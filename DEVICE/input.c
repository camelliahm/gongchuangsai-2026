#include "input.h"
#include "FreeRTOS.h"
#include "task.h"

/*---- 通道硬件资源 ------------------------------------------------*/
static USART_TypeDef *const       s_usart[INPUT_NUM]  = { USART1,       USART3,       USART2       };	//各通道串口
static DMA_Stream_TypeDef *const  s_stream[INPUT_NUM] = { DMA2_Stream5, DMA1_Stream3, DMA1_Stream5 };	//各通道 DMA 流
static const uint32_t             s_chan[INPUT_NUM]   = { DMA_Channel_4, DMA_Channel_4, DMA_Channel_4 };	//各通道 DMA 请求
static const IRQn_Type            s_irqn[INPUT_NUM]   = { USART1_IRQn,   USART3_IRQn,   USART2_IRQn  };	//各通道中断号

#define INPUT_RING_MASK		(INPUT_RING_SIZE - 1u)	//环形缓冲下标掩码；INPUT_RING_SIZE 必须是 2 的幂

//DMA 目标缓冲（4 字节对齐，DMA 直接写入）；双缓冲 ping-pong，每通道两块
static uint8_t s_dmaBuf[INPUT_NUM][2][INPUT_RX_BUF] __attribute__((aligned(4)));	//每通道两块
static volatile uint8_t s_bufSel[INPUT_NUM];	//当前 DMA 写入哪一块：0/1

//环形缓冲：head 在中断里写，tail 在任务里读，单生产者/单消费者
static uint8_t  s_ring[INPUT_NUM][INPUT_RING_SIZE];	//每通道环形缓冲
static volatile uint16_t s_head[INPUT_NUM];			//环形缓冲写指针（中断侧）
static volatile uint16_t s_tail[INPUT_NUM];			//环形缓冲读指针（任务侧）
static volatile uint16_t s_drop[INPUT_NUM];			//环满丢弃的字节数
static volatile uint16_t s_byte[INPUT_NUM];			//累计到达字节数

//通道状态
static InputSlot_t s_slot[INPUT_NUM];	//每通道对外的字节级状态

/*---- 中断服务：IDLE 触发，把本帧从 DMA 缓冲搬进环形缓冲 --------------*/

/**
 * @brief  IDLE 空闲中断处理
 * @param  ch 通道号
 * @note   调用前必须已读 SR 与 DR 清掉 IDLE 标志；本函数不调用任何 FreeRTOS API
 * @author camelliahm
 */
static void input_OnIdle(InputCh_t ch)
{
	uint16_t len;
	uint16_t i;
	uint8_t  cur = s_bufSel[ch];	//本帧落在哪一块（DMA 已冻结，安全可读）

	len = (uint16_t)(INPUT_RX_BUF - DMA_GetCurrDataCounter(s_stream[ch]));

	if (len > 0u)
	{
		s_byte[ch] = (uint16_t)(s_byte[ch] + len);

		//搬的是已冻结块：DMA 即将切走，不再写它，故无需关中断
		for (i = 0u; i < len; i++)
		{
			uint16_t next = (uint16_t)((s_head[ch] + 1u) & INPUT_RING_MASK);

			if (next == s_tail[ch])
			{
				s_drop[ch]++;			//环满：丢新字节，Step 侧由 lockCnt 反映
				continue;
			}

			s_ring[ch][s_head[ch]] = s_dmaBuf[ch][cur][i];
			s_head[ch] = next;
		}
	}

	//切到另一块，DMA 立即在空闲块上重启接收下一帧（Normal 模式传完即停，须手动重置计数）
	s_bufSel[ch] = (uint8_t)(cur ^ 1u);
	DMA_Cmd(s_stream[ch], DISABLE);
	s_stream[ch]->M0AR = (uint32_t)&s_dmaBuf[ch][s_bufSel[ch]][0];
	DMA_SetCurrDataCounter(s_stream[ch], INPUT_RX_BUF);
	DMA_Cmd(s_stream[ch], ENABLE);
}

/**
 * @brief  USART1 中断（视觉模块 1）
 * @note   只认 IDLE；清标志要求"先读 SR 再读 DR"，故先 GetITStatus 再 ReceiveData
 * @author camelliahm
 */
void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		(void)USART_ReceiveData(USART1);
		input_OnIdle(INPUT_VISION1);
	}
}

/**
 * @brief  USART3 中断（视觉模块 2）
 * @author camelliahm
 */
void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
	{
		(void)USART_ReceiveData(USART3);
		input_OnIdle(INPUT_VISION2);
	}
}

/**
 * @brief  USART2 中断（串口屏）
 * @author camelliahm
 */
void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		(void)USART_ReceiveData(USART2);
		input_OnIdle(INPUT_SCREEN);
	}
}

/*---- 周期处理（任务上下文） ----------------------------------------*/

/**
 * @brief  从环形缓冲取一个字节
 * @retval true = 取到
 * @author camelliahm
 */
static bool input_RingPop(InputCh_t ch, uint8_t *b)
{
	if (s_tail[ch] == s_head[ch])
	{
		return false;
	}

	*b = s_ring[ch][s_tail[ch]];
	s_tail[ch] = (uint16_t)((s_tail[ch] + 1u) & INPUT_RING_MASK);
	return true;
}

/**
 * @brief  刷新通道在线标志与丢弃计数
 * @param  ch 通道号
 * @note   在线 = 最近收到过字节；上层应把"离线"解释为"数据不可信"，而非"没有目标"
 * @author camelliahm
 */
static void input_RefreshState(InputCh_t ch)
{
	uint16_t byte = s_byte[ch];

	if (byte != s_slot[ch].rxCnt)
	{
		s_slot[ch].lastTick = xTaskGetTickCount();
	}
	s_slot[ch].rxCnt   = byte;
	s_slot[ch].lockCnt = s_drop[ch];

	if ((xTaskGetTickCount() - s_slot[ch].lastTick) <= pdMS_TO_TICKS(INPUT_TIMEOUT_MS))
	{
		s_slot[ch].online = 1u;
	}
	else
	{
		s_slot[ch].online = 0u;
	}
}

/*---- 摁键（自启动 + 手动调参） --------------------------------------*/

/* 自启动键只在 IDLE / READY 下受理，其余状态（运行中、故障、急停）一律忽略：
   保证摁键不会与状态机或 CAN 后门抢控制权。
   手动调参键同样只在 IDLE / READY 下受理，且与自启动互斥：手动模式下自启动键不动作 */
#define MANUAL_KEY_NUM	(4u)	//手动键数量（模式/减/加 + 双键同按计时位）

//单颗手动键的消抖状态
static struct {
	uint8_t  stable;		//消抖后的稳定电平：1 = 按下
	uint8_t  cnt;			//与 stable 相反的电平已持续的扫描次数
	uint8_t  down;			//1 = 当前稳定按下（供边沿与长按判断）
	uint32_t pressTick;		//按下被确认的时刻
} s_manualKey[MANUAL_KEY_NUM];

//自启动键状态
static struct {
	uint8_t  stable;		//消抖后的稳定电平：1 = 按下
	uint8_t  cnt;			//与 stable 相反的电平已持续的扫描次数
	uint8_t  wait;			//1 = 倒计时中，到点投启动事件
	uint32_t pressTick;		//按下被确认的时刻
} s_startKey;

//手动调参状态
static struct {
	uint8_t  active;		//1 = 手动模式中
	uint8_t  obj;			//当前调节对象 ManualObj_t
	uint8_t  pend;			//待消费的动作意图 ManualAct_t（NONE = 空）
	uint8_t  zeroLatched;	//1 = 双键清零已触发，须松开两键才复位（防一直按着反复清零）
} s_manual;

//启停区选择状态（现场抽签结果，PE3 非手动短按切换）
static struct {
	uint8_t  sel;			//当前选定：1 或 2
	uint8_t  pend;			//1 = 发生过一次切换，待上层取走
} s_dock;

/**
 * @brief  读指定引脚电平并折算成"是否按下"
 * @param  port 端口
 * @param  pin  引脚
 * @note   返回 1 = 按下；有效电平由 KEY_ACTIVE_LOW 决定
 * @author camelliahm
 */
static uint8_t input_PinDown(GPIO_TypeDef *port, uint16_t pin)
{
	uint8_t bit = (GPIO_ReadInputDataBit(port, pin) == Bit_SET) ? 1u : 0u;

	return (KEY_ACTIVE_LOW != 0u) ? (uint8_t)(1u - bit) : bit;
}

/**
 * @brief  单颗键消抖扫描
 * @param  k     键状态
 * @param  port  端口
 * @param  pin   引脚
 * @note   返回 1 = 本周期确认到一次"按下边沿"；电平抖动由 cnt 累计判定
 * @author camelliahm
 */
static uint8_t input_KeyScan(uint8_t *stable, uint8_t *cnt, GPIO_TypeDef *port, uint16_t pin)
{
	uint8_t down = input_PinDown(port, pin);

	if (down != *stable)
	{
		(*cnt)++;

		if (*cnt >= KEY_DEBOUNCE_CNT)
		{
			*stable = down;
			*cnt    = 0u;
			return (down != 0u) ? 1u : 0u;		//只在"按下"边沿返回 1
		}
	}
	else
	{
		*cnt = 0u;
	}

	return 0u;
}

/**
 * @brief  自启动键扫描：消抖 → 按下即起倒计时 → 到点投 EV_ENABLE + EV_START
 * @note   按下即锁存，中途松手不取消；上电以当前电平为基准，故"上电就被压住"不触发。
 *         手动模式下不扫描自启动键，避免误触发跑图
 * @author camelliahm
 */
static void input_StartKeyStep(void)
{
	uint8_t down;
	uint8_t edge;

	if (s_manual.active != 0u)
	{
		return;		//手动调参中，自启动键失效
	}

	down = input_PinDown(KEY_PORT, KEY_PIN);

	if (down != s_startKey.stable)
	{
		s_startKey.cnt++;

		if (s_startKey.cnt >= KEY_DEBOUNCE_CNT)
		{
			s_startKey.stable = down;
			s_startKey.cnt    = 0u;
			edge = (down != 0u) ? 1u : 0u;

			if ((edge != 0u) &&
				((chassis.state == CHS_IDLE) || (chassis.state == CHS_READY)))
			{
				s_startKey.wait      = 1u;
				s_startKey.pressTick = xTaskGetTickCount();
			}
		}
	}
	else
	{
		s_startKey.cnt = 0u;
	}

	if ((s_startKey.wait != 0u) &&
		((xTaskGetTickCount() - s_startKey.pressTick) >= pdMS_TO_TICKS(KEY_START_DELAY_MS)))
	{
		s_startKey.wait = 0u;

		//延时内状态可能已被后门改动，到点重新判定一次；IDLE 要连使能一起投
		if (chassis.state == CHS_IDLE)
		{
			contract_PostEvent(EV_ENABLE, 0);
			contract_PostEvent(EV_START, 0);
		}
		else if (chassis.state == CHS_READY)
		{
			contract_PostEvent(EV_START, 0);
		}
	}
}

/**
 * @brief  手动调参键扫描：模式键（短按切对象/长按进退）、减键、加键
 * @note   只在 IDLE / READY 下受理；减/加动作只在手动模式下有效。
 *         模式键按下先记边沿与时刻，松手后按持续时长判定短按/长按
 * @author camelliahm
 */
static void input_ManualKeyStep(void)
{
	uint8_t edge;

	//减键、加键：只认按下边沿
	if (s_manual.active != 0u)
	{
		edge = input_KeyScan(&s_manualKey[1].stable, &s_manualKey[1].cnt, KEY_MANUAL_PORT, KEY_DEC_PIN);
		if (edge != 0u)
		{
			s_manual.pend = MANUAL_ACT_DEC;
		}

		edge = input_KeyScan(&s_manualKey[2].stable, &s_manualKey[2].cnt, KEY_MANUAL_PORT, KEY_INC_PIN);
		if (edge != 0u)
		{
			s_manual.pend = MANUAL_ACT_INC;
		}

		/* 双键同按（DEC+INC）保持 KEY_ZERO_PRESS_MS：把升降轴当前位置重标为零。
		 * 同按瞬间 DEC / INC 各会产生一次按下边沿，故此处撤销该意图 —— 否则机构会先动一下。
		 * 一直按着只触发一次，松开两键才复位（zeroLatched） */
		if ((s_manualKey[1].stable != 0u) && (s_manualKey[2].stable != 0u))
		{
			s_manual.pend = MANUAL_ACT_NONE;

			if (s_manual.zeroLatched == 0u)
			{
				if (s_manualKey[3].down == 0u)
				{
					s_manualKey[3].down      = 1u;
					s_manualKey[3].pressTick = xTaskGetTickCount();
				}
				else if ((xTaskGetTickCount() - s_manualKey[3].pressTick) >= pdMS_TO_TICKS(KEY_ZERO_PRESS_MS))
				{
					s_manual.zeroLatched = 1u;
					s_manual.pend        = MANUAL_ACT_ZERO;
				}
			}
		}
		else
		{
			s_manualKey[3].down  = 0u;
			s_manual.zeroLatched = 0u;
		}
	}

	//模式键：消抖后区分短按/长按（松手即判）
	{
		uint8_t down = input_PinDown(KEY_MANUAL_PORT, KEY_MODE_PIN);

		if (down != s_manualKey[0].stable)
		{
			s_manualKey[0].cnt++;

			if (s_manualKey[0].cnt >= KEY_DEBOUNCE_CNT)
			{
				s_manualKey[0].stable = down;
				s_manualKey[0].cnt    = 0u;

				if (down != 0u)
				{
					s_manualKey[0].down      = 1u;
					s_manualKey[0].pressTick = xTaskGetTickCount();
				}
				else
				{
					//松手：按按住时长判定短按/长按
					uint32_t held = xTaskGetTickCount() - s_manualKey[0].pressTick;

					s_manualKey[0].down = 0u;

					if ((chassis.state != CHS_IDLE) && (chassis.state != CHS_READY))
					{
						return;		//非空闲/就绪态不响应模式键
					}

					if (held >= pdMS_TO_TICKS(KEY_LONG_PRESS_MS))
					{
						//长按：进/退手动模式
						if (s_manual.active != 0u)
						{
							s_manual.active = 0u;
							s_manual.pend   = MANUAL_ACT_EXIT;
						}
						else
						{
							s_manual.active      = 1u;
							s_manual.obj         = MANUAL_LIFT;
							s_manual.pend        = MANUAL_ACT_ENTER;
							s_manual.zeroLatched = 0u;		//新一次进入：双键清零复位
						}
					}
					else
					{
						//短按：手动模式下切换对象，非手动时切换启停区（现场抽签）
						if (s_manual.active != 0u)
						{
							s_manual.obj = (uint8_t)((s_manual.obj + 1u) % MANUAL_OBJ_NUM);
							s_manual.pend = MANUAL_ACT_NEXT;
						}
						else
						{
							s_dock.sel  = (uint8_t)((s_dock.sel == 1u) ? 2u : 1u);
							s_dock.pend = 1u;
						}
					}
				}
			}
		}
		else
		{
			s_manualKey[0].cnt = 0u;
		}
	}
}

/*---- 对外接口 ----------------------------------------------------*/

/**
 * @brief  输入汇聚层初始化
 * @note   三条串口分别占 PA9/PA10、PB10/PB11、PA2/PA3；接收走 DMA + IDLE，无接收中断风暴。
 *         本函数不做任何帧解析：返回后三条通道只把到达字节搬进环形缓冲。
 *         摁键只配成输入，不占中断，由控制环轮询消抖
 * @author camelliahm
 */
void input_Init(void)
{
	GPIO_InitTypeDef g;
	USART_InitTypeDef u;
	DMA_InitTypeDef d;
	NVIC_InitTypeDef n;
	uint8_t i;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB |
						   RCC_AHB1Periph_DMA1 | RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2 | RCC_APB1Periph_USART3, ENABLE);

	//PA2/PA3 → USART2，PA9/PA10 → USART1
	g.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_9 | GPIO_Pin_10;
	g.GPIO_Mode  = GPIO_Mode_AF;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_PuPd  = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &g);

	//PB10/PB11 → USART3
	g.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_Init(GPIOB, &g);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2,  GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3,  GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	for (i = 0u; i < INPUT_NUM; i++)
	{
		//串口：8N1，收发均配（当前只用于收，留 TX 便于回包）
		u.USART_BaudRate            = INPUT_BAUD;		u.USART_WordLength          = USART_WordLength_8b;
		u.USART_StopBits            = USART_StopBits_1;
		u.USART_Parity              = USART_Parity_No;
		u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		u.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(s_usart[i], &u);

		//DMA：外设 → 内存，字节宽度，Normal 模式（一帧搬一次）
		DMA_DeInit(s_stream[i]);
		d.DMA_Channel            = s_chan[i];
		d.DMA_PeripheralBaseAddr = (uint32_t)(&(s_usart[i]->DR));
		d.DMA_Memory0BaseAddr    = (uint32_t)s_dmaBuf[i];
		d.DMA_DIR                = DMA_DIR_PeripheralToMemory;
		d.DMA_BufferSize         = INPUT_RX_BUF;		d.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
		d.DMA_MemoryInc          = DMA_MemoryInc_Enable;
		d.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
		d.DMA_MemoryDataSize     = DMA_MemoryDataSize_Byte;
		d.DMA_Mode               = DMA_Mode_Normal;
		d.DMA_Priority           = DMA_Priority_Medium;
		d.DMA_FIFOMode           = DMA_FIFOMode_Disable;
		d.DMA_FIFOThreshold      = DMA_FIFOThreshold_HalfFull;
		d.DMA_MemoryBurst        = DMA_MemoryBurst_Single;
		d.DMA_PeripheralBurst    = DMA_PeripheralBurst_Single;
		DMA_Init(s_stream[i], &d);
		s_bufSel[i] = 0u;	//初始 DMA 写块 0

		//中断：只开 IDLE，不调 FreeRTOS API，故抢占优先级按普通外设给 5
		n.NVIC_IRQChannel                   = s_irqn[i];
		n.NVIC_IRQChannelPreemptionPriority = 5;
		n.NVIC_IRQChannelSubPriority        = 0;
		n.NVIC_IRQChannelCmd                = ENABLE;
		NVIC_Init(&n);

		USART_DMACmd(s_usart[i], USART_DMAReq_Rx, ENABLE);
		USART_ITConfig(s_usart[i], USART_IT_IDLE, ENABLE);
		USART_Cmd(s_usart[i], ENABLE);
		DMA_Cmd(s_stream[i], ENABLE);
	}

	//摁键：自启动键 + 三颗手动键，只配成 GPIO 输入，无中断、无 DMA
	RCC_AHB1PeriphClockCmd(KEY_CLK, ENABLE);

	g.GPIO_Pin   = KEY_PIN | KEY_MODE_PIN | KEY_DEC_PIN | KEY_INC_PIN;
	g.GPIO_Mode  = GPIO_Mode_IN;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_PuPd  = KEY_PUPD;
	g.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(KEY_PORT, &g);

	//以当前电平为基准：上电就被压住时不产生"按下"边沿
	s_startKey.stable    = input_PinDown(KEY_PORT, KEY_PIN);
	s_startKey.cnt       = 0u;
	s_startKey.wait      = 0u;
	s_startKey.pressTick = 0u;

	s_manualKey[0].stable = input_PinDown(KEY_MANUAL_PORT, KEY_MODE_PIN);
	s_manualKey[1].stable = input_PinDown(KEY_MANUAL_PORT, KEY_DEC_PIN);
	s_manualKey[2].stable = input_PinDown(KEY_MANUAL_PORT, KEY_INC_PIN);

	s_manual.active = 0u;
	s_manual.obj    = MANUAL_LIFT;
	s_manual.pend   = MANUAL_ACT_NONE;

	s_dock.sel  = 1u;	//上电默认启停区 1，抽签结果由操作者按 PE3 切换
	s_dock.pend = 0u;
}

/**
 * @brief  周期处理：刷新三条通道的在线标志与丢弃计数，并扫描摁键
 * @note   本模块不做解析：字节只停在环形缓冲里、由各协议模块取用，无人取用时 lockCnt 会持续增长。
 *         串口屏通道由 screen_Step() 消费，视觉两条通道由 vision1_Step() / vision_Step() 消费。
 *         必须每周期调用 —— 摁键的消抖与自启动倒计时都按调用次数/时刻推进
 * @author camelliahm
 */
void input_Step(void)
{
	uint8_t i;

	for (i = 0u; i < INPUT_NUM; i++)
	{
		input_RefreshState((InputCh_t)i);
	}

	input_StartKeyStep();
	input_ManualKeyStep();
}

/**
 * @brief  取走原始字节
 * @param  ch  通道号
 * @param  dst 目标缓冲
 * @param  max 最多拷贝字节数
 * @retval 实际拷贝的字节数；0 = 无数据或参数非法
 * @note   与中断侧构成单生产者/单消费者；须在任务上下文调用（串口屏通道由 screen_Step() 消费）
 * @author camelliahm
 */
uint16_t input_Read(InputCh_t ch, uint8_t *dst, uint16_t max)
{
	uint16_t n = 0u;

	if ((ch >= INPUT_NUM) || (dst == NULL))
	{
		return 0u;
	}

	while ((n < max) && input_RingPop(ch, &dst[n]))
	{
		n++;
	}

	return n;
}

/**
 * @brief  阻塞式发送
 * @param  ch  通道号
 * @param  src 数据起始
 * @param  len 长度
 * @retval 实际发出的字节数；0 = 参数非法
 * @note   忙等 TXE/TC，不可在中断或临界区内调用；总线未接时可能长时间阻塞
 * @author camelliahm
 */
uint16_t input_Write(InputCh_t ch, const uint8_t *src, uint16_t len)
{
	uint16_t i;

	if ((ch >= INPUT_NUM) || (src == NULL))
	{
		return 0u;
	}

	for (i = 0u; i < len; i++)
	{
		while (USART_GetFlagStatus(s_usart[ch], USART_FLAG_TXE) == RESET)
		{
		}
		USART_SendData(s_usart[ch], src[i]);
	}

	while (USART_GetFlagStatus(s_usart[ch], USART_FLAG_TC) == RESET)
	{
	}

	return len;
}

/**
 * @brief  只读槽位
 * @retval 槽位指针；通道号非法时返回 NULL
 * @author camelliahm
 */
const InputSlot_t *input_GetSlot(InputCh_t ch)
{
	return (ch < INPUT_NUM) ? &s_slot[ch] : NULL;
}

/*---- 手动调参对外接口 ---------------------------------------------*/

/**
 * @brief  当前是否处于手动调参模式
 * @note   供 OLED 选页与辅助机构分派判断
 * @author camelliahm
 */
uint8_t input_ManualActive(void)
{
	return s_manual.active;
}

/**
 * @brief  当前调节对象 ManualObj_t
 * @author camelliahm
 */
uint8_t input_ManualObj(void)
{
	return s_manual.obj;
}

/**
 * @brief  取走一个手动动作意图
 * @note   读取后即清空，MANUAL_ACT_NONE 表示无新意图；供辅助机构分派
 * @author camelliahm
 */
uint8_t input_ManualPop(void)
{
	uint8_t act = s_manual.pend;

	s_manual.pend = MANUAL_ACT_NONE;

	return act;
}

/*---- 启停区选择对外接口 -------------------------------------------*/

/**
 * @brief  读当前选定的启停区号
 * @retval 1 或 2
 * @author camelliahm
 */
uint8_t input_DockSel(void)
{
	return s_dock.sel;
}

/**
 * @brief  取走一次启停区切换事件
 * @retval 0 = 本次无切换；1/2 = 刚切到的启停区号
 * @note   读取后即清标志；供编排层转给 nav_SetStart()，本层不直接依赖导航模块
 * @author camelliahm
 */
uint8_t input_DockPop(void)
{
	uint8_t sel = 0u;

	if (s_dock.pend != 0u)
	{
		s_dock.pend = 0u;
		sel = s_dock.sel;
	}

	return sel;
}
