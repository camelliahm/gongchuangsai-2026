#include "input.h"
#include "screen.h"
#include "FreeRTOS.h"
#include "task.h"

/*---- 通道硬件资源 ------------------------------------------------*/
static USART_TypeDef *const       s_usart[INPUT_NUM]  = { USART1,       USART3,       USART2       };
static DMA_Stream_TypeDef *const  s_stream[INPUT_NUM] = { DMA2_Stream5, DMA1_Stream3, DMA1_Stream5 };
static const uint32_t             s_chan[INPUT_NUM]   = { DMA_Channel_4, DMA_Channel_4, DMA_Channel_4 };
static const IRQn_Type            s_irqn[INPUT_NUM]   = { USART1_IRQn,   USART3_IRQn,   USART2_IRQn  };

#define INPUT_RING_MASK		(INPUT_RING_SIZE - 1u)	/* INPUT_RING_SIZE 必须是 2 的幂 */

/* DMA 目标缓冲（4 字节对齐） */
__align(4) static uint8_t s_dmaBuf[INPUT_NUM][INPUT_RX_BUF];

/* 环形缓冲：head 在中断里写，tail 在任务里读，单生产者/单消费者 */
static uint8_t  s_ring[INPUT_NUM][INPUT_RING_SIZE];
static volatile uint16_t s_head[INPUT_NUM];
static volatile uint16_t s_tail[INPUT_NUM];
static volatile uint16_t s_drop[INPUT_NUM];	/* 环满丢弃的字节数 */
static volatile uint16_t s_byte[INPUT_NUM];	/* 累计到达字节数 */

/* 通道状态 */
static InputSlot_t s_slot[INPUT_NUM];

/* 视觉模块 2 的帧槽位；帧格式未定，当前只被 input_Init() 归零 */
__IO VisionSlot_t visionSlot;

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
	uint32_t primask;

	len = (uint16_t)(INPUT_RX_BUF - DMA_GetCurrDataCounter(s_stream[ch]));

	if (len > 0u)
	{
		s_byte[ch] = (uint16_t)(s_byte[ch] + len);

		primask = __get_PRIMASK();
		__disable_irq();

		for (i = 0u; i < len; i++)
		{
			uint16_t next = (uint16_t)((s_head[ch] + 1u) & INPUT_RING_MASK);

			if (next == s_tail[ch])
			{
				s_drop[ch]++;			/* 环满：丢新字节，Step 侧由 lockCnt 反映 */
				continue;
			}

			s_ring[ch][s_head[ch]] = s_dmaBuf[ch][i];
			s_head[ch] = next;
		}

		__set_PRIMASK(primask);
	}

	/* 重启 DMA 接收下一帧（Normal 模式传完即停，须手动重置计数） */
	DMA_Cmd(s_stream[ch], DISABLE);
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

/*---- 对外接口 ----------------------------------------------------*/

/**
 * @brief  输入汇聚层初始化
 * @note   三条串口分别占 PA9/PA10、PB10/PB11、PA2/PA3；接收走 DMA + IDLE，无接收中断风暴。
 *         本函数不做任何帧解析，返回后三条通道只把到达字节搬进环形缓冲
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

	/* PA2/PA3 → USART2，PA9/PA10 → USART1 */
	g.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_9 | GPIO_Pin_10;
	g.GPIO_Mode  = GPIO_Mode_AF;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_PuPd  = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &g);

	/* PB10/PB11 → USART3 */
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
		/* 串口：8N1，收发均配（当前只用于收，留 TX 便于回包） */
		u.USART_BaudRate            = INPUT_BAUD;
		u.USART_WordLength          = USART_WordLength_8b;
		u.USART_StopBits            = USART_StopBits_1;
		u.USART_Parity              = USART_Parity_No;
		u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		u.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
		USART_Init(s_usart[i], &u);

		/* DMA：外设 → 内存，字节宽度，Normal 模式（一帧搬一次） */
		DMA_DeInit(s_stream[i]);
		d.DMA_Channel            = s_chan[i];
		d.DMA_PeripheralBaseAddr = (uint32_t)(&(s_usart[i]->DR));
		d.DMA_Memory0BaseAddr    = (uint32_t)s_dmaBuf[i];
		d.DMA_DIR                = DMA_DIR_PeripheralToMemory;
		d.DMA_BufferSize         = INPUT_RX_BUF;
		d.DMA_PeripheralInc      = DMA_PeripheralInc_Disable;
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

		/* 中断：只开 IDLE，不调 FreeRTOS API，故抢占优先级按普通外设给 5 */
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

	visionSlot.online      = 0u;
	visionSlot.fresh       = 0u;
	visionSlot.count       = 0u;
	visionSlot.seq         = 0u;
	visionSlot.rxCnt       = 0u;
	visionSlot.errCnt      = 0u;
	visionSlot.task.taskId = VISION_TASK_ID_NONE;
	visionSlot.lastTick    = xTaskGetTickCount();

	screen_Init();			// 串口屏软件槽位归零（USART2 硬件已在上面的循环里配好）
}

/**
 * @brief  周期处理：刷新三条通道的在线标志与丢弃计数
 * @note   本模块不做解析：字节只停在环形缓冲里、由 input_Read() 取用，无人取用时 lockCnt 会持续增长。
 *         串口屏通道由 screen_Step() 消费；两路视觉的解析待帧格式定稿后再加
 * @author camelliahm
 */
void input_Step(void)
{
	uint8_t i;

	for (i = 0u; i < INPUT_NUM; i++)
	{
		input_RefreshState((InputCh_t)i);
	}
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

/*---- 视觉模块 2 帧槽位 --------------------------------------------*/

/**
 * @brief  周期处理：刷新视觉槽位在线标志
 * @note   解析未实现，lastTick 只在收到有效帧时才会更新，故当前恒判离线（= 数据不可信）
 * @author camelliahm
 */
void vision_Step(void)
{
	if ((xTaskGetTickCount() - visionSlot.lastTick) <= pdMS_TO_TICKS(VISION_FRAME_TIMEOUT_MS))
	{
		visionSlot.online = 1u;
	}
	else
	{
		visionSlot.online = 0u;
	}
}

/**
 * @brief  取走一帧障碍物
 * @param  dst 目标缓冲（须至少有 max 条容量）
 * @param  max 缓冲区可容纳条数
 * @retval 实际拷贝的条数；0 = 无新帧
 * @note   取走后清 fresh，因此同一帧只会被消费一次
 * @author camelliahm
 */
uint8_t vision_TakeFrame(VisionObst_t *dst, uint8_t max)
{
	uint8_t n;
	uint8_t i;

	if ((dst == NULL) || (max == 0u) || (visionSlot.fresh == 0u))
	{
		return 0u;
	}

	n = visionSlot.count;
	if (n > max)
	{
		n = max;
	}

	for (i = 0u; i < n; i++)
	{
		dst[i] = visionSlot.obst[i];
	}

	visionSlot.fresh = 0u;

	return n;
}

/**
 * @brief  取走任务号（二维码）
 * @param  dst 目标结构
 * @retval true = 取到
 * @note   取走后槽位置回 VISION_TASK_ID_NONE，因此同一任务号只会被消费一次
 * @author camelliahm
 */
bool vision_TakeTask(VisionTask_t *dst)
{
	if ((dst == NULL) || (visionSlot.task.taskId == VISION_TASK_ID_NONE))
	{
		return false;
	}

	dst->taskId = visionSlot.task.taskId;
	visionSlot.task.taskId = VISION_TASK_ID_NONE;

	return true;
}

/**
 * @brief  只读槽位
 * @author camelliahm
 */
const VisionSlot_t *vision_GetSlot(void)
{
	return (const VisionSlot_t *)&visionSlot;
}

/**
 * @brief  查询有效识别距离
 * @retval 距离(mm)；0 = 未知（未标定）
 * @note   0 是"不可信"哨兵而非"距离为零"，调用方不得据此放行速度
 * @author camelliahm
 */
int32_t vision_GetRangeMM(void)
{
	return (int32_t)VISION_RANGE_MM;
}
