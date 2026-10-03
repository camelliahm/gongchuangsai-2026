#include "input.h"
#include "FreeRTOS.h"
#include "task.h"

/*---- 通道硬件资源：USART1 → 视觉，USART2 → 二维码 ----*/
static USART_TypeDef *const       s_usart[INPUT_NUM]  = { USART1,       USART2       };
static DMA_Stream_TypeDef *const  s_stream[INPUT_NUM] = { DMA2_Stream5, DMA1_Stream5 };
static const uint32_t             s_chan[INPUT_NUM]   = { DMA_Channel_4, DMA_Channel_4 };

#define INPUT_BAUD			(115200u)
#define INPUT_RING_MASK		(INPUT_RING_SIZE - 1u)	/* INPUT_RING_SIZE 必须是 2 的幂 */

/* DMA 目标缓冲（4 字节对齐） */
__align(4) static uint8_t s_dmaBuf[INPUT_NUM][INPUT_RX_BUF];

/* 环形缓冲：head 在中断里写，tail 在任务里读，单生产者/单消费者 */
static uint8_t  s_ring[INPUT_NUM][INPUT_RING_SIZE];
static volatile uint16_t s_head[INPUT_NUM];
static volatile uint16_t s_tail[INPUT_NUM];
static volatile uint16_t s_drop[INPUT_NUM];

/* 解析结果 */
static InputSlot_t s_slot[INPUT_NUM];
static uint16_t    s_lineLen[INPUT_NUM];

/*----------------------------------------------------------*/
/*  中断服务：IDLE 触发，把本帧从 DMA 缓冲搬进环形缓冲           */
/*----------------------------------------------------------*/

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
		primask = __get_PRIMASK();
		__disable_irq();

		for (i = 0u; i < len; i++)
		{
			uint16_t next = (uint16_t)((s_head[ch] + 1u) & INPUT_RING_MASK);

			if (next == s_tail[ch])
			{
				s_drop[ch]++;			/* 环满：丢新字节 */
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
 * @brief  USART1 中断（视觉）
 * @note   只认 IDLE；清标志要求"先读 SR 再读 DR"，故先 GetITStatus 再 ReceiveData
 * @author camelliahm
 */
void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		(void)USART_ReceiveData(USART1);
		input_OnIdle(INPUT_VISION);
	}
}

/**
 * @brief  USART2 中断（二维码）
 * @author camelliahm
 */
void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		(void)USART_ReceiveData(USART2);
		input_OnIdle(INPUT_QRCODE);
	}
}

/*----------------------------------------------------------*/
/*  周期处理：拆行 + 在线判断（任务上下文）                      */
/*----------------------------------------------------------*/

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
 * @brief  把环形缓冲里的字节按行切分，填进槽位
 * @note   以 '\n' 为帧尾，'\r' 丢弃；超长行丢弃重来；槽位 line 始终存"最新一行"
 * @author camelliahm
 */
static void input_ParseRing(InputCh_t ch)
{
	uint8_t b;

	while (input_RingPop(ch, &b))
	{
		if (b == (uint8_t)'\n')
		{
			s_slot[ch].line[s_lineLen[ch]] = '\0';
			s_slot[ch].lineLen  = s_lineLen[ch];
			s_slot[ch].rxCnt++;
			s_slot[ch].lastTick = xTaskGetTickCount();
			s_slot[ch].fresh    = 1u;
			s_lineLen[ch] = 0u;
		}
		else if (b != (uint8_t)'\r')
		{
			if (s_lineLen[ch] < (INPUT_LINE_MAX - 1u))
			{
				s_slot[ch].line[s_lineLen[ch]] = (char)b;
				s_lineLen[ch]++;
			}
			else
			{
				s_lineLen[ch] = 0u;		/* 溢出：整行作废，等下一个 '\n' */
				s_slot[ch].dropCnt++;
			}
		}
	}
}

/**
 * @brief  刷新通道在线标志
 * @note   超时即判离线——上层应把"离线"解释为"数据不可信"，而非"没有目标"
 * @author camelliahm
 */
static void input_CheckOnline(InputCh_t ch)
{
	if ((xTaskGetTickCount() - s_slot[ch].lastTick) <= pdMS_TO_TICKS(INPUT_TIMEOUT_MS))
	{
		s_slot[ch].online = 1u;
	}
	else
	{
		s_slot[ch].online = 0u;
	}
}

/*----------------------------------------------------------*/
/*  对外接口                                                    */
/*----------------------------------------------------------*/

/**
 * @brief  输入汇聚层初始化
 * @note   USART1 = PA9/PA10（APB2），USART2 = PA2/PA3（APB1）；接收走 DMA + IDLE，无接收中断风暴
 * @author camelliahm
 */
void input_Init(void)
{
	GPIO_InitTypeDef g;
	USART_InitTypeDef u;
	DMA_InitTypeDef d;
	NVIC_InitTypeDef n;
	uint8_t i;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1 | RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* PA2/PA3 → USART2，PA9/PA10 → USART1 */
	g.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_9 | GPIO_Pin_10;
	g.GPIO_Mode  = GPIO_Mode_AF;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_PuPd  = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &g);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2,  GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3,  GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9,  GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

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
		n.NVIC_IRQChannel                   = (i == INPUT_VISION) ? USART1_IRQn : USART2_IRQn;
		n.NVIC_IRQChannelPreemptionPriority = 5;
		n.NVIC_IRQChannelSubPriority        = 0;
		n.NVIC_IRQChannelCmd                = ENABLE;
		NVIC_Init(&n);

		USART_DMACmd(s_usart[i], USART_DMAReq_Rx, ENABLE);
		USART_ITConfig(s_usart[i], USART_IT_IDLE, ENABLE);
		USART_Cmd(s_usart[i], ENABLE);
		DMA_Cmd(s_stream[i], ENABLE);
	}
}

/**
 * @brief  周期处理：拆行并刷新在线标志
 * @note   暂未接入控制环；接入方式见 README.md。须与 input_TakeLine() 在同一任务上下文调用
 * @author camelliahm
 */
void input_Step(void)
{
	uint8_t i;

	for (i = 0u; i < INPUT_NUM; i++)
	{
		input_ParseRing((InputCh_t)i);
		input_CheckOnline((InputCh_t)i);
	}
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

/**
 * @brief  取走最新一行
 * @param  ch   通道号
 * @param  dst  目标缓冲
 * @param  size 目标缓冲大小（含 '\0'）
 * @retval true = 取到一行
 * @author camelliahm
 */
bool input_TakeLine(InputCh_t ch, char *dst, uint16_t size)
{
	uint16_t i;

	if ((ch >= INPUT_NUM) || (dst == NULL) || (size == 0u) || (s_slot[ch].fresh == 0u))
	{
		return false;
	}

	for (i = 0u; (i < s_slot[ch].lineLen) && (i < (uint16_t)(size - 1u)); i++)
	{
		dst[i] = s_slot[ch].line[i];
	}
	dst[i] = '\0';

	s_slot[ch].fresh = 0u;
	return true;
}

/**
 * @brief  丢弃未读帧（只清 fresh，不取用）
 * @author camelliahm
 */
void input_ClearLine(InputCh_t ch)
{
	if (ch < INPUT_NUM)
	{
		s_slot[ch].fresh = 0u;
	}
}
