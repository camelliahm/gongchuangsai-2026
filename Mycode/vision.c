#include "vision.h"
#include "FreeRTOS.h"
#include "task.h"

/* 视觉/二维码层：接收视觉模块的结构化上报帧，详见 README.md
 * ⚠ 接口与流程已完整落地，仅"内容格式"与"视距"留空，见 vision.h 标注。 */

/* DMA 目标缓冲（4 字节对齐） */
__align(4) static uint8_t s_dmaBuf[VISION_RX_BUF];

/* 解析结果槽位：中断只搬字节，解析全部在 vision_Step()（任务上下文）完成 */
__IO VisionSlot_t visionSlot;

static void vision_OnIdle(const uint8_t *buf, uint16_t len);

/**
 * @brief  USART3 空闲中断（视觉模块 2）
 * @note   清 IDLE 标志要求"先读 SR 再读 DR"；本 ISR 不调用任何 FreeRTOS API
 * @author camelliahm
 */
void USART3_IRQHandler(void)
{
	if (USART_GetITStatus(VISION_USART, USART_IT_IDLE) != RESET)
	{
		uint16_t len;

		(void)USART_ReceiveData(VISION_USART);		/* 读 DR 清 IDLE */

		/* 本帧实际长度；⚠ 就地解析（DMA 缓冲在下一次中断前不会被改写） */
		len = (uint16_t)(VISION_RX_BUF - DMA_GetCurrDataCounter(VISION_DMA_STREAM));

		vision_OnIdle(s_dmaBuf, len);

		/* 重启 DMA 接收下一帧（Normal 模式传完即停） */
		DMA_Cmd(VISION_DMA_STREAM, DISABLE);
		DMA_SetCurrDataCounter(VISION_DMA_STREAM, VISION_RX_BUF);
		DMA_Cmd(VISION_DMA_STREAM, ENABLE);
	}
}

/*---- 帧解析（任务上下文） ----------------------------------*/
/**
 * @brief  校验字节冗余和（帧内除校验位外逐字节相加）
 * @param  buf 帧缓冲  len 帧长（含校验位）
 * @retval true = 校验通过
 * @note   ⚠ 校验方式为占位约定，待视觉模块规格确定后可能替换为 CRC8/CRC16
 * @author camelliahm
 */
static bool vision_CheckSum(const uint8_t *buf, uint16_t len)
{
	uint8_t sum = 0u;
	uint16_t i;

	if (len < 4u)
	{
		return false;
	}

	/* 末字节为校验位，其前逐字节累加 */
	for (i = 0u; i < (uint16_t)(len - 1u); i++)
	{
		sum = (uint8_t)(sum + buf[i]);
	}

	return (sum == buf[len - 1u]);
}

/**
 * @brief  解析一帧并填入槽位
 * @param  buf 起始指针  len 帧长
 * @retval true = 帧有效（槽位已更新）
 * @note   ⚠ 载荷布局为占位约定：帧头 / 障碍条数 / 逐条记录 / 任务信息 / 校验。
 *         视觉模块 2 规格确定后只需改此函数体 + vision.h 的帧头宏与结构体。
 * @author camelliahm
 */
static bool vision_Parse(const uint8_t *buf, uint16_t len)
{
	uint8_t  cnt;
	uint8_t  i;
	uint8_t  taskLen;
	uint16_t need;
	const uint8_t *p;

	if ((len < 5u) || (buf[0] != VISION_FRAME_HEAD) || (buf[len - 1u] != VISION_FRAME_TAIL))
	{
		return false;
	}

	if (!vision_CheckSum(buf, len))
	{
		return false;
	}

	/* 载荷：[1] = 障碍条数，其后每条 7 字节（x/y/width 各 int16 大端 + confidence） */
	cnt = buf[1];
	if (cnt > VISION_OBST_MAX)
	{
		cnt = VISION_OBST_MAX;
	}

	p    = &buf[2];
	need = (uint16_t)(2u + (uint16_t)cnt * 7u + 2u + 1u);	/* 头 + 条数 + 记录 + 任务长度 + 校验 */
	if (len < need)
	{
		return false;
	}

	for (i = 0u; i < cnt; i++)
	{
		visionSlot.obst[i].x    = (int16_t)(((uint16_t)p[0] << 8) | (uint16_t)p[1]);
		visionSlot.obst[i].y    = (int16_t)(((uint16_t)p[2] << 8) | (uint16_t)p[3]);
		visionSlot.obst[i].width= (int16_t)(((uint16_t)p[4] << 8) | (uint16_t)p[5]);
		visionSlot.obst[i].confidence = p[6];
		p += 7;
	}

	/* 任务信息（二维码）：[长度 1 字节][内容 N 字节]；⚠ 语义留空，原样搬运 */
	taskLen = p[0];
	p++;
	if (taskLen > VISION_TASK_LEN)
	{
		taskLen = VISION_TASK_LEN;
	}
	if ((uint16_t)(p - buf) + (uint16_t)taskLen + 1u > len)
	{
		return false;
	}

	for (i = 0u; i < taskLen; i++)
	{
		visionSlot.task.data[i] = p[i];
	}
	visionSlot.task.len = taskLen;

	visionSlot.count    = cnt;
	visionSlot.lastTick = xTaskGetTickCount();
	visionSlot.rxCnt++;
	visionSlot.fresh    = 1u;

	return true;
}

/**
 * @brief  收到一帧原始字节（由 USART3_IRQHandler 调用）
 * @param  buf 字节缓冲  len 长度
 * @note   ⚠ 当前在中断内直接解析：DMA 缓冲在下一次 IDLE 前不会被改写，且解析
 *         只用局部变量与槽位字段，不调 FreeRTOS API，故安全。
 *         若将来帧变长导致 ISR 超时，可改为"中断搬进环形缓冲 + vision_Step() 解析"。
 * @author camelliahm
 */
static void vision_OnIdle(const uint8_t *buf, uint16_t len)
{
	if (!vision_Parse(buf, len))
	{
		visionSlot.errCnt++;
	}
}

/*---- 对外接口 ----------------------------------------*/
/**
 * @brief  初始化（时钟、GPIO、DMA、USART、NVIC）
 * @note   独立占用 USART3（PB10/PB11）与 DMA1 Stream3，与 input.c 的
 *         USART1/USART2 完全物理隔离，互不干扰
 * @author camelliahm
 */
void vision_Init(void)
{
	GPIO_InitTypeDef g;
	USART_InitTypeDef u;
	DMA_InitTypeDef d;
	NVIC_InitTypeDef n;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	g.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
	g.GPIO_Mode  = GPIO_Mode_AF;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_PuPd  = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &g);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3);

	u.USART_BaudRate            = VISION_BAUD;
	u.USART_WordLength          = USART_WordLength_8b;
	u.USART_StopBits            = USART_StopBits_1;
	u.USART_Parity              = USART_Parity_No;
	u.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	u.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(VISION_USART, &u);

	DMA_DeInit(VISION_DMA_STREAM);
	d.DMA_Channel            = VISION_DMA_CHANNEL;
	d.DMA_PeripheralBaseAddr = (uint32_t)(&(VISION_USART->DR));
	d.DMA_Memory0BaseAddr    = (uint32_t)s_dmaBuf;
	d.DMA_DIR                = DMA_DIR_PeripheralToMemory;
	d.DMA_BufferSize         = VISION_RX_BUF;
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
	DMA_Init(VISION_DMA_STREAM, &d);

	/* 中断：只开 IDLE，ISR 不调 FreeRTOS API，故按普通外设给抢占优先级 5 */
	n.NVIC_IRQChannel                   = VISION_IRQn;
	n.NVIC_IRQChannelPreemptionPriority = 5;
	n.NVIC_IRQChannelSubPriority        = 0;
	n.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&n);

	USART_DMACmd(VISION_USART, USART_DMAReq_Rx, ENABLE);
	USART_ITConfig(VISION_USART, USART_IT_IDLE, ENABLE);
	USART_Cmd(VISION_USART, ENABLE);
	DMA_Cmd(VISION_DMA_STREAM, ENABLE);

	visionSlot.online   = 0u;
	visionSlot.fresh    = 0u;
	visionSlot.count    = 0u;
	visionSlot.seq      = 0u;
	visionSlot.rxCnt    = 0u;
	visionSlot.errCnt   = 0u;
	visionSlot.task.len = 0u;
	visionSlot.lastTick = xTaskGetTickCount();
}

/**
 * @brief  周期处理：刷新在线标志
 * @note   解析已在中断内完成，本函数只做新鲜度判定。
 *         须与 vision_TakeFrame() 在同一任务上下文调用。
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
 * @brief  取走一帧
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
 * @brief  取走任务信息（二维码）
 * @param  dst 目标结构（含字节袋与长度）
 * @retval 拷贝的有效字节数；0 = 无新任务信息
 * @note   ⚠ 任务信息语义留空，本函数只做原样搬运；取走后清 len，
 *         因此同一条任务信息只会被消费一次
 * @author camelliahm
 */
uint8_t vision_TakeTask(VisionTask_t *dst)
{
	uint8_t n;
	uint8_t i;

	if ((dst == NULL) || (visionSlot.task.len == 0u))
	{
		return 0u;
	}

	n = visionSlot.task.len;
	if (n > VISION_TASK_LEN)
	{
		n = VISION_TASK_LEN;
	}

	for (i = 0u; i < n; i++)
	{
		dst->data[i] = visionSlot.task.data[i];
	}
	dst->len = n;

	visionSlot.task.len = 0u;

	return n;
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
