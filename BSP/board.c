#include "board.h"

/**
 * @brief  配置NVIC控制器
 * @param  无
 * @retval 无
 * @author camelliahm
 */
void nvic_init(void)
{	
	// 4bit抢占优先级位
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	/**********************************************************
	***	接入 FreeRTOS 后：抢占优先级必须 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5)，
	***	否则中断内调用 FreeRTOS API（如 xQueueSendFromISR）会触发 configASSERT。
	***	数值越大 = 优先级越低。
	**********************************************************/
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief  外设时钟初始化
 * @param  无
 * @retval 无
 * @author camelliahm
 */
void clock_init(void)
{
	// 使能GPIOA、GPIOB外设时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);

	// 使能CAN1外设时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
}

/**
 * @brief  初始化CAN
 * @param  无
 * @retval 无
 * @author camelliahm
 */
void can_init(void)
{
/**********************************************************
***	初始化CAN引脚
**********************************************************/
	// PB8 - CAN_RX、PB9 - CAN_TX
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;					// 复用推挽输出
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;					// 上拉
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	// 将PB8、PB9复用为CAN1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_CAN1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_CAN1);

/**********************************************************
***	初始化CAN
**********************************************************/
	// 初始化CAN引脚
	CAN_InitTypeDef	CAN_InitStructure;
	CAN_StructInit(&CAN_InitStructure);
	CAN_InitStructure.CAN_TTCM = DISABLE;					// 关闭时间触发通讯模式
	CAN_InitStructure.CAN_ABOM = ENABLE;					// 开启自动离线管理
	CAN_InitStructure.CAN_AWUM = DISABLE;					// 关闭自动唤醒模式
	CAN_InitStructure.CAN_NART = ENABLE;					// 关闭自动重传（NART=1 禁止重传）
															// 重传会让失败帧一直占用发送邮箱（共3个），
															// 邮箱满后 CAN_Transmit 静默返回 NoMailBox 丢帧，
															// 导致周期控制指令排队/延迟。控制帧按周期刷新，
															// 丢一帧由下一周期覆盖，故关闭重传换取低延迟。
	CAN_InitStructure.CAN_RFLM = DISABLE;					// 接收FIFO锁定模式		DISABLE-溢出时新报文会覆盖原有报文
	CAN_InitStructure.CAN_TXFP = DISABLE;					// 发送FIFO优先级			DISABLE-优先级取决于报文标识符
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;	// 正常工作模式
	CAN_InitStructure.CAN_SJW = CAN_SJW_1tq;			// 重新同步跳跃宽度为SJW + 1个时间单位
	CAN_InitStructure.CAN_BS1 = CAN_BS1_4tq;
	CAN_InitStructure.CAN_BS2 = CAN_BS2_1tq;
	CAN_InitStructure.CAN_Prescaler = 14;				  // 波特率 = 42M / 14 / (1 + 4 + 1) = 0.5，即500K
	CAN_Init(CAN1, &CAN_InitStructure);
	// 设置STM32的帧ID - 扩展帧格式 - 不过滤任何数据帧
	__IO uint8_t id_o, im_o; __IO uint16_t id_l, id_h, im_l, im_h;
	id_o = (0x00);
	id_h = (uint16_t)((uint16_t)id_o >> 5);										 // 高3位
	id_l = (uint16_t)((uint16_t)id_o << 11) | CAN_Id_Extended; // 低5位
	im_o = (0x00);
	im_h = (uint16_t)((uint16_t)im_o >> 5);
	im_l = (uint16_t)((uint16_t)im_o << 11) | CAN_Id_Extended;
	// 初始化CAN过滤器
	CAN_FilterInitTypeDef	CAN_FilterInitStructure;
	CAN_FilterInitStructure.CAN_FilterNumber = 1;											// 过滤器1
	CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;		// 掩码模式
	CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;	// 32位过滤器位宽
	CAN_FilterInitStructure.CAN_FilterIdHigh = id_h;									// 过滤器标识符的高16位值
	CAN_FilterInitStructure.CAN_FilterIdLow = id_l;										// 过滤器标识符的低16位值
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh = im_h;							// 过滤器屏蔽标识符的高16位值
	CAN_FilterInitStructure.CAN_FilterMaskIdLow = im_l;								// 过滤器屏蔽标识符的低16位值
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment = CAN_FIFO0;			// 指向过滤器的FIFO为0
	CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;						// 使能过滤器
	CAN_FilterInit(&CAN_FilterInitStructure);
	// 初始化CAN中断
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);													// 使能RX0一包数据接收中断
}

/**
 * @brief  板载初始化
 * @param  无
 * @retval 无
 * @author camelliahm
 */
void board_init(void)
{
	nvic_init();
	clock_init();
	can_init();
}
