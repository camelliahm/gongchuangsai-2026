#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

__IO CAN_t can = {0};	//CAN 收发状态与槽位
__IO uint16_t can_TxDropCount = 0;	//发送丢弃计数（诊断）

static QueueHandle_t     s_canRxQueue = NULL;	//CAN 接收队列句柄
static SemaphoreHandle_t s_canTxMutex = NULL;	//CAN 发送互斥锁句柄

/**
 * @brief  创建接收队列与发送互斥锁
 * @param  无
 * @retval true-成功 false-堆内存不足
 * @note   需在创建任务之前、调度器启动之前调用
 * @author camelliahm
 */
bool can_RtosInit(void)
{
	s_canRxQueue = xQueueCreate(CAN_RX_QUEUE_LEN, sizeof(CAN_Frame_t));
	s_canTxMutex = xSemaphoreCreateMutex();

	return ((s_canRxQueue != NULL) && (s_canTxMutex != NULL));
}

/**
 * @brief  阻塞等待一帧
 * @param  out 输出帧
 * @param  timeout_ms 超时时间(ms)，0 表示不等待
 * @retval true-取到一帧 false-超时或未初始化
 * @author camelliahm
 */
bool can_RecvFrame(CAN_Frame_t *out, uint32_t timeout_ms)
{
	if ((s_canRxQueue == NULL) || (out == NULL))
	{
		return false;
	}

	return (xQueueReceive(s_canRxQueue, out, pdMS_TO_TICKS(timeout_ms)) == pdPASS);
}

/**
 * @brief  发送互斥加锁
 * @param  无
 * @retval 无
 * @note   调度器未启动时退化为空操作（此时不存在并发）
 * @author camelliahm
 */
void can_TxLock(void)
{
	if ((s_canTxMutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
	{
		xSemaphoreTake(s_canTxMutex, portMAX_DELAY);
	}
}

/**
 * @brief  发送互斥解锁
 * @param  无
 * @retval 无
 * @author camelliahm
 */
void can_TxUnlock(void)
{
	if ((s_canTxMutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
	{
		xSemaphoreGive(s_canTxMutex);
	}
}

/**
 * @brief  CAN1_RX0接收中断
 * @param  无
 * @retval 无
 * @note   中断内只做最小搬运：写入全局（便于调试器直接观察）并投递到队列，
 *         所有解析一律放到任务里做。队列满则丢弃，绝不阻塞中断。
 * @author camelliahm
 */
void CAN1_RX0_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	CAN_Frame_t frame;
	uint8_t i;

	// 接收一包数据（同时会释放 FIFO0，清除挂起标志）
	CAN_Receive(CAN1, CAN_FIFO0, (CanRxMsg *)(&can.CAN_RxMsg));

	// 一帧数据接收完成，置位帧标志位
	can.rxFrameFlag = true;

	// 打包成可投递的帧
	frame.ExtId = can.CAN_RxMsg.ExtId;
	frame.DLC   = can.CAN_RxMsg.DLC;

	if (frame.DLC > 8)
	{
		frame.DLC = 8;
	}

	for (i = 0; i < 8; i++)
	{
		frame.Data[i] = (i < frame.DLC) ? can.CAN_RxMsg.Data[i] : 0;
	}

	if (s_canRxQueue != NULL)
	{
		(void)xQueueSendFromISR(s_canRxQueue, &frame, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

/**
 * @brief  发送一帧，邮箱满即计数丢弃（不等待）
 * @param  msg 待发送报文
 * @retval 邮箱编号 0~2；0x04(CAN_TxStatus_NoMailBox) 表示发送失败
 * @note   已关闭自动重传（CAN_NART = ENABLE），失败帧不会占着邮箱反复重试；
 *         控制帧由下一周期覆盖，因此丢弃是可接受的。
 * @author camelliahm
 */
static uint8_t can_TransmitNoWait(__IO CanTxMsg *msg)
{
	uint8_t box = CAN_Transmit(CAN1, (CanTxMsg *)msg);

	if (box == CAN_TxStatus_NoMailBox)
	{
		can_TxDropCount++;
	}

	return box;
}

/**
 * @brief  CAN发送多个字节
 * @param  cmd 命令缓冲：[addr, 功能码, 参数..., 校验字节]
 * @param  len 缓冲总长度
 * @author camelliahm
 */
void can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
	__IO uint8_t i = 0, j = 0, k = 0, l = 0, packNum = 0;

	// 除去ID地址和功能码后的数据长度
	j = len - 2;

	// 发送数据
	while(i < j)
	{
		// 数据个数
		k = j - i;

		// 填充缓存
		can.CAN_TxMsg.StdId = 0x00;
		can.CAN_TxMsg.ExtId = ((uint32_t)cmd[0] << 8) | (uint32_t)packNum;
		can.CAN_TxMsg.Data[0] = cmd[1];
		can.CAN_TxMsg.IDE = CAN_Id_Extended;
		can.CAN_TxMsg.RTR = CAN_RTR_Data;

		// 小于8字节命令
		if(k < 8)
		{
			for(l=0; l < k; l++,i++) { can.CAN_TxMsg.Data[l + 1] = cmd[i + 2]; } can.CAN_TxMsg.DLC = k + 1;
		}
		// 大于8字节命令，分包发送，每包数据最多发送8个字节
		else
		{
			for(l=0; l < 7; l++,i++) { can.CAN_TxMsg.Data[l + 1] = cmd[i + 2]; } can.CAN_TxMsg.DLC = 8;
		}

		// 发送数据（邮箱满则丢弃并计数，不阻塞）
		(void)can_TransmitNoWait(&can.CAN_TxMsg);

		// 记录发送的第几包的数据
		++packNum;
	}
}
