/* can_stub.c —— POSIX 模拟层：can.c 空壳
 * 真实实现见 BSP/can.c（STM32 CAN1 + 中断 + 邮箱发送），本机不编译它。
 * 这里保留 FreeRTOS 队列/互斥锁语义（状态机依赖 can_RecvFrame 阻塞等待），
 * 但物理收发为空操作：无真实电机，反馈帧不会来。 */
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

__IO CAN_t can = {0};                 /* CAN 收发状态与槽位 */
__IO uint16_t can_TxDropCount = 0;    /* 发送丢弃计数（诊断） */

static QueueHandle_t     s_canRxQueue = NULL;    /* CAN 接收队列句柄 */
static SemaphoreHandle_t s_canTxMutex = NULL;    /* CAN 发送互斥锁句柄 */

/**
 * @brief  创建接收队列与发送互斥锁
 * @note   语义与 BSP/can.c 一致；本机无真实 CAN，仅保证队列/锁可创建
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
 * @note   本机无真实 CAN 中断投递，此函数会一直等到超时
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

void can_TxLock(void)
{
    if ((s_canTxMutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
    {
        xSemaphoreTake(s_canTxMutex, portMAX_DELAY);
    }
}

void can_TxUnlock(void)
{
    if ((s_canTxMutex != NULL) && (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED))
    {
        xSemaphoreGive(s_canTxMutex);
    }
}

/**
 * @brief  发送命令（空操作）
 * @note   本机无真实 CAN 硬件，指令直接丢弃；保留签名让 Emm_V5.c 照常编译
 * @author camelliahm
 */
void can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
    (void)cmd;
    (void)len;
}
