/* can.c 的 Linux 实现（socketcan 底座）
 * 替代 STM32 的 BSP/can.c（CAN1 中断 + 邮箱发送）。本机不编译 BSP/can.c。
 *
 * 与 STM32 版差异：
 *   - 物理收发：STM32 CAN1 邮箱 → socketcan（write/read 内核套接字）
 *   - 中断投递 CAN1_RX0_IRQHandler → socketcan 轮询 recv 后投队列
 *   其余（FreeRTOS 队列/互斥、can_SendCmd 分包逻辑）与 STM32 版一字不差。 */
#include "can.h"
#include "socketcan.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include <string.h>

__IO CAN_t can = {0};                 /* CAN 收发状态与槽位 */
__IO uint16_t can_TxDropCount = 0;    /* 发送丢弃计数（诊断） */

static QueueHandle_t     s_canRxQueue = NULL;    /* CAN 接收队列句柄 */
static SemaphoreHandle_t s_canTxMutex = NULL;    /* CAN 发送互斥锁句柄 */

/* 扩展帧标志（socketcan 规定 29 位 ID 须带 CAN_EFF_FLAG） */
#define CAN_EFF_FLAG    (0x80000000u)

/**
 * @brief  创建接收队列与发送互斥锁，并打开 socketcan
 * @param  无
 * @retval true-成功 false-堆内存不足
 * @note   需在创建任务之前、调度器启动之前调用。
 *         打开 "can0" 失败时静默降级（本机无 vcan0 属预期），收发退化为丢弃。
 * @author camelliahm
 */
bool can_RtosInit(void)
{
    s_canRxQueue = xQueueCreate(CAN_RX_QUEUE_LEN, sizeof(CAN_Frame_t));
    s_canTxMutex = xSemaphoreCreateMutex();

    /* 打开 CAN 接口；失败不致命（无接口时帧照常丢弃） */
    (void)socketcan_Init("can0");

    return ((s_canRxQueue != NULL) && (s_canTxMutex != NULL));
}

/**
 * @brief  CAN 接收桥接：从 socketcan 读一帧投进 FreeRTOS 队列
 * @note   替代 STM32 的 CAN1_RX0_IRQHandler。读到的裸 29 位 ID 去掉
 *         CAN_EFF_FLAG 后存入 CAN_Frame_t.ExtId。
 * @author camelliahm
 */
void can_PollRx(void)
{
    SocketcanFrame_t sf;
    CAN_Frame_t frame;
    uint8_t i;

    if (s_canRxQueue == NULL)
    {
        return;
    }

    while (socketcan_Recv(&sf))
    {
        frame.ExtId = sf.id & ~CAN_EFF_FLAG;   /* 去掉 EFF 标志，还原裸 29 位 ID */
        frame.DLC   = sf.dlc;

        if (frame.DLC > 8)
        {
            frame.DLC = 8;
        }

        for (i = 0; i < 8; i++)
        {
            frame.Data[i] = (i < frame.DLC) ? sf.data[i] : 0;
        }

        /* 队列满则丢弃，绝不阻塞调用方 */
        (void)xQueueSend(s_canRxQueue, &frame, 0);
    }
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

    /* 先桥接 socketcan 的帧进队列，再阻塞收；对上层完全透明 */
    can_PollRx();

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
 * @brief  CAN 发送多个字节（分包）
 * @param  cmd 命令缓冲：[addr, 功能码, 参数..., 校验字节]
 * @param  len 缓冲总长度
 * @note   分包逻辑与 STM32 版一致：ExtId = (addr<<8)|packNum，扩展帧；
 *         每包数据最多 8 字节（含功能码）。物理发送改走 socketcan。
 * @author camelliahm
 */
void can_SendCmd(__IO uint8_t *cmd, uint8_t len)
{
    uint8_t i = 0, j = 0, l = 0, packNum = 0;
    SocketcanFrame_t sf;

    /* 除去 ID 地址和功能码后的数据长度 */
    j = (uint8_t)(len - 2);

    while (i < j)
    {
        uint8_t k = (uint8_t)(j - i);   /* 本包剩余数据个数 */

        memset(&sf, 0, sizeof(sf));

        /* 扩展帧：29 位 ID = (addr << 8) | 包序号 */
        sf.id  = CAN_EFF_FLAG | (((uint32_t)cmd[0] << 8) | (uint32_t)packNum);
        sf.data[0] = cmd[1];            /* 功能码占 Data[0] */

        if (k < 8)
        {
            /* 小于 8 字节：一包发完 */
            for (l = 0; l < k; l++, i++)
            {
                sf.data[l + 1] = cmd[i + 2];
            }
            sf.dlc = (uint8_t)(k + 1);
        }
        else
        {
            /* 大于等于 8 字节：本包发满 8 字节（1 功能码 + 7 数据） */
            for (l = 0; l < 7; l++, i++)
            {
                sf.data[l + 1] = cmd[i + 2];
            }
            sf.dlc = 8;
        }

        if (!socketcan_Send(&sf))
        {
            can_TxDropCount++;   /* 发送失败计数，替代 STM32 的邮箱满丢弃 */
        }

        ++packNum;
    }
}
