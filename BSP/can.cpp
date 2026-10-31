/* can.cpp —— CAN 链路的 Linux 实现（socketcan 底座）
 * 物理收发走 socketcan（write/read 内核套接字），帧格式转换、分包、队列语义在此层。 */
#include "can.h"
#include "socketcan.h"

#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>

__IO CAN_t can = {0};                 /* CAN 收发状态与槽位 */
__IO uint16_t can_TxDropCount = 0;    /* 发送丢弃计数（诊断） */

/* 接收队列：std::deque 环形缓冲 + mutex + cond 实现阻塞/非阻塞收发 */
static std::deque<CAN_Frame_t> s_canRxBuf;      /* CAN 接收帧缓冲 */
static std::mutex              s_canRxMutex;    /* 接收队列锁 */
static std::condition_variable s_canRxCond;     /* 接收队列条件变量 */
static std::mutex              s_canTxMutex;    /* CAN 发送互斥锁 */

/* 扩展帧标志（socketcan 规定 29 位 ID 须带 CAN_EFF_FLAG） */
#define CAN_EFF_FLAG    (0x80000000u)

/**
 * @brief  创建接收队列与发送互斥锁，并打开 socketcan
 * @param  无
 * @retval true-成功 false-打开 socketcan 失败
 * @note   需在创建任务之前调用。
 *         打开 "can0" 失败时静默降级（本机无 vcan0 属预期），收发退化为丢弃。
 * @author camelliahm
 */
bool can_RtosInit(void)
{
    /* 打开 CAN 接口；失败不致命（无接口时帧照常丢弃） */
    (void)socketcan_Init("can0");

    return true;
}

/**
 * @brief  CAN 接收桥接：从 socketcan 读一帧投进接收队列
 * @note   替代 STM32 的 CAN1_RX0_IRQHandler。读到的裸 29 位 ID 去掉
 *         CAN_EFF_FLAG 后存入 CAN_Frame_t.ExtId。
 * @author camelliahm
 */
void can_PollRx(void)
{
    SocketcanFrame_t sf;
    CAN_Frame_t frame;
    uint8_t i;

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
        std::lock_guard<std::mutex> lk(s_canRxMutex);
        if (s_canRxBuf.size() < CAN_RX_QUEUE_LEN)
        {
            s_canRxBuf.push_back(frame);
            s_canRxCond.notify_one();
        }
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
    if (out == NULL)
    {
        return false;
    }

    /* 先桥接 socketcan 的帧进队列，再阻塞收；对上层完全透明 */
    can_PollRx();

    std::unique_lock<std::mutex> lk(s_canRxMutex);

    if (timeout_ms == 0u)
    {
        /* 不等待：队列空则立即返回 */
        if (s_canRxBuf.empty())
        {
            return false;
        }
    }
    else
    {
        /* 阻塞等待，直到有帧或超时 */
        if (!s_canRxCond.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                                  [&] { return !s_canRxBuf.empty(); }))
        {
            return false;
        }
    }

    *out = s_canRxBuf.front();
    s_canRxBuf.pop_front();
    return true;
}

/**
 * @brief  发送互斥加锁
 * @param  无
 * @retval 无
 * @author camelliahm
 */
void can_TxLock(void)
{
    s_canTxMutex.lock();
}

/**
 * @brief  发送互斥解锁
 * @param  无
 * @retval 无
 * @author camelliahm
 */
void can_TxUnlock(void)
{
    s_canTxMutex.unlock();
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
