#ifndef __SOCKETCAN_H
#define __SOCKETCAN_H

#include <stdint.h>
#include <stdbool.h>

/* socketcan 底座：把 Linux CAN 总线（can0/vcan0）封装成与 BSP/can.h 对齐的最小接口。
 * 本文件只做"物理收发"，帧格式转换、分包、队列语义都在上层（can.c 的 Linux 实现）里做。 */

/* 一帧 socketcan 报文，与内核 struct can_frame 对齐（can_id 已含 EFF 标志） */
typedef struct {
    uint32_t id;     /* CAN ID（标准帧 11 位，扩展帧 29 位，扩展帧须带 CAN_EFF_FLAG） */
    uint8_t  dlc;    /* 数据长度 0~8 */
    uint8_t  data[8];/* 数据 */
} SocketcanFrame_t;

/* 初始化：打开并绑定指定接口名（如 "vcan0"/"can0"），返回是否成功 */
bool socketcan_Init(const char *ifname);

/* 发送一帧（非阻塞），返回是否成功 */
bool socketcan_Send(const SocketcanFrame_t *frame);

/* 接收一帧（非阻塞），返回是否取到；无数据返回 false */
bool socketcan_Recv(SocketcanFrame_t *frame);

/* 关闭并释放 */
void socketcan_Close(void);

#endif
