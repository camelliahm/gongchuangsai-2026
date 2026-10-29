#ifndef __CAN_H
#define __CAN_H

#include "board.h"
#include "fifo.h"

/**********************************************************
***	一帧 CAN 报文
***	作为"中断 → 任务"的传递单元。直接带上 ExtId / DLC / Data，
***	任务侧不必再去读全局单缓冲（连续多帧时会被覆盖）。
**********************************************************/
typedef struct {
	uint32_t ExtId;	//扩展帧 ID（ExtId[28:8] = 地址，[7:0] = 包序号）
	uint8_t  DLC;	//数据长度
	uint8_t  Data[8];	//数据内容
} CAN_Frame_t;

typedef struct {
	__IO CanRxMsg CAN_RxMsg;	//接收中断填入的报文
	__IO CanTxMsg CAN_TxMsg;	//发送用报文

	__IO bool rxFrameFlag;	//收到帧标志（中断置位）
} CAN_t;

//接收队列深度（帧数），16 帧足以吸收一次突发
#define CAN_RX_QUEUE_LEN	(16)					//接收队列深度（帧数）

void can_SendCmd(__IO uint8_t *cmd, uint8_t len);	//多包发送：cmd = [addr, 功能码, 参数..., 校验字节]

bool can_RtosInit(void);	//创建接收队列与发送互斥锁；须在创建任务之前调用

//阻塞等待一帧；timeout_ms = 0 表示不等待；retval: true-取到 false-超时/未初始化
bool can_RecvFrame(CAN_Frame_t *out, uint32_t timeout_ms);

/**********************************************************
***	发送互斥
***	Emm_V5_* 各命令函数内部使用的是文件级 static 缓冲，
***	多任务并发调用会互相覆盖，必须把整个"装填 + 发送"过程包住：
***		can_TxLock();
***		Emm_V5_Vel_Control(...);
***		can_TxUnlock();
***	调度器未启动时自动退化为空操作。
**********************************************************/
void can_TxLock(void);
void can_TxUnlock(void);

//因发送邮箱满而丢弃的帧数（诊断用）
extern __IO uint16_t can_TxDropCount;	//发送丢弃计数（诊断）

extern __IO CAN_t can;

#endif
