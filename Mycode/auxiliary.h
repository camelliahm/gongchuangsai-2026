#ifndef __AUXILIARY_H
#define __AUXILIARY_H

#include "contract.h"
#include "grip_seq.h"

/* 辅助机构接入层：
 * 对外（contract）提供指令分发与周期任务入口，对内聚合升降轴与夹持机构；
 * 升 / 降实现在 lift_axis.c，夹持与动作序列在 grip_seq.c。
 */

//初始化（在 contract_Init() 中调用）
void aux_Init(void);

//指令分发：是本模块指令则处理并返回 true（受 CFG_HOST_CAN_ENABLED 控制）
bool aux_HandleCmd(ChassisCmd_t cmd, const CAN_Frame_t *f);

//解析升降轴反馈帧（地址 0x05）
void aux_HandleLiftFeedback(const CAN_Frame_t *f);

//辅助机构周期任务：由控制环每 5ms 调用
void aux_Step(void);

#endif
