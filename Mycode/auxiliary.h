#ifndef __AUXILIARY_H
#define __AUXILIARY_H

#include "chassis.h"

/* 辅助机构层（底盘四轮之外的执行机构）*/

/* 控制环周期：须与 chassis_drive.h 的 CHASSIS_CTRL_PERIOD_MS 一致（到位计时按此累加） */
#define AUX_CTRL_PERIOD_MS		(5u)

/* 升降轴丝杆：T8 梯形丝杆（自锁），转一圈升 2 mm。⚠ 细分为占位值，待实测 */
#define LIFT_SCREW_LEAD_MM		(2.0f)	/* 导程(mm/圈)，已实测确认 */
#define LIFT_SCREW_MICROSTEP	(16u)	/* 驱动器细分 */
#define LIFT_SCREW_STEP_PER_REV	(200u)	/* 整步数/圈（1.8° 电机） */

/* 脉冲/mm = 整步数 × 细分 / 导程 = 200 × 16 / 2 = 1600 */
#define LIFT_PULSE_PER_MM \
	((float)LIFT_SCREW_STEP_PER_REV * (float)LIFT_SCREW_MICROSTEP / LIFT_SCREW_LEAD_MM)

/* mm ↔ 脉冲 换算（四舍五入到整脉冲）；速度换算：mm/s = RPM / 60 × 导程 */
#define LIFT_MM_TO_PULSE(mm) \
	((int32_t)(((float)(mm) * LIFT_PULSE_PER_MM) + (((float)(mm) >= 0.0f) ? 0.5f : -0.5f)))

#define LIFT_PULSE_TO_MM(clk) \
	((float)(clk) / LIFT_PULSE_PER_MM)

#define LIFT_RPM_TO_MMPS(rpm) \
	((float)(rpm) / 60.0f * LIFT_SCREW_LEAD_MM)

#define LIFT_MMPS_TO_RPM(mmps) \
	((uint16_t)((float)(mmps) * 60.0f / LIFT_SCREW_LEAD_MM))

#define LIFT_STROKE_MM			(300.0f)	/* 有效行程(mm)  ⚠ 占位值，待实测 */

#define LIFT_VEL_DEFAULT		(300u)	/* 默认速度(RPM) = 10 mm/s */
#define LIFT_ACC_DEFAULT		(10u)	/* 默认加速度(0=直接启动) */

/* 回零模式：3 = 多圈有限位开关回零（丝杆轴最常用，靠限位开关定零） */
#define LIFT_HOME_MODE			(3u)

#define LIFT_FB_TIMEOUT_MS		(200u)	/* 反馈超时：上报周期 50ms，200ms 未收到判掉线 */
#define LIFT_FB_PERIOD_MS		(50u)	/* 定时上报周期：比四轮慢，丝杆响应本就慢，省带宽 */
#define LIFT_HOME_TIMEOUT_MS	(15000u)/* 回零总超时：限位开关失效时防止电机一直撞到底 */

/* 到位判据：位置窗口(按 mm 给，再换算成脉冲) + 转速阈值，两者同时满足并持续一段时间 */
#define LIFT_ARRIVE_LEN_MM		(0.05f)	/* 到位位移窗口(mm)  ⚠ 占位值，待实测 */
#define LIFT_ARRIVE_WINDOW		((int32_t)(LIFT_ARRIVE_LEN_MM * LIFT_PULSE_PER_MM))	/* = 80 脉冲 */
#define LIFT_ARRIVE_VEL_TH		(30)	/* 转速阈值(RPM) */
#define LIFT_ARRIVE_HOLD_MS		(200u)	/* 持续时长 */

/* 升降轴状态 */
typedef struct {
	uint8_t	 online;			/* 1 = 反馈帧未超时 */
	uint8_t	 homed;				/* 1 = 已完成回零（本次上电）或已人工标定原点 */
	uint8_t	 homing;			/* 1 = 回零进行中（收到 0x3B 且值为 0 时清除） */
	uint8_t	 moving;			/* 1 = 已下发位置指令且尚未判到位 */
	int32_t	 pos;				/* S_CPOS 实时位置(脉冲)，有符号 */
	int16_t	 vel;				/* S_VEL 实时转速(RPM) */
	int32_t	 target;			/* 目标位置(脉冲)，有符号；moving 期间有效 */
	uint32_t lastTick;			/* 最后一次收到升降轴帧的 tick */
	uint16_t rxCount;			/* 累计收帧数（诊断用） */
} Lift_t;

extern __IO Lift_t lift;

/* 初始化（在 chassis_Init() 中调用） */
void aux_Init(void);

/* 以下接口内部统一用 can_TxLock/Unlock 包住发送；
 * 移动指令只在 READY / RUNNING 下被接受，IDLE / FAULT / ESTOP 一律拒绝 */
void lift_SetEnable(bool en);			/* 使能 / 失能，由状态机自动调用 */
void lift_EnableFeedback(void);			/* 打开 S_CPOS 定时上报，进入 READY 时调用 */
void lift_ClearOnline(void);			/* 清在线标志，进入 IDLE 时调用 */
bool lift_MoveTo(int32_t clk, uint8_t dir, uint16_t vel, uint8_t acc);	/* 绝对位置(脉冲) */
bool lift_MoveBy(int32_t delta, uint16_t vel, uint8_t acc);				/* 相对位移(脉冲) */

/* 以 mm 为单位的同一组接口：内部换算后转调上面的脉冲接口，零点 = 回零/标定原点 */
bool lift_MoveToMM(float mm, uint16_t vel, uint8_t acc);		/* 升到绝对高度(mm)，正 = 向上 */
bool lift_MoveByMM(float deltaMM, uint16_t vel, uint8_t acc);	/* 相对升降(mm)，正升负降 */
float lift_GetHeightMM(void);									/* 当前高度(mm) */

/* 触发回零（模式 LIFT_HOME_MODE） */
void lift_Home(void);

/* 立即停止升降轴 */
void lift_StopNow(void);

/* 将当前位置清零（需先卸掉丝杆负载，否则零点会漂） */
void lift_SetZero(void);

/* 解除升降轴堵转保护 */
void lift_ClearClog(void);

/* 是否仍在移动中（供应用层查询） */
bool lift_IsMoving(void);

/* 指令分发与周期任务（由 chassis.c 调用） */
bool aux_HandleCmd(ChassisCmd_t cmd, const CAN_Frame_t *f);	/* 是本模块指令则处理，返回 true */

/* 解析升降轴反馈帧（地址 0x05） */
void aux_HandleLiftFeedback(const CAN_Frame_t *f);

/* 辅助机构周期任务：各机构的看门狗 / 到位判断，由控制环每 5ms 调用 */
void aux_Step(void);

#endif
