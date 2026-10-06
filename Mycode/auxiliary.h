#ifndef __AUXILIARY_H
#define __AUXILIARY_H

#include "contract.h"

//辅助机构层（底盘四轮之外的执行机构）

//控制环周期：须与 chassis_drive.h 的 CHASSIS_CTRL_PERIOD_MS 一致（到位计时按此累加）
#define AUX_CTRL_PERIOD_MS		(5u)	//辅助机构控制环周期(ms)

//升降轴丝杆：T8 梯形丝杆（自锁），转一圈升 2 mm。⚠ 细分为占位值，待实测
#define LIFT_SCREW_LEAD_MM		(2.0f)	//导程(mm/圈)，已实测确认
#define LIFT_SCREW_MICROSTEP	(16u)	//驱动器细分
#define LIFT_SCREW_STEP_PER_REV	(200u)	//整步数/圈（1.8° 电机）

//脉冲/mm = 整步数 × 细分 / 导程 = 200 × 16 / 2 = 1600
#define LIFT_PULSE_PER_MM \
	((float)LIFT_SCREW_STEP_PER_REV * (float)LIFT_SCREW_MICROSTEP / LIFT_SCREW_LEAD_MM)

//mm → 脉冲（四舍五入到整脉冲）
#define LIFT_MM_TO_PULSE(mm) \
	((int32_t)(((float)(mm) * LIFT_PULSE_PER_MM) + (((float)(mm) >= 0.0f) ? 0.5f : -0.5f)))

//脉冲 → mm
#define LIFT_PULSE_TO_MM(clk) \
	((float)(clk) / LIFT_PULSE_PER_MM)

//RPM → mm/s（mm/s = RPM / 60 × 导程）
#define LIFT_RPM_TO_MMPS(rpm) \
	((float)(rpm) / 60.0f * LIFT_SCREW_LEAD_MM)

//mm/s → RPM
#define LIFT_MMPS_TO_RPM(mmps) \
	((uint16_t)((float)(mmps) * 60.0f / LIFT_SCREW_LEAD_MM))

#define LIFT_STROKE_MM			(300.0f)	//有效行程(mm)  ⚠ 占位值，待实测

#define LIFT_VEL_DEFAULT		(300u)	//默认速度(RPM) = 10 mm/s
#define LIFT_ACC_DEFAULT		(10u)	//默认加速度(0=直接启动)

//回零模式：3 = 多圈有限位开关回零（丝杆轴最常用，靠限位开关定零）
#define LIFT_HOME_MODE			(3u)	//回零模式：3 = 多圈有限位开关回零

#define LIFT_FB_TIMEOUT_MS		(200u)	//反馈超时：上报周期 50ms，200ms 未收到判掉线
#define LIFT_FB_PERIOD_MS		(50u)	//定时上报周期：比四轮慢，丝杆响应本就慢，省带宽
#define LIFT_HOME_TIMEOUT_MS	(15000u)//回零总超时：限位开关失效时防止电机一直撞到底

//到位判据：位置窗口(按 mm 给，再换算成脉冲) + 转速阈值，两者同时满足并持续一段时间
#define LIFT_ARRIVE_LEN_MM		(0.05f)	//到位位移窗口(mm)  ⚠ 占位值，待实测
#define LIFT_ARRIVE_WINDOW		((int32_t)(LIFT_ARRIVE_LEN_MM * LIFT_PULSE_PER_MM))	//= 80 脉冲
#define LIFT_ARRIVE_VEL_TH		(30)	//转速阈值(RPM)
#define LIFT_ARRIVE_HOLD_MS		(200u)	//持续时长

//升降轴状态
typedef struct {
	uint8_t	 online;			//1 = 反馈帧未超时
	uint8_t	 homed;				//1 = 已完成回零（本次上电）或已人工标定原点
	uint8_t	 homing;			//1 = 回零进行中（收到 0x3B 且值为 0 时清除）
	uint8_t	 moving;			//1 = 已下发位置指令且尚未判到位
	int32_t	 pos;				//S_CPOS 实时位置(脉冲)，有符号
	int16_t	 vel;				//S_VEL 实时转速(RPM)
	int32_t	 target;			//目标位置(脉冲)，有符号；moving 期间有效
	uint32_t lastTick;			//最后一次收到升降轴帧的 tick
	uint16_t rxCount;			//累计收帧数（诊断用）
} Lift_t;

extern __IO Lift_t lift;

//初始化（在 contract_Init() 中调用）
void aux_Init(void);

/* 以下接口内部统一用 can_TxLock/Unlock 包住发送；
 * 移动指令只在 READY / RUNNING 下被接受，IDLE / FAULT / ESTOP 一律拒绝 */
void lift_SetEnable(bool en);			//使能 / 失能，由状态机自动调用
void lift_EnableFeedback(void);			//打开 S_CPOS 定时上报，进入 READY 时调用
void lift_ClearOnline(void);			//清在线标志，进入 IDLE 时调用
bool lift_MoveTo(int32_t clk, uint8_t dir, uint16_t vel, uint8_t acc);	//绝对位置(脉冲)
bool lift_MoveBy(int32_t delta, uint16_t vel, uint8_t acc);				//相对位移(脉冲)

//以 mm 为单位的同一组接口：内部换算后转调上面的脉冲接口，零点 = 回零/标定原点
bool lift_MoveToMM(float mm, uint16_t vel, uint8_t acc);		//升到绝对高度(mm)，正 = 向上
bool lift_MoveByMM(float deltaMM, uint16_t vel, uint8_t acc);	//相对升降(mm)，正升负降
float lift_GetHeightMM(void);									//当前高度(mm)

//触发回零（模式 LIFT_HOME_MODE）
void lift_Home(void);

//立即停止升降轴
void lift_StopNow(void);

//将当前位置清零（需先卸掉丝杆负载，否则零点会漂）
void lift_SetZero(void);

//解除升降轴堵转保护
void lift_ClearClog(void);

//是否仍在移动中（供应用层查询）
bool lift_IsMoving(void);

/*---- 夹持机构（旋转轴 = TIM3 CH1，夹子 = TIM3 CH2）----*/
/* 无水平伸缩机构：水平定位靠底盘全向移动（编排层）+ 旋转轴在取料位 / 落盘位之间二选一；
 * 旋转轴只有两个端点角，中间角不需标定，不可当连续角度轴使用 */

#define GRIP_ROT_CH			(0u)		//旋转轴舵机通道（TIM3 CH1 = PC6）
#define GRIP_JAW_CH			(1u)		//夹子舵机通道（TIM3 CH2 = PC7）

#define GRIP_ROT_PICK_DEG	(0.0f)		//旋转轴取料位(°)：朝原料区
#define GRIP_ROT_PLACE_DEG	(180.0f)	//旋转轴落盘位(°)：朝车上物料盘
#define GRIP_ROT_SAFE_DEG	(90.0f)		//旋转轴安全位(°)：两工位中间，急停 / 上电姿态

#define GRIP_ROT_SLEW_DEG	(1.0f)		//旋转轴单周期最大角度增量(°)：限速使动作柔和；180° 约 0.9 s  ⚠ 待实测
#define GRIP_ROT_TOL_DEG	(2.0f)		//旋转轴到位判据(°)

/*---- 车载物料盘（旋转舵机 = TIM3 CH3 = PC8，270° 行程）----*/
/* 三槽位按搬运顺序落料：任务码第 1 组的第 k 位 → 槽位 k；
 * 槽位角即舵机指令角，机械上把槽位 k 装成该角正对夹持机构的落料点（顺序 1 = 位置一 = 0°） */

#define GRIP_TRAY_CH		(2u)		//物料盘舵机通道（TIM3 CH3 = PC8）
#define GRIP_TRAY_POS_NUM	(3u)		//槽位数量，须与 GRIP_LOAD_TOTAL 一致

#define GRIP_TRAY_POS1_DEG	(0.0f)		//槽位一(°)：顺序 1 的物料落点，兼上电 / 换批复位角
#define GRIP_TRAY_POS2_DEG	(120.0f)	//槽位二(°)：顺序 2 的物料落点
#define GRIP_TRAY_POS3_DEG	(240.0f)	//槽位三(°)：顺序 3 的物料落点

#define GRIP_TRAY_SLEW_DEG	(1.0f)		//物料盘单周期最大角度增量(°)：限速使动作柔和；240° 约 1.2 s  ⚠ 待实测
#define GRIP_TRAY_TOL_DEG	(2.0f)		//物料盘到位判据(°)

#define GRIP_JAW_OPEN_DEG	(10.0f)		//夹子张开角(°)  ⚠ 占位值，待实测
#define GRIP_JAW_CLOSE_DEG	(80.0f)		//夹子夹紧角(°)  ⚠ 占位值，待实测
#define GRIP_JAW_SETTLE_MS	(300u)		//夹子动作后的等待时长(ms)，等舵机走完

#define GRIP_H_PICK_MM		(60.0f)		//抓取高度(mm)      ⚠ 占位值，待实测
#define GRIP_H_PLACE_MM		(150.0f)	//落盘高度(mm)      ⚠ 占位值，待实测
#define GRIP_H_SAFE_MM		(200.0f)	//抬升 / 安全高度(mm)  ⚠ 占位值，待实测

/*---- 卸载（把车上物料放回车外目标区）----*/
/* 复用装载的两个端点角，只是角色对调：车内位取料、车外位放置。
 * 车怎么停在目标区前由编排层决定（本层只管一次取放），故不需要新的摆位机构 */

#define GRIP_ROT_U_PICK_DEG	(GRIP_ROT_PLACE_DEG)	//卸载取料位(°)：朝车内物料盘，即装载落盘位
#define GRIP_ROT_U_DROP_DEG	(GRIP_ROT_PICK_DEG)		//卸载放置位(°)：朝车外目标区，即装载取料位

#define GRIP_H_U_PICK_MM	(GRIP_H_PLACE_MM)	//槽内取料高度(mm)：暂沿用落盘高度  ⚠ 待实测，料落槽后或需再降一个料高
#define GRIP_H_U_DROP_MM	(GRIP_H_PICK_MM)	//车外放置高度(mm)：暂沿用原料区抓取高度  ⚠ 待实测，粗加工区台面高度或不同

#define GRIP_LIFT_VEL		(LIFT_VEL_DEFAULT)	//夹持动作的升降速度(RPM)，沿用升降轴默认
#define GRIP_LIFT_ACC		(LIFT_ACC_DEFAULT)	//夹持动作的升降加速度(0-255)

#define GRIP_MOVE_TIMEOUT_MS (1000u)	//单个升降动作的超时(ms)
#define GRIP_ROT_TIMEOUT_MS	 (3000u)	//旋转动作的超时(ms)
#define GRIP_LOAD_TOTAL		 (3u)		//一个原料区需完成的装载轮数

//一轮装载的动作阶段（顺序即 phase 递增顺序）
typedef enum {
	GRIP_IDLE = 0,		//空闲
	GRIP_ROT_PICK,		//旋转到取料位
	GRIP_LOWER_PICK,	//下降到抓取高度
	GRIP_JAW_CLOSE,		//夹紧
	GRIP_LIFT_UP,		//抬起到安全高度
	GRIP_ROT_PLACE,		//旋转到落盘位
	GRIP_LOWER_PLACE,	//下降到落盘高度
	GRIP_JAW_OPEN,		//松开
	GRIP_LIFT_CLEAR,	//升高让开料盘已放物料
	GRIP_ROT_SAFE,		//旋转回安全位
	GRIP_TRAY_HOME,		//物料盘回位置一（仅三轮装满后）

	/*---- 卸载序列（车上物料 → 车外目标区），与装载反向 ----*/
	GRIP_U_ROT_TRAY,	//转到车内取料位，同时物料盘把待取槽位转到取料点
	GRIP_U_LOWER,		//下降到槽内取料高度
	GRIP_U_JAW_CLOSE,	//夹紧
	GRIP_U_LIFT_UP,		//抬起到安全高度
	GRIP_U_ROT_DROP,	//转到车外放置位
	GRIP_U_LOWER_DROP,	//下降到放置高度
	GRIP_U_JAW_OPEN,	//松开
	GRIP_U_LIFT_CLEAR,	//升高让开已放物料
	GRIP_U_ROT_SAFE,	//旋转回安全位（序列完成）

	GRIP_PHASE_NUM
} GripPhase_t;

typedef struct {
	uint8_t	 phase;			//当前动作阶段 GripPhase_t
	uint8_t	 busy;			//1 = 动作序列进行中
	uint8_t	 fault;			//1 = 本序列因超时 / 被拒绝执行而中断
	uint8_t	 loadCount;		//已完成的装载轮数（满 GRIP_LOAD_TOTAL 即完成一个原料区）
	uint8_t	 unloadSlot;		//卸载待取槽位(1~GRIP_TRAY_POS_NUM)
	float	 rotTarget;		//旋转轴目标角(°)（由斜坡逐步逼近）
	float	 rotNow;		//旋转轴当前角(°)（斜坡中间值，非舵机反馈）
	float	 trayTarget;	//物料盘目标角(°)（由斜坡逐步逼近）
	float	 trayNow;		//物料盘当前角(°)（斜坡中间值，非舵机反馈）
	uint32_t phaseTick;		//进入当前阶段的 tick（超时判据）
} Grip_t;

extern __IO Grip_t grip;

/* 安全姿态：旋转轴回 90° + 夹子张开 + 中止序列；IDLE / FAULT / ESTOP 由 contract_OnEnter() 调用；
 * 物料盘不在安全姿态内 —— 盘上可能已有物料，急停时转动会把它甩出去，故保持当前角不动 */
void grip_Safe(void);

/* 启动一轮装载（取料 → 转 180° 并同步把物料盘转到本轮槽位 → 落到物料盘 → 回安全位）；
 * 返回 false = 前置条件不满足（状态不符 / 序列忙 / 升降轴未回零） */
bool grip_LoadOnce(void);

/* 启动一次卸载（从物料盘第 slot 槽取料 → 转到车外位 → 放到台面 → 回安全位）；
 * 返回 false = 前置条件不满足（状态不符 / 序列忙 / 升降轴未回零 / slot 越界） */
bool grip_UnloadOnce(uint8_t slot);

//中止动作序列（保持已下发的姿态，不改变旋转轴与夹子）
void grip_Abort(void);

//设定旋转轴目标角(°)，内部按 GRIP_ROT_SLEW_DEG 斜坡推进，不会一步跳到位
void grip_RotTo(float deg);

//夹子开 / 合：true = 夹紧，false = 张开
void grip_SetJaw(bool close);

//物料盘转到第 pos 个槽位（1~GRIP_TRAY_POS_NUM），内部按 GRIP_TRAY_SLEW_DEG 斜坡推进
void grip_TrayTo(uint8_t pos);

//物料盘回位置一（上电姿态；换批复位）
void grip_TrayHome(void);

//装载计数清零（每批装载开始调用；不清则 grip_AllLoaded() 恒真，装载段会秒过）
void grip_ResetLoad(void);

//序列是否进行中 / 三轮是否全部装完
bool grip_IsBusy(void);
bool grip_AllLoaded(void);

//指令分发与周期任务（由 contract.c 调用）
bool aux_HandleCmd(ChassisCmd_t cmd, const CAN_Frame_t *f);	//是本模块指令则处理，返回 true

//解析升降轴反馈帧（地址 0x05）
void aux_HandleLiftFeedback(const CAN_Frame_t *f);

//辅助机构周期任务：各机构的看门狗 / 到位判断，由控制环每 5ms 调用
void aux_Step(void);

#endif
