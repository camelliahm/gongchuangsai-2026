#ifndef __NAV_H
#define __NAV_H

#include "contract.h"

//编排层（任务层）：航段序列 → 逐周期底盘速度与目标航向；nav_Step() 须在 RUNNING 闸门之前调用。详见 README.md

/*---- 航段类型 ----*/
typedef enum {
	NAV_SEG_END = 0,	//终点：清速度并结束
	NAV_SEG_TURN,		//原地转向：value = 相对转角(0.1 度)，正 = 逆时针
	NAV_SEG_FORWARD,	//直行：value = 相对位移(mm)，正 = 前进
	NAV_SEG_STRAFE,		//横移：value = 相对位移(mm)，正 = 右移
	NAV_SEG_DWELL,		//停留：value = 时长(ms)
	NAV_SEG_HOME,		//升降轴回零：车停原地等 lift.homed，value 忽略
	NAV_SEG_ALIGN,		//视觉对准：读视觉1 相对距离微调底盘，value 忽略
	NAV_SEG_LOAD,		//三轮装载：每轮先对准再抓取，轮数由 GRIP_LOAD_TOTAL 定，value 忽略
} NavSegType_t;

//航段：位移/转角用 value + 符号表达，speed 只对平移段有效
typedef struct {
	uint8_t  type;		//NavSegType_t
	int16_t  value;		//位移(mm) / 转角(0.1 度) / 时长(ms)
	int16_t  speed;		//平移速度(mm/s)，恒正；转向与停留段忽略
	uint32_t timeout;	//本段超时(ms)；0 = 用 NAV_SEG_TIMEOUT_MS。转向/对准/装载/回零段应显式给（默认值会提前打断）
} NavSeg_t;

/*---- 导航状态 ----*/
typedef enum {
	NAV_IDLE = 0,		//未启动
	NAV_RUN,			//执行航段中
	NAV_DONE,			//全部航段完成，已投 EV_STOP
	NAV_ABORT,			//被中止（故障 / 急停 / 外部调用 nav_Abort）
} NavState_t;

/*---- 参数 ----*/
#define NAV_SEG_MAX			(48u)		//单张地图最大航段数
#define NAV_CTRL_PERIOD_MS	(5u)		//须与 CHASSIS_CTRL_PERIOD_MS 一致
#define NAV_TURN_WINDOW_DEG	(2.0f)		//转向到位窗口(度)
#define NAV_TURN_HOLD_MS	(200u)		//转向到位后需保持的时长
#define NAV_DECEL_MM		(80)		//剩余距离小于此值即降速，减小过冲
#define NAV_DECEL_VEL		(120)		//降速后的速度(mm/s)
#define NAV_SEG_TIMEOUT_MS	(20000u)	//平移/停留段的默认超时；转向段应显式给 timeout
#define NAV_TURN_TIMEOUT_MS	(40000u)	//转向段建议超时（姿态环收敛慢，180° 约需 24s）

/*---- 对准段（NAV_SEG_ALIGN）：⚠ 除窗口外全为占位值，待实测 ----*/
#define NAV_ALIGN_TOL_MM		(10)		//对准到位窗口(mm)：前后 / 左右同时进入才算居中
#define NAV_ALIGN_HOLD_MS		(200u)		//居中后需保持的时长：停住等车稳，防带速进入抓取
#define NAV_ALIGN_KP			(0.8f)		//相对距离(mm) → 底盘速度(mm/s)，单位 1/s
#define NAV_ALIGN_SLOW_MM		(30)		//剩余距离小于此值即降速，减小过冲
#define NAV_ALIGN_SLOW_VEL		(30)		//降速后的速度(mm/s)
#define NAV_ALIGN_VEL_MAX		(100)		//对准速度上限(mm/s)
#define NAV_ALIGN_TRY_TIMEOUT_MS (6000u)	//单次对准的时限(ms)
#define NAV_ALIGN_RETRY			(3u)		//对准超时后的重试次数（含首次共 4 次尝试）
#define NAV_ALIGN_TIMEOUT_MS	(30000u)	//对准段的段超时(ms)：须覆盖全部重试

/*---- 装载段（NAV_SEG_LOAD）----*/
#define NAV_LOAD_RETRY			(2u)		//装载失败后的重试次数（与段内对准共用该计数）
#define NAV_LOAD_TIMEOUT_MS		(240000u)	//装载段的段超时(ms)：须覆盖三轮"等色 + 对准 + 动作"与全部重试  ⚠ 占位值

/*---- 挑色（装载段每轮的前置）：等本轮目标色转到取料点 ----*/
#define NAV_COLOR_HOLD_FRAMES	(3u)		//同色需连续通过的帧数，确认转盘已停稳  ⚠ 占位值
#define NAV_COLOR_TIMEOUT_MS	(25000u)	//单轮等色的上限(ms)：无任务码 / 目标色迟迟不出现即判故障

/*---- 回零段（NAV_SEG_HOME）----*/
#define NAV_HOME_TIMEOUT_MS		(20000u)	//回零段的段超时(ms)：aux 侧的 15s 通常先投 EV_FAULT

void nav_Init(void);

//装订地图（只存指针，调用者须保证生命周期，通常用 static const 表）
bool nav_Load(const NavSeg_t *segs, uint16_t num);

//请求启动（只置标志；实际装载在 nav_Step() 的控制环上下文里完成）
void nav_Start(void);

//中止并清速度。正常路径由 nav_Step() 在离开 RUNNING 时自动调用
void nav_Abort(void);

//控制环周期调用（5ms）。须置于 RUNNING 闸门之前，见 .c 内说明
void nav_Step(void);

NavState_t nav_GetState(void);
uint16_t   nav_GetSeg(void);

//姿态环给定：平移段锁"进入本段时的航向"，转向段推到目标航向
//驱动层经黑板 chassis.yawTarget 读取；本函数仅供观察调试
float nav_GetTargetYaw(void);

#endif
