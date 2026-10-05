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
} NavSegType_t;

//航段：位移/转角用 value + 符号表达，speed 只对平移段有效
typedef struct {
	uint8_t  type;		//NavSegType_t
	int16_t  value;		//位移(mm) / 转角(0.1 度) / 时长(ms)
	int16_t  speed;		//平移速度(mm/s)，恒正；转向与停留段忽略
	uint16_t timeout;	//本段超时(ms)，0 = 用 NAV_SEG_TIMEOUT_MS 默认值
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
