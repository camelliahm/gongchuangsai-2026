#include "nav.h"
#include "chassis_drive.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

/*----------------------------------------------------------*/
/*  默认地图                                                    */
/*----------------------------------------------------------*/

/* ⚠ 示例路线，按赛题替换；末段必须是 NAV_SEG_END */
static const NavSeg_t s_defaultMap[] = {
	{ NAV_SEG_FORWARD,  600, 300 },		/* 前进 600 mm */
	{ NAV_SEG_TURN,    -900,   0 },		/* 右转 90.0 度 */
	{ NAV_SEG_STRAFE,  -400, 250 },		/* 左移 400 mm */
	{ NAV_SEG_TURN,    1800,   0 },		/* 左转 180.0 度 */
	{ NAV_SEG_FORWARD, -300, 200 },		/* 后退 300 mm */
	{ NAV_SEG_END,        0,   0 },
};

#define NAV_DEFAULT_NUM		((uint16_t)(sizeof(s_defaultMap) / sizeof(s_defaultMap[0])))
#define NAV_TURN_HOLD_CNT	(NAV_TURN_HOLD_MS / NAV_CTRL_PERIOD_MS)
#define NAV_DEG2RAD			(0.01745329252f)

/*----------------------------------------------------------*/
/*  内部状态                                                    */
/*----------------------------------------------------------*/

static struct {
	NavState_t		state;
	const NavSeg_t *map;
	uint16_t		mapNum;
	uint16_t		seg;
	uint32_t		segTick;
	uint16_t		hold;

	/* 连续航向：自累加，规避 chassisDrive.odomYaw 的 ±180 归一化跳变 */
	float			yaw;
	float			lastYaw;
	float			targetYaw;

	/* 本段起点：位移判据在"段方向"上做投影，横向漂移不干扰 */
	float			startX;
	float			startY;
	float			startYaw;
} s_nav;

/**
 * @brief  把角度折到 [-180, 180]
 * @author camelliahm
 */
static float nav_WrapDeg(float deg)
{
	while (deg > 180.0f)  { deg -= 360.0f; }
	while (deg < -180.0f) { deg += 360.0f; }
	return deg;
}

/**
 * @brief  用里程计的航向增量维护连续航向
 * @note   里程计航向归一到 ±180，跨过该边界会跳变 360 度；改用增量累加后，
 *         航向单调连续，转向判据与控制都不必处理环绕
 * @author camelliahm
 */
static void nav_UpdateYaw(void)
{
	float now = chassisDrive.odomYaw;

	s_nav.yaw += nav_WrapDeg(now - s_nav.lastYaw);
	s_nav.lastYaw = now;
}

/**
 * @brief  开始当前航段：记录起点、推出目标航向、清速度
 * @note   目标航向以"进入本段时的航向"为基准，故里程计的绝对漂移不会累积到路线里；
 *         平移段把目标航向锁在当前航向，顺带得到跑偏抑制
 * @author camelliahm
 */
static void nav_SegStart(void)
{
	const NavSeg_t *seg = &s_nav.map[s_nav.seg];
	float offset = 0.0f;

	if (seg->type == NAV_SEG_TURN)
	{
		offset = (float)seg->value / 10.0f;		/* 0.1 度 → 度 */
	}

	s_nav.startX   = chassisDrive.odomX;
	s_nav.startY   = chassisDrive.odomY;
	s_nav.startYaw = s_nav.yaw;
	s_nav.hold     = 0u;
	s_nav.segTick  = xTaskGetTickCount();

	s_nav.targetYaw = s_nav.yaw + offset;

	chassis.vx = 0;
	chassis.vy = 0;
	chassis.w  = 0;
}

/**
 * @brief  执行当前航段
 * @retval 1 = 本段完成
 * @note   转向段只判到位，转角速度由姿态环 PID 产生；平移段按"位移在段方向上的投影"判到位
 * @author camelliahm
 */
static uint8_t nav_SegStep(void)
{
	const NavSeg_t *seg = &s_nav.map[s_nav.seg];
	float rad;
	float dx;
	float dy;
	float proj;
	float left;
	float err;
	int16_t v;

	switch (seg->type)
	{
	case NAV_SEG_DWELL:
		chassis.vx = 0;
		chassis.vy = 0;
		return ((xTaskGetTickCount() - s_nav.segTick)
				>= pdMS_TO_TICKS((uint32_t)seg->value)) ? 1u : 0u;

	case NAV_SEG_TURN:
		chassis.vx = 0;
		chassis.vy = 0;
		chassis.w  = 0;

		err = s_nav.yaw - s_nav.targetYaw;
		if (err < 0.0f) { err = -err; }

		if (err > NAV_TURN_WINDOW_DEG)
		{
			s_nav.hold = 0u;
			return 0u;
		}

		s_nav.hold++;
		return (s_nav.hold >= NAV_TURN_HOLD_CNT) ? 1u : 0u;

	case NAV_SEG_END:
	default:
		chassis.vx = 0;
		chassis.vy = 0;
		return 1u;

	case NAV_SEG_FORWARD:
	case NAV_SEG_STRAFE:
		break;
	}

	/* 车体轴在本段起始航向下映射到里程计坐标：
	 * 前进(+y) → (-sin, cos)，右移(+x) → (cos, sin) */
	rad = s_nav.startYaw * NAV_DEG2RAD;
	dx  = chassisDrive.odomX - s_nav.startX;
	dy  = chassisDrive.odomY - s_nav.startY;

	if (seg->type == NAV_SEG_FORWARD)
	{
		proj = dx * (-sinf(rad)) + dy * cosf(rad);
	}
	else
	{
		proj = dx * cosf(rad) + dy * sinf(rad);
	}

	if (seg->value < 0) { proj = -proj; }

	left = (float)seg->value;
	if (left < 0.0f) { left = -left; }
	left -= proj;

	if (left <= 0.0f)
	{
		chassis.vx = 0;
		chassis.vy = 0;
		return 1u;
	}

	/* 进入减速窗口后降速，减小过冲；到位判据仍按剩余距离 */
	v = (left <= (float)NAV_DECEL_MM) ? (int16_t)NAV_DECEL_VEL : seg->speed;
	if (seg->value < 0) { v = (int16_t)(-v); }

	if (seg->type == NAV_SEG_FORWARD)
	{
		chassis.vx = 0;
		chassis.vy = v;
	}
	else
	{
		chassis.vx = v;
		chassis.vy = 0;
	}

	return 0u;
}

/*----------------------------------------------------------*/
/*  对外接口                                                    */
/*----------------------------------------------------------*/

/**
 * @brief  编排层初始化（装载默认地图，不启动）
 * @author camelliahm
 */
void nav_Init(void)
{
	s_nav.state     = NAV_IDLE;
	s_nav.map       = s_defaultMap;
	s_nav.mapNum    = NAV_DEFAULT_NUM;
	s_nav.seg       = 0u;
	s_nav.segTick   = 0u;
	s_nav.hold      = 0u;
	s_nav.yaw       = 0.0f;
	s_nav.lastYaw   = 0.0f;
	s_nav.targetYaw = 0.0f;
	s_nav.startX    = 0.0f;
	s_nav.startY    = 0.0f;
	s_nav.startYaw  = 0.0f;
}

/**
 * @brief  装订地图
 * @param  segs 航段表（只存指针，调用者须保证生命周期，通常用 static const）
 * @param  num  航段数，不得大于 NAV_SEG_MAX
 * @retval true = 已装订
 * @author camelliahm
 */
bool nav_Load(const NavSeg_t *segs, uint16_t num)
{
	if ((segs == NULL) || (num == 0u) || (num > NAV_SEG_MAX))
	{
		return false;
	}

	s_nav.map    = segs;
	s_nav.mapNum = num;
	return true;
}

/**
 * @brief  从第一个航段开始执行
 * @note   航向基准取当前里程计航向，故从任意位置启动都不会引入绝对角误差
 * @author camelliahm
 */
void nav_Start(void)
{
	if ((s_nav.map == NULL) || (s_nav.mapNum == 0u))
	{
		return;
	}

	s_nav.seg     = 0u;
	s_nav.yaw     = chassisDrive.odomYaw;
	s_nav.lastYaw = chassisDrive.odomYaw;
	s_nav.state   = NAV_RUN;

	if (s_nav.map[0].type == NAV_SEG_END)
	{
		s_nav.state = NAV_DONE;
		return;
	}

	nav_SegStart();
}

/**
 * @brief  中止导航并清速度（故障 / 急停 / 外部取消时调用）
 * @author camelliahm
 */
void nav_Abort(void)
{
	s_nav.state = NAV_ABORT;

	chassis.vx = 0;
	chassis.vy = 0;
	chassis.w  = 0;
}

/**
 * @brief  编排层周期推进
 * @note   暂未接入 chassis_ControlStep()；接入点应放在 RUNNING 闸门之后、chassis_DriveStep() 之前，
 *         这样急停与故障的保护可原样继承，本函数只写 chassis.vx/vy，不直接驱动车轮
 * @author camelliahm
 */
void nav_Step(void)
{
	if (s_nav.state != NAV_RUN)
	{
		return;
	}

	nav_UpdateYaw();

	/* 一旦离开 RUNNING（故障 / 急停 / 被停车），立即交出控制权 */
	if (chassis.state != CHS_RUNNING)
	{
		nav_Abort();
		return;
	}

	/* 单段超时：收敛到 FAULT，不自行复位 */
	if ((xTaskGetTickCount() - s_nav.segTick) > pdMS_TO_TICKS(NAV_SEG_TIMEOUT_MS))
	{
		chassis_PostEvent(EV_FAULT, 0);
		nav_Abort();
		return;
	}

	if (nav_SegStep() == 0u)
	{
		return;
	}

	s_nav.seg++;

	if ((s_nav.seg >= s_nav.mapNum) || (s_nav.map[s_nav.seg].type == NAV_SEG_END))
	{
		chassis.vx = 0;
		chassis.vy = 0;
		chassis.w  = 0;
		chassis_PostEvent(EV_STOP, 0);		/* 复用现成的减速停链路收尾 */
		s_nav.state = NAV_DONE;
		return;
	}

	nav_SegStart();
}

/**
 * @brief  读导航状态
 * @author camelliahm
 */
NavState_t nav_GetState(void)
{
	return s_nav.state;
}

/**
 * @brief  读当前航段序号
 * @author camelliahm
 */
uint16_t nav_GetSeg(void)
{
	return s_nav.seg;
}

/**
 * @brief  读姿态环给定航向
 * @note   接入时把 chassis_DriveStep() 中姿态环的 0.0f 换成本函数返回值，转向才会生效
 * @author camelliahm
 */
float nav_GetTargetYaw(void)
{
	return s_nav.targetYaw;
}
