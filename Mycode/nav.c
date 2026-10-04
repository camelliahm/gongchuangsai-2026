#include "nav.h"
#include "nav_map.h"
#include "chassis_drive.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#define NAV_TURN_HOLD_CNT	(NAV_TURN_HOLD_MS / NAV_CTRL_PERIOD_MS)
#define NAV_DEG2RAD			(0.01745329252f)

/*---- 内部状态 ----------------------------------------*/

static struct {
	NavState_t		state;
	const NavSeg_t *map;
	uint16_t		mapNum;
	uint16_t		seg;
	uint32_t		segTick;
	uint16_t		hold;
	uint8_t			pendStart;		/* 1 = 收到启动请求，等控制环上下文里装载 */

	/* 航向不再本层维护：直接用 chassisDrive.odomYawCont（连续域） */
	float			targetYaw;

	/* 本段起点：位移判据在"段方向"上做投影，横向漂移不干扰 */
	float			startX;
	float			startY;
	float			startYaw;
} s_nav;

/**
 * @brief  把角度折到 [-180, 180]
 * @note   只用于段起始航向 —— 位移投影用 sinf/cosf，折回后参数更小、精度更好
 * @author camelliahm
 */
static float nav_WrapDeg(float deg)
{
	while (deg > 180.0f)  { deg -= 360.0f; }
	while (deg < -180.0f) { deg += 360.0f; }
	return deg;
}

/**
 * @brief  开始当前航段：记录起点、推出目标航向、清速度
 * @note   目标航向以"进入本段时的连续航向"为基准，故里程计的绝对漂移不会累积到路线里；
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
	s_nav.startYaw = nav_WrapDeg(chassisDrive.odomYawCont);
	s_nav.hold     = 0u;
	s_nav.segTick  = xTaskGetTickCount();

	/* 目标航向与 odomYawCont 同在连续域，姿态环可直接相减 */
	s_nav.targetYaw   = chassisDrive.odomYawCont + offset;
	chassis.yawTarget = s_nav.targetYaw;

	chassis.vx = 0;
	chassis.vy = 0;
	chassis.w  = 0;
}

/**
 * @brief  执行当前航段，本段完成时返回 1
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

		err = chassisDrive.odomYawCont - s_nav.targetYaw;
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

/**
 * @brief  真正启动：装载第一个航段，确立航向与位移基准
 * @note   只在控制环上下文调用（由 nav_Step() 消费启动请求时进入）
 * @author camelliahm
 */
static void nav_Begin(void)
{
	if ((s_nav.map == NULL) || (s_nav.mapNum == 0u))
	{
		return;
	}

	s_nav.seg  = 0u;
	s_nav.hold = 0u;

	if (s_nav.map[0].type == NAV_SEG_END)
	{
		s_nav.state = NAV_DONE;
		return;
	}

	/* 航向基准取当前连续航向，故从任意位置启动都不会引入绝对角误差 */
	s_nav.state = NAV_RUN;
	nav_SegStart();
}

/*---- 对外接口 ----------------------------------------*/

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
	s_nav.pendStart = 0u;
	s_nav.targetYaw = 0.0f;
	s_nav.startX    = 0.0f;
	s_nav.startY    = 0.0f;
	s_nav.startYaw  = 0.0f;
}

/**
 * @brief  装订地图，成功返回 true
 * @param  segs 航段表（只存指针，调用者须保证生命周期，通常用 static const）
 * @param  num  航段数，不得大于 NAV_SEG_MAX
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
 * @brief  请求从第一个航段开始执行（只置标志，不直接改状态）
 * @note   调用方通常是状态机任务，优先级低于控制环任务；直接写 s_nav 会在
 *         装载中途被抢占，让控制环读到半新半旧的段基准。改由 nav_Step()
 *         在控制环上下文里统一装载，该跨任务窗口即被消除
 * @author camelliahm
 */
void nav_Start(void)
{
	s_nav.pendStart = 1u;
}

/**
 * @brief  中止导航并清速度
 * @note   正常路径由 nav_Step() 在控制环上下文里调用（离开 RUNNING 时归位）；
 *         外部调用亦可，但需自行承担与 nav_Step() 的并发
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
 * @brief  编排层周期推进（控制环调用，5ms）
 * @note   在 contract_ControlStep() 的 RUNNING 闸门**之前**调用是必须的：
 *         本函数自带"离开 RUNNING 即中止并交还控制权"的归位逻辑，放到
 *         闸门之后会让该逻辑永远不可达 —— 于是停车或急停后再进 RUNNING，
 *         导航会拿着失效的段基准接着往下跑。
 *         只写 chassis.vx/vy/w 与 chassis.yawTarget，不直接驱动车轮
 * @author camelliahm
 */
void nav_Step(void)
{
	uint16_t tmo;

	/* 启动请求在本上下文消费：装载过程不会被状态机任务打断 */
	if (s_nav.pendStart)
	{
		s_nav.pendStart = 0u;
		nav_Begin();
	}

	if (s_nav.state != NAV_RUN)
	{
		return;
	}

	/* 一旦离开 RUNNING（故障 / 急停 / 被停车），立即交出控制权 */
	if (chassis.state != CHS_RUNNING)
	{
		nav_Abort();
		return;
	}

	/* 单段超时（按段取）：收敛到 FAULT，不自行复位 */
	tmo = s_nav.map[s_nav.seg].timeout;
	if (tmo == 0u)
	{
		tmo = NAV_SEG_TIMEOUT_MS;
	}

	if ((xTaskGetTickCount() - s_nav.segTick) > pdMS_TO_TICKS((uint32_t)tmo))
	{
		contract_PostEvent(EV_FAULT, 0);
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
		contract_PostEvent(EV_STOP, 0);		/* 复用现成的减速停链路收尾 */
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
 * @brief  读姿态环给定航向（调试 / 观察用）
 * @note   驱动层通过黑板 chassis.yawTarget 取该给定，不依赖本函数
 * @author camelliahm
 */
float nav_GetTargetYaw(void)
{
	return s_nav.targetYaw;
}
