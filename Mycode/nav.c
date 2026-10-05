#include "nav.h"
#include "nav_map.h"
#include "chassis_drive.h"
#include "auxiliary.h"
#include "vision.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#define NAV_TURN_HOLD_CNT	(NAV_TURN_HOLD_MS / NAV_CTRL_PERIOD_MS)	//转向保持判据的周期数
#define NAV_ALIGN_HOLD_CNT	(NAV_ALIGN_HOLD_MS / NAV_CTRL_PERIOD_MS)	//对准保持判据的周期数
#define NAV_DEG2RAD			(0.01745329252f)	//度 → 弧度

/*---- 内部状态 ----------------------------------------*/

static struct {
	NavState_t		state;			//编排状态：是否在跑
	const NavSeg_t *map;			//当前地图（段数组）
	uint16_t		mapNum;			//地图段数
	uint16_t		seg;			//当前段下标
	uint32_t		segTick;		//当前段开始的 tick
	uint16_t		hold;			//判据需持续的控制周期数
	uint8_t			pendStart;		//1 = 收到启动请求，等控制环上下文里装载

	//航向不再本层维护：直接用 chassisDrive.odomYawCont（连续域）
	float			targetYaw;		//本段目标航向(度)

	//本段起点：位移判据在"段方向"上做投影，横向漂移不干扰
	float			startX;			//本段起点 X(mm)
	float			startY;			//本段起点 Y(mm)
	float			startYaw;		//本段起点航向(度)

	//对准 / 装载段
	uint8_t			alignHold;		//已居中的连续周期数（保持判据）
	uint8_t			loadAligned;	//1 = 本轮装载已对准，可以进行抓取
	uint8_t			loadColored;	//1 = 本轮已等到目标色（等色步完成）
	uint8_t			colorHold;		//同色连续帧数（等色判据）
	uint16_t		rxSeen;			//上一帧的 vision1Slot.rxCnt，用于逐帧计数
	uint8_t			retry;			//本段已重试次数（对准与抓取共用该计数）
	uint32_t		tryTick;		//本次尝试开始的 tick；重试时重置

	//任务码第 1 组（第一批物料的搬运顺序）：第 k 位 = 第 k 轮要抓的颜色编号（1~6）
	uint8_t			order[GRIP_TRAY_POS_NUM];	//搬运顺序的颜色编号
	uint8_t			orderGot;					//1 = 已解析到任务码顺序
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
 * @brief  本段判故障：投事件并中止导航
 * @note   收敛到 FAULT 由状态机完成，本层不自行复位
 * @author camelliahm
 */
static void nav_Fault(void)
{
	contract_PostEvent(EV_FAULT, 0);
	nav_Abort();
}

/**
 * @brief  取绝对值（避免引入 math 库）
 * @author camelliahm
 */
static float nav_AbsF(float x)
{
	return (x < 0.0f) ? -x : x;
}

/**
 * @brief  单轴对准速度：相对距离 → 底盘速度（比例控制 + 两档限幅）
 * @param  err 相对距离(mm)，正 = 车需沿本轴正向移动
 * @retval 目标速度(mm/s)，带符号；限幅决定大小
 * @note   逼近窗口时改用降速档，否则 Kp 越大过冲越明显
 * @author camelliahm
 */
static int16_t nav_AlignVel(int32_t err)
{
	float lim = (nav_AbsF((float)err) <= (float)NAV_ALIGN_SLOW_MM)
				? (float)NAV_ALIGN_SLOW_VEL : (float)NAV_ALIGN_VEL_MAX;
	float v = (float)err * NAV_ALIGN_KP;

	if (v > lim)  { v = lim; }
	if (v < -lim) { v = -lim; }

	return (int16_t)v;
}

/**
 * @brief  本次尝试是否超时，并对超时做出计数与重开
 * @param  tryMs    单次尝试的时限(ms)
 * @param  retryMax 允许的重试次数
 * @retval 0 = 仍在时限内；1 = 已重开一次尝试；2 = 重试次数用尽
 * @note   返回 1 时由调用方复位本次尝试的中间量（如 alignHold）；
 *         段的总时长由 nav_Step() 的段超时兜底，本函数只算单次尝试
 * @author camelliahm
 */
static uint8_t nav_TryTimeout(uint32_t tryMs, uint8_t retryMax)
{
	if ((xTaskGetTickCount() - s_nav.tryTick) <= pdMS_TO_TICKS(tryMs))
	{
		return 0u;
	}

	if (s_nav.retry >= retryMax)
	{
		return 2u;
	}

	s_nav.retry++;
	s_nav.tryTick = xTaskGetTickCount();
	return 1u;
}

/**
 * @brief  对准控制律：按视觉1 的相对距离微调底盘，居中后停住保持
 * @retval 1 = 已居中并保持够久；0 = 仍在收敛（含视觉离线）
 * @note   只写 chassis.vx/vy，航向仍锁在进入本段时的姿态；
 *         front 正 = 车需前进 → +y，left 正 = 车需左移 → −x（vx 右移为正）
 * @author camelliahm
 */
static uint8_t nav_AlignCtrl(void)
{
	const Vision1Slot_t *vs = vision1_GetSlot();
	float errF;
	float errL;

	//对准期间默认停住：视觉离线、居中等待都不留上一周期的残余速度
	chassis.vx = 0;
	chassis.vy = 0;

	if (vs->online == 0u)
	{
		s_nav.alignHold = 0u;		//距离不可信：不动作，等恢复或超时
		return 0u;
	}

	errF = (float)vs->front;
	errL = (float)vs->left;

	if ((nav_AbsF(errF) > (float)NAV_ALIGN_TOL_MM)
			|| (nav_AbsF(errL) > (float)NAV_ALIGN_TOL_MM))
	{
		s_nav.alignHold = 0u;
		chassis.vy = nav_AlignVel((int32_t)vs->front);
		chassis.vx = nav_AlignVel(-(int32_t)vs->left);
		return 0u;
	}

	//已进入窗口：停住并保持，避免带速进入抓取
	s_nav.alignHold++;
	return (s_nav.alignHold >= NAV_ALIGN_HOLD_CNT) ? 1u : 0u;
}

/**
 * @brief  对准段：单次时限 + 超时重试，重试用尽才判故障
 * @retval 1 = 对准完成；0 = 进行中或已判故障
 * @author camelliahm
 */
static uint8_t nav_AlignStep(void)
{
	uint8_t t;

	//重开尝试 / 判故障的那一周期也不留上一周期的残余速度
	chassis.vx = 0;
	chassis.vy = 0;

	t = nav_TryTimeout(NAV_ALIGN_TRY_TIMEOUT_MS, NAV_ALIGN_RETRY);

	if (t == 2u)
	{
		nav_Fault();
		return 0u;
	}

	if (t == 1u)
	{
		s_nav.alignHold = 0u;		//重开一次尝试：从当前位置重新收敛
		return 0u;
	}

	return nav_AlignCtrl();
}

/**
 * @brief  解析任务码第 1 组：三个颜色编号即第一批物料的搬运顺序
 * @note   视觉模块 2 上报的是 ASCII 原文（如 "156+123+516+231"），本函数只取
 *         第一个 '+' 之前的至多三位数字；没有新任务码时保留上次结果，
 *         避免视觉短暂丢帧把顺序清成空
 * @author camelliahm
 */
static void nav_TaskOrder(void)
{
	VisionTask_t t;
	uint8_t i;
	uint8_t n = 0u;

	if (vision_TakeTask(&t) == false)
	{
		return;
	}

	for (i = 0u; (i < t.len) && (n < (uint8_t)GRIP_TRAY_POS_NUM); i++)
	{
		if ((t.code[i] >= '1') && (t.code[i] <= '6'))
		{
			s_nav.order[n] = (uint8_t)(t.code[i] - '0');
			n++;
		}
		else if (n > 0u)
		{
			break;		//第 1 组结束（遇 '+' 或其它分隔符）
		}
	}

	if (n == (uint8_t)GRIP_TRAY_POS_NUM)
	{
		s_nav.orderGot = 1u;
	}
}

/**
 * @brief  本轮的目标颜色：任务码第 1 组里第 (已完成轮数 + 1) 位
 * @retval 颜色代号 1~6；未取到任务码或序号越界时返回 VISION1_COLOR_NONE
 * @note   槽位与轮次同序，故目标色直接取 order[loadCount]，不需按色查表
 * @author camelliahm
 */
static uint8_t nav_TargetColor(void)
{
	if ((s_nav.orderGot == 0u) || (grip.loadCount >= (uint8_t)GRIP_TRAY_POS_NUM))
	{
		return (uint8_t)VISION1_COLOR_NONE;
	}

	return s_nav.order[grip.loadCount];
}

/**
 * @brief  等色步：原地等本轮目标色转到取料点，同色连续 NAV_COLOR_HOLD_FRAMES 帧算停稳
 * @retval 1 = 目标色已稳定停住；0 = 仍在等（或已判故障）
 * @note   转盘停位随机，不能保证首次停的就是目标色，故先等色再对准抓取；
 *         等色期间车原地泊住（转盘在换料，追没有意义），颜色不可信时同样不动作；
 *         任务码可能晚到，本步会补解析一次；超时用尽即判故障
 * @author camelliahm
 */
static uint8_t nav_ColorWait(void)
{
	const Vision1Slot_t *vs = vision1_GetSlot();
	uint8_t target;

	//等色期间车停住：转盘在换料，底盘不必动作
	chassis.vx = 0;
	chassis.vy = 0;

	if (s_nav.orderGot == 0u)
	{
		nav_TaskOrder();		//段入口没取到任务码：这里再试一次
	}

	if (nav_TryTimeout(NAV_COLOR_TIMEOUT_MS, 0u) == 2u)
	{
		nav_Fault();
		return 0u;
	}

	target = nav_TargetColor();

	//以视觉帧为节拍：没有新帧不重复计数，避免同一帧被算成多次
	if (vs->rxCnt == s_nav.rxSeen)
	{
		return 0u;
	}
	s_nav.rxSeen = vs->rxCnt;

	if ((vs->online == 0u) || (vs->color == VISION1_COLOR_NONE) || (vs->color != target))
	{
		s_nav.colorHold = 0u;
		return 0u;
	}

	s_nav.colorHold++;
	return (s_nav.colorHold >= NAV_COLOR_HOLD_FRAMES) ? 1u : 0u;
}

/**
 * @brief  装载段：每轮先等目标色、再对准、再抓取，装满 GRIP_LOAD_TOTAL 轮即完成
 * @retval 1 = 三轮装满；0 = 进行中或已判故障
 * @note   机构空闲时才发起下一轮，抓取失败按 NAV_LOAD_RETRY 重发（grip_LoadOnce 会清故障位）；
 *         前置条件不满足（状态不符 / 升降轴未回零）不重试，直接判故障
 * @author camelliahm
 */
static uint8_t nav_LoadStep(void)
{
	uint8_t t;

	//机构动作期间车必须停稳，底盘只由对准步微调
	chassis.vx = 0;
	chassis.vy = 0;

	if (grip_AllLoaded() == true)
	{
		//装满后还有"物料盘归位"的收尾动作，等机构空闲才算本段走完
		return (grip_IsBusy() == false) ? 1u : 0u;
	}

	if (grip_IsBusy() == true)
	{
		return 0u;
	}

	//第 1 步：等本轮目标色（挑色抓取：转盘停位随机，先等到目标色再动手）
	if (s_nav.loadColored == 0u)
	{
		if (nav_ColorWait() == 0u)
		{
			return 0u;
		}

		s_nav.loadColored = 1u;
		s_nav.alignHold   = 0u;
		s_nav.tryTick     = xTaskGetTickCount();	//对准计时从本轮重新起算
	}

	//第 2 步：视觉闭环对准
	if (s_nav.loadAligned == 0u)
	{
		t = nav_TryTimeout(NAV_ALIGN_TRY_TIMEOUT_MS, NAV_ALIGN_RETRY);
		if (t == 2u)
		{
			nav_Fault();
			return 0u;
		}
		if (t == 1u)
		{
			s_nav.alignHold = 0u;
			return 0u;
		}

		if (nav_AlignCtrl() == 0u)
		{
			return 0u;
		}

		s_nav.loadAligned = 1u;
	}

	//第 3 步：发起一轮抓取
	if (grip.fault != 0u)		//上一轮失败：计数后重发，重试用尽才判故障
	{
		if (s_nav.retry >= NAV_LOAD_RETRY)
		{
			nav_Fault();
			return 0u;
		}
		s_nav.retry++;
	}

	if (grip_LoadOnce() == false)
	{
		nav_Fault();
		return 0u;
	}

	//下一轮回到等色步
	s_nav.loadColored = 0u;
	s_nav.loadAligned = 0u;
	s_nav.tryTick     = xTaskGetTickCount();
	return 0u;
}

/**
 * @brief  开始当前航段：记录起点、推出目标航向、清速度、复位本段重试与判据
 * @note   目标航向以"进入本段时的连续航向"为基准，故里程计的绝对漂移不会累积到路线里；
 *         平移段把目标航向锁在当前航向，顺带得到跑偏抑制
 * @author camelliahm
 */
static void nav_SegStart(void)
{
	const NavSeg_t *seg = &s_nav.map[s_nav.seg];
	uint32_t now = xTaskGetTickCount();
	float offset = 0.0f;

	if (seg->type == NAV_SEG_TURN)
	{
		offset = (float)seg->value / 10.0f;		//0.1 度 → 度
	}

	s_nav.startX   = chassisDrive.odomX;
	s_nav.startY   = chassisDrive.odomY;
	s_nav.startYaw = nav_WrapDeg(chassisDrive.odomYawCont);
	s_nav.hold     = 0u;
	s_nav.segTick  = now;

	//本段的重试与判据都从零开始，段间不继承
	s_nav.tryTick     = now;
	s_nav.retry       = 0u;
	s_nav.alignHold   = 0u;
	s_nav.loadAligned = 0u;
	s_nav.loadColored = 0u;
	s_nav.colorHold   = 0u;
	s_nav.rxSeen      = vision1_GetSlot()->rxCnt;	//以当前帧为基准，免把旧帧当本轮新帧

	//装载的前置条件是升降轴已回零，故进入回零段时触发一次（完成由 aux 侧置 homed）
	if ((seg->type == NAV_SEG_HOME) && (lift.homed == 0u))
	{
		lift_Home();
	}

	//装载段入口：取任务码第 1 组作为搬运顺序，并清装载计数
	//（不清则 grip_AllLoaded() 恒真 —— 跑完一批再启动导航，本段会秒过）
	if (seg->type == NAV_SEG_LOAD)
	{
		nav_TaskOrder();
		grip_ResetLoad();
	}

	//目标航向与 odomYawCont 同在连续域，姿态环可直接相减
	s_nav.targetYaw   = chassisDrive.odomYawCont + offset;
	chassis.yawTarget = s_nav.targetYaw;

	chassis.vx = 0;
	chassis.vy = 0;
	chassis.w  = 0;
}

/**
 * @brief  执行当前航段，本段完成时返回 1
 * @note   转向段只判到位，转角速度由姿态环 PID 产生；平移段按"位移在段方向上的投影"判到位；
 *         回零 / 对准 / 装载三段各自管住自己的重试，段超时只作兜底
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

	case NAV_SEG_HOME:
		chassis.vx = 0;
		chassis.vy = 0;
		return (lift.homed != 0u) ? 1u : 0u;	//回零在进入本段时已触发，这里只等结果

	case NAV_SEG_ALIGN:
		return nav_AlignStep();

	case NAV_SEG_LOAD:
		return nav_LoadStep();

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

	//进入减速窗口后降速，减小过冲；到位判据仍按剩余距离
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

	//航向基准取当前连续航向，故从任意位置启动都不会引入绝对角误差
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
	uint8_t i;

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

	s_nav.alignHold   = 0u;
	s_nav.loadAligned = 0u;
	s_nav.loadColored = 0u;
	s_nav.colorHold   = 0u;
	s_nav.rxSeen      = 0u;
	s_nav.retry       = 0u;
	s_nav.tryTick     = 0u;

	for (i = 0u; i < (uint8_t)GRIP_TRAY_POS_NUM; i++)
	{
		s_nav.order[i] = 0u;
	}
	s_nav.orderGot = 0u;
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
	uint32_t tmo;

	//启动请求在本上下文消费：装载过程不会被状态机任务打断
	if (s_nav.pendStart)
	{
		s_nav.pendStart = 0u;
		nav_Begin();
	}

	if (s_nav.state != NAV_RUN)
	{
		return;
	}

	//一旦离开 RUNNING（故障 / 急停 / 被停车），立即交出控制权
	if (chassis.state != CHS_RUNNING)
	{
		nav_Abort();
		return;
	}

	//单段超时（按段取）：收敛到 FAULT，不自行复位
	tmo = s_nav.map[s_nav.seg].timeout;
	if (tmo == 0u)
	{
		tmo = NAV_SEG_TIMEOUT_MS;
	}

	if ((xTaskGetTickCount() - s_nav.segTick) > pdMS_TO_TICKS(tmo))
	{
		nav_Fault();
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
		contract_PostEvent(EV_STOP, 0);		//复用现成的减速停链路收尾
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
