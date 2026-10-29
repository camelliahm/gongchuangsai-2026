#include "nav_priv.h"
#include "nav_map.h"
#include <math.h>

//编排层（任务层）：航段调度与对外入口。详见 README.md
//段类型实现见 nav_seg.c / 对准挑色见 nav_align.c / 工具与读接口见 nav_util.c

/*---- 内部状态（跨文件共享，声明在 nav_priv.h）----------------------------------------*/

Nav_t s_nav;

/**
 * @brief  本段判故障：投事件并中止导航
 * @note   收敛到 FAULT 由状态机完成，本层不自行复位；
 *         取放族的段在这里计入失败次数，其余段（对准超时 / 位号越界）不计
 * @author camelliahm
 */
void nav_Fault(void)
{
	if ((s_nav.map != NULL) && (s_nav.seg < s_nav.mapNum))
	{
		switch (s_nav.map[s_nav.seg].type)
		{
		case NAV_SEG_LOAD:
		case NAV_SEG_RELOAD:	s_nav.errGet++;	break;
		case NAV_SEG_UNLOAD:
		case NAV_SEG_STACKED:	s_nav.errPut++;	break;
		default:								break;
		}
	}

	contract_PostEvent(EV_FAULT, 0);
	nav_Abort();
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
uint8_t nav_TryTimeout(uint32_t tryMs, uint8_t retryMax)
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
 * @brief  本段类型是否属于"目标区取放"一族（区域内连续横移 / 取放，位号跨段保持）
 * @retval 1 = 位号沿用；0 = 车已离开目标区，位号复位到入口停靠点
 * @author camelliahm
 */
uint8_t nav_IsRingFamily(uint8_t type)
{
	if ((type == (uint8_t)NAV_SEG_RING) || (type == (uint8_t)NAV_SEG_STACK)
			|| (type == (uint8_t)NAV_SEG_UNLOAD) || (type == (uint8_t)NAV_SEG_RELOAD)
			|| (type == (uint8_t)NAV_SEG_STACKED))
	{
		return 1u;
	}

	return 0u;
}

/**
 * @brief  开始当前航段：记录起点、推出目标航向、清速度、复位本段重试与判据
 * @note   目标航向以"进入本段时的连续航向"为基准，故里程计的绝对漂移不会累积到路线里；
 *         平移段把目标航向锁在当前航向，顺带得到跑偏抑制
 * @author camelliahm
 */
void nav_SegStart(void)
{
	const NavSeg_t *seg = &s_nav.map[s_nav.seg];
	uint32_t now = xTaskGetTickCount();
	float offset = 0.0f;
	uint8_t ring = 0u;

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
	s_nav.tryTick       = now;
	s_nav.retry         = 0u;
	s_nav.alignHold     = 0u;
	s_nav.loadAligned   = 0u;
	s_nav.loadColored   = 0u;
	s_nav.colorHold     = 0u;
	s_nav.gripStarted = 0u;
	s_nav.rxSeen        = vision1_GetSlot()->rxCnt;	//以当前帧为基准，免把旧帧当本轮新帧

	//位号基准：非取放族的那一段意味着车刚进（或刚离开）目标区，位号从入口停靠点重算
	if (nav_IsRingFamily(seg->type) == 0u)
	{
		s_nav.ringAt = (uint8_t)NAV_MAP_RING_DOCK_NUM;
	}

	//装载的前置条件是升降轴已回零，故进入回零段时触发一次（完成由 aux 侧置 homed）
	if ((seg->type == NAV_SEG_HOME) && (lift.homed == 0u))
	{
		lift_Home();
	}

	//装载段入口：按段值定的颜色组取任务码，并清装载计数
	//（不清则 grip_AllLoaded() 恒真 —— 跑完一批再启动导航，本段会秒过）
	if (seg->type == NAV_SEG_LOAD)
	{
		s_nav.loadGroup = (uint8_t)seg->value;
		nav_TaskOrder();
		grip_ResetLoad();
		s_nav.cntLoad   = 0u;		//统计影子与装载计数同步清零
	}

	//本段位移：平移段直接取段值；目标区横移段按"目标位号 − 当前位号"折算
	s_nav.segDisp = 0;
	if ((seg->type == NAV_SEG_FORWARD) || (seg->type == NAV_SEG_STRAFE))
	{
		s_nav.segDisp = (int32_t)seg->value;
	}
	else if (seg->type == NAV_SEG_RING)
	{
		if (s_nav.taskGot == 0u)
		{
			nav_TaskOrder();		//段入口没取到任务码：这里再试一次
		}
		ring = nav_TaskRing((uint8_t)(seg->value / 10), (uint8_t)(seg->value % 10));
	}
	else if (seg->type == NAV_SEG_STACK)
	{
		if (s_nav.taskGot == 0u)
		{
			nav_TaskOrder();
		}
		ring = nav_StackRing((uint8_t)seg->value);
	}

	if ((seg->type == NAV_SEG_RING) || (seg->type == NAV_SEG_STACK))
	{
		if ((ring < 1u) || (ring > (uint8_t)NAV_TASK_GROUP_LEN))
		{
			nav_Fault();		//位号取不到 / 越界：任务码不符，不猜
			return;
		}

		s_nav.segDisp = ((int32_t)ring - (int32_t)s_nav.ringAt)
						* (int32_t)NAV_MAP_RING_PITCH_MM * (int32_t)NAV_MAP_RING_DIR;
		s_nav.ringAt  = ring;
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
 * @note   转向 / 停留 / 回零段在此就地判定，其余段分派给 nav_seg.c；
 *         转向段只判到位，转角速度由姿态环 PID 产生；平移与横移段共用到位判据
 * @author camelliahm
 */
uint8_t nav_SegStep(void)
{
	const NavSeg_t *seg = &s_nav.map[s_nav.seg];
	float err;

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

	case NAV_SEG_UNLOAD:
		return nav_GripStep((uint8_t)seg->value, NAV_GRIP_UNLOAD);

	case NAV_SEG_RELOAD:
		return nav_GripStep((uint8_t)seg->value, NAV_GRIP_RELOAD);

	case NAV_SEG_STACKED:
		return nav_GripStep((uint8_t)seg->value, NAV_GRIP_STACK);

	case NAV_SEG_END:
	default:
		chassis.vx = 0;
		chassis.vy = 0;
		return 1u;

	case NAV_SEG_FORWARD:
	case NAV_SEG_STRAFE:
	case NAV_SEG_RING:
	case NAV_SEG_STACK:
		return nav_MotionStep();
	}
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
	uint8_t k;

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

	s_nav.gripStarted = 0u;
	s_nav.segDisp     = 0;
	s_nav.ringAt      = (uint8_t)NAV_MAP_RING_DOCK_NUM;
	s_nav.loadGroup   = (uint8_t)NAV_TASK_G_ORDER1;

	for (i = 0u; i < (uint8_t)NAV_TASK_GROUP_NUM; i++)
	{
		for (k = 0u; k < (uint8_t)NAV_TASK_GROUP_LEN; k++)
		{
			s_nav.task[i][k] = 0u;
		}
	}
	s_nav.taskGot = 0u;

	s_nav.okGet   = 0u;
	s_nav.okPut   = 0u;
	s_nav.errGet  = 0u;
	s_nav.errPut  = 0u;
	s_nav.cntLoad = 0u;
	s_nav.codeLen = 0u;
	s_nav.code[0] = '\0';
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
