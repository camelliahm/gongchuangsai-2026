#include "nav_priv.h"
#include <math.h>

//编排层航段实现：平移 / 横移到位判据、对准段、装载段、取放段

/**
 * @brief  平移 / 横移段：按"位移在段方向上的投影"判到位并给速度
 * @retval 1 = 已到位；0 = 进行中
 * @note   前进(+y) 映射到 (-sin, cos)，右移(+x) 映射到 (cos, sin)；
 *         位移来源由 nav_SegStart() 定（平移取段值，横移按位号折算），本函数只看 s_nav.segDisp
 * @author camelliahm
 */
uint8_t nav_MotionStep(void)
{
	const NavSeg_t *seg = &s_nav.map[s_nav.seg];
	float rad;
	float dx;
	float dy;
	float proj;
	float left;
	int32_t disp;
	int16_t v;

	/* 车体轴在本段起始航向下映射到里程计坐标：
	 * 前进(+y) → (-sin, cos)，右移(+x) → (cos, sin) */
	rad  = s_nav.startYaw * NAV_DEG2RAD;
	dx   = chassisDrive.odomX - s_nav.startX;
	dy   = chassisDrive.odomY - s_nav.startY;
	disp = s_nav.segDisp;

	if (seg->type == NAV_SEG_FORWARD)
	{
		proj = dx * (-sinf(rad)) + dy * cosf(rad);
	}
	else
	{
		proj = dx * cosf(rad) + dy * sinf(rad);
	}

	if (disp < 0) { proj = -proj; }

	left = (float)disp;
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
	if (disp < 0) { v = (int16_t)(-v); }

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
 * @brief  对准段：单次时限 + 超时重试，重试用尽才判故障
 * @retval 1 = 对准完成；0 = 进行中或已判故障
 * @author camelliahm
 */
uint8_t nav_AlignStep(void)
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
 * @brief  装载段：每轮先等目标色、再对准、抓取前复查颜色、再抓取，装满 GRIP_LOAD_TOTAL 轮即完成
 * @retval 1 = 三轮装满；0 = 进行中或已判故障
 * @note   机构空闲时才发起下一轮，抓取失败按 NAV_LOAD_RETRY 重发（grip_LoadOnce 会清故障位）；
 *         前置条件不满足（状态不符 / 升降轴未回零）不重试，直接判故障；
 *         复查不符则退回等色步重来（本轮时限与重试都重算），长期卡住由段超时兜底
 * @author camelliahm
 */
uint8_t nav_LoadStep(void)
{
	uint8_t t;

	//机构动作期间车必须停稳，底盘只由对准步微调
	chassis.vx = 0;
	chassis.vy = 0;

	//每完成一轮装载计一次抓取（cntLoad 是本层影子，段入口随 grip_ResetLoad() 清零）
	if (grip.loadCount > s_nav.cntLoad)
	{
		s_nav.okGet  += (uint8_t)(grip.loadCount - s_nav.cntLoad);
		s_nav.cntLoad = grip.loadCount;
	}

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

	//第 3 步：抓取前复查颜色 —— 对准耗时期间转盘可能又转过一颗，不符就退回等色重来
	if (nav_ColorMatched() == 0u)
	{
		s_nav.loadColored = 0u;
		s_nav.loadAligned = 0u;
		s_nav.retry       = 0u;		//本轮重新开始：等色与对准的时限都重算
		s_nav.tryTick     = xTaskGetTickCount();
		return 0u;
	}

	//第 4 步：发起一轮抓取
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
 * @brief  取放段：车外目标区与物料盘之间搬一件，一次一个
 * @param  slot 物料盘槽位(1~GRIP_TRAY_POS_NUM)，由段值给定
 * @param  mode NAV_GRIP_UNLOAD = 平放卸料；NAV_GRIP_RELOAD = 取回；NAV_GRIP_STACK = 码垛卸料
 * @retval 1 = 本次取放已完成；0 = 进行中或已判故障
 * @note   车须已由上游平移段停在目标区前，本段只管一次取放；
 *         前置条件不满足（状态不符 / 升降轴未回零 / 槽位越界）直接判故障，不重试
 * @author camelliahm
 */
uint8_t nav_GripStep(uint8_t slot, uint8_t mode)
{
	bool ok = false;

	//机构动作期间车必须停稳
	chassis.vx = 0;
	chassis.vy = 0;

	if (s_nav.gripStarted == 0u)
	{
		switch (mode)
		{
		case NAV_GRIP_RELOAD:	ok = grip_ReloadOnce(slot); break;
		case NAV_GRIP_STACK:	ok = grip_StackOnce(slot);  break;
		default:				ok = grip_UnloadOnce(slot); break;
		}

		if (ok == false)
		{
			nav_Fault();
			return 0u;
		}

		s_nav.gripStarted = 1u;
		return 0u;
	}

	if (grip_IsBusy() == true)
	{
		return 0u;
	}

	if (grip.fault != 0u)		//序列内部超时
	{
		nav_Fault();
		return 0u;
	}

	//本次取放完成：取回算抓取，卸料 / 码垛算放置
	if (mode == NAV_GRIP_RELOAD)
	{
		s_nav.okGet++;
	}
	else
	{
		s_nav.okPut++;
	}

	return 1u;
}
