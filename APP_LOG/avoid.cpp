#include "avoid.h"
#include <chrono>
#include "nav.h"
#include "vision.h"

//避障层：把视觉2 的障碍横向偏移转成底盘横向让行分量，叠加到 nav 的速度上

/*---- 内部状态 ----------------------------------------------------*/
static struct {
	uint32_t lastTick;		//最后一次取到障碍帧的时刻
} s_av;

/**
 * @brief  初始化：清避障层状态
 * @author camelliahm
 */
void avoid_Init(void)
{
	s_av.lastTick = 0u;
}

/**
 * @brief  判断本航段是否允许让行
 * @note   返回 1 = 平移族（前进 / 横移 / 位号对准 / 码垛对准）。
 *         转向 / 对准 / 取放 / 停留 / 回零段一律不介入 ——
 *         这些段的速度是"机构动作的一部分"，叠加横向分量会把动作冲掉
 * @author camelliahm
 */
static uint8_t avoid_IsTranslatable(void)
{
	uint8_t t = nav_GetSegType();

	if ((t == (uint8_t)NAV_SEG_FORWARD) ||
		(t == (uint8_t)NAV_SEG_STRAFE)  ||
		(t == (uint8_t)NAV_SEG_RING)    ||
		(t == (uint8_t)NAV_SEG_STACK))
	{
		return 1u;
	}

	return 0u;
}

/**
 * @brief  周期处理：障碍存在时叠加上横向让行分量
 * @note   调用点必须在 nav_Step() 之后。本函数只"叠加"不"覆盖" ——
 *         nav 本段算出的 vx/vy 原样保留，只在 vx 上加一个让行分量，
 *         故横移段的横移目标不会丢失。
 *         只在 RUNNING 且本段属平移族时生效，其余情况不动黑板
 * @author camelliahm
 */
void avoid_Step(void)
{
	VisionObst_t obst;
	int16_t      x;
	int32_t      vx;
	uint32_t     now;

	//非 RUNNING、或本段不属平移族：不改写黑板，nav 的速度原样生效
	if ((chassis.state != CHS_RUNNING) || (avoid_IsTranslatable() == 0u))
	{
		return;
	}

	now = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

	/* 取障碍帧；取不到就看上一帧是否仍在有效期内。
	 * 视觉在障碍存在期间持续发帧，故"最近一帧"的时效即代表障碍是否还在 */
	if (vision_TakeFrame(&obst, 1u) > 0u)
	{
		x = obst.x;
		s_av.lastTick = now;
	}
	else if ((now - s_av.lastTick) > AVOID_HOLD_MS)
	{
		//障碍超时消失：不叠加，nav 本周期算出的速度原样生效
		return;
	}
	else
	{
		//帧还没到但仍在有效期内：沿用槽位里最后一次的偏移
		x = vision_GetSlot()->obst[0].x;
	}

	/* 让行方向：障碍偏左(x>0) → 车右移(vx>0)，符号相同。
	 * |x| 在死区内时障碍视为正前方，固定往右绕，避免零点两侧来回摆 */
	if (x < -(int16_t)AVOID_DEADBAND_CM)
	{
		vx = (int32_t)chassis.vx - (int32_t)AVOID_STRAFE_VEL;	//障碍偏右：往左让
	}
	else
	{
		vx = (int32_t)chassis.vx + (int32_t)AVOID_STRAFE_VEL;	//障碍偏左或正前方：往右让
	}

	//叠加后限幅到 int16（逆运动学会再按轮速上限二次限幅）
	if (vx >  32767) { vx =  32767; }
	if (vx < -32768) { vx = -32768; }

	chassis.vx = (int16_t)vx;
}
