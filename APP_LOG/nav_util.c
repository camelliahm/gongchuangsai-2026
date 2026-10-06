#include "nav_priv.h"

//编排层工具与读接口：纯函数 + 状态 / 统计 / 任务码 / 地图装订

/**
 * @brief  把角度折到 [-180, 180]
 * @note   只用于段起始航向 —— 位移投影用 sinf/cosf，折回后参数更小、精度更好
 * @author camelliahm
 */
float nav_WrapDeg(float deg)
{
	while (deg > 180.0f)  { deg -= 360.0f; }
	while (deg < -180.0f) { deg += 360.0f; }
	return deg;
}

/**
 * @brief  取绝对值（避免引入 math 库）
 * @author camelliahm
 */
float nav_AbsF(float x)
{
	return (x < 0.0f) ? -x : x;
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
 * @brief  读当前航段类型
 * @note   返回 NavSegType_t；段号越界时返回 NAV_SEG_END。
 *         供避障层判断"本段是否属于平移族"，不暴露航段细节
 * @author camelliahm
 */
uint8_t nav_GetSegType(void)
{
	if (s_nav.seg >= s_nav.mapNum)
	{
		return (uint8_t)NAV_SEG_END;
	}

	return s_nav.map[s_nav.seg].type;
}

/**
 * @brief  读任务统计（上屏 / 诊断）
 * @param  dst 输出结构；为空指针时不动
 * @author camelliahm
 */
void nav_GetStats(NavStats_t *dst)
{
	if (dst == NULL)
	{
		return;
	}

	dst->okGet  = s_nav.okGet;
	dst->okPut  = s_nav.okPut;
	dst->errGet = s_nav.errGet;
	dst->errPut = s_nav.errPut;
}

/**
 * @brief  读任务码原文（视觉2 上报的数字串）
 * @param  buf 输出缓冲  max 缓冲容量（含结尾 '\0'）
 * @retval 拷入的字符数；0 = 尚未取到任务码
 * @note   原文可能带 '+' 分隔符，原样拷贝；容量不足则截断
 * @author camelliahm
 */
uint8_t nav_GetTaskCode(char *buf, uint8_t max)
{
	uint8_t n;
	uint8_t i;

	if ((buf == NULL) || (max == 0u))
	{
		return 0u;
	}

	buf[0] = '\0';

	if (s_nav.codeLen == 0u)
	{
		return 0u;
	}

	n = (s_nav.codeLen < (uint8_t)(max - 1u)) ? s_nav.codeLen : (uint8_t)(max - 1u);
	for (i = 0u; i < n; i++)
	{
		buf[i] = s_nav.code[i];
	}
	buf[n] = '\0';

	return n;
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
