#include "nav_priv.h"

//编排层对准与挑色：视觉1 闭环对准 + 任务码解析与目标色判定

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
 * @brief  对准控制律：按视觉1 的相对距离微调底盘，居中后停住保持
 * @retval 1 = 已居中并保持够久；0 = 仍在收敛（含视觉离线）
 * @note   只写 chassis.vx/vy，航向仍锁在进入本段时的姿态；
 *         front 正 = 车需前进 → +y，left 正 = 车需左移 → −x（vx 右移为正）
 * @author camelliahm
 */
uint8_t nav_AlignCtrl(void)
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
 * @brief  解析任务码：抽出全部数字按每组 3 位切分，校验后写入 s_nav.task
 * @param  code ASCII 原文  len 有效字符数
 * @retval 通过校验的组数（全部通过时等于 NAV_TASK_GROUP_NUM，其余情况为 0）
 * @note   视觉模块 2 上报的是 12 位纯数字（如 "156123516231"），'+' 等分隔符一律跳过，
 *         故带分隔符的原文同样可解析；颜色组只认 1~6，有任一位越界即整码作废；
 *         位置组不做范围校验 —— 现场圆环数未定，交由消费方判断
 * @author camelliahm
 */
static uint8_t nav_ParseTask(const char *code, uint8_t len)
{
	uint8_t buf[NAV_TASK_GROUP_NUM][NAV_TASK_GROUP_LEN];
	uint8_t total = (uint8_t)(NAV_TASK_GROUP_NUM * NAV_TASK_GROUP_LEN);
	uint8_t n = 0u;
	uint8_t i;
	uint8_t g;
	uint8_t k;

	for (i = 0u; (i < len) && (n < total); i++)
	{
		if ((code[i] >= '0') && (code[i] <= '9'))
		{
			buf[n / NAV_TASK_GROUP_LEN][n % NAV_TASK_GROUP_LEN] = (uint8_t)(code[i] - '0');
			n++;
		}
	}

	if (n < total)
	{
		return 0u;		//位数不足：整码作废，保留上次结果
	}

	for (g = 0u; g < (uint8_t)NAV_TASK_GROUP_NUM; g++)
	{
		if ((g != (uint8_t)NAV_TASK_G_ORDER1) && (g != (uint8_t)NAV_TASK_G_ORDER2))
		{
			continue;	//位置组不校验范围
		}

		for (k = 0u; k < (uint8_t)NAV_TASK_GROUP_LEN; k++)
		{
			if ((buf[g][k] < (uint8_t)NAV_TASK_COLOR_MIN)
					|| (buf[g][k] > (uint8_t)NAV_TASK_COLOR_MAX))
			{
				return 0u;
			}
		}
	}

	for (g = 0u; g < (uint8_t)NAV_TASK_GROUP_NUM; g++)
	{
		for (k = 0u; k < (uint8_t)NAV_TASK_GROUP_LEN; k++)
		{
			s_nav.task[g][k] = buf[g][k];
		}
	}

	//原文一并留存：上屏显示的是视觉上报的字符串，不是解析后的数字表
	s_nav.codeLen = 0u;
	for (i = 0u; (i < len) && (i < (uint8_t)(VISION_TASK_CODE_MAX - 1u)); i++)
	{
		s_nav.code[i] = code[i];
		s_nav.codeLen++;
	}
	s_nav.code[s_nav.codeLen] = '\0';

	return (uint8_t)NAV_TASK_GROUP_NUM;
}

/**
 * @brief  取并解析任务码：4 组全部通过校验才置 taskGot
 * @note   视觉模块 2 上报的是 ASCII 数字原文（如 "156123516231"）；
 *         没有新任务码时保留上次结果，避免视觉短暂丢帧把任务清空
 * @author camelliahm
 */
void nav_TaskOrder(void)
{
	VisionTask_t t;

	if (vision_TakeTask(&t) == false)
	{
		return;
	}

	if (nav_ParseTask(t.code, t.len) == (uint8_t)NAV_TASK_GROUP_NUM)
	{
		s_nav.taskGot = 1u;
	}
}

/**
 * @brief  本轮的目标颜色：本轮颜色组里第 (已完成轮数 + 1) 位
 * @retval 颜色代号 1~6；未取到任务码或序号越界时返回 VISION1_COLOR_NONE
 * @note   本轮读哪一组由装载段的段值给定（第一批读第 1 组 / 第二批读第 3 组）；
 *         槽位与轮次同序，故目标色直接取 task[loadGroup][loadCount]，不需按色查表
 * @author camelliahm
 */
uint8_t nav_TargetColor(void)
{
	if ((s_nav.taskGot == 0u) || (grip.loadCount >= (uint8_t)NAV_TASK_GROUP_LEN))
	{
		return (uint8_t)VISION1_COLOR_NONE;
	}

	return s_nav.task[s_nav.loadGroup][grip.loadCount];
}

/**
 * @brief  任务码指定位置：第 group 组第 idx 位
 * @retval 位置号；未取到任务码或下标越界时返回 0
 * @author camelliahm
 */
uint8_t nav_TaskRing(uint8_t group, uint8_t idx)
{
	if ((s_nav.taskGot == 0u) || (group >= (uint8_t)NAV_TASK_GROUP_NUM)
			|| (idx >= (uint8_t)NAV_TASK_GROUP_LEN))
	{
		return 0u;
	}

	return s_nav.task[group][idx];
}

/**
 * @brief  码垛位：第二批第 k 件颜色在第一批暂存区所占的位置号
 * @param  k 第二批件序号(0~2)
 * @retval 位置号(1~3)；0 = 第一批里没有同色物料（任务码异常）
 * @note   码垛要求"颜色一致"，故按颜色反查第一批的位置，而不是按件序号对应
 * @author camelliahm
 */
uint8_t nav_StackRing(uint8_t k)
{
	uint8_t color;
	uint8_t j;

	if (k >= (uint8_t)NAV_TASK_GROUP_LEN)
	{
		return 0u;
	}

	color = s_nav.task[(uint8_t)NAV_TASK_G_ORDER2][k];

	for (j = 0u; j < (uint8_t)NAV_TASK_GROUP_LEN; j++)
	{
		if (s_nav.task[(uint8_t)NAV_TASK_G_ORDER1][j] == color)
		{
			return s_nav.task[(uint8_t)NAV_TASK_G_POS1][j];
		}
	}

	return 0u;
}

/**
 * @brief  等色步：原地等本轮目标色转到取料点，同色连续 NAV_COLOR_HOLD_FRAMES 帧算停稳
 * @retval 1 = 目标色已稳定停住；0 = 仍在等（或已判故障）
 * @note   转盘停位随机，不能保证首次停的就是目标色，故先等色再对准抓取；
 *         等色期间车原地泊住（转盘在换料，追没有意义），颜色不可信时同样不动作；
 *         任务码可能晚到，本步会补解析一次；超时用尽即判故障
 * @author camelliahm
 */
uint8_t nav_ColorWait(void)
{
	const Vision1Slot_t *vs = vision1_GetSlot();
	uint8_t target;

	//等色期间车停住：转盘在换料，底盘不必动作
	chassis.vx = 0;
	chassis.vy = 0;

	if (s_nav.taskGot == 0u)
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
 * @brief  抓取前的颜色复查：此刻视野里的物料是不是本轮目标色
 * @retval 1 = 在线且颜色等于目标色；0 = 颜色不符或不可信
 * @note   等色判据是"对准之前"的快照，对准耗时期间转盘可能又转过一颗，
 *         那时对准收敛到的已不是目标色物料，故动手前按当前帧复查一次
 * @author camelliahm
 */
uint8_t nav_ColorMatched(void)
{
	const Vision1Slot_t *vs = vision1_GetSlot();

	if ((vs->online == 0u) || (vs->color == VISION1_COLOR_NONE))
	{
		return 0u;
	}

	return (vs->color == nav_TargetColor()) ? 1u : 0u;
}
