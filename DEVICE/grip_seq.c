#include "grip_seq.h"
#include "servo.h"
#include "FreeRTOS.h"
#include "task.h"

/*---- 夹持机构（旋转轴 = TIM3 CH1，夹子 = TIM3 CH2）----*/

__IO Grip_t grip;	//夹持机构状态

//槽位序号(1~3) → 物料盘角度，次序与 GRIP_TRAY_POS*_DEG 一致
static const float s_trayDeg[GRIP_TRAY_POS_NUM] = {
	GRIP_TRAY_POS1_DEG, GRIP_TRAY_POS2_DEG, GRIP_TRAY_POS3_DEG
};

/**
 * @brief  设定旋转轴目标角
 * @param  deg 目标角(度)
 * @note   只记目标，实际转动由 rot_Step() 按 GRIP_ROT_SLEW_DEG 每周期逼近；
 *         直接写舵机会一步跳到位，机械冲击大
 * @author camelliahm
 */
void grip_RotTo(float deg)
{
	grip.rotTarget = deg;
}

/**
 * @brief  夹子开 / 合
 * @param  close true = 夹紧，false = 张开
 * @note   夹子不需斜坡，直接给目标角；舵机走完所需时间由序列里的等待时长覆盖
 * @author camelliahm
 */
void grip_SetJaw(bool close)
{
	(void)servo_SetAngle(GRIP_JAW_CH, close ? GRIP_JAW_CLOSE_DEG : GRIP_JAW_OPEN_DEG);
}

/**
 * @brief  物料盘转到指定槽位
 * @param  pos 槽位序号 1~GRIP_TRAY_POS_NUM，越界忽略
 * @note   只记目标，实际转动由 tray_Step() 按 GRIP_TRAY_SLEW_DEG 每周期逼近；
 *         270° 舵机一步跳 240° 会把盘上的物料甩出去
 * @author camelliahm
 */
void grip_TrayTo(uint8_t pos)
{
	if ((pos >= 1u) && (pos <= GRIP_TRAY_POS_NUM))
	{
		grip.trayTarget = s_trayDeg[pos - 1u];
	}
}

/**
 * @brief  物料盘回位置一
 * @note   上电姿态与换批复位共用；同样走斜坡，不会一步跳
 * @author camelliahm
 */
void grip_TrayHome(void)
{
	grip_TrayTo(1u);
}

/**
 * @brief  装载计数清零
 * @note   每批装载由编排层在段入口调用；不清则 grip_AllLoaded() 恒真，装载段会秒过。
 *         顺带把物料盘归位 —— 第 1 轮本来也要转到位置一，提前转可省掉那一段等待
 * @author camelliahm
 */
void grip_ResetLoad(void)
{
	grip.loadCount = 0u;
	grip_TrayHome();
}

/**
 * @brief  中止动作序列
 * @note   只清序列状态，不改变旋转轴与夹子已下发的姿态
 * @author camelliahm
 */
void grip_Abort(void)
{
	grip.busy      = 0u;
	grip.phase     = (uint8_t)GRIP_IDLE;
	grip.phaseTick = xTaskGetTickCount();
}

/**
 * @brief  安全姿态：旋转轴回中间位 + 夹子张开 + 中止序列
 * @note   IDLE / FAULT / ESTOP 由 contract_OnEnter() 调用；
 *         旋转轴仍由 rot_Step() 斜坡推进，不会一次跳 90°
 * @author camelliahm
 */
void grip_Safe(void)
{
	grip_Abort();
	grip.fault = 0u;
	grip_RotTo(GRIP_ROT_SAFE_DEG);
	grip_SetJaw(false);
}

/**
 * @brief  取绝对值（避免引入 math 库）
 * @author camelliahm
 */
static float grip_Abs(float x)
{
	return (x < 0.0f) ? -x : x;
}

/**
 * @brief  当前阶段是否超时
 * @param  ms 本阶段的时限(ms)
 * @author camelliahm
 */
static bool grip_Timeout(uint32_t ms)
{
	return ((xTaskGetTickCount() - grip.phaseTick) > pdMS_TO_TICKS(ms));
}

/**
 * @brief  序列失败：置故障位并中止
 * @note   中止后姿态保持在上一步下发的位置，便于人工观察停在哪一阶段
 * @author camelliahm
 */
static void grip_Fail(void)
{
	grip.fault = 1u;
	grip_Abort();
}

/**
 * @brief  让升降轴到指定高度
 * @param  mm 目标高度(mm)
 * @note   被拒绝（未使能 / 状态不符）即判本序列失败
 * @author camelliahm
 */
static void grip_LiftTo(float mm)
{
	if (lift_MoveToMM(mm, GRIP_LIFT_VEL, GRIP_LIFT_ACC) == false)
	{
		grip_Fail();
	}
}

/**
 * @brief  切入下一阶段并执行该阶段的动作
 * @param  p 目标阶段
 * @note   升降只在阶段入口下发一次，靠 lift_IsMoving() 判到位；
 *         旋转只改目标角，继续交给 rot_Step() 推进
 * @author camelliahm
 */
static void grip_Enter(GripPhase_t p)
{
	grip.phase     = (uint8_t)p;
	grip.phaseTick = xTaskGetTickCount();

	switch (p)
	{
	case GRIP_ROT_PICK:    grip_RotTo(GRIP_ROT_PICK_DEG);  break;
	case GRIP_LOWER_PICK:  grip_LiftTo(grip.pickH);        break;
	case GRIP_JAW_CLOSE:   grip_SetJaw(true);              break;
	case GRIP_LIFT_UP:     grip_LiftTo(GRIP_H_SAFE_MM);    break;
	case GRIP_ROT_PLACE:	//与夹持轴同时起步，两轴并行
		grip_RotTo(GRIP_ROT_PLACE_DEG);
		grip_TrayTo(grip.traySlot);
		break;
	case GRIP_LOWER_PLACE: grip_LiftTo(GRIP_H_PLACE_MM);   break;
	case GRIP_JAW_OPEN:    grip_SetJaw(false);             break;
	case GRIP_LIFT_CLEAR:  grip_LiftTo(GRIP_H_SAFE_MM);    break;
	case GRIP_ROT_SAFE:    grip_RotTo(GRIP_ROT_SAFE_DEG);  break;
	case GRIP_TRAY_HOME:   grip_TrayHome();                break;

	/*---- 卸载序列 ----*/
	case GRIP_U_ROT_TRAY:	//与物料盘同时起步，两轴并行
		grip_RotTo(GRIP_ROT_U_PICK_DEG);
		grip_TrayTo(grip.unloadSlot);
		break;
	case GRIP_U_LOWER:      grip_LiftTo(GRIP_H_U_PICK_MM);   break;
	case GRIP_U_JAW_CLOSE:  grip_SetJaw(true);               break;
	case GRIP_U_LIFT_UP:    grip_LiftTo(GRIP_H_SAFE_MM);     break;
	case GRIP_U_ROT_DROP:   grip_RotTo(GRIP_ROT_U_DROP_DEG); break;
	case GRIP_U_LOWER_DROP: grip_LiftTo(grip.dropH);        break;
	case GRIP_U_JAW_OPEN:   grip_SetJaw(false);              break;
	case GRIP_U_LIFT_CLEAR: grip_LiftTo(GRIP_H_SAFE_MM);     break;
	case GRIP_U_ROT_SAFE:   grip_RotTo(GRIP_ROT_SAFE_DEG);   break;

	default:
		break;
	}
}

/*---- 序列推进表：每相位登记"完成判据 + 下一相 + 超时时限" ----*/
/* 判据按类型复用（升降停 / 夹子等时各复用多次），真正去重；带状态的分支在 grip_SeqStep 内特判 */

/**
 * @brief  旋转轴是否已到目标角
 * @param  deg 目标角(°)
 * @note   到位判据 = 当前角与目标角之差在 GRIP_ROT_TOL_DEG 内
 * @author camelliahm
 */
static bool rot_Arrived(float deg)
{
	return (grip_Abs(grip.rotNow - deg) <= GRIP_ROT_TOL_DEG);
}

/**
 * @brief  物料盘是否已到本轮目标槽位角
 * @note   目标角取 grip.trayTarget（由 grip_TrayTo / grip_TrayHome 设定）
 * @author camelliahm
 */
static bool tray_Arrived(void)
{
	return (grip_Abs(grip.trayNow - grip.trayTarget) <= GRIP_TRAY_TOL_DEG);
}

/**
 * @brief  升降轴是否已停止
 * @note   到位或故障都会停，两者靠超时兜底区分
 * @author camelliahm
 */
static bool lift_Done(void)
{
	return (lift_IsMoving() == false);
}

//各相位的完成判据：返回 true = 本相位动作已完成，可进下一相
static bool judge_RotPick(void)   { return rot_Arrived(GRIP_ROT_PICK_DEG); }
static bool judge_LiftDone(void)  { return lift_Done(); }
static bool judge_Settle(void)    { return grip_Timeout(GRIP_JAW_SETTLE_MS); }
static bool judge_RotPlace(void)  { return rot_Arrived(GRIP_ROT_PLACE_DEG) && tray_Arrived(); }
static bool judge_RotSafe(void)   { return rot_Arrived(GRIP_ROT_SAFE_DEG); }
static bool judge_TrayHome(void)  { return (grip_Abs(grip.trayNow - GRIP_TRAY_POS1_DEG) <= GRIP_TRAY_TOL_DEG); }
static bool judge_U_RotTray(void) { return rot_Arrived(GRIP_ROT_U_PICK_DEG) && tray_Arrived(); }
static bool judge_U_RotDrop(void) { return rot_Arrived(GRIP_ROT_U_DROP_DEG); }

//序列推进表项：完成判据 + 下一位相 + 超时时限
typedef struct {
	bool     (*done)(void);	//完成判据
	uint8_t  next;			//下一位相 GripPhase_t
	uint32_t timeout;		//超时时限(ms)
} GripStep_t;

static const GripStep_t s_steps[GRIP_PHASE_NUM] = {	//序列推进表
	/*---- 装载序列 ----*/
	[GRIP_ROT_PICK]    = { judge_RotPick,  GRIP_LOWER_PICK,  GRIP_ROT_TIMEOUT_MS  },
	[GRIP_LOWER_PICK]  = { judge_LiftDone, GRIP_JAW_CLOSE,   GRIP_MOVE_TIMEOUT_MS },
	[GRIP_JAW_CLOSE]   = { judge_Settle,   GRIP_LIFT_UP,     GRIP_JAW_SETTLE_MS   },
	[GRIP_LIFT_UP]     = { judge_LiftDone, GRIP_ROT_PLACE,   GRIP_MOVE_TIMEOUT_MS },
	[GRIP_ROT_PLACE]   = { judge_RotPlace, GRIP_LOWER_PLACE, GRIP_ROT_TIMEOUT_MS  },
	[GRIP_LOWER_PLACE] = { judge_LiftDone, GRIP_JAW_OPEN,    GRIP_MOVE_TIMEOUT_MS },
	[GRIP_JAW_OPEN]    = { judge_Settle,   GRIP_LIFT_CLEAR,  GRIP_JAW_SETTLE_MS   },
	[GRIP_LIFT_CLEAR]  = { judge_LiftDone, GRIP_ROT_SAFE,    GRIP_MOVE_TIMEOUT_MS },
	[GRIP_ROT_SAFE]    = { judge_RotSafe,  GRIP_TRAY_HOME,   GRIP_ROT_TIMEOUT_MS  },
	[GRIP_TRAY_HOME]   = { judge_TrayHome, GRIP_IDLE,        GRIP_ROT_TIMEOUT_MS  },

	/*---- 卸载序列 ----*/
	[GRIP_U_ROT_TRAY]    = { judge_U_RotTray, GRIP_U_LOWER,     GRIP_ROT_TIMEOUT_MS  },
	[GRIP_U_LOWER]       = { judge_LiftDone,  GRIP_U_JAW_CLOSE, GRIP_MOVE_TIMEOUT_MS },
	[GRIP_U_JAW_CLOSE]   = { judge_Settle,    GRIP_U_LIFT_UP,   GRIP_JAW_SETTLE_MS   },
	[GRIP_U_LIFT_UP]     = { judge_LiftDone,  GRIP_U_ROT_DROP,  GRIP_MOVE_TIMEOUT_MS },
	[GRIP_U_ROT_DROP]    = { judge_U_RotDrop, GRIP_U_LOWER_DROP,GRIP_ROT_TIMEOUT_MS  },
	[GRIP_U_LOWER_DROP]  = { judge_LiftDone,  GRIP_U_JAW_OPEN,  GRIP_MOVE_TIMEOUT_MS },
	[GRIP_U_JAW_OPEN]    = { judge_Settle,    GRIP_U_LIFT_CLEAR,GRIP_JAW_SETTLE_MS   },
	[GRIP_U_LIFT_CLEAR]  = { judge_LiftDone,  GRIP_U_ROT_SAFE,  GRIP_MOVE_TIMEOUT_MS },
	[GRIP_U_ROT_SAFE]    = { judge_RotSafe,   GRIP_IDLE,        GRIP_ROT_TIMEOUT_MS  },
};

/**
 * @brief  旋转轴斜坡：每个控制周期向目标角逼近一步
 * @note   无条件推进（安全姿态也靠它）；已到位就不再写舵机，避免无谓的比较值更新
 * @author camelliahm
 */
static void rot_Step(void)
{
	float d = grip.rotTarget - grip.rotNow;

	if (d > GRIP_ROT_SLEW_DEG)
	{
		d = GRIP_ROT_SLEW_DEG;
	}
	else if (d < -GRIP_ROT_SLEW_DEG)
	{
		d = -GRIP_ROT_SLEW_DEG;
	}

	if (d != 0.0f)
	{
		grip.rotNow += d;
		(void)servo_SetAngle(GRIP_ROT_CH, grip.rotNow);
	}
}

/**
 * @brief  物料盘斜坡：每个控制周期向目标角逼近一步
 * @note   与 rot_Step() 同构，两者在同一个控制周期里各自推进一格 ——
 *         "夹持轴转 180° 的同时物料盘转到本轮槽位"就是靠这一层实现并行，而非时间配平；
 *         已到位就不再写舵机，避免无谓的比较值更新
 * @author camelliahm
 */
static void tray_Step(void)
{
	float d = grip.trayTarget - grip.trayNow;

	if (d > GRIP_TRAY_SLEW_DEG)
	{
		d = GRIP_TRAY_SLEW_DEG;
	}
	else if (d < -GRIP_TRAY_SLEW_DEG)
	{
		d = -GRIP_TRAY_SLEW_DEG;
	}

	if (d != 0.0f)
	{
		grip.trayNow += d;
		(void)servo_SetAngle(GRIP_TRAY_CH, grip.trayNow);
	}
}

/**
 * @brief  初始化夹持机构：上电姿态 = 安全位 + 夹子张开
 * @note   上电时以当前电平为基准先给出姿态，避免上电瞬间舵机乱甩
 * @author camelliahm
 */
void grip_Init(void)
{
	grip.phase     = (uint8_t)GRIP_IDLE;
	grip.busy      = 0u;
	grip.fault     = 0u;
	grip.loadCount = 0u;
	grip.unloadSlot = 1u;
	grip.traySlot   = 1u;
	grip.seqLoad    = 0u;
	grip.pickH      = GRIP_H_PICK_MM;
	grip.dropH      = GRIP_H_U_DROP_MM;
	grip.rotTarget  = GRIP_ROT_SAFE_DEG;
	grip.rotNow     = GRIP_ROT_SAFE_DEG;
	grip.trayTarget = GRIP_TRAY_POS1_DEG;
	grip.trayNow    = GRIP_TRAY_POS1_DEG;
	grip.phaseTick  = xTaskGetTickCount();

	(void)servo_SetAngle(GRIP_ROT_CH, GRIP_ROT_SAFE_DEG);
	(void)servo_SetAngle(GRIP_JAW_CH, GRIP_JAW_OPEN_DEG);
	(void)servo_SetAngle(GRIP_TRAY_CH, GRIP_TRAY_POS1_DEG);
}

/**
 * @brief  推进一轮动作序列（装载 / 卸载共用）
 * @note   仅在 busy 时动作：升降阶段等 lift_IsMoving() 归零，旋转阶段等角度到位
 *         （落盘阶段还要等物料盘转到本轮槽位），都有超时兜底；
 *         一轮装完 loadCount 自增，装满 GRIP_LOAD_TOTAL 轮则把物料盘转回位置一
 * @author camelliahm
 */
static void grip_SeqStep(void)
{
	if (grip.busy == 0u)
	{
		return;
	}

	GripPhase_t ph = (GripPhase_t)grip.phase;

	/* 完成即结束的相位：到位后交还控制权，而非进入下一相 */
	if ((ph == GRIP_TRAY_HOME) || (ph == GRIP_U_ROT_SAFE))
	{
		if (s_steps[ph].done()) { grip_Abort(); }
		else if (grip_Timeout(s_steps[ph].timeout)) { grip_Fail(); }
		return;
	}

	/* 旋转回安全位有"装完计数 / 归位"的分支，不是线性"到位→下一相"，单独处理 */
	if (ph == GRIP_ROT_SAFE)
	{
		if (s_steps[ph].done())
		{
			if (grip.seqLoad == 0u)
			{
				grip_Abort();		//取回是一次性的，不计轮次
				return;
			}

			if (grip.loadCount < GRIP_LOAD_TOTAL)
			{
				grip.loadCount++;
			}

			if (grip.loadCount >= GRIP_LOAD_TOTAL)
			{
				grip_Enter(GRIP_TRAY_HOME);		//本批装满：物料盘归位，等换批 / 卸料
			}
			else
			{
				grip_Abort();
			}
		}
		else if (grip_Timeout(s_steps[ph].timeout))
		{
			grip_Fail();
		}
		return;
	}

	/* 线性推进的相位：唯一一份 if/else */
	if (s_steps[ph].done()) { grip_Enter((GripPhase_t)s_steps[ph].next); }
	else if (grip_Timeout(s_steps[ph].timeout)) { grip_Fail(); }
}

/**
 * @brief  启动一轮装载（取料 → 转 180° → 落到物料盘 → 回安全位）
 * @retval true = 已启动；false = 前置条件不满足
 * @note   车须已由编排层对准并停稳，本函数只管机构动作，不含底盘移动；
 *         三轮循环由调用方（编排层）在每轮结束后重新调用
 * @author camelliahm
 */
bool grip_LoadOnce(void)
{
	if ((chassis.state != CHS_READY) && (chassis.state != CHS_RUNNING))
	{
		return false;
	}
	if (grip.busy != 0u)
	{
		return false;
	}
	if (lift.homed == 0u)
	{
		return false;		//未回零：高度基准不可信，拒绝动作
	}

	grip.fault    = 0u;
	grip.busy     = 1u;
	grip.seqLoad  = 1u;
	grip.pickH    = GRIP_H_PICK_MM;
	grip.traySlot = (uint8_t)(grip.loadCount + 1u);
	grip_Enter(GRIP_ROT_PICK);
	return true;
}

/**
 * @brief  启动一次车外放置序列（卸料 / 码垛共用），只差放置高度
 * @param  slot  待取槽位(1~GRIP_TRAY_POS_NUM)
 * @param  dropH 车外放置高度(mm)：平放 = GRIP_H_U_DROP_MM，码垛 = GRIP_H_STACK_MM
 * @retval true = 已启动；false = 前置条件不满足
 * @note   与 grip_LoadOnce() 反向：车须已由编排层停在目标区前，本函数只管一次取放；
 *         slot 越界（0 或超过槽位数）直接拒绝，调用方据此判故障
 * @author camelliahm
 */
static bool grip_UnloadStart(uint8_t slot, float dropH)
{
	if ((chassis.state != CHS_READY) && (chassis.state != CHS_RUNNING))
	{
		return false;
	}
	if (grip.busy != 0u)
	{
		return false;
	}
	if (lift.homed == 0u)
	{
		return false;		//未回零：高度基准不可信，拒绝动作
	}
	if ((slot < 1u) || (slot > (uint8_t)GRIP_TRAY_POS_NUM))
	{
		return false;		//槽位越界：编排出错，不猜
	}

	grip.fault      = 0u;
	grip.busy       = 1u;
	grip.unloadSlot = slot;
	grip.dropH      = dropH;
	grip_Enter(GRIP_U_ROT_TRAY);
	return true;
}

/**
 * @brief  启动一次卸载（从物料盘第 slot 槽取料 → 转到车外位 → 放到台面 → 回安全位）
 * @param  slot 待取槽位(1~GRIP_TRAY_POS_NUM)
 * @retval true = 已启动；false = 前置条件不满足
 * @author camelliahm
 */
bool grip_UnloadOnce(uint8_t slot)
{
	return grip_UnloadStart(slot, GRIP_H_U_DROP_MM);
}

/**
 * @brief  启动一次码垛卸载（放置高度抬一个料高，叠到已有的物料上）
 * @param  slot 待取槽位(1~GRIP_TRAY_POS_NUM)
 * @retval true = 已启动；false = 前置条件不满足
 * @note   仅放置高度与 grip_UnloadOnce() 不同，其余（取料 / 旋转 / 松开）完全一致
 * @author camelliahm
 */
bool grip_StackOnce(uint8_t slot)
{
	return grip_UnloadStart(slot, GRIP_H_STACK_MM);
}

/**
 * @brief  启动一次取回（车外目标区取料 → 转 180° 装回物料盘第 slot 槽 → 回安全位）
 * @param  slot 装回槽位(1~GRIP_TRAY_POS_NUM)，与卸料时的槽位一一对应
 * @retval true = 已启动；false = 前置条件不满足
 * @note   与 grip_LoadOnce() 同向（车外抓取 → 车内落盘），只是抓取高度取圆环高度、
 *         且完成时不计装载轮次；车须已由编排层停在目标区前
 * @author camelliahm
 */
bool grip_ReloadOnce(uint8_t slot)
{
	if ((chassis.state != CHS_READY) && (chassis.state != CHS_RUNNING))
	{
		return false;
	}
	if (grip.busy != 0u)
	{
		return false;
	}
	if (lift.homed == 0u)
	{
		return false;		//未回零：高度基准不可信，拒绝动作
	}
	if ((slot < 1u) || (slot > (uint8_t)GRIP_TRAY_POS_NUM))
	{
		return false;		//槽位越界：编排出错，不猜
	}

	grip.fault    = 0u;
	grip.busy     = 1u;
	grip.seqLoad  = 0u;
	grip.pickH    = GRIP_H_R_PICK_MM;
	grip.traySlot = slot;
	grip_Enter(GRIP_ROT_PICK);
	return true;
}

/**
 * @brief  动作序列是否进行中
 * @author camelliahm
 */
bool grip_IsBusy(void)
{
	return (grip.busy != 0u);
}

/**
 * @brief  三轮装载是否全部完成
 * @author camelliahm
 */
bool grip_AllLoaded(void)
{
	return (grip.loadCount >= GRIP_LOAD_TOTAL);
}

/**
 * @brief  夹持机构周期任务：推进斜坡与动作序列
 * @note   两个斜坡无条件推进（安全姿态 / 归位都靠它们）；序列在 busy = 0 时直接返回
 * @author camelliahm
 */
void grip_Step(void)
{
	rot_Step();		//旋转轴斜坡：无条件推进，安全姿态也靠它
	tray_Step();	//物料盘斜坡：同样无条件推进，归位 / 换批也靠它
	grip_SeqStep();	//装载 / 卸载动作序列
}
