#include "auxiliary.h"
#include "Emm_V5.h"
#include "servo.h"
#include "FreeRTOS.h"
#include "task.h"

//静态存储期变量由启动代码清零；初值在 aux_Init() 中显式赋值
__IO Lift_t lift;	//升降轴状态

//升降轴到位判据的持续计时（仅控制周期任务访问）
static uint32_t s_liftHoldMs = 0;	//到位判据的持续计时(ms)

/*---- 升降轴（丝杆步进，CAN 地址 0x05）----*/

/**
 * @brief  下发一条升降轴位置指令（不检查状态，调用方负责状态校验）
 * @param  clk 目标位置(脉冲，有符号，驱动器坐标系)
 * @param  vel 速度(RPM)
 * @param  acc 加速度(0-255，0 = 直接启动)
 * @note   方向由目标与当前位置比较得出，故用绝对值运动(raF=true)，
 *         否则 Emm_V5 会按 dir 单向走而无法正负双向定位
 * @author camelliahm
 */
static void lift_SendMove(int32_t clk, uint16_t vel, uint8_t acc)
{
	uint8_t dir;

	can_TxLock();
	dir = (clk >= lift.pos) ? 0u : 1u;	//绝对值运动(raF=true)，方向由目标与当前位置比较得出
	Emm_V5_Pos_Control(CHASSIS_ADDR_LIFT, dir, vel, acc, (uint32_t)clk, true, false);
	can_TxUnlock();

	lift.target  = clk;
	lift.moving  = 1;
	s_liftHoldMs = 0;
}

/**
 * @brief  升降轴使能 / 失能
 * @param  en true-使能 false-失能
 * @note   断使能会撤掉保持电流，负载全靠 T8 梯形丝杆自锁停住；
 *         若换成不自锁的丝杆，急停须改为"保持使能 + Stop_Now"并加抱闸
 * @author camelliahm
 */
void lift_SetEnable(bool en)
{
	if (en)
	{
		lift.online = 0;	//使能后要重新等第一帧反馈
	}
	else
	{
		lift.moving  = 0;
		lift.online  = 0;
		s_liftHoldMs = 0;
	}

	can_TxLock();
	Emm_V5_En_Control(CHASSIS_ADDR_LIFT, en, false);
	can_TxUnlock();
}

/**
 * @brief  打开升降轴的定时位置上报
 * @note   上报 S_CPOS（实时位置），是判断"是否到位"的唯一依据
 * @author camelliahm
 */
void lift_EnableFeedback(void)
{
	lift.lastTick = xTaskGetTickCount();

	can_TxLock();
	Emm_V5_Auto_Return_Sys_Params_Timed(CHASSIS_ADDR_LIFT, S_CPOS, LIFT_FB_PERIOD_MS);
	can_TxUnlock();
}

/**
 * @brief  清在线标志
 * @author camelliahm
 */
void lift_ClearOnline(void)
{
	lift.online = 0;
}

bool lift_MoveTo(int32_t clk, uint8_t dir, uint16_t vel, uint8_t acc)
{
	(void)dir;	//方向由 target 与 pos 决定，dir 参数保留是为了兼容帧格式

	if ((chassis.state != CHS_READY) && (chassis.state != CHS_RUNNING))
	{
		return false;
	}

	lift_SendMove(clk, vel, acc);
	return true;
}

bool lift_MoveBy(int32_t delta, uint16_t vel, uint8_t acc)
{
	int32_t target;

	if ((chassis.state != CHS_READY) && (chassis.state != CHS_RUNNING))
	{
		return false;
	}

	target = lift.pos + delta;

	lift_SendMove(target, vel, acc);
	return true;
}

/**
 * @brief  把目标高度限制在机械行程内
 * @param  mm 期望高度(mm)
 * @retval 夹到 [0, LIFT_STROKE_MM] 之后的高度
 * @note   只在 homed = 1（已回零或 SetZero）后有意义；未回零时零点不在行程最低点
 * @author camelliahm
 */
static float lift_ClampMM(float mm)
{
	if (mm < 0.0f)
	{
		return 0.0f;
	}
	if (mm > LIFT_STROKE_MM)
	{
		return LIFT_STROKE_MM;
	}
	return mm;
}

/**
 * @brief  升降到绝对高度（mm）—— 控制升降的主接口
 * @param  mm  目标高度(mm)：0 = 原点，向上为正
 * @param  vel 速度(RPM)，按 mm/s 给时用 LIFT_MMPS_TO_RPM() 换算
 * @param  acc 加速度(0-255，0 = 直接启动)
 * @retval true-已下发 false-当前状态不接受（未使能 / FAULT / ESTOP）
 * @note   目标先经 lift_ClampMM() 夹到行程内，撞不到机械极限
 * @author camelliahm
 */
bool lift_MoveToMM(float mm, uint16_t vel, uint8_t acc)
{
	mm = lift_ClampMM(mm);

	return lift_MoveTo(LIFT_MM_TO_PULSE(mm), 0u, vel, acc);
}

/**
 * @brief  相对当前高度升降（mm）
 * @param  deltaMM 位移量(mm)：正 = 上升，负 = 下降
 * @param  vel 速度(RPM)
 * @param  acc 加速度(0-255，0 = 直接启动)
 * @retval true-已下发 false-当前状态不接受
 * @note   先算绝对目标再夹行程，故"已到顶再升"会停在上限而非发出越界指令
 * @author camelliahm
 */
bool lift_MoveByMM(float deltaMM, uint16_t vel, uint8_t acc)
{
	return lift_MoveToMM(lift_GetHeightMM() + deltaMM, vel, acc);
}

/**
 * @brief  读当前高度（mm）
 * @retval 由 S_CPOS 反馈脉冲换算的高度
 * @note   未回零时是相对多圈计数，不是绝对行程高度
 * @author camelliahm
 */
float lift_GetHeightMM(void)
{
	return LIFT_PULSE_TO_MM(lift.pos);
}

/**
 * @brief  触发回零（模式 LIFT_HOME_MODE = 3，多圈有限位开关回零）
 * @note   异步：完成由 S_OFLAG(0x3B) 反馈，置 homed=1。
 *         限位开关必须已接好，否则会一直撞到底（有 15s 超时兜底）
 * @author camelliahm
 */
void lift_Home(void)
{
	lift.homed   = 0;
	lift.homing  = 1;
	lift.moving  = 0;
	s_liftHoldMs = 0;

	can_TxLock();
	Emm_V5_Origin_Trigger_Return(CHASSIS_ADDR_LIFT, LIFT_HOME_MODE, false);
	can_TxUnlock();
}

/**
 * @brief  立即停止升降轴（不影响底盘）
 * @note   Stop_Now 是保持使能的减速停止，不撤扭矩，对丝杆负载安全；
 *         真正要断电的急停走 lift_SetEnable(false)
 * @author camelliahm
 */
void lift_StopNow(void)
{
	lift.moving  = 0;
	s_liftHoldMs = 0;

	can_TxLock();
	Emm_V5_Stop_Now(CHASSIS_ADDR_LIFT, false);
	can_TxUnlock();
}

/**
 * @brief  将升降轴当前位置清零
 * @note   软件标定原点（非回零）：必须在真实零位且已卸负载时调用，
 *         否则整段行程的基准都是错的；调用后置 homed=1
 * @author camelliahm
 */
void lift_SetZero(void)
{
	can_TxLock();
	Emm_V5_Reset_CurPos_To_Zero(CHASSIS_ADDR_LIFT);
	can_TxUnlock();

	lift.pos     = 0;
	lift.target  = 0;
	lift.moving  = 0;
	lift.homed   = 1;	//人工建立了坐标基准
	lift.homing  = 0;
	s_liftHoldMs = 0;
}

void lift_ClearClog(void)
{
	can_TxLock();
	Emm_V5_Reset_Clog_Pro(CHASSIS_ADDR_LIFT);
	can_TxUnlock();
}

bool lift_IsMoving(void)
{
	return (lift.moving != 0);
}

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
	case GRIP_LOWER_PICK:  grip_LiftTo(GRIP_H_PICK_MM);    break;
	case GRIP_JAW_CLOSE:   grip_SetJaw(true);              break;
	case GRIP_LIFT_UP:     grip_LiftTo(GRIP_H_SAFE_MM);    break;
	case GRIP_ROT_PLACE:	//与夹持轴同时起步，两轴并行：本轮槽位 = 已完成轮数 + 1
		grip_RotTo(GRIP_ROT_PLACE_DEG);
		grip_TrayTo((uint8_t)(grip.loadCount + 1u));
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
	case GRIP_U_LOWER_DROP: grip_LiftTo(GRIP_H_U_DROP_MM);   break;
	case GRIP_U_JAW_OPEN:   grip_SetJaw(false);              break;
	case GRIP_U_LIFT_CLEAR: grip_LiftTo(GRIP_H_SAFE_MM);     break;
	case GRIP_U_ROT_SAFE:   grip_RotTo(GRIP_ROT_SAFE_DEG);   break;

	default:
		break;
	}
}

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
static void grip_Init(void)
{
	grip.phase     = (uint8_t)GRIP_IDLE;
	grip.busy      = 0u;
	grip.fault     = 0u;
	grip.loadCount = 0u;
	grip.unloadSlot = 1u;
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
 * @brief  夹持机构周期任务：推进一轮装载动作序列
 * @note   仅在 busy 时动作：升降阶段等 lift_IsMoving() 归零，旋转阶段等角度到位
 *         （落盘阶段还要等物料盘转到本轮槽位），都有超时兜底；
 *         一轮装完 loadCount 自增，装满 GRIP_LOAD_TOTAL 轮则把物料盘转回位置一
 * @author camelliahm
 */
void grip_Step(void)
{
	if (grip.busy == 0u)
	{
		return;
	}

	switch ((GripPhase_t)grip.phase)
	{
	case GRIP_ROT_PICK:
		if (grip_Abs(grip.rotNow - GRIP_ROT_PICK_DEG) <= GRIP_ROT_TOL_DEG)
		{
			grip_Enter(GRIP_LOWER_PICK);
		}
		else if (grip_Timeout(GRIP_ROT_TIMEOUT_MS))
		{
			grip_Fail();
		}
		break;

	case GRIP_LOWER_PICK:
		if (lift_IsMoving() == false)
		{
			grip_Enter(GRIP_JAW_CLOSE);
		}
		else if (grip_Timeout(GRIP_MOVE_TIMEOUT_MS))
		{
			grip_Fail();
		}
		break;

	case GRIP_JAW_CLOSE:
		if (grip_Timeout(GRIP_JAW_SETTLE_MS))
		{
			grip_Enter(GRIP_LIFT_UP);
		}
		break;

	case GRIP_LIFT_UP:
		if (lift_IsMoving() == false)
		{
			grip_Enter(GRIP_ROT_PLACE);
		}
		else if (grip_Timeout(GRIP_MOVE_TIMEOUT_MS))
		{
			grip_Fail();
		}
		break;

	case GRIP_ROT_PLACE:
		//双轴到位才算完：夹持轴到落盘位，且物料盘已把本轮槽位转到落料点
		if ((grip_Abs(grip.rotNow - GRIP_ROT_PLACE_DEG) <= GRIP_ROT_TOL_DEG) &&
			(grip_Abs(grip.trayNow - grip.trayTarget) <= GRIP_TRAY_TOL_DEG))
		{
			grip_Enter(GRIP_LOWER_PLACE);
		}
		else if (grip_Timeout(GRIP_ROT_TIMEOUT_MS))
		{
			grip_Fail();
		}
		break;

	case GRIP_LOWER_PLACE:
		if (lift_IsMoving() == false)
		{
			grip_Enter(GRIP_JAW_OPEN);
		}
		else if (grip_Timeout(GRIP_MOVE_TIMEOUT_MS))
		{
			grip_Fail();
		}
		break;

	case GRIP_JAW_OPEN:
		if (grip_Timeout(GRIP_JAW_SETTLE_MS))
		{
			grip_Enter(GRIP_LIFT_CLEAR);
		}
		break;

	case GRIP_LIFT_CLEAR:
		if (lift_IsMoving() == false)
		{
			grip_Enter(GRIP_ROT_SAFE);
		}
		else if (grip_Timeout(GRIP_MOVE_TIMEOUT_MS))
		{
			grip_Fail();
		}
		break;

	case GRIP_ROT_SAFE:
		if (grip_Abs(grip.rotNow - GRIP_ROT_SAFE_DEG) <= GRIP_ROT_TOL_DEG)
		{
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
		else if (grip_Timeout(GRIP_ROT_TIMEOUT_MS))
		{
			grip_Fail();
		}
		break;

	case GRIP_TRAY_HOME:
		if (grip_Abs(grip.trayNow - GRIP_TRAY_POS1_DEG) <= GRIP_TRAY_TOL_DEG)
		{
			grip_Abort();
		}
		else if (grip_Timeout(GRIP_ROT_TIMEOUT_MS))
		{
			grip_Fail();
		}
		break;

	/*---- 卸载序列 ----*/
	case GRIP_U_ROT_TRAY:
		//双轴到位才算完：旋转轴到车内取料位，且物料盘已把待取槽位转到取料点
		if ((grip_Abs(grip.rotNow - GRIP_ROT_U_PICK_DEG) <= GRIP_ROT_TOL_DEG) &&
			(grip_Abs(grip.trayNow - grip.trayTarget) <= GRIP_TRAY_TOL_DEG))
		{
			grip_Enter(GRIP_U_LOWER);
		}
		else if (grip_Timeout(GRIP_ROT_TIMEOUT_MS))
		{
			grip_Fail();
		}
		break;

	case GRIP_U_LOWER:
		if (lift_IsMoving() == false)
		{
			grip_Enter(GRIP_U_JAW_CLOSE);
		}
		else if (grip_Timeout(GRIP_MOVE_TIMEOUT_MS))
		{
			grip_Fail();
		}
		break;

	case GRIP_U_JAW_CLOSE:
		if (grip_Timeout(GRIP_JAW_SETTLE_MS))
		{
			grip_Enter(GRIP_U_LIFT_UP);
		}
		break;

	case GRIP_U_LIFT_UP:
		if (lift_IsMoving() == false)
		{
			grip_Enter(GRIP_U_ROT_DROP);
		}
		else if (grip_Timeout(GRIP_MOVE_TIMEOUT_MS))
		{
			grip_Fail();
		}
		break;

	case GRIP_U_ROT_DROP:
		if (grip_Abs(grip.rotNow - GRIP_ROT_U_DROP_DEG) <= GRIP_ROT_TOL_DEG)
		{
			grip_Enter(GRIP_U_LOWER_DROP);
		}
		else if (grip_Timeout(GRIP_ROT_TIMEOUT_MS))
		{
			grip_Fail();
		}
		break;

	case GRIP_U_LOWER_DROP:
		if (lift_IsMoving() == false)
		{
			grip_Enter(GRIP_U_JAW_OPEN);
		}
		else if (grip_Timeout(GRIP_MOVE_TIMEOUT_MS))
		{
			grip_Fail();
		}
		break;

	case GRIP_U_JAW_OPEN:
		if (grip_Timeout(GRIP_JAW_SETTLE_MS))
		{
			grip_Enter(GRIP_U_LIFT_CLEAR);
		}
		break;

	case GRIP_U_LIFT_CLEAR:
		if (lift_IsMoving() == false)
		{
			grip_Enter(GRIP_U_ROT_SAFE);
		}
		else if (grip_Timeout(GRIP_MOVE_TIMEOUT_MS))
		{
			grip_Fail();
		}
		break;

	case GRIP_U_ROT_SAFE:
		if (grip_Abs(grip.rotNow - GRIP_ROT_SAFE_DEG) <= GRIP_ROT_TOL_DEG)
		{
			grip_Abort();	//一次卸载完成，交还控制权
		}
		else if (grip_Timeout(GRIP_ROT_TIMEOUT_MS))
		{
			grip_Fail();
		}
		break;

	default:
		grip_Abort();
		break;
	}
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

	grip.fault = 0u;
	grip.busy  = 1u;
	grip_Enter(GRIP_ROT_PICK);
	return true;
}

/**
 * @brief  启动一次卸载（从物料盘第 slot 槽取料 → 转到车外位 → 放到台面 → 回安全位）
 * @param  slot 待取槽位(1~GRIP_TRAY_POS_NUM)
 * @retval true = 已启动；false = 前置条件不满足
 * @note   与 grip_LoadOnce() 反向：车须已由编排层停在目标区前，本函数只管一次取放；
 *         slot 越界（0 或超过槽位数）直接拒绝，调用方据此判故障
 * @author camelliahm
 */
bool grip_UnloadOnce(uint8_t slot)
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
	grip_Enter(GRIP_U_ROT_TRAY);
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

/*---- 消息分发（只处理"受消息控制"的机构）----*/

#if CFG_HOST_CAN_ENABLED
/**
 * @brief  夹持机构的非安全指令是否被当前状态接受
 * @retval true = READY / RUNNING（已使能且不在故障）
 * @author camelliahm
 */
static bool aux_GripAllowed(void)
{
	return ((chassis.state == CHS_READY) || (chassis.state == CHS_RUNNING));
}
#endif

/**
 * @brief  尝试把一条消息按"辅助机构"处理
 * @param  cmd 消息码
 * @param  f   收到的帧
 * @retval true-已受理 false-未受理（交后续路由，最终计入 evtIgnored）
 * @note   升降轴与夹持机构的动作类指令只在 READY / RUNNING 下被接受
 *         （低风险动作除外：升降轴的停止 / 清堵转，夹持机构的 ABORT / SAFE），
 *         被拒绝时计入 evtIgnored
 * @author camelliahm
 */
bool aux_HandleCmd(ChassisCmd_t cmd, const CAN_Frame_t *f)
{
#if CFG_HOST_CAN_ENABLED
	uint16_t vel;
	uint8_t  acc;
	int32_t  delta;
	float    deltaMM;

	switch (cmd)
	{
	/*---- 升降轴指令 ----*/
	case CMD_LIFT_POS:
		/*   [1] 方向 0=上升/1=下降  [2..3] 位移量(脉冲, u16 大端)
		 *   [4..5] 速度(RPM)  [6] 加速度  [7] 保留
		 *   1600 脉冲/mm 下 uint16 单帧上限 ≈ 41 mm，长行程用 CMD_LIFT_MM
		 */
		if (f->DLC >= 9)
		{
			vel   = (uint16_t)(((uint16_t)f->Data[4] << 8) | (uint16_t)f->Data[5]);
			acc   = f->Data[6];
			delta = (int32_t)(((uint16_t)f->Data[2] << 8) | (uint16_t)f->Data[3]);

			if (f->Data[1] != 0u)
			{
				delta = -delta;		//Data[1]=1 → 下降（负方向）
			}

			if (lift_MoveBy(delta, vel, acc) == false)
			{
				chassis.evtIgnored++;
			}
		}
		break;

	case CMD_LIFT_MM:
		//帧格式同 CMD_LIFT_POS，位移量单位换成 0.1 mm（120 = 12.0 mm）
		if (f->DLC >= 9)
		{
			vel     = (uint16_t)(((uint16_t)f->Data[4] << 8) | (uint16_t)f->Data[5]);
			acc     = f->Data[6];
			deltaMM = (float)(((uint16_t)f->Data[2] << 8) | (uint16_t)f->Data[3]) * 0.1f;

			if (f->Data[1] != 0u)
			{
				deltaMM = -deltaMM;	//Data[1]=1 → 下降（负方向）
			}

			if (lift_MoveByMM(deltaMM, vel, acc) == false)
			{
				chassis.evtIgnored++;
			}
		}
		break;

	case CMD_LIFT_HOME:
		if ((chassis.state == CHS_READY) || (chassis.state == CHS_RUNNING))
		{
			lift_Home();
		}
		else
		{
			chassis.evtIgnored++;
		}
		break;

	case CMD_LIFT_STOP:
		//停止属于安全动作，任何已使能状态都接受
		lift_StopNow();
		break;

	case CMD_LIFT_SETZERO:
		if ((chassis.state == CHS_READY) || (chassis.state == CHS_RUNNING))
		{
			lift_SetZero();
		}
		else
		{
			chassis.evtIgnored++;
		}
		break;

	case CMD_LIFT_CLEARF:
		lift_ClearClog();
		break;

	/*---- 夹持机构指令 ----*/
	//安全动作（ABORT / SAFE）任何状态都受理；其余要求 aux_GripAllowed()
	case CMD_GRIP_LOAD:
		if ((aux_GripAllowed() == true) && (grip_LoadOnce() == true))
		{
			break;
		}
		chassis.evtIgnored++;
		break;

	case CMD_GRIP_ABORT:
		grip_Abort();
		break;

	case CMD_GRIP_SAFE:
		grip_Safe();
		break;

	case CMD_GRIP_OPEN:
		if (aux_GripAllowed()) { grip_SetJaw(false); } else { chassis.evtIgnored++; }
		break;

	case CMD_GRIP_CLOSE:
		if (aux_GripAllowed()) { grip_SetJaw(true); } else { chassis.evtIgnored++; }
		break;

	case CMD_GRIP_ROTPICK:
		if (aux_GripAllowed()) { grip_RotTo(GRIP_ROT_PICK_DEG); } else { chassis.evtIgnored++; }
		break;

	case CMD_GRIP_ROTPLACE:
		if (aux_GripAllowed()) { grip_RotTo(GRIP_ROT_PLACE_DEG); } else { chassis.evtIgnored++; }
		break;

	case CMD_GRIP_ROTSAFE:
		if (aux_GripAllowed()) { grip_RotTo(GRIP_ROT_SAFE_DEG); } else { chassis.evtIgnored++; }
		break;

	//后续由上位机 CAN 指令驱动的机构在这里加 case

	default:
		return false;	//不是本模块的指令
	}

	return true;
#else
	(void)cmd;
	(void)f;

	return false;	//无上位机：升降轴 / 夹持机构段无属主，动作只由编排层直接调用
#endif
}

/**
 * @brief  解析升降轴反馈帧
 * @param  f 收到的帧（地址已确认为 0x05）
 * @note   与四轮共用同一套信息功能码：0x35 S_VEL 转速 / 0x36 S_CPOS 位置 /
 *         0x3B S_OFLAG 回零标志（0 = 回零完成）；数据为大端有符号
 * @author camelliahm
 */
void aux_HandleLiftFeedback(const CAN_Frame_t *f)
{
	lift.lastTick = xTaskGetTickCount();
	lift.online   = 1;
	lift.rxCount++;

	switch (f->Data[0])
	{
	case 0x35:	//S_VEL 实时转速
		if (f->DLC >= 3)
		{
			lift.vel = (int16_t)(((uint16_t)f->Data[1] << 8) | (uint16_t)f->Data[2]);
		}
		break;

	case 0x36:	//S_CPOS 实时位置
		if (f->DLC >= 5)
		{
			lift.pos = (int32_t)(((uint32_t)f->Data[1] << 24) |
								 ((uint32_t)f->Data[2] << 16) |
								 ((uint32_t)f->Data[3] <<  8) |
								  (uint32_t)f->Data[4]);
		}
		break;

	case 0x3B:	//S_OFLAG 回零状态标志位：0 = 回零已完成
		if (f->DLC >= 2)
		{
			if (f->Data[1] == 0)
			{
				lift.homed  = 1;
				lift.homing = 0;
			}
		}
		break;

	default:
		break;
	}
}

/*---- 周期任务 ----*/

/**
 * @brief  升降轴反馈看门狗
 * @note   仅 READY / RUNNING 下判定；只在"在线 → 掉线"跳变沿投一次 EV_FAULT
 * @author camelliahm
 */
static void lift_Watchdog(void)
{
	uint32_t now = xTaskGetTickCount();

	if ((chassis.state != CHS_READY) && (chassis.state != CHS_RUNNING))
	{
		return;
	}

	if ((now - lift.lastTick) > pdMS_TO_TICKS(LIFT_FB_TIMEOUT_MS))
	{
		if (lift.online)
		{
			lift.online = 0;
			contract_PostEvent(EV_FAULT, 0x05u);
		}
	}
	else
	{
		lift.online = 1;
	}
}

/**
 * @brief  升降轴到位判断与回零超时保护
 * @note   ① 回零超时（限位失效会一直撞到底）→ 中断回零并报故障；
 *         ② 到位判定同时看位置与转速，只更新标志、不投递事件
 * @author camelliahm
 */
static void lift_CheckArrive(void)
{
	int32_t err;
	int16_t v;

	if (lift.homing)
	{
		if ((xTaskGetTickCount() - lift.lastTick) > pdMS_TO_TICKS(LIFT_HOME_TIMEOUT_MS))
		{
			lift.homing = 0;
			can_TxLock();
			Emm_V5_Origin_Interrupt(CHASSIS_ADDR_LIFT);
			can_TxUnlock();
			contract_PostEvent(EV_FAULT, 0x05u);
		}
		return;
	}

	if (!lift.moving)
	{
		s_liftHoldMs = 0;
		return;
	}

	err = lift.pos - lift.target;
	if (err < 0)
	{
		err = -err;
	}

	v = lift.vel;
	if (v < 0)
	{
		v = (int16_t)(-v);
	}

	if ((err > (int32_t)LIFT_ARRIVE_WINDOW) || (v > LIFT_ARRIVE_VEL_TH))
	{
		s_liftHoldMs = 0;
		return;
	}

	s_liftHoldMs += AUX_CTRL_PERIOD_MS;

	if (s_liftHoldMs >= LIFT_ARRIVE_HOLD_MS)
	{
		s_liftHoldMs = 0;
		lift.moving  = 0;
	}
}

/**
 * @brief  辅助机构周期任务：由控制环每 5ms 调用
 * @author camelliahm
 */
void aux_Step(void)
{
	/*---- 升降轴 ----*/
	lift_Watchdog();
	lift_CheckArrive();

	/*---- 夹持机构 ----*/
	rot_Step();		//旋转轴斜坡：无条件推进，安全姿态也靠它
	tray_Step();	//物料盘斜坡：同样无条件推进，归位 / 换批也靠它
	grip_Step();	//装载动作序列（busy = 0 时直接返回）
}

/**
 * @brief  初始化辅助机构（在 contract_Init() 中调用）
 * @author camelliahm
 */
void aux_Init(void)
{
	//homed 必须为 0：上电时驱动器的 pos 是它自己的多圈计数，未经回零标定
	lift.online   = 0;
	lift.homed    = 0;
	lift.homing   = 0;
	lift.moving   = 0;
	lift.pos      = 0;
	lift.vel      = 0;
	lift.target   = 0;
	lift.rxCount  = 0;
	lift.lastTick = xTaskGetTickCount();

	s_liftHoldMs = 0;

	grip_Init();
}
