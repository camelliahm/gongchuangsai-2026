#include "lift_axis.h"
#include "Emm_V5.h"
#include "FreeRTOS.h"
#include "task.h"

//静态存储期变量由启动代码清零；初值在 lift_Init() 中显式赋值
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

/**
 * @brief  解析升降轴反馈帧
 * @param  f 收到的帧（地址已确认为 0x05）
 * @note   与四轮共用同一套信息功能码：0x35 S_VEL 转速 / 0x36 S_CPOS 位置 /
 *         0x3B S_OFLAG 回零标志（0 = 回零完成）；数据为大端有符号
 * @author camelliahm
 */
void lift_HandleFeedback(const CAN_Frame_t *f)
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

	s_liftHoldMs += LIFT_CTRL_PERIOD_MS;

	if (s_liftHoldMs >= LIFT_ARRIVE_HOLD_MS)
	{
		s_liftHoldMs = 0;
		lift.moving  = 0;
	}
}

/**
 * @brief  升降轴周期任务：反馈看门狗 + 到位判断
 * @author camelliahm
 */
void lift_Step(void)
{
	lift_Watchdog();
	lift_CheckArrive();
}

/**
 * @brief  初始化升降轴
 * @note   homed 必须为 0：上电时驱动器的 pos 是它自己的多圈计数，未经回零标定
 * @author camelliahm
 */
void lift_Init(void)
{
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
}
