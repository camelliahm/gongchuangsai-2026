#include "lift_axis.h"
#include "Emm_V5.h"
#include "FreeRTOS.h"
#include "task.h"

//静态存储期变量由启动代码清零；初值在 lift_Init() 中显式赋值
__IO Lift_t lift;	//升降轴状态

//升降轴到位判据的持续计时（仅控制周期任务访问）
static uint32_t s_liftHoldMs = 0;	//到位判据的持续计时(ms)

//上电自动标零状态（延时到点把当前位置记为 0）
static struct {
	uint8_t  done;		//1 = 本周期的自动标零已完成
	uint32_t tick;		//记录的上电时刻(tick)
} s_bootZero;

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
 * @note   只在 homed = 1（已标零）后有意义；未标零时零点不在行程最低点
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
 * @note   未标零时是相对多圈计数，不是绝对行程高度
 * @author camelliahm
 */
float lift_GetHeightMM(void)
{
	return LIFT_PULSE_TO_MM(lift.pos);
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
 * @note   软件标定原点：必须在真实零位且已卸负载时调用，
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
	lift.homed   = 1;	//零点基准已建立
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
 * @note   与四轮共用同一套信息功能码：0x35 S_VEL 转速 / 0x36 S_CPOS 位置；
 *         数据为大端有符号
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

	default:
		break;
	}
}

/*---- 周期任务 ----*/

/**
 * @brief  升降轴反馈看门狗
 * @note   仅 READY / RUNNING 下判定；只在"在线 → 掉线"跳变沿置离线标志。
 *         不投 EV_FAULT：升降轴掉线不算终态，后续依赖它的取放航点由导航层
 *         跳过续走，最大限度跑完路线
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
		}
	}
	else
	{
		lift.online = 1;
	}
}

/**
 * @brief  升降轴到位判断
 * @note   到位判定同时看位置与转速，只更新标志、不投递事件
 * @author camelliahm
 */
static void lift_CheckArrive(void)
{
	int32_t err;
	int16_t v;

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
 * @brief  上电自动标零：延时 LIFT_BOOT_ZERO_MS 后把当前位置记为 0
 * @note   零点由上电时丝杆所处位置确定；延时是为了等驱动器上电稳定 ——
 *         清零命令是单向下发，发早了会被它丢掉，那样 homed 虽置 1 而实际基准并未建立
 * @author camelliahm
 */
static void lift_BootZero(void)
{
	if (s_bootZero.done != 0u)
	{
		return;
	}

	if ((xTaskGetTickCount() - s_bootZero.tick) < pdMS_TO_TICKS(LIFT_BOOT_ZERO_MS))
	{
		return;
	}

	s_bootZero.done = 1u;
	lift_SetZero();
}

/**
 * @brief  升降轴周期任务：上电标零 + 反馈看门狗 + 到位判断
 * @author camelliahm
 */
void lift_Step(void)
{
	lift_BootZero();
	lift_Watchdog();
	lift_CheckArrive();
}

/**
 * @brief  初始化升降轴
 * @note   homed 先置 0：上电时驱动器的 pos 是它自己的多圈计数，基准尚未建立；
 *         随后由 lift_BootZero() 延时自动标零，到点才置 homed=1
 * @author camelliahm
 */
void lift_Init(void)
{
	lift.online   = 0;
	lift.homed    = 0;
	lift.moving   = 0;
	lift.pos      = 0;
	lift.vel      = 0;
	lift.target   = 0;
	lift.rxCount  = 0;
	lift.lastTick = xTaskGetTickCount();

	s_liftHoldMs = 0;

	s_bootZero.done = 0u;
	s_bootZero.tick = xTaskGetTickCount();
}
