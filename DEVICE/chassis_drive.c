#include "chassis_drive.h"
#include "Emm_V5.h"
#include "PID_Mod.h"
#include "FreeRTOS.h"
#include "task.h"

//静态存储期变量由启动代码清零；初值在 contract_Init() 中显式赋值
__IO ChassisDrive_t chassisDrive;		//底盘里程计与运动学状态
__IO ChassisMotor_t chassisMotor[CHASSIS_MOTOR_NUM];	//四台电机的反馈槽位

/*---- 姿态环 PID 实例 ----*/
//四轮闭环在驱动器内部完成，MCU 只做底盘级姿态修正：yaw 误差叠加到 w
//⚠ 调度器启动前不调用 PID_Calculate()，避免 xTaskGetTickCount() 未就绪
static PID_TypeDef s_yawPid;			//航向姿态环 PID 实例

#define YAW_PID_KP			(20.0f)		//姿态环比例增益
#define YAW_PID_KI			(0.0f)		//姿态环积分增益（当前未启用）
#define YAW_PID_KD			(0.0f)		//姿态环微分增益（当前未启用）
#define YAW_PID_OUT_MAX		(200.0f)	//姿态修正量上限(0.1度/s 单位)

/*---- 内部换算 ----*/

/**
 * @brief  底盘线速度(mm/s) → 轮子转速(RPM)
 * @param  v_mm_s 轮子线速度(mm/s)
 * @retval 转速(RPM)，带符号
 * @note   周长 = π × 直径；转速 = 线速度 / 周长 × 60 × 减速比
 * @author camelliahm
 */
static int16_t chassis_VelToRpm(float v_mm_s)
{
	float circumference = 3.14159265f * CHASSIS_WHEEL_DIAMETER_MM;
	float rpm           = (v_mm_s / circumference) * 60.0f * CHASSIS_GEAR_RATIO;

	//限幅：防止逆解算出的极端值打爆电机
	if (rpm >  (float)CHASSIS_DRIVE_VEL_MAX_RPM) { rpm =  (float)CHASSIS_DRIVE_VEL_MAX_RPM; }
	if (rpm < -(float)CHASSIS_DRIVE_VEL_MAX_RPM) { rpm = -(float)CHASSIS_DRIVE_VEL_MAX_RPM; }

	return (int16_t)rpm;
}

/**
 * @brief  轮子转速(RPM) → 轮子线速度(mm/s)
 * @param  rpm 转速(RPM)，带符号
 * @retval 线速度(mm/s)
 * @author camelliahm
 */
static float chassis_RpmToVel(int16_t rpm)
{
	float circumference = 3.14159265f * CHASSIS_WHEEL_DIAMETER_MM;

	return ((float)rpm / CHASSIS_GEAR_RATIO) * circumference / 60.0f;
}

/*---- 四台电机的动作与上报 ----*/

/**
 * @brief  向四台驱动器广播一条"无参数"动作命令
 * @param  action 序号 0=断使能 1=使能 2=立即停止
 * @note   必须整段包在 can_TxLock/Unlock 内：Emm_V5_* 用的是
 *         文件级 static 缓冲，跨任务并发会互相覆盖
 * @author camelliahm
 */
void chassis_DriveBroadcast(uint8_t action)
{
	uint8_t i;
	uint8_t addr;

	can_TxLock();
	for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
	{
		addr = (uint8_t)(CHASSIS_ADDR_MOTOR_BASE + i);

		if (action == 2)
		{
			Emm_V5_Stop_Now(addr, false);
		}
		else
		{
			Emm_V5_En_Control(addr, (action == 1) ? true : false, false);
		}
	}
	can_TxUnlock();
}

/**
 * @brief  向四台驱动器下发零速（软停），由驱动器按 acc 斜率自行减速
 * @note   Emm_V5_Vel_Control 的 vel=0 是合法命令，效果是"目标速度 0"而非急停；
 *         与 chassis_DriveBroadcast(2)（Stop_Now 立即停止）区分。
 *         BRAKING 期间须由控制环逐周期调用：只发一次会被同一周期内
 *         已进入 chassis_DriveStep() 的残留速度指令覆盖
 * @author camelliahm
 */
void chassis_BrakeWheels(void)
{
	uint8_t i;
	uint8_t addr;

	can_TxLock();
	for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
	{
		addr = (uint8_t)(CHASSIS_ADDR_MOTOR_BASE + i);
		Emm_V5_Vel_Control(addr, 0, 0, CHASSIS_DRIVE_ACC_DEFAULT, false);
	}
	can_TxUnlock();
}

/**
 * @brief  打开四台驱动器的定时上报（掉线检测与停稳判断的依据）
 * @note   上报周期 CHASSIS_FB_PERIOD_MS，4 台合计约 200 帧/s，
 *         在 500 kbit/s 总线上约占 5% 带宽
 * @author camelliahm
 */
void chassis_EnableFeedback(void)
{
	uint8_t i;

	can_TxLock();
	for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
	{
		Emm_V5_Auto_Return_Sys_Params_Timed((uint8_t)(CHASSIS_ADDR_MOTOR_BASE + i),
											S_VEL,
											CHASSIS_FB_PERIOD_MS);
	}
	can_TxUnlock();
}

/**
 * @brief  把所有电机心跳重置为"现在"并清在线标志
 * @note   进入 READY 时调用：给每台电机一个完整的超时窗口，
 *         避免刚使能就被看门狗误判掉线
 * @author camelliahm
 */
void chassis_ResetHeartbeat(void)
{
	uint32_t now = xTaskGetTickCount();
	uint8_t  i;

	for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
	{
		chassisMotor[i].online   = 0;
		chassisMotor[i].lastTick = now;
	}
}

/**
 * @brief  清全部在线标志
 * @note   进入 IDLE（未使能）时调用：此时不该期待任何反馈
 * @author camelliahm
 */
void chassis_ClearOnline(void)
{
	uint8_t i;

	for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
	{
		chassisMotor[i].online = 0;
	}
}

/*---- 反馈处理 ----*/

/**
 * @brief  解析电机反馈帧
 * @param  idx 电机序号 0~3
 * @param  f   收到的帧
 * @note   Data[0] 为信息功能码，与读取命令的功能码相同：
 *           0x35 = S_VEL 实时转速（2 字节，有符号）
 *           0x36 = S_CPOS 实时位置（4 字节，有符号）
 *           0x3A = S_FLAG 状态标志位（1 字节）
 *         ⚠ 回包字节序按 ZDT 手册的大端约定书写，需上硬件实测确认
 * @author camelliahm
 */
void chassis_HandleFeedback(uint8_t idx, const CAN_Frame_t *f)
{
	volatile ChassisMotor_t *m = &chassisMotor[idx];

	m->lastTick = xTaskGetTickCount();
	m->online   = 1;
	m->rxCount++;

	switch (f->Data[0])
	{
	case 0x35:	//S_VEL 实时转速
		if (f->DLC >= 3)
		{
			m->vel = (int16_t)(((uint16_t)f->Data[1] << 8) | (uint16_t)f->Data[2]);
		}
		break;

	case 0x36:	//S_CPOS 实时位置
		if (f->DLC >= 5)
		{
			m->pos = (int32_t)(((uint32_t)f->Data[1] << 24) |
							   ((uint32_t)f->Data[2] << 16) |
							   ((uint32_t)f->Data[3] <<  8) |
								(uint32_t)f->Data[4]);
		}
		break;

	case 0x3A:	//S_FLAG 状态标志位
		if (f->DLC >= 2)
		{
			m->flag = f->Data[1];
		}
		break;

	default:
		break;
	}
}

/*---- 周期任务 ----*/

/**
 * @brief  反馈看门狗：电机应在线却没回帧 → 故障
 * @note   仅在 READY / RUNNING / BRAKING 下判定。
 *         只在"在线 → 掉线"的跳变沿投递一次 EV_FAULT：否则每个控制
 *         周期都会投递，把 8 深的队列刷爆并淹没真正的事件。
 * @author camelliahm
 */
void chassis_DriveWatchdog(void)
{
	uint32_t now = xTaskGetTickCount();
	uint8_t  i;
	uint8_t  lost = 0;

	if ((chassis.state != CHS_READY) &&
		(chassis.state != CHS_RUNNING) &&
		(chassis.state != CHS_BRAKING))
	{
		return;
	}

	for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
	{
		if ((now - chassisMotor[i].lastTick) > pdMS_TO_TICKS(CHASSIS_FB_TIMEOUT_MS))
		{
			if (chassisMotor[i].online)
			{
				lost++;		//本周期刚从在线变掉线
			}
			chassisMotor[i].online = 0;
		}
		else
		{
			chassisMotor[i].online = 1;
		}
	}

	//只在跳变沿投递；已掉线状态下无需重复投递
	if (lost > 0)
	{
		contract_PostEvent(EV_FAULT, lost);
	}
}

/**
 * @brief  停稳判断：BRAKING 下四轮转速持续接近 0 即认为停稳
 * @note   只判四轮。升降轴有自己的到位判据（aux_LiftStep），
 *         且丝杆运动与底盘停车无关，把它算进来会让底盘永远停不下来。
 * @author camelliahm
 */
void chassis_CheckBraking(void)
{
	static uint32_t holdMs = 0;	//停稳计时(ms)（静态，跨周期保留）
	uint8_t  i;
	uint8_t  moving = 0;
	int16_t  v;

	if (chassis.state != CHS_BRAKING)
	{
		holdMs = 0;
		return;
	}

	for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
	{
		v = chassisMotor[i].vel;
		if (v < 0)
		{
			v = (int16_t)(-v);
		}
		if (v > CHASSIS_STOP_VEL_TH)
		{
			moving = 1;
			break;
		}
	}

	if (moving)
	{
		holdMs = 0;
	}
	else
	{
		holdMs += CHASSIS_CTRL_PERIOD_MS;
	}

	if (holdMs >= CHASSIS_STOP_HOLD_MS)
	{
		holdMs = 0;
		contract_PostEvent(EV_ARRIVED, 0);
	}
}

/**
 * @brief  里程计推算：由四轮反馈转速反推车体位移与偏航
 * @note   轮序约定：0=左前 1=右前 2=左后 3=右后；用"反馈转速 × 周期"估算每轮位移。
 *         航向输出两个版本：odomYawCont 连续（航向误差一律用它），
 *         odomYaw 是它的归一化副本（仅供内部泰勒展开近似）。
 *         ⚠ 骨架实现：轮径/减速比/轮距均为占位值，轮序与符号必须与实车装配一致，
 *           未标定前里程计不可依赖（否则方向相反、距离失准）
 * @author camelliahm
 */
static void chassis_UpdateOdom(void)
{
	float d[CHASSIS_MOTOR_NUM];
	float vLeft;
	float vRight;
	float dx;
	float dy;
	float dyaw;
	float yawRad;
	float c;
	float s;
	float x2;
	uint8_t i;

	//各轮本周期位移(mm) = 线速度(mm/s) × 周期(s)
	for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
	{
		d[i] = chassis_RpmToVel(chassisMotor[i].vel) * ((float)CHASSIS_CTRL_PERIOD_MS / 1000.0f);
	}

	/* 车体坐标系位移（简化正解）：
	 *   dy(纵向) = 四轮平均
	 *   dx(横向) = (右前 + 左后 - 左前 - 右后) / 4
	 * 轮序：0=左前 1=右前 2=左后 3=右后 */
	dy = (d[0] + d[1] + d[2] + d[3]) * 0.25f;
	dx = (d[1] + d[2] - d[0] - d[3]) * 0.25f;

	//偏航：左右两侧速度差 / 轮距 → 角度增量(度)
	vLeft  = (d[0] + d[2]) * 0.5f;
	vRight = (d[1] + d[3]) * 0.5f;
	dyaw   = ((vRight - vLeft) / CHASSIS_TRACK_WIDTH_MM) * (180.0f / 3.14159265f);

	/* 把位移增量投影到世界坐标系（用当前 yaw 旋转）。
	 * 用泰勒展开算 sin/cos，避免引入 math 库的重型 sinf/cosf；
	 * ⚠ 泰勒展开只在 |角| 小时准确，故这里取归一化副本 odomYaw 而非常量累加的 odomYawCont。 */
	yawRad = chassisDrive.odomYaw * 3.14159265f / 180.0f;
	x2     = yawRad * yawRad;
	s      = yawRad * (1.0f - x2 / 6.0f + x2 * x2 / 120.0f);	//sin 近似
	c      = 1.0f - x2 / 2.0f + x2 * x2 / 24.0f;				//cos 近似

	chassisDrive.odomX += dx * c - dy * s;
	chassisDrive.odomY += dx * s + dy * c;

	/* 航向走连续域：单调累加，跨 ±180 不跳变，导航到位判据与姿态环
	 * 的误差才有意义（否则 179°→-180° 会算出几百度的假误差） */
	chassisDrive.odomYawCont += dyaw;

	//归一化副本：内部三角函数近似与对外显示用
	chassisDrive.odomYaw = chassisDrive.odomYawCont;
	while (chassisDrive.odomYaw >  180.0f) { chassisDrive.odomYaw -= 360.0f; }
	while (chassisDrive.odomYaw < -180.0f) { chassisDrive.odomYaw += 360.0f; }
}

/**
 * @brief  麦轮逆运动学：车体速度(vx, vy, w) → 四轮目标转速(RPM)
 * @note   轮序约定：0=左前 1=右前 2=左后 3=右后；车体坐标 +x 向右、+y 向前、w 逆时针为正。
 *         标准麦轮逆解（wz = (lx+ly) × w，w 已由 0.1 度/s 换算为 rad/s）：
 *           v0(左前)=vy+vx-wz  v1(右前)=vy-vx+wz  v2(左后)=vy-vx-wz  v3(右后)=vy+vx+wz
 *         ⚠ 辊子旋向须与上式配套；若实车装配是另一种对角组合，需交换 vx 的符号
 * @param  vx 横向速度(mm/s)  vy 纵向速度(mm/s)  w 角速度(0.1度/s)
 * @author camelliahm
 */
static void chassis_InverseKinematics(int16_t vx, int16_t vy, int16_t w)
{
	float lxly;
	float wz;
	float v[CHASSIS_MOTOR_NUM];
	uint8_t i;

	lxly = CHASSIS_LX_MM + CHASSIS_LY_MM;

	//0.1度/s → rad/s，再 × 半径和 → 轮子线速度(mm/s)
	wz = ((float)w / 10.0f) * (3.14159265f / 180.0f) * lxly;

	v[0] = (float)vy + (float)vx - wz;	//左前
	v[1] = (float)vy - (float)vx + wz;	//右前
	v[2] = (float)vy - (float)vx - wz;	//左后
	v[3] = (float)vy + (float)vx + wz;	//右后

	for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
	{
		chassisDrive.wheelTarget[i] = chassis_VelToRpm(v[i]);
	}
}

/**
 * @brief  把四轮目标转速下发到驱动器
 * @note   必须整段包在 can_TxLock/Unlock 内（Emm_V5_* 用文件级 static 缓冲）。
 *         Emm_V5_Vel_Control 的 dir 只取 0 或非 0，速度值本身必须为正，
 *         故这里按符号拆出 dir 与绝对值。
 * @author camelliahm
 */
static void chassis_SendWheelVel(void)
{
	uint8_t i;
	uint8_t addr;
	uint8_t dir;
	uint16_t vel;

	can_TxLock();
	for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
	{
		addr = (uint8_t)(CHASSIS_ADDR_MOTOR_BASE + i);

		if (chassisDrive.wheelTarget[i] < 0)
		{
			dir = 1;
			vel = (uint16_t)(-chassisDrive.wheelTarget[i]);
		}
		else
		{
			dir = 0;
			vel = (uint16_t)chassisDrive.wheelTarget[i];
		}

		Emm_V5_Vel_Control(addr, dir, vel, CHASSIS_DRIVE_ACC_DEFAULT, false);
	}
	can_TxUnlock();
}

/**
 * @brief  底盘控制环：仅在 RUNNING 下被调用
 * @note   流程：里程计 → 姿态环 PID 修正 w → 逆运动学 → 下发。
 *         四轮的位置/速度闭环在驱动器内部完成，故 MCU 这一环
 *         只做"底盘级"运动学与姿态修正，5ms 节拍足够。
 * @author camelliahm
 */
void chassis_DriveStep(void)
{
	float yawFix;
	int16_t wCmd;

	//1. 里程计：由四轮反馈推算当前位姿
	chassis_UpdateOdom();

	/* 2. 姿态环：给定航向由编排层写在黑板（chassis.yawTarget），误差在
	 *    连续航向域上计算，跨 ±180 不会算出整圈的假误差。
	 *    PID_Calculate 按 Err = target - measure 计算 */
	yawFix = PID_Calculate(&s_yawPid, chassisDrive.odomYawCont, chassis.yawTarget);
	//3. 姿态修正量叠加到角速度指令上（限幅，防止修正量盖过用户指令）
	if (yawFix >  YAW_PID_OUT_MAX) { yawFix =  YAW_PID_OUT_MAX; }
	if (yawFix < -YAW_PID_OUT_MAX) { yawFix = -YAW_PID_OUT_MAX; }

	wCmd = (int16_t)((float)chassis.w + yawFix);

	//4. 麦轮逆解 + 下发
	chassis_InverseKinematics(chassis.vx, chassis.vy, wCmd);
	chassis_SendWheelVel();
}

/**
 * @brief  重置底盘驱动层状态（在 contract_Init() 中调用）
 * @note   里程计清零、PID 实例初始化、心跳置零
 * @author camelliahm
 */
void chassis_DriveInit(void)
{
	uint8_t i;

	for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
	{
		chassisMotor[i].online   = 0;
		chassisMotor[i].flag     = 0;
		chassisMotor[i].vel      = 0;
		chassisMotor[i].pos      = 0;
		chassisMotor[i].rxCount  = 0;
		chassisMotor[i].lastTick = xTaskGetTickCount();

		chassisDrive.wheelTarget[i] = 0;
	}

	chassisDrive.odomX       = 0.0f;
	chassisDrive.odomY       = 0.0f;
	chassisDrive.odomYaw     = 0.0f;
	chassisDrive.odomYawCont = 0.0f;

	/* 姿态环必开积分限幅（抗饱和）+ 微分先行（避免目标突变冲击）；参数顺序：
	 * max_out / integral_limit / deadband / kp / ki / kd / A / B / 输出滤波 / 微分滤波 / 改进位掩码
	 * ⚠ 该库无控制周期无关性，改 CHASSIS_CTRL_PERIOD_MS 必须重新整定 */
	PID_Init(&s_yawPid,
			 (uint16_t)YAW_PID_OUT_MAX,
			 (uint16_t)(YAW_PID_OUT_MAX * 0.2f),	//积分限幅取输出的 20%
			 0.1f,									//死区：误差 <0.1° 不调节，抑制抖动
			 YAW_PID_KP, YAW_PID_KI, YAW_PID_KD,
			 0.0f, 0.0f,							//变速积分 A/B，本环未启用
			 1.0f,									//输出滤波系数 1.0 = 不滤波
			 0.5f,									//微分滤波系数
			 (uint8_t)(Integral_Limit | Derivative_On_Measurement | DerivativeFilter));
}
