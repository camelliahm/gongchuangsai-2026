#include "auxiliary.h"
#include "input.h"

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
 * @brief  解析升降轴反馈帧（地址 0x05）
 * @param  f 收到的帧
 * @note   反馈解析属升降轴，此处只转发
 * @author camelliahm
 */
void aux_HandleLiftFeedback(const CAN_Frame_t *f)
{
	lift_HandleFeedback(f);
}

/**
 * @brief  辅助机构周期任务：由控制环每 5ms 调用
 * @author camelliahm
 */
void aux_Step(void)
{
	lift_Step();	//升降轴：反馈看门狗 + 到位判断
	grip_Step();	//夹持机构：旋转 / 物料盘斜坡 + 装载 / 卸载序列
}

/**
 * @brief  手动调参动作分派：消费 input 的手动意图，转调升降轴/夹持机构
 * @note   只在 IDLE / READY 下受理；旋转轴三档、物料盘三槽循环，夹爪二值，
 *         升降轴按 KEY_MANUAL_LIFT_STEP_MM 单步相对位移（不破坏零点基准）。
 *         由 contract_ControlStep() 在 input_Step() 之后调用
 * @author camelliahm
 */
void aux_ManualStep(void)
{
	uint8_t act;
	uint8_t obj;

	if ((chassis.state != CHS_IDLE) && (chassis.state != CHS_READY))
	{
		return;		//非空闲/就绪态不响应手动调参
	}

	act = input_ManualPop();

	if (act == MANUAL_ACT_NONE)
	{
		return;
	}

	obj = input_ManualObj();

	switch (act)
	{
	case MANUAL_ACT_DEC:
	case MANUAL_ACT_INC:
		break;

	case MANUAL_ACT_ZERO:
		//双键同按：把升降轴当前位置重标为零，与当前调节对象无关（专属于升降轴）
		lift_SetZero();
		return;

	case MANUAL_ACT_NEXT:		//切对象：OLED 已由对象索引选页，这里无需动作
	default:					//ENTER / EXIT / NEXT 无需机构动作
		return;
	}

	switch (obj)
	{
	case MANUAL_LIFT:
		lift_MoveByMM((act == MANUAL_ACT_INC) ? KEY_MANUAL_LIFT_STEP_MM : -KEY_MANUAL_LIFT_STEP_MM,
					  LIFT_VEL_DEFAULT, LIFT_ACC_DEFAULT);
		break;

	case MANUAL_JAW:
		grip_SetJaw(act == MANUAL_ACT_INC);	//加=夹紧，减=张开
		break;

	case MANUAL_ROT:
	{
		//三档循环：0° → 90° → 180° → 0°
		float now = grip.rotNow;
		float tgt;

		if (act == MANUAL_ACT_INC)
		{
			if (now < GRIP_ROT_SAFE_DEG - GRIP_ROT_TOL_DEG)			tgt = GRIP_ROT_SAFE_DEG;
			else if (now < GRIP_ROT_PLACE_DEG - GRIP_ROT_TOL_DEG)	tgt = GRIP_ROT_PLACE_DEG;
			else													tgt = GRIP_ROT_PICK_DEG;
		}
		else
		{
			if (now > GRIP_ROT_SAFE_DEG + GRIP_ROT_TOL_DEG)			tgt = GRIP_ROT_SAFE_DEG;
			else if (now > GRIP_ROT_PICK_DEG + GRIP_ROT_TOL_DEG)	tgt = GRIP_ROT_PICK_DEG;
			else													tgt = GRIP_ROT_PLACE_DEG;
		}

		grip_RotTo(tgt);
		break;
	}

	case MANUAL_TRAY:
	{
		//三槽循环：1 → 2 → 3 → 1
		uint8_t pos = 1u;	//缺省回槽位一

		if (grip.trayNow >= GRIP_TRAY_POS2_DEG - GRIP_TRAY_TOL_DEG)	pos = 2u;
		if (grip.trayNow >= GRIP_TRAY_POS3_DEG - GRIP_TRAY_TOL_DEG)	pos = 3u;

		if (act == MANUAL_ACT_INC)
		{
			pos = (pos % GRIP_TRAY_POS_NUM) + 1u;
		}
		else
		{
			pos = (pos == 1u) ? GRIP_TRAY_POS_NUM : (pos - 1u);
		}

		grip_TrayTo(pos);
		break;
	}

	default:
		break;
	}
}

/**
 * @brief  初始化辅助机构（在 contract_Init() 中调用）
 * @author camelliahm
 */
void aux_Init(void)
{
	lift_Init();
	grip_Init();
}
