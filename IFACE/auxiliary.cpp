#include "auxiliary.h"

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
 * @brief  解析升降轴反馈帧（地址 0x05）
 * @param  f 收到的帧
 * @note   反馈解析属升降轴，实现见 lift_axis.c，此处只转发以保持本文件为唯一接入点
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
 * @brief  初始化辅助机构（在 contract_Init() 中调用）
 * @author camelliahm
 */
void aux_Init(void)
{
	lift_Init();
	grip_Init();
}
