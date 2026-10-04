#include "contract.h"
#include "chassis_drive.h"
#include "auxiliary.h"
#include "can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* 底盘应用层：状态机 + 系统级调度（不含控制算法），详见工程根目录 README.md */

/* 静态存储期变量由启动代码清零；state / lastCmd 在 contract_Init() 中显式赋值 */
__IO Chassis_t chassis;

#define ARRAY_SIZE(a)	(sizeof(a) / sizeof((a)[0]))

/* 事件队列：事件稀疏，8 条足够 */
#define CHASSIS_EVT_QUEUE_LEN	(8)

typedef struct {
	ChassisEvent_t ev;
	uint32_t       param;
} ChassisEvtMsg_t;

static QueueHandle_t s_evtQueue = NULL;

/*---- 状态迁移表（EV_ESTOP 全局，在查表之前单独处理）----*/
typedef struct {
	ChassisState_t from;
	ChassisEvent_t ev;
	ChassisState_t to;
} ChassisTrans_t;

static const ChassisTrans_t s_transTable[] = {
	{ CHS_IDLE,    EV_ENABLE,      CHS_READY   },
	{ CHS_READY,   EV_START,       CHS_RUNNING },
	{ CHS_READY,   EV_DISABLE,     CHS_IDLE    },
	{ CHS_READY,   EV_FAULT,       CHS_FAULT   },	/* 使能后掉线同样要报故障 */
	{ CHS_RUNNING, EV_STOP,        CHS_BRAKING },
	{ CHS_RUNNING, EV_FAULT,       CHS_FAULT   },
	{ CHS_RUNNING, EV_DISABLE,     CHS_IDLE    },
	{ CHS_BRAKING, EV_ARRIVED,     CHS_READY   },
	{ CHS_BRAKING, EV_FAULT,       CHS_FAULT   },
	{ CHS_BRAKING, EV_DISABLE,     CHS_IDLE    },
	{ CHS_FAULT,   EV_CLEAR_FAULT, CHS_READY   },	/* 清除后回 READY，需重新 START */
	{ CHS_FAULT,   EV_DISABLE,     CHS_IDLE    },
	{ CHS_ESTOP,   EV_CLEAR_FAULT, CHS_IDLE    },	/* 急停解除一律回 IDLE，须重新使能 */
};

/*---- 状态进入动作（只编排各模块动作，不含控制算法）----*/

/**
 * @brief  状态进入动作
 * @param  s 目标状态
 * @note   新增机构时，在下面各 case 里补一行该机构的使能/失能调用即可
 * @author camelliahm
 */
static void contract_OnEnter(ChassisState_t s)
{
	chassis.state     = s;
	chassis.stateTick = xTaskGetTickCount();

	switch (s)
	{
	case CHS_IDLE:
		/* 断使能，电机可自由拖动 */
		chassis_DriveBroadcast(0);
		chassis_ClearOnline();
		lift_SetEnable(false);
		lift_ClearOnline();
		chassis.vx = 0; chassis.vy = 0; chassis.w = 0;
		break;

	case CHS_READY:
		/* 使能 + 打开定时上报；心跳重置为"现在"，
		 * 之后若在超时窗口内收不到帧即判掉线 */
		chassis_DriveBroadcast(1);
		chassis_EnableFeedback();
		chassis_ResetHeartbeat();
		lift_SetEnable(true);
		lift_EnableFeedback();
		chassis.vx = 0; chassis.vy = 0; chassis.w = 0;
		break;

	case CHS_RUNNING:
		chassis.vx = 0; chassis.vy = 0; chassis.w = 0;
		break;

	case CHS_BRAKING:
		/* 只把目标速度清零，由控制环逐周期减速，而不是急停 */
		chassis.vx = 0; chassis.vy = 0; chassis.w = 0;
		break;

	case CHS_FAULT:
	case CHS_ESTOP:
		/* 先清目标速度，再立即停止四台电机 */
		chassis.vx = 0; chassis.vy = 0; chassis.w = 0;
		chassis_DriveBroadcast(2);
		/* 升降轴单独处理：断使能立即撤扭矩并彻底断电，比 Stop_Now
		 * （保持使能的减速停止）更彻底；负载靠 T8 梯形丝杆自锁停住 */
		lift_SetEnable(false);
		chassis.faultCnt++;
		break;

	default:
		break;
	}
}

/**
 * @brief  事件处理：查表迁移
 * @param  ev    事件
 * @param  param 事件参数（当前未使用）
 * @author camelliahm
 */
static void contract_Step(ChassisEvent_t ev, uint32_t param)
{
	uint8_t i;

	(void)param;

	/* 急停最高优先：任何状态都直接进入 ESTOP */
	if ((ev == EV_ESTOP) && (chassis.state != CHS_ESTOP))
	{
		contract_OnEnter(CHS_ESTOP);
		return;
	}

	for (i = 0; i < ARRAY_SIZE(s_transTable); i++)
	{
		if ((s_transTable[i].from == chassis.state) &&
			(s_transTable[i].ev   == ev))
		{
			contract_OnEnter(s_transTable[i].to);
			return;
		}
	}

	/* 当前状态不接受该事件：忽略并计数（调试时可观察 evtIgnored） */
	chassis.evtIgnored++;
}

/*---- 消息处理（由 CAN 接收任务调用）----*/

/**
 * @brief  系统段消息（0x00~0x0F）：转投状态机事件
 * @retval true-已受理 false-段内无属主（计入 evtIgnored）
 * @note   只投请求，能否迁移由状态机查表决定（闸门在 s_transTable）
 * @author camelliahm
 */
static bool contract_SysCmd(ChassisCmd_t cmd, const CAN_Frame_t *f)
{
	(void)f;

	switch (cmd)
	{
	case CMD_ENABLE:      contract_PostEvent(EV_ENABLE, 0);      return true;
	case CMD_DISABLE:     contract_PostEvent(EV_DISABLE, 0);     return true;
	case CMD_START:       contract_PostEvent(EV_START, 0);       return true;
	case CMD_STOP:        contract_PostEvent(EV_STOP, 0);        return true;
	case CMD_ESTOP:       contract_PostEvent(EV_ESTOP, 0);       return true;
	case CMD_CLEAR_FAULT: contract_PostEvent(EV_CLEAR_FAULT, 0); return true;
	default:                                                    return false;
	}
}

/**
 * @brief  底盘段消息（0x10~0x1F）：速度指令
 * @retval true-已受理 false-段内无属主（计入 evtIgnored）
 * @note   解析前先校验 DLC；速度只在 RUNNING 下写入黑板，其余状态一律忽略
 * @author camelliahm
 */
static bool contract_DriveCmd(ChassisCmd_t cmd, const CAN_Frame_t *f)
{
	int16_t vx;
	int16_t vy;
	int16_t w;

	if (cmd != CMD_SET_VEL)
	{
		return false;
	}

	/* 参数 6 字节 + 消息码 = DLC 至少 7 */
	if (f->DLC >= 7)
	{
		vx = (int16_t)(((uint16_t)f->Data[1] << 8) | (uint16_t)f->Data[2]);
		vy = (int16_t)(((uint16_t)f->Data[3] << 8) | (uint16_t)f->Data[4]);
		w  = (int16_t)(((uint16_t)f->Data[5] << 8) | (uint16_t)f->Data[6]);

		/* 速度指令只在 RUNNING 下生效，其他状态一律忽略 */
		if (chassis.state == CHS_RUNNING)
		{
			chassis.vx = vx;
			chassis.vy = vy;
			chassis.w  = w;
		}
	}

	return true;
}

/*---- 消息路由表：按"码 & mask"查表派发；新增模块加一行 ----*/
typedef struct {
	uint8_t          seg;		/* 段基址，如 CHASSIS_SEG_LIFT */
	uint8_t          mask;		/* 段掩码，如 CHASSIS_SEG_MASK */
	ChassisHandler_t handler;	/* 该段消息的属主模块 */
} ChassisRoute_t;

static const ChassisRoute_t s_routeTable[] = {
	{ CHASSIS_SEG_SYS,   CHASSIS_SEG_MASK, contract_SysCmd   },	/* 0x00~0x0F 系统   */
	{ CHASSIS_SEG_DRIVE, CHASSIS_SEG_MASK, contract_DriveCmd },	/* 0x10~0x1F 底盘   */
	{ CHASSIS_SEG_LIFT,  CHASSIS_SEG_MASK, aux_HandleCmd    },	/* 0x20~0x2F 升降轴 */
};

/**
 * @brief  消息入口：按段查路由表，同步派发给属主模块
 * @param  f 收到的帧（已确认地址为本机）
 * @note   同一拍内处理完；无属主则忽略并计数，不当成错误
 * @author camelliahm
 */
static void contract_HandleCmd(const CAN_Frame_t *f)
{
	ChassisCmd_t cmd = (ChassisCmd_t)f->Data[0];
	uint8_t i;

	for (i = 0; i < ARRAY_SIZE(s_routeTable); i++)
	{
		if ((cmd & s_routeTable[i].mask) == s_routeTable[i].seg)
		{
			if (s_routeTable[i].handler(cmd, f))
			{
				chassis.lastCmd = cmd;
				return;
			}
		}
	}

	/* 段未注册 / 段内未知码：忽略并计数（调试时可观察 evtIgnored） */
	chassis.evtIgnored++;
	chassis.lastCmd = cmd;
}

/*---- 对外接口 ----*/

/**
 * @brief  初始化（创建事件队列），需在创建任务前调用
 * @note   依次初始化事件队列、底盘驱动层、辅助机构层
 * @author camelliahm
 */
void contract_Init(void)
{
	s_evtQueue = xQueueCreate(CHASSIS_EVT_QUEUE_LEN, sizeof(ChassisEvtMsg_t));
	configASSERT(s_evtQueue != NULL);

	chassis.state   = CHS_IDLE;
	chassis.lastCmd = CMD_NONE;

	chassis_DriveInit();	/* 四轮槽位、里程计、姿态环 PID */
	aux_Init();				/* 升降轴槽位 */
}

/**
 * @brief  处理一帧接收报文：按地址分流
 * @param  f 收到的帧
 * @note   0xFE → 消息；0x01~0x04 → 四轮反馈；0x05 → 升降轴反馈；
 *         其余地址（含 0x00 广播）忽略
 * @author camelliahm
 */
void contract_HandleFrame(const CAN_Frame_t *f)
{
	uint8_t addr = (uint8_t)((f->ExtId >> 8) & 0xFFu);

	if (addr == CHASSIS_ADDR_HOST)
	{
		contract_HandleCmd(f);
	}
	else if (addr == CHASSIS_ADDR_LIFT)
	{
		aux_HandleLiftFeedback(f);
	}
	else if ((addr >= CHASSIS_ADDR_MOTOR_BASE) &&
			 (addr <  (CHASSIS_ADDR_MOTOR_BASE + CHASSIS_MOTOR_NUM)))
	{
		chassis_HandleFeedback((uint8_t)(addr - CHASSIS_ADDR_MOTOR_BASE), f);
	}
	/* 其它地址（含 0x00 广播）：忽略 */
}

/**
 * @brief  投递状态机事件（非阻塞，队列满则丢弃）
 * @param  ev    事件
 * @param  param 事件参数
 * @note   投递方可能是控制环，绝不能在这里挂住，故超时为 0
 * @author camelliahm
 */
void contract_PostEvent(ChassisEvent_t ev, uint32_t param)
{
	ChassisEvtMsg_t msg;

	if (s_evtQueue == NULL)
	{
		return;
	}

	msg.ev    = ev;
	msg.param = param;

	/* 非阻塞投递：投递方可能是控制环，绝不能在这里挂住 */
	(void)xQueueSend(s_evtQueue, &msg, 0);
}

/*---- 任务体 ----*/

/**
 * @brief  控制环任务
 * @param  pvParameters 未使用
 * @note   周期 CHASSIS_CTRL_PERIOD_MS，需保证节拍绝对稳定。
 *         本任务只负责"节拍"与状态相关的调度，算法在各模块内。
 * @author camelliahm
 */
void contract_CtrlTask(void *pvParameters)
{
	TickType_t xLastWakeTime = xTaskGetTickCount();

	(void)pvParameters;

	for (;;)
	{
		contract_ControlStep();

		/* 保证固定周期，不随其他任务波动 */
		vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CHASSIS_CTRL_PERIOD_MS));
	}
}

/**
 * @brief  CAN 接收任务
 * @param  pvParameters 未使用
 * @note   阻塞等待接收队列（CAN1_RX0 中断投递），
 *         取到帧后交由 contract_HandleFrame() 按地址分流
 * @author camelliahm
 */
void contract_CanRecvTask(void *pvParameters)
{
	CAN_Frame_t frame;

	(void)pvParameters;

	for (;;)
	{
		/* 超时 100ms 只为周期性回头；正常情况下中断一到就唤醒 */
		if (can_RecvFrame(&frame, 100))
		{
			contract_HandleFrame(&frame);
		}
	}
}

/**
 * @brief  状态机任务
 * @param  pvParameters 未使用
 * @note   阻塞在事件队列上，无事件时不占 CPU
 * @author camelliahm
 */
void contract_StateTask(void *pvParameters)
{
	ChassisEvtMsg_t msg;

	(void)pvParameters;

	for (;;)
	{
		if (xQueueReceive(s_evtQueue, &msg, portMAX_DELAY) == pdPASS)
		{
			contract_Step(msg.ev, msg.param);
		}
	}
}

/**
 * @brief  控制环周期内调用（每 CHASSIS_CTRL_PERIOD_MS 一次）
 * @note   编排顺序：各模块看门狗 → 各模块周期判断 → RUNNING 下驱动底盘。
 *         各模块内部只做"自己的事"，互不干涉。
 * @author camelliahm
 */
void contract_ControlStep(void)
{
	/* 1. 看门狗（可能投递 EV_FAULT） */
	chassis_DriveWatchdog();
	aux_Step();

	/* 2. 停车过程的停稳判断（可能投递 EV_ARRIVED） */
	chassis_CheckBraking();

	/* 3. 只有在 RUNNING 下才驱动底盘 */
	if (chassis.state != CHS_RUNNING)
	{
		return;
	}

	chassis_DriveStep();	/* 里程计 → 逆运动学 → 姿态环 → 下发四轮 */
}

/**
 * @brief  取状态名（调试打印用）
 * @param  s 状态枚举
 * @retval 状态名字符串；越界时返回 "UNKNOWN"
 * @author camelliahm
 */
const char *contract_StateName(ChassisState_t s)
{
	static const char *name[CHS_STATE_NUM] = {
		"IDLE", "READY", "RUNNING", "BRAKING", "FAULT", "ESTOP"
	};

	if (s < CHS_STATE_NUM)
	{
		return name[s];
	}

	return "UNKNOWN";
}
