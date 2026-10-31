#include "contract.h"
#include "chassis_drive.h"
#include "auxiliary.h"
#include "nav.h"
#include "avoid.h"
#include "input.h"
#include "vision.h"
#include "screen.h"
#include "can.h"
#include "oled.h"

#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>

//底盘应用层：状态机 + 系统级调度（不含控制算法），详见工程根目录 README.md

//静态存储期变量由启动代码清零；state / lastCmd 在 contract_Init() 中显式赋值
__IO Chassis_t chassis;	//全局黑板：跨层共享的底盘状态

#define ARRAY_SIZE(a)	(sizeof(a) / sizeof((a)[0]))	//数组元素个数

//事件队列：事件稀疏，8 条足够
#define CHASSIS_EVT_QUEUE_LEN	(8)	//状态机事件队列深度

//上屏节拍：屏指令是阻塞发送（115200 下最长约 6 ms），不能落在 5 ms 控制环上
#define CHASSIS_UI_PERIOD_MS	(200u)	//STATE 任务的等待超时，兼作上屏刷新周期

//待上屏项：一个刷新周期只发一条，把连发的阻塞摊到多个周期
#define UI_PEND_TASK			(1u << 0)	//任务码待上屏
#define UI_PEND_GET				(1u << 1)	//抓取次数待上屏
#define UI_PEND_PUT				(1u << 2)	//放置次数待上屏
#define UI_PEND_ERR				(1u << 3)	//失败次数待上屏

typedef struct {
	ChassisEvent_t ev;	//事件码
	uint32_t       param;	//事件参数
} ChassisEvtMsg_t;

static std::deque<ChassisEvtMsg_t> s_evtQueue;	//状态机事件队列
static std::mutex                  s_evtMutex;	//事件队列锁
static std::condition_variable     s_evtCond;	//事件队列条件变量

/*---- 状态迁移表（EV_ESTOP 全局，在查表之前单独处理）----*/
typedef struct {
	ChassisState_t from;							//迁移前状态
	ChassisEvent_t ev;								//触发事件
	ChassisState_t to;								//迁移后状态
} ChassisTrans_t;

static const ChassisTrans_t s_transTable[] = {
	{ CHS_IDLE,    EV_ENABLE,      CHS_READY   },
	{ CHS_READY,   EV_START,       CHS_RUNNING },
	{ CHS_READY,   EV_DISABLE,     CHS_IDLE    },
	{ CHS_READY,   EV_FAULT,       CHS_FAULT   },	//使能后掉线同样要报故障
	{ CHS_RUNNING, EV_STOP,        CHS_BRAKING },
	{ CHS_RUNNING, EV_FAULT,       CHS_FAULT   },
	{ CHS_RUNNING, EV_DISABLE,     CHS_IDLE    },
	{ CHS_BRAKING, EV_ARRIVED,     CHS_READY   },
	{ CHS_BRAKING, EV_FAULT,       CHS_FAULT   },
	{ CHS_BRAKING, EV_DISABLE,     CHS_IDLE    },
	{ CHS_FAULT,   EV_CLEAR_FAULT, CHS_READY   },	//清除后回 READY，需重新 START
	{ CHS_FAULT,   EV_DISABLE,     CHS_IDLE    },
	{ CHS_ESTOP,   EV_CLEAR_FAULT, CHS_IDLE    },	//急停解除一律回 IDLE，须重新使能
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
	chassis.stateTick = (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();

	switch (s)
	{
	case CHS_IDLE:
		//断使能，电机可自由拖动
		chassis_DriveBroadcast(0);
		chassis_ClearOnline();
		lift_SetEnable(false);
		lift_ClearOnline();
		grip_Safe();
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
		nav_Start();	//从第 0 段重跑；实际装载由控制环在 nav_Step() 里完成
		break;

	case CHS_BRAKING:
		//只清目标速度；实际减速由控制环在 BRAKING 分支里逐周期下发零速
		chassis.vx = 0; chassis.vy = 0; chassis.w = 0;
		break;

	case CHS_FAULT:
	case CHS_ESTOP:
		//先清目标速度，再立即停止四台电机
		chassis.vx = 0; chassis.vy = 0; chassis.w = 0;
		chassis_DriveBroadcast(2);
		/* 升降轴单独处理：断使能立即撤扭矩并彻底断电，比 Stop_Now
		 * （保持使能的减速停止）更彻底；负载靠 T8 梯形丝杆自锁停住 */
		lift_SetEnable(false);
		grip_Safe();
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

	//急停最高优先：任何状态都直接进入 ESTOP
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

	//当前状态不接受该事件：忽略并计数（调试时可观察 evtIgnored）
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

#if CFG_HOST_CAN_ENABLED
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
#else
	return false;		//无上位机：系统段无属主
#endif
}

/**
 * @brief  底盘段消息（0x10~0x1F）：速度指令
 * @retval true-已受理 false-段内无属主（计入 evtIgnored）
 * @note   解析前先校验 DLC；速度只在 RUNNING 下写入黑板，其余状态一律忽略
 * @author camelliahm
 */
static bool contract_DriveCmd(ChassisCmd_t cmd, const CAN_Frame_t *f)
{
#if CFG_HOST_CAN_ENABLED
	int16_t vx;
	int16_t vy;
	int16_t w;

	if (cmd != CMD_SET_VEL)
	{
		return false;
	}

	//参数 6 字节 + 消息码 = DLC 至少 7
	if (f->DLC >= 7)
	{
		vx = (int16_t)(((uint16_t)f->Data[1] << 8) | (uint16_t)f->Data[2]);
		vy = (int16_t)(((uint16_t)f->Data[3] << 8) | (uint16_t)f->Data[4]);
		w  = (int16_t)(((uint16_t)f->Data[5] << 8) | (uint16_t)f->Data[6]);

		//速度指令只在 RUNNING 下生效，其他状态一律忽略
		if (chassis.state == CHS_RUNNING)
		{
			chassis.vx = vx;
			chassis.vy = vy;
			chassis.w  = w;
		}
	}

	return true;
#else
	(void)cmd;
	(void)f;

	return false;		//无上位机：底盘段无属主，速度只由 nav 产生
#endif
}

/*---- 消息路由表：按"码 & mask"查表派发；新增模块加一行 ----*/
typedef struct {
	uint8_t          seg;		//段基址，如 CHASSIS_SEG_LIFT
	uint8_t          mask;		//段掩码，如 CHASSIS_SEG_MASK
	ChassisHandler_t handler;	//该段消息的属主模块
} ChassisRoute_t;

static const ChassisRoute_t s_routeTable[] = {
	{ CHASSIS_SEG_SYS,   CHASSIS_SEG_MASK, contract_SysCmd   },	//0x00~0x0F 系统
	{ CHASSIS_SEG_DRIVE, CHASSIS_SEG_MASK, contract_DriveCmd },	//0x10~0x1F 底盘
	{ CHASSIS_SEG_LIFT,  CHASSIS_SEG_MASK, aux_HandleCmd    },	//0x20~0x2F 升降轴
	{ CHASSIS_SEG_SERVO, CHASSIS_SEG_MASK, aux_HandleCmd    },	//0x30~0x3F 夹持机构（舵机）
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

	//段未注册 / 段内未知码：忽略并计数（调试时可观察 evtIgnored）
	chassis.evtIgnored++;
	chassis.lastCmd = cmd;
}

/*---- 对外接口 ----*/

/**
 * @brief  初始化，需在创建任务前调用
 * @note   依次初始化底盘驱动层、辅助机构层
 * @author camelliahm
 */
void contract_Init(void)
{
	chassis.state     = CHS_IDLE;
	chassis.lastCmd   = CMD_NONE;
	chassis.yawTarget = 0.0f;

	chassis_DriveInit();	//四轮槽位、里程计、姿态环 PID
	aux_Init();				//升降轴槽位
	avoid_Init();			//避障层状态
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
	//其它地址（含 0x00 广播）：忽略
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

	msg.ev    = ev;
	msg.param = param;

	//非阻塞投递：投递方可能是控制环，绝不能在这里挂住，队列满则丢弃
	std::lock_guard<std::mutex> lk(s_evtMutex);

	if (s_evtQueue.size() < CHASSIS_EVT_QUEUE_LEN)
	{
		s_evtQueue.push_back(msg);
		s_evtCond.notify_one();
	}
}

/*---- 任务体 ----*/

/**
 * @brief  控制环任务
 * @note   周期 CHASSIS_CTRL_PERIOD_MS，需保证节拍绝对稳定。
 *         本任务只负责"节拍"与状态相关的调度，算法在各模块内。
 * @author camelliahm
 */
void contract_CtrlTask(void)
{
	std::chrono::steady_clock::time_point next = std::chrono::steady_clock::now();

	for (;;)
	{
		contract_ControlStep();

		//保证固定周期，不随其他任务波动
		next += std::chrono::milliseconds(CHASSIS_CTRL_PERIOD_MS);
		std::this_thread::sleep_until(next);
	}
}

/**
 * @brief  CAN 接收任务
 * @note   阻塞等待接收队列（socketcan 轮询桥接），
 *         取到帧后交由 contract_HandleFrame() 按地址分流
 * @author camelliahm
 */
void contract_CanRecvTask(void)
{
	CAN_Frame_t frame;

	for (;;)
	{
		//超时 100ms 只为周期性回头；正常情况下有帧即唤醒
		if (can_RecvFrame(&frame, 100))
		{
			contract_HandleFrame(&frame);
		}
	}
}

/**
 * @brief  比较两个字符串是否相同（不引入 string 库）
 * @param  a 字符串一  b 字符串二
 * @retval 1 = 相同
 * @author camelliahm
 */
static uint8_t contract_StrSame(const char *a, const char *b)
{
	while ((*a != '\0') && (*a == *b))
	{
		a++;
		b++;
	}

	return (*a == *b) ? 1u : 0u;
}

/**
 * @brief  上屏刷新：与屏上当前内容比较，有差异才登记；每个周期最多发一条
 * @note   只在本任务上下文调用。屏指令是阻塞发送（115200 下最长约 6 ms），
 *         故不放进 5 ms 控制环；连发多条会拖住事件消费，故摊到多个超时周期
 * @author camelliahm
 */
static void contract_UiStep(void)
{
	static uint8_t    inited;							//1 = 初值已上屏过
	static uint8_t    pend;								//待上屏项位图（UI_PEND_*）
	static char       pendCode[VISION_TASK_CODE_MAX];	//待上屏的任务码
	static NavStats_t pendStat;							//待上屏的统计
	static char       lastCode[VISION_TASK_CODE_MAX];	//屏上当前的任务码
	static NavStats_t lastStat;							//屏上当前的统计
	char              code[VISION_TASK_CODE_MAX];
	NavStats_t        st;
	uint8_t           n;
	uint8_t           i;

	nav_GetStats(&st);
	n = nav_GetTaskCode(code, (uint8_t)sizeof(code));

	//与屏上值比较：有差异才登记；首次无条件登记，把 0/12 的初值刷上去
	//没有任务码时不动，避免把屏上已有显示清空
	if ((n > 0u) && (contract_StrSame(code, lastCode) == 0u))
	{
		for (i = 0u; i < (uint8_t)sizeof(pendCode); i++)
		{
			pendCode[i] = code[i];
			if (code[i] == '\0')
			{
				break;
			}
		}
		pend |= UI_PEND_TASK;
	}

	if ((inited == 0u) || (st.okGet != lastStat.okGet))
	{
		pendStat.okGet = st.okGet;
		pend |= UI_PEND_GET;
	}
	if ((inited == 0u) || (st.okPut != lastStat.okPut))
	{
		pendStat.okPut = st.okPut;
		pend |= UI_PEND_PUT;
	}
	if ((inited == 0u) || (st.errGet != lastStat.errGet) || (st.errPut != lastStat.errPut))
	{
		pendStat.errGet = st.errGet;
		pendStat.errPut = st.errPut;
		pend |= UI_PEND_ERR;
	}

	inited = 1u;

	//每周期只发一条：任务码 → 抓取 → 放置 → 失败；发完即记屏上现状并清位
	if ((pend & UI_PEND_TASK) != 0u)
	{
		screen_ShowTask(pendCode);

		for (i = 0u; i < (uint8_t)sizeof(lastCode); i++)
		{
			lastCode[i] = pendCode[i];
			if (pendCode[i] == '\0')
			{
				break;
			}
		}
		pend &= (uint8_t)~UI_PEND_TASK;
	}
	else if ((pend & UI_PEND_GET) != 0u)
	{
		screen_ShowGet(pendStat.okGet, (uint8_t)NAV_STAT_GET_TOTAL);
		lastStat.okGet = pendStat.okGet;
		pend &= (uint8_t)~UI_PEND_GET;
	}
	else if ((pend & UI_PEND_PUT) != 0u)
	{
		screen_ShowPut(pendStat.okPut, (uint8_t)NAV_STAT_PUT_TOTAL);
		lastStat.okPut = pendStat.okPut;
		pend &= (uint8_t)~UI_PEND_PUT;
	}
	else if ((pend & UI_PEND_ERR) != 0u)
	{
		screen_ShowErr(pendStat.errGet, pendStat.errPut);
		lastStat.errGet = pendStat.errGet;
		lastStat.errPut = pendStat.errPut;
		pend &= (uint8_t)~UI_PEND_ERR;
	}
}

/**
 * @brief  状态机任务
 * @note   带超时等事件队列：超时与事件两条路都汇到上屏刷新，故本任务兼作低速后台。
 *         不影响 5 ms 控制节拍
 * @author camelliahm
 */
void contract_StateTask(void)
{
	ChassisEvtMsg_t msg;

	for (;;)
	{
		bool got;

		{
			std::unique_lock<std::mutex> lk(s_evtMutex);
			got = s_evtCond.wait_for(lk, std::chrono::milliseconds(CHASSIS_UI_PERIOD_MS),
									 [&] { return !s_evtQueue.empty(); });

			if (got)
			{
				msg = s_evtQueue.front();
				s_evtQueue.pop_front();
			}
		}

		if (got)
		{
			contract_Step(msg.ev, msg.param);
		}

		contract_UiStep();
		oled_Step();
	}
}

/**
 * @brief  控制环周期内调用（每 CHASSIS_CTRL_PERIOD_MS 一次）
 * @note   编排顺序：各模块看门狗 → 各模块周期判断 → 编排层推进 → 按状态驱动底盘。
 *         各模块内部只做"自己的事"，互不干涉，进行解耦式编排。
 * @author camelliahm
 */
void contract_ControlStep(void)
{
	//1. 各模块周期处理。输入汇聚层负责摁键自启动
	chassis_DriveWatchdog();	//看门狗可能投递 EV_FAULT；
	aux_Step();
	input_Step();  //（按下延时 3 s 后投 EV_ENABLE + EV_START），并刷新三条串口的在线标志
	vision1_Step();  //视觉模块 1 帧解析（物块颜色 + 对准相对距离）
	vision_Step();  //视觉模块 2 帧解析（二维码任务码 / 避障列表）
	screen_Step();  //串口屏上报帧解析（USART2；只收不阻塞，发送在 STATE 任务里）

	//2. 停车过程的停稳判断（可能投递 EV_ARRIVED）
	chassis_CheckBraking();

	/* 3. 编排层推进：内部自判状态，离开 RUNNING 即中止并交还控制权。
	 *    放在闸门之前是必须的 —— 它的归位逻辑靠"每周期都被调用"才能生效 */
	nav_Step();

	/* 4. 避障叠加：必须在 nav_Step() 之后 —— 本层只往 vx 上加让行分量，
	 *    靠"后写"压过导航本周期算出的横向速度，故顺序不能颠倒 */
	avoid_Step();

	//5. 按状态驱动底盘
	if (chassis.state == CHS_RUNNING)
	{
		chassis_DriveStep();	//里程计 → 逆运动学 → 姿态环 → 下发四轮
	}
	else if (chassis.state == CHS_BRAKING)
	{
		/* 减速停：目标速度已清零，这里逐周期重发零速，
		 * 覆盖同一周期内可能已下发的残留 RUNNING 速度指令 */
		chassis_BrakeWheels();
	}
	//其余状态（IDLE / READY / FAULT / ESTOP）不下发任何速度
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
