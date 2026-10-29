#ifndef __CONTRACT_H
#define __CONTRACT_H

#include "board.h"
#include "can.h"

//公共契约层：地址宏 / 消息码 / 状态与事件枚举 / Chassis_t / 任务参数 / 对外接口，详见 README.md

/*---- 功能开关 ----*/
//1 = 有上位机，CAN 指令层生效；0 = 无上位机，指令层不生效（纯自治）
//只闸指令解析（SysCmd / DriveCmd / aux_HandleCmd），反馈接收与各模块周期任务不受影响
#ifndef CFG_HOST_CAN_ENABLED
#define CFG_HOST_CAN_ENABLED	(0)		//是否启用上位机 CAN 指令层
#endif

/*---- 地址分配（ZDT 帧：ExtId[28:8] = 地址，ExtId[7:0] = 包序号）----*/
//0x01~0x04 四轮 / 0x05 升降轴 / 0xFE 本机；0x00 是 ZDT 广播地址，禁止占用。全表见 README.md
#define CHASSIS_ADDR_HOST			(0xFEu)		//本机（上位机）地址
#define CHASSIS_ADDR_MOTOR_BASE		(0x01u)	//四轮驱动器起始地址（0x01~0x04）
#define CHASSIS_MOTOR_NUM			(4u)	//底盘车轮数量
#define CHASSIS_ADDR_LIFT			(0x05u)	//升降轴驱动器地址

/*---- 消息码分段：段基址 = 码 & CHASSIS_SEG_MASK ----*/
//0x00 系统 / 0x10 底盘 / 0x20 升降轴 / 0x30 夹持机构（舵机）；0x40 任务 / 0x50 视觉（后两段规划中）
#define CHASSIS_SEG_SYS			(0x00u)		//0x00~0x0F：系统（使能/启动/急停）
#define CHASSIS_SEG_DRIVE		(0x10u)		//0x10~0x1F：底盘（速度指令）
#define CHASSIS_SEG_LIFT		(0x20u)		//0x20~0x2F：升降轴
#define CHASSIS_SEG_SERVO		(0x30u)		//0x30~0x3F：夹持机构（舵机）
#define CHASSIS_SEG_NAV			(0x40u)		//0x40~0x4F：任务层（规划中）
#define CHASSIS_SEG_VISION		(0x50u)		//0x50~0x5F：视觉（规划中）
#define CHASSIS_SEG_MASK		(0xF0u)		//段掩码：段基址 = 码 & MASK

/*---- 消息码：Data[0] = 消息码，Data[1..] = 参数；各段属主见 contract.c 的 s_routeTable[] ----*/
typedef enum {
	CMD_NONE		= 0x00,		//无消息码（占位）
	CMD_ENABLE		= 0x01,		//参数无：IDLE  → READY
	CMD_DISABLE		= 0x02,		//参数无：任意  → IDLE
	CMD_START		= 0x03,		//参数无：READY → RUNNING
	CMD_STOP		= 0x04,		//参数无：RUNNING → BRAKING
	CMD_ESTOP		= 0x05,		//参数无：任意  → ESTOP（最高优先）
	CMD_CLEAR_FAULT	= 0x06,		//参数无：FAULT/ESTOP → READY/IDLE
	CMD_SET_VEL		= 0x10,		//参数：vx(int16) vy(int16) w(int16) 大端，共 6 字节

	/*---- 升降轴指令（0x20~0x2F）：只影响升降轴，不触发底盘状态机迁移 ----*/
	//除 CMD_LIFT_STOP / CMD_LIFT_CLEARF 外，仅在 READY / RUNNING 下生效
	//CMD_LIFT_POS 参数 8 字节，DLC=9（单帧塞满）：
	//  [1] 方向 0=上升/1=下降
	//  [2..3] 位移量(脉冲, u16 大端)
	//  [4..5] 速度(RPM, u16 大端)
	//  [6] 加速度(0-255，0=直接启动)
	//  [7] 保留填 0
	//  位移量是"相对当前位置"，不是绝对位置
	CMD_LIFT_POS	= 0x20,		//参数 8 字节
	CMD_LIFT_MM		= 0x25,		//参数：同 0x20，位移量单位换成 0.1 mm（120 = 12.0 mm），量程更大
	CMD_LIFT_HOME	= 0x21,		//参数无：触发回零（模式 3 = 多圈有限位开关回零）
	CMD_LIFT_STOP	= 0x22,		//参数无：立即停止升降轴（不影响底盘）
	CMD_LIFT_SETZERO= 0x23,		//参数无：将当前位置清零（需先卸掉丝杆负载）
	CMD_LIFT_CLEARF	= 0x24,		//参数无：解除升降轴堵转保护

	/*---- 夹持机构指令（0x30~0x3F）：只影响舵机机构，不触发底盘状态机迁移 ----*/
	//除 CMD_GRIP_ABORT / CMD_GRIP_SAFE 外，仅在 READY / RUNNING 下生效
	CMD_GRIP_LOAD	= 0x30,		//参数无：跑一轮装载（取料 → 转 180° → 落盘 → 回 90°）
	CMD_GRIP_ABORT	= 0x31,		//参数无：中止装载动作序列（保持当前姿态）
	CMD_GRIP_SAFE	= 0x32,		//参数无：安全姿态（旋转 90° + 夹子张开 + 中止序列）
	CMD_GRIP_OPEN	= 0x33,		//参数无：夹子张开
	CMD_GRIP_CLOSE	= 0x34,		//参数无：夹子夹紧
	CMD_GRIP_ROTPICK= 0x35,		//参数无：旋转轴 → 取料位(0°)
	CMD_GRIP_ROTPLACE=0x36,		//参数无：旋转轴 → 落盘位(180°)
	CMD_GRIP_ROTSAFE= 0x37,		//参数无：旋转轴 → 安全位(90°)
} ChassisCmd_t;

/*---- 消息处理器（按段注册到 contract.c 的 s_routeTable[]）----*/
//返回 true = 已受理；false = 未受理（段内无属主，计入 evtIgnored）
typedef bool (*ChassisHandler_t)(ChassisCmd_t cmd, const CAN_Frame_t *f);

/*---- 状态机状态 ----*/
typedef enum {
	CHS_IDLE = 0,		//初始：电机失能
	CHS_READY,			//就绪：已使能，等待运动指令
	CHS_RUNNING,		//运行：底盘控制环生效
	CHS_BRAKING,		//停车：目标速度置零，等停稳
	CHS_FAULT,			//故障：堵转/掉线/超时，需 CMD_CLEAR_FAULT
	CHS_ESTOP,			//急停：最高优先，需 CMD_CLEAR_FAULT
	CHS_STATE_NUM		//状态总数（哨兵）
} ChassisState_t;

/*---- 状态机事件 ----*/
typedef enum {
	EV_NONE = 0,			//无事件（占位）
	EV_ENABLE,			//使能，IDLE -> READY
	EV_DISABLE,			//断使能，回 IDLE
	EV_START,			//启动，READY -> RUNNING
	EV_STOP,			//请求停车，RUNNING -> BRAKING
	EV_ESTOP,			//急停：不查表，任何状态直入 ESTOP
	EV_CLEAR_FAULT,			//清故障：FAULT -> READY、ESTOP -> IDLE
	EV_ARRIVED,			//内部：四轮已停稳
	EV_FAULT,			//内部：掉线/超时等
	EV_EVENT_NUM			//事件总数（哨兵）
} ChassisEvent_t;

/*---- 底盘整体状态（系统级）----*/
//各模块自己的状态：chassisMotor[] / chassisDrive[] 见 chassis_drive.h，lift 见 auxiliary.h
typedef struct {
	ChassisState_t state;		//当前状态
	ChassisCmd_t   lastCmd;		//最后一条收到的消息码（诊断用）
	uint32_t	   evtIgnored;	//被忽略的事件数（诊断用）
	uint32_t	   faultCnt;	//进入 FAULT/ESTOP 的次数

	int16_t		   vx, vy, w;	//目标底盘速度：vx/vy = mm/s，w = 0.1 度/s
	float		   yawTarget;	//姿态环给定航向(度，连续域)：编排层写，驱动层读

	uint32_t	   stateTick;	//进入当前状态的 tick
} Chassis_t;

/*---- 任务参数（实际创建在 APP/main.c 的任务表里，优先级依据见 README.md）----*/
#define CHASSIS_CTRL_TASK_PRIO		( tskIDLE_PRIORITY + 4 )	//控制环任务优先级（最高，5 ms 硬节拍）
#define CHASSIS_CTRL_TASK_STACK		( 512 )		//需做浮点运算，栈留大一些

#define CHASSIS_CAN_TASK_PRIO		( tskIDLE_PRIORITY + 3 )	//CAN 接收任务优先级
#define CHASSIS_CAN_TASK_STACK		( 256 )		//阻塞等接收队列，栈需求小

#define CHASSIS_STATE_TASK_PRIO		( tskIDLE_PRIORITY + 2 )	//状态机任务优先级（最低）
#define CHASSIS_STATE_TASK_STACK	( 256 )		//阻塞等事件队列，栈需求小

extern __IO Chassis_t chassis;

void contract_Init(void);			//创建事件队列；须在创建任务之前调用

void contract_HandleFrame(const CAN_Frame_t *f);	//按地址分流：0xFE 消息 / 0x01~0x04 四轮反馈 / 0x05 升降反馈

void contract_PostEvent(ChassisEvent_t ev, uint32_t param);	//投递状态机事件（非阻塞，队列满丢弃）

void contract_CtrlTask(void *pvParameters);		//控制环任务体：固定 5ms 节拍
void contract_CanRecvTask(void *pvParameters);	//CAN 接收任务体：阻塞等队列并分流
void contract_StateTask(void *pvParameters);		//状态机任务体：阻塞等事件，空闲零 CPU

void contract_ControlStep(void);					//控制环周期内调用：只做编排，不写控制算法

const char *contract_StateName(ChassisState_t s);	//状态名（调试用）

#endif
