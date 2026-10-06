#ifndef __GRIP_SEQ_H
#define __GRIP_SEQ_H

#include "lift_axis.h"

//夹持机构层（旋转轴 = TIM3 CH1，夹子 = TIM3 CH2，车载物料盘 = TIM3 CH3）
//分层结构与扩展指引见工程根目录 README.md

/*---- 夹持机构（旋转轴 = TIM3 CH1，夹子 = TIM3 CH2）----*/
/* 无水平伸缩机构：水平定位靠底盘全向移动（编排层）+ 旋转轴在取料位 / 落盘位之间二选一；
 * 旋转轴只有两个端点角，中间角不需标定，不可当连续角度轴使用 */

#define GRIP_ROT_CH			(0u)		//旋转轴舵机通道（TIM3 CH1 = PC6）
#define GRIP_JAW_CH			(1u)		//夹子舵机通道（TIM3 CH2 = PC7）

#define GRIP_ROT_PICK_DEG	(0.0f)		//旋转轴取料位(°)：朝原料区
#define GRIP_ROT_PLACE_DEG	(180.0f)	//旋转轴落盘位(°)：朝车上物料盘
#define GRIP_ROT_SAFE_DEG	(90.0f)		//旋转轴安全位(°)：两工位中间，急停 / 上电姿态

#define GRIP_ROT_SLEW_DEG	(1.0f)		//旋转轴单周期最大角度增量(°)：限速使动作柔和；180° 约 0.9 s  ⚠ 待实测
#define GRIP_ROT_TOL_DEG	(2.0f)		//旋转轴到位判据(°)

/*---- 车载物料盘（旋转舵机 = TIM3 CH3 = PC8，270° 行程）----*/
/* 三槽位按搬运顺序落料：任务码第 1 组的第 k 位 → 槽位 k；
 * 槽位角即舵机指令角，机械上把槽位 k 装成该角正对夹持机构的落料点（顺序 1 = 位置一 = 0°） */

#define GRIP_TRAY_CH		(2u)		//物料盘舵机通道（TIM3 CH3 = PC8）
#define GRIP_TRAY_POS_NUM	(3u)		//槽位数量，须与 GRIP_LOAD_TOTAL 一致

#define GRIP_TRAY_POS1_DEG	(0.0f)		//槽位一(°)：顺序 1 的物料落点，兼上电 / 换批复位角
#define GRIP_TRAY_POS2_DEG	(120.0f)	//槽位二(°)：顺序 2 的物料落点
#define GRIP_TRAY_POS3_DEG	(240.0f)	//槽位三(°)：顺序 3 的物料落点

#define GRIP_TRAY_SLEW_DEG	(1.0f)		//物料盘单周期最大角度增量(°)：限速使动作柔和；240° 约 1.2 s  ⚠ 待实测
#define GRIP_TRAY_TOL_DEG	(2.0f)		//物料盘到位判据(°)

#define GRIP_JAW_OPEN_DEG	(10.0f)		//夹子张开角(°)  ⚠ 占位值，待实测
#define GRIP_JAW_CLOSE_DEG	(80.0f)		//夹子夹紧角(°)  ⚠ 占位值，待实测
#define GRIP_JAW_SETTLE_MS	(300u)		//夹子动作后的等待时长(ms)，等舵机走完

#define GRIP_H_PICK_MM		(60.0f)		//抓取高度(mm)      ⚠ 占位值，待实测
#define GRIP_H_PLACE_MM		(150.0f)	//落盘高度(mm)      ⚠ 占位值，待实测
#define GRIP_H_SAFE_MM		(200.0f)	//抬升 / 安全高度(mm)  ⚠ 占位值，待实测

/*---- 卸载（把车上物料放回车外目标区）----*/
/* 复用装载的两个端点角，只是角色对调：车内位取料、车外位放置。
 * 车怎么停在目标区前由编排层决定（本层只管一次取放），故不需要新的摆位机构 */

#define GRIP_ROT_U_PICK_DEG	(GRIP_ROT_PLACE_DEG)	//卸载取料位(°)：朝车内物料盘，即装载落盘位
#define GRIP_ROT_U_DROP_DEG	(GRIP_ROT_PICK_DEG)		//卸载放置位(°)：朝车外目标区，即装载取料位

#define GRIP_H_U_PICK_MM	(GRIP_H_PLACE_MM)	//槽内取料高度(mm)：暂沿用落盘高度  ⚠ 待实测，料落槽后或需再降一个料高
#define GRIP_H_U_DROP_MM	(GRIP_H_PICK_MM)	//车外放置高度(mm)：暂沿用原料区抓取高度  ⚠ 待实测，粗加工区台面高度或不同

/*---- 取回（把车外目标区的物料装回车上物料盘）----*/
/* 与装载同向（车外抓取 → 车内落盘），故复用装载的动作阶段，只多一个抓取高度：
 * 装载在原料区转盘上取，取回在地面圆环里取，两者台面高度不同 */

#define GRIP_H_R_PICK_MM	(GRIP_H_U_DROP_MM)	//圆环取料高度(mm)：暂沿用卸载放置高度  ⚠ 待实测，地面圆环台面高度

/*---- 码垛（第二批在暂存区叠到第一批物料之上）----*/
/* 放置高度比平放高一个料高：夹爪托着物料下到已有物料的顶面即停，不靠自由落体 */

#define GRIP_ITEM_H_MM		(80.0f)	//物料最大高度(mm)，取命题上限  ⚠ 占位值，待实测
#define GRIP_H_STACK_MM		(GRIP_H_U_DROP_MM + GRIP_ITEM_H_MM)	//码垛放置高度(mm)：第二层 = 第一层 + 一个料高  ⚠ 占位值，待实测

#define GRIP_LIFT_VEL		(LIFT_VEL_DEFAULT)	//夹持动作的升降速度(RPM)，沿用升降轴默认
#define GRIP_LIFT_ACC		(LIFT_ACC_DEFAULT)	//夹持动作的升降加速度(0-255)

#define GRIP_MOVE_TIMEOUT_MS (1000u)	//单个升降动作的超时(ms)
#define GRIP_ROT_TIMEOUT_MS	 (3000u)	//旋转动作的超时(ms)
#define GRIP_LOAD_TOTAL		 (3u)		//一个原料区需完成的装载轮数

//一轮装载的动作阶段（顺序即 phase 递增顺序）
typedef enum {
	GRIP_IDLE = 0,		//空闲
	GRIP_ROT_PICK,		//旋转到取料位
	GRIP_LOWER_PICK,	//下降到抓取高度
	GRIP_JAW_CLOSE,		//夹紧
	GRIP_LIFT_UP,		//抬起到安全高度
	GRIP_ROT_PLACE,		//旋转到落盘位
	GRIP_LOWER_PLACE,	//下降到落盘高度
	GRIP_JAW_OPEN,		//松开
	GRIP_LIFT_CLEAR,	//升高让开料盘已放物料
	GRIP_ROT_SAFE,		//旋转回安全位
	GRIP_TRAY_HOME,		//物料盘回位置一（仅三轮装满后）

	/*---- 卸载序列（车上物料 → 车外目标区），与装载反向 ----*/
	GRIP_U_ROT_TRAY,	//转到车内取料位，同时物料盘把待取槽位转到取料点
	GRIP_U_LOWER,		//下降到槽内取料高度
	GRIP_U_JAW_CLOSE,	//夹紧
	GRIP_U_LIFT_UP,		//抬起到安全高度
	GRIP_U_ROT_DROP,	//转到车外放置位
	GRIP_U_LOWER_DROP,	//下降到放置高度
	GRIP_U_JAW_OPEN,	//松开
	GRIP_U_LIFT_CLEAR,	//升高让开已放物料
	GRIP_U_ROT_SAFE,	//旋转回安全位（序列完成）

	GRIP_PHASE_NUM
} GripPhase_t;

typedef struct {
	uint8_t	 phase;			//当前动作阶段 GripPhase_t
	uint8_t	 busy;			//1 = 动作序列进行中
	uint8_t	 fault;			//1 = 本序列因超时 / 被拒绝执行而中断
	uint8_t	 loadCount;		//已完成的装载轮数（满 GRIP_LOAD_TOTAL 即完成一个原料区）
	uint8_t	 unloadSlot;	//卸载待取槽位(1~GRIP_TRAY_POS_NUM)
	uint8_t	 traySlot;		//本轮落料槽位(1~GRIP_TRAY_POS_NUM)：装载 = 轮次，取回 = 段指定
	uint8_t	 seqLoad;		//1 = 装载轮次序列（完成时累计 loadCount）；0 = 取回序列（完成即结束）
	float	 rotTarget;		//旋转轴目标角(°)（由斜坡逐步逼近）
	float	 rotNow;		//旋转轴当前角(°)（斜坡中间值，非舵机反馈）
	float	 trayTarget;	//物料盘目标角(°)（由斜坡逐步逼近）
	float	 trayNow;		//物料盘当前角(°)（斜坡中间值，非舵机反馈）
	float	 pickH;			//本轮抓取高度(mm)：装载 = 原料区转盘，取回 = 地面圆环
	float	 dropH;			//本轮车外放置高度(mm)：平放 = GRIP_H_U_DROP_MM，码垛 = GRIP_H_STACK_MM
	uint32_t phaseTick;		//进入当前阶段的 tick（超时判据）
} Grip_t;

extern __IO Grip_t grip;

//初始化（在 aux_Init() 中调用）
void grip_Init(void);

//周期任务：旋转轴 / 物料盘斜坡 + 装载 / 卸载序列推进（在 aux_Step() 中调用）
void grip_Step(void);

/* 安全姿态：旋转轴回 90° + 夹子张开 + 中止序列；IDLE / FAULT / ESTOP 由 contract_OnEnter() 调用；
 * 物料盘不在安全姿态内 —— 盘上可能已有物料，急停时转动会把它甩出去，故保持当前角不动 */
void grip_Safe(void);

/* 启动一轮装载（取料 → 转 180° 并同步把物料盘转到本轮槽位 → 落到物料盘 → 回安全位）；
 * 返回 false = 前置条件不满足（状态不符 / 序列忙 / 升降轴未回零） */
bool grip_LoadOnce(void);

/* 启动一次卸载（从物料盘第 slot 槽取料 → 转到车外位 → 放到台面 → 回安全位）；
 * 返回 false = 前置条件不满足（状态不符 / 序列忙 / 升降轴未回零 / slot 越界） */
bool grip_UnloadOnce(uint8_t slot);

/* 启动一次码垛卸载：动作与 grip_UnloadOnce() 相同，只是放置高度抬一个料高，叠到已有物料上；
 * 返回 false = 前置条件不满足 */
bool grip_StackOnce(uint8_t slot);

/* 启动一次取回（车外目标区取料 → 转 180° 装回物料盘第 slot 槽 → 回安全位）；
 * 返回 false = 前置条件不满足（状态不符 / 序列忙 / 升降轴未回零 / slot 越界） */
bool grip_ReloadOnce(uint8_t slot);

//中止动作序列（保持已下发的姿态，不改变旋转轴与夹子）
void grip_Abort(void);

//设定旋转轴目标角(°)，内部按 GRIP_ROT_SLEW_DEG 斜坡推进，不会一步跳到位
void grip_RotTo(float deg);

//夹子开 / 合：true = 夹紧，false = 张开
void grip_SetJaw(bool close);

//物料盘转到第 pos 个槽位（1~GRIP_TRAY_POS_NUM），内部按 GRIP_TRAY_SLEW_DEG 斜坡推进
void grip_TrayTo(uint8_t pos);

//物料盘回位置一（上电姿态；换批复位）
void grip_TrayHome(void);

//装载计数清零（每批装载开始调用；不清则 grip_AllLoaded() 恒真，装载段会秒过）
void grip_ResetLoad(void);

//序列是否进行中 / 三轮是否全部装完
bool grip_IsBusy(void);
bool grip_AllLoaded(void);

#endif
