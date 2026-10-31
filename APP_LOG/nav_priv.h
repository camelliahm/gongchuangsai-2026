#ifndef __NAV_PRIV_H
#define __NAV_PRIV_H

#include "nav.h"
#include <chrono>
#include "chassis_drive.h"
#include "auxiliary.h"
#include "vision.h"

//nav 内部共享：状态结构、内部宏与跨文件函数原型。对外接口见 nav.h
//拆分只为缩短单文件；函数与调用关系不变，故运行负担不变

/*---- 内部宏 ----------------------------------------*/
#define NAV_TURN_HOLD_CNT	(NAV_TURN_HOLD_MS / NAV_CTRL_PERIOD_MS)	//转向保持判据的周期数
#define NAV_ALIGN_HOLD_CNT	(NAV_ALIGN_HOLD_MS / NAV_CTRL_PERIOD_MS)	//对准保持判据的周期数
#define NAV_DEG2RAD			(0.01745329252f)	//度 → 弧度

#define NAV_GRIP_UNLOAD		(0u)	//取放模式：卸料（物料盘 → 车外目标区，平放）
#define NAV_GRIP_RELOAD		(1u)	//取放模式：取回（车外目标区 → 物料盘）
#define NAV_GRIP_STACK		(2u)	//取放模式：码垛卸料（同卸料，放置高度抬一个料高）

/*---- 内部状态（定义在 nav.c）----------------------------------------*/

typedef struct {
	NavState_t		state;			//编排状态：是否在跑
	const NavSeg_t *map;			//当前地图（段数组）
	uint16_t		mapNum;			//地图段数
	uint16_t		seg;			//当前段下标
	uint32_t		segTick;		//当前段开始的 tick
	uint16_t		hold;			//判据需持续的控制周期数
	uint8_t			pendStart;		//1 = 收到启动请求，等控制环上下文里装载

	//航向不再本层维护：直接用 chassisDrive.odomYawCont（连续域）
	float			targetYaw;		//本段目标航向(度)

	//本段起点：位移判据在"段方向"上做投影，横向漂移不干扰
	float			startX;			//本段起点 X(mm)
	float			startY;			//本段起点 Y(mm)
	float			startYaw;		//本段起点航向(度)

	//对准 / 装载段
	uint8_t			alignHold;		//已居中的连续周期数（保持判据）
	uint8_t			loadAligned;	//1 = 本轮装载已对准，可以进行抓取
	uint8_t			loadColored;	//1 = 本轮已等到目标色（等色步完成）
	uint8_t			colorHold;		//同色连续帧数（等色判据）
	uint16_t		rxSeen;			//上一帧的 vision1Slot.rxCnt，用于逐帧计数
	uint8_t			retry;			//本段已重试次数（对准与抓取共用该计数）
	uint32_t		tryTick;		//本次尝试开始的 tick；重试时重置

	//卸载 / 取回段
	uint8_t			gripStarted;	//1 = 本段已发起取放动作，等序列走完

	//目标区位置横移（RING / STACK）
	int32_t			segDisp;		//本段位移(mm)：平移段直接取段值，横移段按位号折算
	uint8_t			ringAt;			//车当前正对的位号(1~3)；区域内的段间保持，离开区域即复位

	//装载段
	uint8_t			loadGroup;		//本轮装载读哪一组颜色：NAV_TASK_G_ORDER1 / ORDER2

	//任务码（视觉2 上报的 12 位数字，4 组 × 3 位；组序见 nav.h 的 NAV_TASK_G_*）
	uint8_t			task[NAV_TASK_GROUP_NUM][NAV_TASK_GROUP_LEN];	//各组数字：颜色组为编号 1~6 / 位置组为环号
	uint8_t			taskGot;					//1 = 4 组已全部解析通过

	//任务统计（上屏用）：抓取 = 装载 + 取回，放置 = 卸料 + 码垛
	uint8_t			okGet;			//成功完成的抓取动作数
	uint8_t			okPut;			//成功完成的放置动作数
	uint8_t			errGet;			//判失败的抓取次数
	uint8_t			errPut;			//判失败的放置次数
	uint8_t			cntLoad;		//已计入统计的装载轮数（grip.loadCount 的影子，防重复计数）
	char			code[VISION_TASK_CODE_MAX];	//任务码原文（上屏用，与 task[][] 同时更新）
	uint8_t			codeLen;		//任务码有效字符数；0 = 尚未取到
} Nav_t;

extern Nav_t s_nav;		//导航内部状态

/*---- 内部函数（跨文件共享）----------------------------------------*/

//nav.c
void    nav_Fault(void);				//本段判故障：投事件并中止导航
uint8_t nav_TryTimeout(uint32_t tryMs, uint8_t retryMax);	//单次尝试超时判定与重开
void    nav_SegStart(void);				//开始当前航段：记录起点、推出目标航向
uint8_t nav_SegStep(void);				//执行当前航段，完成时返回 1
uint8_t nav_IsRingFamily(uint8_t type);	//本段是否属"目标区取放"族（位号跨段保持）

//nav_seg.c
uint8_t nav_MotionStep(void);			//平移 / 横移段到位判据与速度给定
uint8_t nav_AlignStep(void);			//对准段：单次时限 + 超时重试
uint8_t nav_LoadStep(void);				//装载段：等色 → 对准 → 复查 → 抓取
uint8_t nav_GripStep(uint8_t slot, uint8_t mode);	//取放段：搬一件

//nav_align.c
uint8_t nav_AlignCtrl(void);			//对准控制律：读视觉1 微调底盘
uint8_t nav_ColorWait(void);			//等色步：等本轮目标色转到取料点
uint8_t nav_ColorMatched(void);			//抓取前的颜色复查
void    nav_TaskOrder(void);			//取并解析任务码
uint8_t nav_TargetColor(void);			//本轮目标颜色
uint8_t nav_TaskRing(uint8_t group, uint8_t idx);	//任务码指定位置
uint8_t nav_StackRing(uint8_t k);		//码垛位：按颜色反查第一批位置

//nav_util.c
float nav_WrapDeg(float deg);			//把角度折到 [-180, 180]
float nav_AbsF(float x);				//取绝对值

#endif
