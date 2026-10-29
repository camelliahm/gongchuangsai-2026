#ifndef __CHASSIS_DRIVE_H
#define __CHASSIS_DRIVE_H

#include "contract.h"

//底盘驱动层（四轮麦轮）：驱动器动作 / 反馈看门狗 / 停稳判断 / 底盘运动学，详见 README.md

/*---- 控制环参数 ----*/
#define CHASSIS_CTRL_PERIOD_MS		(5u)	//控制环周期（须与 lift_axis.h 的 LIFT_CTRL_PERIOD_MS 一致）
#define CHASSIS_FB_PERIOD_MS		(20u)	//驱动器定时上报周期
#define CHASSIS_FB_TIMEOUT_MS		(100u)	//超过则判掉线
#define CHASSIS_STOP_VEL_TH			(30)	//停稳判据：转速阈值(RPM)
#define CHASSIS_STOP_HOLD_MS		(200u)	//停稳判据：持续时长

/*---- 麦轮运动学参数：⚠ 以下数值均为占位，必须按实车实测后修正 ----*/
//WHEEL_DIAMETER 轮径 / GEAR_RATIO 减速比 / WHEEL_BASE 轴距 / TRACK_WIDTH 轮距 / LX·LY 轮心到车体中心
#define CHASSIS_WHEEL_DIAMETER_MM	(60.0f)	//麦轮直径(mm)：⚠ 占位值
#define CHASSIS_GEAR_RATIO			(1.0f)		//电机到车轮的减速比：⚠ 占位值
#define CHASSIS_WHEEL_BASE_MM		(200.0f)	//轴距：前后轮中心距(mm)：⚠ 占位值
#define CHASSIS_TRACK_WIDTH_MM		(200.0f)	//轮距：左右轮中心距(mm)：⚠ 占位值
#define CHASSIS_LX_MM				(100.0f)	//轴距/2
#define CHASSIS_LY_MM				(100.0f)	//轮距/2

/*---- 速度指令下发参数 ----*/
#define CHASSIS_DRIVE_ACC_DEFAULT	(20u)	//速度模式加速度(0-255，0=直接启动)
#define CHASSIS_DRIVE_VEL_MAX_RPM	(500u)	//单轮速度上限(RPM)，防止指令打爆电机

/*---- 单台电机的反馈状态 ----*/
typedef struct {
	uint8_t	 online;			//1 = 反馈帧未超时
	uint8_t	 flag;				//S_FLAG 状态标志位
	int16_t	 vel;				//S_VEL 实时转速(RPM)
	int32_t	 pos;				//S_CPOS 实时位置(脉冲)
	uint32_t lastTick;			//最后一次收到该电机帧的 tick
	uint16_t rxCount;			//累计收帧数（诊断用）
} ChassisMotor_t;

/*---- 底盘运动学状态（里程计推算结果）----*/
typedef struct {
	float	 odomX;				//里程计累计位移(mm)
	float	 odomY;				//里程计累计位移(mm)
	float	 odomYaw;			//偏航角(度)，归一化到 [-180,180]：仅供内部 sin/cos 近似与显示
	float	 odomYawCont;		//偏航角(度)，连续累加不归一化：航向误差一律读它
	int16_t	 wheelTarget[CHASSIS_MOTOR_NUM];	//四轮目标转速(RPM，带符号)
} ChassisDrive_t;

/* 航向坐标域约定：跨 ±180 的两处误差（导航到位判据、姿态环 PID）必须在
   odomYawCont 上计算。odomYaw 归一化副本只用于内部的三角函数近似。 */

extern __IO ChassisDrive_t chassisDrive;
extern __IO ChassisMotor_t chassisMotor[CHASSIS_MOTOR_NUM];

/*---- 初始化 ----*/
//里程计清零、姿态环 PID 初始化、电机槽位复位；在 contract_Init() 中调用
void chassis_DriveInit(void);

/*---- 四台电机的动作与上报 ----*/
//向四台驱动器广播无参数动作：0=断使能 1=使能 2=立即停止
void chassis_DriveBroadcast(uint8_t action);

/* 向四台驱动器下发零速软停（速度模式，驱动器按 acc 斜率自行减速）。
   与 chassis_DriveBroadcast(2) 的 Stop_Now 急停区分；BRAKING 期间由控制环周期调用 */
void chassis_BrakeWheels(void);

//打开四轮定时上报（S_VEL），进入 READY 时调用
void chassis_EnableFeedback(void);

//把所有电机心跳重置为"现在"并清在线标志，进入 READY 时调用
void chassis_ResetHeartbeat(void);

//清全部在线标志，进入 IDLE 时调用
void chassis_ClearOnline(void);

/*---- 反馈处理与周期任务 ----*/
//解析四轮反馈帧，idx = 0~3
void chassis_HandleFeedback(uint8_t idx, const CAN_Frame_t *f);

//四轮掉线看门狗（跳变沿投 EV_FAULT）
void chassis_DriveWatchdog(void);

//停稳判断：BRAKING 下四轮都静止 → 投 EV_ARRIVED
void chassis_CheckBraking(void);

//底盘控制环：仅 RUNNING 下生效。里程计 → 逆运动学 → 姿态环 → 下发
void chassis_DriveStep(void);

#endif
