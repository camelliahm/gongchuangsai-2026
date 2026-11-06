#ifndef __IMU_FILTER_H
#define __IMU_FILTER_H

#include "stm32f4xx.h"
#include "FreeRTOS.h"

//IMU 滤波层（一阶低通 + 互补滤波），纯算法不碰硬件，输入由 bmi088 读出后喂入，详见 README.md

/*---- 低通滤波系数（一阶 IIR，α 越大越灵敏、越不平滑）----*/
#define IMU_LPF_ALPHA_GYR	(0.15f)		//陀螺仪低通系数
#define IMU_LPF_ALPHA_ACC	(0.10f)		//加速度计低通系数

/*---- 互补滤波增益（β 越大越信任加速度计，越小越信任陀螺仪）----*/
#define IMU_CF_BETA			(0.02f)		//互补滤波增益，典型 0.02~0.05

//复位滤波状态（清历史值与 tick 基准），可安全在调度器启动前调用一次
void imu_filter_Init(void);

//周期更新：读 BMI088 → 低通 + 互补融合，更新内部姿态；由控制环每周期调用
void imu_filter_Step(void);

//读回当前姿态（rad，yaw/pitch/roll）；无数据时返回上次结果
void imu_filter_GetEuler(float euler[3]);

//喂入角速度（rad/s）与加速度（m/s²），低通 + 互补融合，输出欧拉角（rad，yaw/pitch/roll）
void imu_filter_Update(float gyr[3], float acc[3], float euler[3]);

#endif
