#include "imu_filter.h"
#include "bmi088.h"
#include "math.h"
#include "FreeRTOS.h"
#include "task.h"

/* IMU 滤波：一阶低通去噪 + 互补滤波解算姿态。
 * 陀螺仪积分得短期姿态（噪声累积会漂），加速度计反推重力方向得长期姿态（稳但怕振动），
 * 按 β 加权融合，抑制漂移又滤掉振动。 */

static float s_lpf_gyr[3];	//陀螺仪低通后的角速度
static float s_lpf_acc[3];	//加速度计低通后的加速度
static float s_euler[3];	//当前姿态（rad）：yaw/pitch/roll
static uint32_t s_lastTick;	//上次更新的 tick，用于算 dt
static uint8_t s_dtValid;	//1 = tick 基准已建立（首次 Update 后置位）

/**
 * @brief  复位滤波状态
 * @note   不取 tick（调度器启动前可安全调用）；tick 基准在首次 Update 建立
 * @author camelliahm
 */
void imu_filter_Init(void)
{
	s_lpf_gyr[0] = 0.0f; s_lpf_gyr[1] = 0.0f; s_lpf_gyr[2] = 0.0f;
	s_lpf_acc[0] = 0.0f; s_lpf_acc[1] = 0.0f; s_lpf_acc[2] = 0.0f;
	s_euler[0] = 0.0f; s_euler[1] = 0.0f; s_euler[2] = 0.0f;
	s_lastTick = 0u;
	s_dtValid = 0u;
}

/**
 * @brief  喂入原始数据并更新姿态
 * @param  gyr   陀螺仪角速度（rad/s，3 轴）
 * @param  acc   加速度计比力（m/s²，3 轴）
 * @param  euler 输出欧拉角（rad，yaw/pitch/roll）
 * @note   输入由 bmi088_ReadGyrRad / ReadAccMss 提供；dt 用 tick 差值自算，自适应任务抢占抖动
 * @author camelliahm
 */
void imu_filter_Update(float gyr[3], float acc[3], float euler[3])
{
	float norm, accPitch, accRoll, dt;
	float ax, ay, az;
	uint32_t now;

	/*---- 0. tick 差值算 dt（秒）----*/
	now = xTaskGetTickCount();
	if (s_dtValid == 0u)
	{
		s_lastTick = now;		//首次调用：只建立 tick 基准，不算 dt
		s_dtValid = 1u;
		dt = 0.005f;
	}
	else
	{
		dt = (float)(now - s_lastTick) / (float)configTICK_RATE_HZ;
		s_lastTick = now;
		if (dt <= 0.0f || dt > 0.1f) { dt = 0.005f; }	//异常钳到标称 5ms
	}

	/*---- 1. 一阶低通去噪 ----*/
	s_lpf_gyr[0] += IMU_LPF_ALPHA_GYR * (gyr[0] - s_lpf_gyr[0]);
	s_lpf_gyr[1] += IMU_LPF_ALPHA_GYR * (gyr[1] - s_lpf_gyr[1]);
	s_lpf_gyr[2] += IMU_LPF_ALPHA_GYR * (gyr[2] - s_lpf_gyr[2]);
	s_lpf_acc[0] += IMU_LPF_ALPHA_ACC * (acc[0] - s_lpf_acc[0]);
	s_lpf_acc[1] += IMU_LPF_ALPHA_ACC * (acc[1] - s_lpf_acc[1]);
	s_lpf_acc[2] += IMU_LPF_ALPHA_ACC * (acc[2] - s_lpf_acc[2]);

	/*---- 2. 陀螺仪积分（短期姿态）----*/
	s_euler[0] += s_lpf_gyr[2] * dt;	//yaw 绕 z 轴
	s_euler[1] += s_lpf_gyr[0] * dt;	//pitch 绕 x 轴
	s_euler[2] += s_lpf_gyr[1] * dt;	//roll 绕 y 轴

	/*---- 3. 加速度计反推重力方向（长期姿态）----*/
	ax = s_lpf_acc[0];
	ay = s_lpf_acc[1];
	az = s_lpf_acc[2];
	norm = sqrtf(ax * ax + ay * ay + az * az);
	if (norm > 1e-6f)		//防除零
	{
		ax /= norm;
		ay /= norm;
		az /= norm;
	}
	accPitch = atan2f(ax, sqrtf(ay * ay + az * az));	//绕 x 轴倾角
	accRoll  = atan2f(-ay, az);							//绕 y 轴倾角

	/*---- 4. 互补融合：pitch/roll 用加速度计校正，yaw 仅靠陀螺仪 ----*/
	s_euler[1] = (1.0f - IMU_CF_BETA) * s_euler[1] + IMU_CF_BETA * accPitch;
	s_euler[2] = (1.0f - IMU_CF_BETA) * s_euler[2] + IMU_CF_BETA * accRoll;
	//yaw 无法靠重力校正（重力沿 z 轴不提供航向），保持陀螺仪积分

	euler[0] = s_euler[0];
	euler[1] = s_euler[1];
	euler[2] = s_euler[2];
}

/**
 * @brief  周期更新：读 BMI088 → 低通 + 互补融合
 * @note   由控制环每周期调用；内部读原始数据并换算物理量后喂入 Update
 * @author camelliahm
 */
void imu_filter_Step(void)
{
	float gyr[3], acc[3], euler[3];

	bmi088_ReadGyrRad(gyr);
	bmi088_ReadAccMss(acc);
	imu_filter_Update(gyr, acc, euler);
}

/**
 * @brief  读回当前姿态
 * @param  euler 输出欧拉角（rad，yaw/pitch/roll）
 * @note   返回内部暂存的姿态，不触发新采样
 * @author camelliahm
 */
void imu_filter_GetEuler(float euler[3])
{
	euler[0] = s_euler[0];
	euler[1] = s_euler[1];
	euler[2] = s_euler[2];
}
