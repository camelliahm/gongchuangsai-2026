#ifndef __SERVO_H
#define __SERVO_H

#include "types.h"

//舵机 PWM 层：3 路 50Hz PWM（TIM3 CH1/CH2/CH3 = PC6/PC7/PC8），详见 README.md

#define SERVO_NUM			(3u)		//舵机路数

/*---- 脉宽 ↔ 角度标定：按通道给，不同型号舵机行程不同 ----*/
#define SERVO_FREQ_HZ		(50u)		//舵机标准频率
#define SERVO_PULSE_MIN_US	(500u)		//CH1/CH2 最小角度对应脉宽
#define SERVO_PULSE_MAX_US	(2500u)		//CH1/CH2 最大角度对应脉宽
#define SERVO_ANGLE_MAX		(180.0f)	//CH1/CH2 脉宽范围对应的角度

//CH3（通道索引 2，PC8）单独标定：车载物料盘用 270° 舵机，量程与另两路不同
#define SERVO_PULSE_MIN_US_CH3	(500u)		//CH3 最小角度对应脉宽(us)  ⚠ 待实测
#define SERVO_PULSE_MAX_US_CH3	(2500u)		//CH3 最大角度对应脉宽(us)  ⚠ 待实测
#define SERVO_ANGLE_MAX_CH3		(270.0f)	//CH3 脉宽范围对应的角度

/*---- 时基：TIM3 挂 APB1，定时器时钟 84MHz ----*/
//PSC = 84-1 → 计数 1MHz（1 格 = 1us）；ARR = 1000000/50-1 → 周期 20ms
#define SERVO_TIM_PSC		(84u - 1u)	//预分频：84 MHz → 1 MHz 计数（1 格 = 1 us）
#define SERVO_TIM_ARR		(1000000u / SERVO_FREQ_HZ - 1u)	//重装值：1 MHz / 50 Hz − 1 → 周期 20 ms

//初始化（时钟、GPIO 复用、PWM 输出）
void servo_Init(void);

//设置角度（度，0~SERVO_ANGLE_MAX，越界截断）；返回 false = 通道号非法
bool servo_SetAngle(uint8_t ch, float deg);

//直接设置脉宽（us），用于标定行程或驱动非标准舵机
bool servo_SetPulse(uint8_t ch, uint16_t us);

//读回当前角度（读的是设定值，非实际反馈）
float servo_GetAngle(uint8_t ch);

//关闭该路 PWM 输出（上电瞬间防抖 / 省电；不影响其它通道）
void servo_Disable(uint8_t ch);

#endif
