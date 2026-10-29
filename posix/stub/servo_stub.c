/* servo_stub.c —— POSIX 模拟层：servo.c 空壳
 * 真实实现见 DEVICE/servo.c（TIM3 三路 PWM），本机不编译它。
 * 保留角度设定值语义，让夹持机构序列照常跑逻辑，只是不驱动真实舵机。 */
#include "servo.h"

static float s_angle[SERVO_NUM];    /* 各通道当前设定角度（度） */

void servo_Init(void)
{
    uint8_t i;

    for (i = 0u; i < SERVO_NUM; i++)
    {
        s_angle[i] = 0.0f;
    }
}

bool servo_SetAngle(uint8_t ch, float deg)
{
    if (ch >= SERVO_NUM)
    {
        return false;
    }

    if (deg < 0.0f)
    {
        deg = 0.0f;
    }

    float maxAng = (ch == 2u) ? SERVO_ANGLE_MAX_CH3 : SERVO_ANGLE_MAX;

    if (deg > maxAng)
    {
        deg = maxAng;
    }

    s_angle[ch] = deg;
    return true;
}

bool servo_SetPulse(uint8_t ch, uint16_t us)
{
    (void)ch;
    (void)us;
    return false;
}

float servo_GetAngle(uint8_t ch)
{
    return (ch < SERVO_NUM) ? s_angle[ch] : 0.0f;
}

void servo_Disable(uint8_t ch)
{
    (void)ch;
}
