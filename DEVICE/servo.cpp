/* servo.cpp —— 三路舵机的 Linux 实现（PCA9685 底座）
 * 物理输出走 PCA9685（I2C PWM 驱动芯片），角度↔脉宽线性映射见 servo.h 标定宏。 */
#include "servo.h"
#include "pca9685.h"

/* 每通道量程：CH1/CH2 是 180° 舵机，CH3 是 270° 舵机（车载物料盘） */
static const float    s_degMax[SERVO_NUM] = { SERVO_ANGLE_MAX, SERVO_ANGLE_MAX, SERVO_ANGLE_MAX_CH3 };      /* 各通道角度量程(°) */
static const uint16_t s_usMin[SERVO_NUM]  = { SERVO_PULSE_MIN_US, SERVO_PULSE_MIN_US, SERVO_PULSE_MIN_US_CH3 }; /* 各通道最小脉宽(us) */
static const uint16_t s_usMax[SERVO_NUM]  = { SERVO_PULSE_MAX_US, SERVO_PULSE_MAX_US, SERVO_PULSE_MAX_US_CH3 }; /* 各通道最大脉宽(us) */

static float s_angle[SERVO_NUM];   /* 当前设定角度（非实际反馈） */

/* PCA9685 地址（7 位，默认 0x40）；设备节点与地址均待实测（M2） */
#define PCA9685_DEV     "/dev/i2c-1"
#define PCA9685_ADDR    0x40

/* PWM 周期 20ms = 20000us，对应 4096 计数（12 位） */
#define PWM_PERIOD_US   (20000u)
#define PWM_MAX_COUNT   (4096u)

/**
 * @brief  三路舵机初始化
 * @note   STM32 版配 TIM3 + GPIO 复用；本版只需打开 PCA9685。
 * @author camelliahm
 */
void servo_Init(void)
{
    uint8_t i;

    for (i = 0u; i < SERVO_NUM; i++)
    {
        s_angle[i] = 0.0f;
    }

    /* 打开 PCA9685；失败静默降级（无硬件时角度只记录不输出） */
    (void)pca9685_Init(PCA9685_DEV, PCA9685_ADDR);
}

/**
 * @brief  设置通道脉宽
 * @param  ch 通道号 0~SERVO_NUM-1
 * @param  us 脉宽(us)，越界截断到标定范围
 * @retval true = 已设置；false = 通道号非法
 * @note   脉宽 → PCA9685 off 计数：off = us * 4096 / 20000
 * @author camelliahm
 */
bool servo_SetPulse(uint8_t ch, uint16_t us)
{
    uint16_t off;

    if (ch >= SERVO_NUM)
    {
        return false;
    }

    if (us < s_usMin[ch]) { us = s_usMin[ch]; }
    if (us > s_usMax[ch]) { us = s_usMax[ch]; }

    /* us → 12 位计数（四舍五入） */
    off = (uint16_t)(((uint32_t)us * PWM_MAX_COUNT + PWM_PERIOD_US / 2u) / PWM_PERIOD_US);

    (void)pca9685_SetPWM(ch, 0u, off);

    s_angle[ch] = (float)(us - s_usMin[ch]) * s_degMax[ch]
                / (float)(s_usMax[ch] - s_usMin[ch]);
    return true;
}

/**
 * @brief  设置通道角度
 * @param  ch  通道号 0~SERVO_NUM-1
 * @param  deg 角度(度)，越界截断到 0~该通道量程
 * @retval true = 已设置；false = 通道号非法
 * @note   脉宽与角度按 s_usMin/s_usMax 线性映射，换舵机只改 servo.h 的标定宏
 * @author camelliahm
 */
bool servo_SetAngle(uint8_t ch, float deg)
{
    uint16_t us;

    if (ch >= SERVO_NUM)
    {
        return false;
    }

    if (deg < 0.0f)         { deg = 0.0f; }
    if (deg > s_degMax[ch]) { deg = s_degMax[ch]; }

    us = (uint16_t)((float)s_usMin[ch]
        + (float)(s_usMax[ch] - s_usMin[ch]) * (deg / s_degMax[ch]) + 0.5f);

    (void)servo_SetPulse(ch, us);
    s_angle[ch] = deg;
    return true;
}

/**
 * @brief  读回通道当前设定角度
 * @note   读的是设定值，不是舵机实际角度（无反馈线）
 * @author camelliahm
 */
float servo_GetAngle(uint8_t ch)
{
    return (ch < SERVO_NUM) ? s_angle[ch] : 0.0f;
}

/**
 * @brief  关闭某通道 PWM 输出
 * @note   只影响该通道，其它通道照常输出；重新 SetPulse/SetAngle 不会自动恢复输出
 * @author camelliahm
 */
void servo_Disable(uint8_t ch)
{
    if (ch >= SERVO_NUM)
    {
        return;
    }

    (void)pca9685_Disable(ch);
}
