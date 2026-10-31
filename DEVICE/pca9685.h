#ifndef __PCA9685_H
#define __PCA9685_H

#include <stdint.h>
#include <stdbool.h>

/* PCA9685 底座：16 通道 12 位 PWM 驱动芯片（I2C）。
 * 用 Linux 内核 I2C 用户态接口（ioctl I2C_SLAVE）访问，替换 STM32 的 TIM3 PWM。 */

/* 初始化：打开指定 I2C 总线设备并配好芯片（内部时钟 + 自动递增），返回是否成功 */
bool pca9685_Init(const char *dev, uint8_t addr);

/* 设置某通道 PWM 输出（on/off 计数，12 位：0~4095） */
bool pca9685_SetPWM(uint8_t ch, uint16_t on, uint16_t off);

/* 关闭某通道输出（off 全 1 = 常低，等效舵机断电） */
bool pca9685_Disable(uint8_t ch);

/* 关闭并释放 */
void pca9685_Close(void);

#endif
