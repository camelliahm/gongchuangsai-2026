#include "pca9685.h"

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

/* PCA9685 寄存器 */
#define PCA9685_MODE1        0x00   /* 模式 1 */
#define PCA9685_PRESCALE     0xFE   /* 预分频 */
#define PCA9685_LED0_ON_L    0x06   /* 通道 0 起始寄存器（每通道 4 字节） */

/* MODE1 位 */
#define MODE1_SLEEP          0x10   /* 睡眠位 */
#define MODE1_AI             0x20   /* 自动递增位 */
#define MODE1_RESTART        0x80   /* 重启位 */

/* 关闭通道时 off 用 0x1000（第 13 位 = 常低） */
#define PCA9685_OFF_FULL     0x1000

static int s_fd = -1;   /* I2C 设备描述符，-1 = 未打开 */

/**
 * @brief  写单个寄存器字节
 * @author camelliahm
 */
static bool pca9685_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };

    if (s_fd < 0)
    {
        return false;
    }

    return (write(s_fd, buf, 2) == 2);
}

/**
 * @brief  初始化：打开 I2C 设备并配好芯片
 * @param  dev  设备节点（如 "/dev/i2c-1"）
 * @param  addr 芯片地址（7 位，默认 0x40）
 * @note   内部 25MHz 时钟，预分频后得到约 50Hz 的 PWM 周期（配合 servo 层换算）
 * @author camelliahm
 */
bool pca9685_Init(const char *dev, uint8_t addr)
{
    int fd;

    fd = open(dev, O_RDWR);
    if (fd < 0)
    {
        perror("pca9685: open");
        return false;
    }

    if (ioctl(fd, I2C_SLAVE, addr) < 0)
    {
        perror("pca9685: ioctl(I2C_SLAVE)");
        close(fd);
        return false;
    }

    s_fd = fd;

    /* 复位：先睡眠再唤醒，并开启自动递增 */
    if (!pca9685_WriteReg(PCA9685_MODE1, MODE1_SLEEP | MODE1_AI))
    {
        pca9685_Close();
        return false;
    }

    /* 预分频：目标 50Hz。内部 25MHz → prescale = 25e6 / (4096*50) - 1 ≈ 121 */
    if (!pca9685_WriteReg(PCA9685_PRESCALE, 121u))
    {
        pca9685_Close();
        return false;
    }

    if (!pca9685_WriteReg(PCA9685_MODE1, MODE1_AI))
    {
        pca9685_Close();
        return false;
    }

    return true;
}

/**
 * @brief  设置某通道 PWM
 * @param  ch  通道号 0~15
 * @param  on  上升沿计数（0~4095）
 * @param  off 下降沿计数（0~4095）
 * @author camelliahm
 */
bool pca9685_SetPWM(uint8_t ch, uint16_t on, uint16_t off)
{
    uint8_t reg;
    uint8_t buf[5];

    if ((s_fd < 0) || (ch >= 16u))
    {
        return false;
    }

    reg = (uint8_t)(PCA9685_LED0_ON_L + (ch * 4u));

    buf[0] = reg;
    buf[1] = (uint8_t)(on & 0xFF);
    buf[2] = (uint8_t)(on >> 8);
    buf[3] = (uint8_t)(off & 0xFF);
    buf[4] = (uint8_t)(off >> 8);

    return (write(s_fd, buf, 5) == 5);
}

/**
 * @brief  关闭某通道（off 全 1 = 常低，等效舵机断电）
 * @author camelliahm
 */
bool pca9685_Disable(uint8_t ch)
{
    return pca9685_SetPWM(ch, 0u, PCA9685_OFF_FULL);
}

/**
 * @brief  关闭并释放
 * @author camelliahm
 */
void pca9685_Close(void)
{
    if (s_fd >= 0)
    {
        close(s_fd);
        s_fd = -1;
    }
}
