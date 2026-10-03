#ifndef __OLED_H
#define __OLED_H

#include "stm32f4xx.h"

/* OLED 显示层：SSD1306 128x64 四针 I2C，软件模拟时序（不占用硬件 I2C、无需延时），详见 README.md */

/*---- 软件 I2C 引脚：默认 PB6/PB7（标准 I2C1 位置，接线方便）----*/
/* ⚠ 避开 PB8/PB9，那是 CAN1；也不要用 PC6/PC7/PC8，那是舵机 PWM */
#define OLED_SCL_PORT		GPIOB
#define OLED_SCL_PIN		GPIO_Pin_6
#define OLED_SDA_PORT		GPIOB
#define OLED_SDA_PIN		GPIO_Pin_7

/*---- 面板参数 ----*/
#define OLED_WIDTH			(128u)
#define OLED_PAGE_NUM		(8u)	/* 64 / 8：SSD1306 一页 = 8 个像素行 */
#define OLED_LINE_NUM		(4u)	/* 8x16 字体下可显示 4 行 */
#define OLED_COL_NUM		(16u)	/* 每行 16 个字符 */

#define OLED_I2C_ADDR		(0x78u)	/* SSD1306 从机地址（7 位 0x3C 左移 1 位） */

#define OLED_REFRESH_MS		(200u)	/* 状态页刷新间隔：OLED 只是给人看，不必每周期刷 */

/* 初始化并清屏 */
void oled_Init(void);

/* 清显存（不立即刷新面板） */
void oled_Clear(void);

/* 把整块显存刷到面板 */
void oled_Refresh(void);

/* 写显存：line = 1~4，col = 1~16 */
void oled_ShowString(uint8_t line, uint8_t col, const char *str);
void oled_ShowNum(uint8_t line, uint8_t col, int32_t num, uint8_t len);

/* 组装整车状态页并刷新（读 chassis / chassisDrive / chassisMotor / lift） */
void oled_ShowStatus(void);

/* 周期调用：限速刷新状态页。暂未接入控制环 */
void oled_Step(void);

#endif
