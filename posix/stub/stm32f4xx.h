/* stm32f4xx.h —— POSIX 模拟层 stub
 * 仅供 posix/ 目录的本机模拟编译使用，替代真实 CMSIS 头，
 * 让 board.h / can.h / input.h / servo.h / oled.h / delay.h 这条 include 链在本机编过。
 * 只定义头文件层用到的少量符号；真正的硬件寄存器访问在真实 STM32 构建里，
 * 本目录不编译 board.c / can.c / input.c / servo.c / oled.c 等硬件实现。 */
#ifndef __STM32F4XX_H
#define __STM32F4XX_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/*---- CMSIS 访问限定宏 ---------------------------------------*/
#define __I     volatile const   /* 只读 */
#define __O     volatile         /* 只写 */
#define __IO    volatile         /* 读写 */
#define __WEAK  __attribute__((weak))

/*---- 内核时钟（FreeRTOSConfig.h 的 configCPU_CLOCK_HZ 用到）----*/
#define SystemCoreClock      (168000000UL)

/*---- 枚举 / 标志类型（ST 标准外设库风格，仅占位）------------*/
typedef enum { RESET = 0, SET = !RESET } FlagStatus, ITStatus;
typedef enum { DISABLE = 0, ENABLE = !DISABLE } FunctionalState;
typedef enum { ERROR = 0, SUCCESS = !ERROR } ErrorStatus;

/*---- 外设结构占位（can.h 的 CAN_t 结构体成员用到）-----------*/
typedef struct { uint32_t ExtId; uint8_t RTR; uint8_t DLC; uint8_t Data[8]; } CanRxMsg;
typedef struct { uint32_t ExtId; uint8_t RTR; uint8_t DLC; uint8_t Data[8]; } CanTxMsg;

/*---- 外设结构占位（头文件防御性引用）------------------------*/
typedef struct { uint32_t R; } GPIO_TypeDef;
typedef struct { uint32_t R; } GPIO_InitTypeDef;
typedef struct { uint32_t R; } USART_TypeDef;
typedef struct { uint32_t R; } TIM_TypeDef;

/*---- GPIO 端口与引脚（input.h / oled.h 的宏用到）-------------*/
#define GPIOA                   ((GPIO_TypeDef *)0x00000000UL)
#define GPIOB                   ((GPIO_TypeDef *)0x00000004UL)
#define GPIOC                   ((GPIO_TypeDef *)0x00000008UL)

#define GPIO_Pin_0              ((uint16_t)0x0001)
#define GPIO_Pin_2              ((uint16_t)0x0004)
#define GPIO_Pin_3              ((uint16_t)0x0008)
#define GPIO_Pin_6              ((uint16_t)0x0040)
#define GPIO_Pin_7              ((uint16_t)0x0080)

#define GPIO_PuPd_UP            ((uint32_t)0x0001)
#define GPIO_PuPd_DOWN          ((uint32_t)0x0002)

/*---- 时钟外设（input.h 的 KEY_CLK 宏用到）-------------------*/
#define RCC_AHB1Periph_GPIOA    ((uint32_t)0x0001)

#endif /* __STM32F4XX_H */
