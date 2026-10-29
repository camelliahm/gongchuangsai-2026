#ifndef __INPUT_H
#define __INPUT_H

#include "contract.h"

/* 输入汇聚层：三条串口的硬件配置、DMA 与字节管道 + 摁键（板内自举）；
 * 视觉帧解析见 vision.c，串口屏协议见 screen.c。分层与扩展指引见工程根目录 README.md */

/* USART1：PA9(TX)/PA10(RX)，APB2，DMA2 Stream5 Ch4 —— 视觉模块 1（物块颜色 + 对准相对距离）
 * USART3：PB10(TX)/PB11(RX)，APB1，DMA1 Stream3 Ch4 —— 视觉模块 2（障碍物 + 任务号）
 * USART2：PA2(TX)/PA3(RX)，APB1，DMA1 Stream5 Ch4 —— 串口屏（TJC 协议见 screen.h）
 * ⚠ 不用 USART6：它的 PC6/PC7 与舵机 PWM 是同一对引脚 */
typedef enum {
	INPUT_VISION1 = 0,	//USART1 → 视觉模块 1
	INPUT_VISION2,		//USART3 → 视觉模块 2
	INPUT_SCREEN,		//USART2 → 串口屏（TJC）
	INPUT_NUM			//通道数量
} InputCh_t;

#define INPUT_BAUD			(115200u)	//三通道同波特率；串口屏若需单独取值再拆成数组
#define INPUT_RX_BUF		(256u)		//每通道 DMA 接收缓冲字节数
#define INPUT_RING_SIZE		(512u)		//每通道环形缓冲字节数（2 的幂）
#define INPUT_TIMEOUT_MS	(500u)		//超过此时长未收到字节判离线

/* 每通道接收状态（字节级）。本层只搬字节与刷新计数，
 * 帧解析在各自的协议模块里（视觉见 vision.c，串口屏见 screen.c） */
typedef struct {
	uint8_t	 online;			//1 = 最后收到字节未超时
	uint32_t lastTick;			//最后收到字节的 tick
	uint16_t rxCnt;				//累计到达字节数
	uint16_t lockCnt;			//环形缓冲满导致的丢弃字节数
} InputSlot_t;

//初始化三条串口（时钟/GPIO/DMA/NVIC）并复位通道状态；返回后接收立即开始
void input_Init(void);

//周期处理：刷新通道状态 + 摁键扫描（含自启动倒计时）；由 contract_ControlStep() 每周期调用
void input_Step(void);

//取走原始字节：从环形缓冲拷最多 max 字节，返回实际拷贝数
uint16_t input_Read(InputCh_t ch, uint8_t *dst, uint16_t max);

//阻塞式发送：逐字节等 TXE，返回实际发出的字节数
uint16_t input_Write(InputCh_t ch, const uint8_t *src, uint16_t len);

//只读槽位（诊断）
const InputSlot_t *input_GetSlot(InputCh_t ch);

/*---- 摁键（自启动）：按下后延时 KEY_START_DELAY_MS 再投 EV_ENABLE + EV_START ----*/
//引脚、接线与状态约束见 README.md「摁键自启动（板内自举）」；换引脚只改下面 KEY_PORT / KEY_CLK / KEY_PIN 三行
#define KEY_PORT			GPIOA					//摁键所在端口，改端口须同步 KEY_CLK
#define KEY_CLK				RCC_AHB1Periph_GPIOA	//KEY_PORT 对应的 AHB1 时钟
#define KEY_PIN				GPIO_Pin_0				//摁键引脚（PA0，当前工程未占用）
#define KEY_ACTIVE_LOW		(1u)					//1 = 按下为低（内部上拉）；0 = 按下为高（内部下拉）
#define KEY_PUPD			(KEY_ACTIVE_LOW ? GPIO_PuPd_UP : GPIO_PuPd_DOWN)	//随有效电平联动，勿单独改
#define KEY_DEBOUNCE_CNT	(4u)					//消抖：连续同电平的扫描次数（控制环 5 ms → 20 ms）
#define KEY_START_DELAY_MS	(3000u)					//按下到自启动的延时(ms)

#endif
