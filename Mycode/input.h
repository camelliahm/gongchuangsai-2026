#ifndef __INPUT_H
#define __INPUT_H

#include "chassis.h"

/* 输入汇聚层：两路只收串口（USART1 = 视觉，USART2 = 二维码），详见 README.md */

/* USART1：PA9(TX)/PA10(RX)，挂 APB2，DMA2 Stream5 Ch4 */
/* USART2：PA2(TX)/PA3(RX)，挂 APB1，DMA1 Stream5 Ch4 */
/* ⚠ 不能用 USART6：它的 PC6/PC7 与舵机 PWM 是同一对引脚 */

typedef enum {
	INPUT_VISION = 0,	/* USART1 */
	INPUT_QRCODE,		/* USART2 */
	INPUT_NUM
} InputCh_t;

#define INPUT_LINE_MAX		(64u)	/* 单行缓冲（含结尾 '\0'） */
#define INPUT_RX_BUF		(256u)	/* DMA 接收缓冲 */
#define INPUT_RING_SIZE		(512u)	/* 每通道环形缓冲（2 的幂） */
#define INPUT_TIMEOUT_MS	(500u)	/* 超过此时长无完整帧判离线 */

/* 每通道的输入槽位，形状同 ChassisMotor_t：值 + 新鲜度 + 丢帧计数 */
typedef struct {
	uint8_t	 online;				/* 1 = 最后一条完整帧未超时 */
	uint32_t lastTick;				/* 最后收到完整帧的 tick */
	uint16_t rxCnt;					/* 累计完整帧数 */
	uint16_t dropCnt;				/* 环形缓冲溢出丢弃的字节数 */
	char	 line[INPUT_LINE_MAX];	/* 最近一条完整帧（ASCII，'\0' 结尾） */
	uint16_t lineLen;
	uint8_t	 fresh;					/* 1 = 有新帧待上层取走 */
} InputSlot_t;

/* 初始化（时钟、GPIO、DMA、USART、NVIC；接收随即开始） */
void input_Init(void);

/* 周期处理：拆行 + 刷新 online。暂未接入控制环 */
void input_Step(void);

/* 只读槽位（诊断 / 直接取用） */
const InputSlot_t *input_GetSlot(InputCh_t ch);

/* 取走一行：有未读帧时拷进 dst 并清 fresh，返回 true */
bool input_TakeLine(InputCh_t ch, char *dst, uint16_t size);

/* 丢弃未读帧（不取用，只清 fresh） */
void input_ClearLine(InputCh_t ch);

#endif
