#ifndef __INPUT_H
#define __INPUT_H

#include "chassis.h"

/* 输入汇聚层：三条串口的硬件配置与字节管道；串口屏协议实现见 screen.c；
 * 分层与扩展指引见工程根目录 README.md */

/* USART1：PA9(TX)/PA10(RX)，APB2，DMA2 Stream5 Ch4 —— 视觉模块 1（物块颜色）
 * USART3：PB10(TX)/PB11(RX)，APB1，DMA1 Stream3 Ch4 —— 视觉模块 2（障碍物 + 任务号）
 * USART2：PA2(TX)/PA3(RX)，APB1，DMA1 Stream5 Ch4 —— 串口屏（TJC 协议见 screen.h）
 * ⚠ 不用 USART6：它的 PC6/PC7 与舵机 PWM 是同一对引脚 */
typedef enum {
	INPUT_VISION1 = 0,	/* USART1 → 视觉模块 1 */
	INPUT_VISION2,		/* USART3 → 视觉模块 2 */
	INPUT_SCREEN,		/* USART2 → 串口屏（TJC） */
	INPUT_NUM
} InputCh_t;

#define INPUT_BAUD			(115200u)	/* 三通道同波特率；串口屏若需单独取值再拆成数组 */
#define INPUT_RX_BUF		(256u)		/* 每通道 DMA 接收缓冲 */
#define INPUT_RING_SIZE		(512u)		/* 每通道环形缓冲（2 的幂） */
#define INPUT_TIMEOUT_MS	(500u)		/* 超过此时长未收到字节判离线 */

/* 每通道接收状态（字节级）。⚠ 本模块只搬字节不做帧解析：
 * 串口屏的解析在 screen.c，两路视觉的帧格式仍留空 */
typedef struct {
	uint8_t	 online;			/* 1 = 最后收到字节未超时 */
	uint32_t lastTick;			/* 最后收到字节的 tick */
	uint16_t rxCnt;				/* 累计到达字节数 */
	uint16_t lockCnt;			/* 环形缓冲满导致的丢弃字节数 */
} InputSlot_t;

/* 初始化三条串口（时钟/GPIO/DMA/NVIC）并复位上层槽位；返回后接收立即开始 */
void input_Init(void);

/* 周期处理：刷新通道状态。暂未接入控制环 */
void input_Step(void);

/* 取走原始字节：从环形缓冲拷最多 max 字节，返回实际拷贝数 */
uint16_t input_Read(InputCh_t ch, uint8_t *dst, uint16_t max);

/* 阻塞式发送：逐字节等 TXE，返回实际发出的字节数 */
uint16_t input_Write(InputCh_t ch, const uint8_t *src, uint16_t len);

/* 只读槽位（诊断） */
const InputSlot_t *input_GetSlot(InputCh_t ch);

/*---- 视觉模块 2 帧槽位（内容留空，见 README.md） --------------------*/

/* ⚠ 帧头/帧尾/校验/字段量纲全是占位约定，解析尚未实现；定稿后只改本段宏与 input.c 的解析 */
#define VISION_FRAME_HEAD		(0xA5u)		/* ⚠ 占位 */
#define VISION_FRAME_TAIL		(0x5Au)		/* ⚠ 占位 */
#define VISION_OBST_MAX			(8u)		/* 一帧障碍物条数上限，改大需核对 VISION_RX_BUF */
#define VISION_FRAME_TIMEOUT_MS	(500u)		/* 超过此时长无完整帧即判离线 */

typedef struct {
	int16_t x;				/* 障碍中心横向(车体坐标, mm)  ⚠ 待定 */
	int16_t y;				/* 障碍中心纵向(车体坐标, mm)  ⚠ 待定 */
	int16_t width;			/* 障碍宽度(mm)                ⚠ 待定 */
	uint8_t confidence;		/* 置信度 0~100                ⚠ 待定 */
} VisionObst_t;

/* 任务号（二维码）：视觉模块 2 已解析成整数后上报；⚠ 帧内承载位置待帧格式确定 */
#define VISION_TASK_ID_NONE		(0xFFu)		/* 无任务信息哨兵 */
typedef struct {
	uint8_t taskId;			/* 任务号；VISION_TASK_ID_NONE = 本帧无任务 */
} VisionTask_t;

/* 视觉模块 2 有效识别距离(mm)，实测 50 cm；⚠ 0 = 未知（哨兵），调用方不得据此放行速度 */
#define VISION_RANGE_MM			(500)

/* 视场角(度) / 安装高度(mm) / 俯角(度)：⚠ 均留空，影响障碍物反投影与盲区判定 */
#define VISION_FOV_DEG			(0.0f)		/* ⚠ 留空：0 = 未知 */
#define VISION_MOUNT_H_MM		(0.0f)		/* ⚠ 留空：0 = 未知 */
#define VISION_MOUNT_PITCH_DEG	(0.0f)		/* ⚠ 留空：0 = 未知 */

typedef struct {
	uint8_t  online;					/* 1 = 最后完整帧未超时 */
	uint8_t  fresh;						/* 1 = 有新帧待上层取走 */
	uint8_t  count;						/* 本帧障碍物条数 */
	uint8_t  seq;						/* 帧序号，用于丢帧检测 */
	uint32_t lastTick;					/* 最后收到完整帧的 tick */
	uint16_t rxCnt;						/* 累计完整帧数 */
	uint16_t errCnt;					/* 校验/长度错误帧数 */
	VisionObst_t obst[VISION_OBST_MAX];	/* 障碍物列表（本帧快照） */
	VisionTask_t task;					/* 任务号（二维码） */
} VisionSlot_t;

extern __IO VisionSlot_t visionSlot;

/* 周期处理：刷新视觉槽位 online；解析未实现期间恒为离线 */
void vision_Step(void);

/* 取走一帧：返回条数(0 = 无)，取后清 fresh */
uint8_t vision_TakeFrame(VisionObst_t *dst, uint8_t max);

/* 取走任务号：返回 true = 取到，取后置回 VISION_TASK_ID_NONE */
bool vision_TakeTask(VisionTask_t *dst);

/* 只读槽位（诊断） */
const VisionSlot_t *vision_GetSlot(void);

/* 视距查询：返回 mm；0 = 未知（未标定），须按"不可信"处理 */
int32_t vision_GetRangeMM(void);

#endif
