#ifndef __INPUT_H
#define __INPUT_H

#include "contract.h"

/* 输入汇聚层：三条串口的硬件配置与字节管道，以及视觉模块 2 的帧解析；
 * 串口屏协议实现见 screen.c；分层与扩展指引见工程根目录 README.md */

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

/* 每通道接收状态（字节级）。本层只搬字节与刷新计数，
 * 帧解析在各自的协议模块里（串口屏见 screen.c，视觉模块 2 见本文件视觉段） */
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

/*---- 视觉模块 2（USART3）：匿名协议精简帧 -------------------------*/

/* 帧格式：0xAA | ID | LEN | DATA(LEN) | SC | AC（去掉了匿名协议的目标地址 D_ADDR）
 * SC = 从帧头起逐字节累加取低 8 位，累加范围为帧头到 DATA 结束共 LEN+3 字节；
 * AC = 每累加一字节就把当时的 SC 再累加一次，取低 8 位。
 * 故 SC 位于下标 LEN+3、AC 位于 LEN+4；DATA 为小端（多字节字段低字节在前） */
#define VISION_FRAME_HEAD		(0xAAu)		/* 帧头（固定值） */
#define VISION_ID_OBST			(0xF3u)		/* 功能码：避障信息；数据内容未定，暂不解析 */
#define VISION_ID_TASK			(0xF4u)		/* 功能码：任务码（二维码），数据为 ASCII 原文 */
#define VISION_FRAME_OVERHEAD	(5u)		/* 帧头+功能码+长度+SC+AC，不含数据内容 */
#define VISION_DATA_MAX			(128u)		/* 单帧数据内容上限；调大须保证 +5 后小于 INPUT_RX_BUF */
#define VISION_FRAME_MAX		(VISION_DATA_MAX + VISION_FRAME_OVERHEAD)

#define VISION_OBST_MAX			(8u)		/* 障碍物条数上限；数据内容未定，当前恒为 0 条 */
#define VISION_TASK_CODE_MAX	(16u)		/* 任务码 ASCII 原文上限；当前 4 组共 12 位 */
#define VISION_FRAME_TIMEOUT_MS	(500u)		/* 超过此时长无完整帧即判离线 */

/* 视觉模块 2 有效识别距离(mm)，实测 50 cm；⚠ 0 = 未知（哨兵），调用方不得据此放行速度 */
#define VISION_RANGE_MM			(500)

/* 视场角(度) / 安装高度(mm) / 俯角(度)：⚠ 均留空，影响障碍物反投影与盲区判定 */
#define VISION_FOV_DEG			(0.0f)		/* ⚠ 留空：0 = 未知 */
#define VISION_MOUNT_H_MM		(0.0f)		/* ⚠ 留空：0 = 未知 */
#define VISION_MOUNT_PITCH_DEG	(0.0f)		/* ⚠ 留空：0 = 未知 */

typedef struct {
	int16_t x;				/* 障碍中心横向(车体坐标, mm)  ⚠ 待定 */
	int16_t y;				/* 障碍中心纵向(车体坐标, mm)  ⚠ 待定 */
	int16_t width;			/* 障碍宽度(mm)                ⚠ 待定 */
	uint8_t confidence;		/* 置信度 0~100                ⚠ 待定 */
} VisionObst_t;

/* 任务码（二维码）：视觉模块 2 原样上报的数字串，如 "156123516231"（4 组各 3 位） */
typedef struct {
	uint8_t len;						/* 有效字符数；0 = 无任务码 */
	char    code[VISION_TASK_CODE_MAX];	/* ASCII 原文，末尾补 '\0' */
} VisionTask_t;

typedef struct {
	uint8_t  online;					/* 1 = 最后完整帧未超时 */
	uint32_t lastTick;					/* 最后收到完整帧的 tick */
	uint16_t rxCnt;						/* 累计通过校验的完整帧数 */
	uint16_t errCnt;					/* 失步 / 长度越界 / 校验失败 / 未知功能码帧数 */

	uint8_t  obstFresh;					/* 1 = 有新避障帧待取走；数据内容未定，暂不会置位 */
	uint8_t  count;						/* 本帧障碍物条数 */
	VisionObst_t obst[VISION_OBST_MAX];

	uint8_t  taskFresh;					/* 1 = 有新任务码待取走 */
	VisionTask_t task;					/* 任务码（二维码） */
} VisionSlot_t;

extern __IO VisionSlot_t visionSlot;

/* 周期处理：收帧 + 校验 + 刷新在线标志；须与 input_Step() 一样每周期调用 */
void vision_Step(void);

/* 取走一帧避障数据：返回条数(0 = 无)，取后清 obstFresh */
uint8_t vision_TakeFrame(VisionObst_t *dst, uint8_t max);

/* 取走任务码：返回 true = 取到，取后清 taskFresh（任务码本身留在槽位里） */
bool vision_TakeTask(VisionTask_t *dst);

/* 只读槽位（诊断） */
const VisionSlot_t *vision_GetSlot(void);

/* 视距查询：返回 mm；0 = 未知（未标定），须按"不可信"处理 */
int32_t vision_GetRangeMM(void);

#endif
