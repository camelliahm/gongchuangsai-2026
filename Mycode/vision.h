#ifndef __VISION_H
#define __VISION_H

#include "chassis.h"

/* 视觉/二维码层：接收视觉模块经串口上报的**结构化帧**，详见 README.md
 *
 * ⚠ 本模块为"接口已定、内容留空"形态：
 *   ① 通信内容（帧字段与语义）待视觉模块规格确定后填写 —— 见 §内容留空
 *   ② 视距（有效识别距离）待实测 —— 见 VISION_RANGE_MM
 *   ③ 本模块**尚未接入控制环**，且避障层尚未建立
 *
 * 与 input.c 的关系：input.c 是"视觉模块 1"（通用文本行汇聚层，ASCII + '\n'）；
 * 本模块是**独立的第二路串口**，接收"视觉模块 2"的二进制结构帧（障碍物 + 任务信息）。
 * 两路物理隔离，互不干扰。 */

/*----------------------------------------------------------*/
/*  一、物理资源（已确定，可直接接线）                              */
/*----------------------------------------------------------*/

/* 视觉模块 2 走 USART3：PB10(TX)/PB11(RX)，DMA1 Stream3 Channel4
 * ⚠ 引脚选择理由：PB10/PB11 为 USART3 的标准位置，全工程未被占用。
 *   已占用清单（勿动）：USART1 = PA9/PA10（视觉1）、USART2 = PA2/PA3（二维码）、
 *   舵机 PWM = PC6/PC7/PC8、CAN1 = PB8/PB9、OLED 软 I2C = PB6/PB7 */
#define VISION_USART			USART3
#define VISION_DMA_STREAM		DMA1_Stream3
#define VISION_DMA_CHANNEL		DMA_Channel_4
#define VISION_IRQn				USART3_IRQn
#define VISION_BAUD				(115200u)	/* ⚠ 约定值，待与视觉模块 2 核对 */
#define VISION_RX_BUF			(256u)		/* DMA 接收缓冲（一帧上限） */
#define VISION_FRAME_TIMEOUT_MS	(500u)		/* 超过此时长无完整帧即判离线 */

/*----------------------------------------------------------*/
/*  二、内容留空：帧格式（协议槽位，值待定）                        */
/*----------------------------------------------------------*/

/* 帧头/帧尾：⚠ 占位值，待视觉模块 2 规格确定后改。改这两个宏即可，解析代码不用动 */
#define VISION_FRAME_HEAD		(0xA5u)
#define VISION_FRAME_TAIL		(0x5Au)

/* 障碍物上限：受一帧长度约束（VISION_RX_BUF），改大需同步核对带宽 */
#define VISION_OBST_MAX			(8u)

/* 障碍物单条记录：⚠ 字段语义与量纲待定，当前按"车体坐标"预留 */
typedef struct {
	int16_t x;				/* 障碍中心横向(车体坐标, mm)  ⚠ 待定 */
	int16_t y;				/* 障碍中心纵向(车体坐标, mm)  ⚠ 待定 */
	int16_t width;			/* 障碍宽度(mm)                ⚠ 待定 */
	uint8_t confidence;		/* 置信度 0~100                ⚠ 待定 */
} VisionObst_t;

/* 任务信息（二维码）：⚠ 内容与格式全部留空，待用户确定
 * 视觉模块 2 识别二维码后，把"任务信息"随帧上报；当前仅预留一个定长字节袋，
 * 语义（是任务号 / 目标点序列索引 / 字符串）与长度都由使用者决定。
 * 若最终是字符串，用 VISION_TASK_LEN 并直接存 ASCII；若是编号，前 2 字节按大端解析。 */
#define VISION_TASK_LEN			(16u)	/* ⚠ 留空：字节袋容量，待定 */
typedef struct {
	uint8_t data[VISION_TASK_LEN];	/* ⚠ 留空：任务信息原始字节，语义待定 */
	uint8_t len;					/* 有效字节数（0 = 本帧无任务信息） */
} VisionTask_t;

/*----------------------------------------------------------*/
/*  三、视距：留空待实测                                           */
/*----------------------------------------------------------*/

/* 有效识别距离(mm)。
 * ⚠ 留空原因：由视觉模块硬件与算法决定，未实测前无法填写，故置 0 作为"未知"哨兵。
 *   置 0 的语义 = 车速上限不可计算 → 避障层不得据此放行任何速度。
 *   实测后填写，避障层用它与"当前车速"比较决定是否需要降速：
 *     制动距离 = v²/(2a) + v×总延迟 + 车体半径 + 障碍半径
 * 参考（延迟 100ms / 减速度 800mm/s² / 车体+障碍 200mm）：
 *   300mm/s → 约 286mm；600 → 约 485mm；900 → 约 796mm */
#define VISION_RANGE_MM			(0)		/* ⚠ 留空：0 = 未知 */

/* 视场角(度)与安装高度/俯角：⚠ 均留空，影响障碍物反投影与盲区判定 */
#define VISION_FOV_DEG			(0.0f)	/* ⚠ 留空：0 = 未知 */
#define VISION_MOUNT_H_MM		(0.0f)	/* ⚠ 留空：0 = 未知 */
#define VISION_MOUNT_PITCH_DEG	(0.0f)	/* ⚠ 留空：0 = 未知 */

/*----------------------------------------------------------*/
/*  四、汇聚槽位（形状照 ChassisMotor_t：值 + 新鲜度 + 计数）        */
/*----------------------------------------------------------*/

typedef struct {
	uint8_t  online;					/* 1 = 最后完整帧未超时 */
	uint8_t  fresh;						/* 1 = 有新帧待上层取走 */
	uint8_t  count;						/* 本帧障碍物条数 (0 ~ VISION_OBST_MAX) */
	uint8_t  seq;						/* 帧序号，用于丢帧检测 */
	uint32_t lastTick;					/* 最后收到完整帧的 tick */
	uint16_t rxCnt;						/* 累计完整帧数 */
	uint16_t errCnt;					/* 校验/长度错误帧数 */
	VisionObst_t obst[VISION_OBST_MAX];	/* 障碍物列表（本帧快照） */
	VisionTask_t task;					/* ⚠ 任务信息（二维码），格式留空 */
} VisionSlot_t;

extern __IO VisionSlot_t visionSlot;

/*----------------------------------------------------------*/
/*  五、对外接口                                                   */
/*----------------------------------------------------------*/

/* 初始化（时钟/GPIO/DMA/USART/NVIC；接收随即开始） */
void vision_Init(void);

/* 周期处理：刷新 online。须挂进控制环（由上层调用） */
void vision_Step(void);

/* 取走一帧：有未读帧时把障碍列表拷进 dst 并清 fresh，返回条数(0 = 无) */
uint8_t vision_TakeFrame(VisionObst_t *dst, uint8_t max);

/* 取走任务信息：有未读任务信息时拷进 dst 并清标志，返回有效字节数(0 = 无) */
uint8_t vision_TakeTask(VisionTask_t *dst);

/* 只读槽位（诊断 / 直接取用） */
const VisionSlot_t *vision_GetSlot(void);

/* 视距查询：返回有效识别距离(mm)；返回 0 = 未知（未标定），调用方须按"不可信"处理 */
int32_t vision_GetRangeMM(void);

#endif
