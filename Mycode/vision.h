#ifndef __VISION_H
#define __VISION_H

#include "contract.h"

/* 视觉层：两个视觉模块的帧解析（匿名协议精简帧），只做组帧 / 校验 / 槽位刷新；
 * 三条串口的时钟 / GPIO / DMA / 环形缓冲在 input.c。分层与扩展指引见工程根目录 README.md */

/*---- 帧壳：视觉模块 1 / 2 共用 -----------------------------------*/
/* 帧格式：0xAA | ID | LEN | DATA(LEN) | SC | AC（去掉了匿名协议的目标地址 D_ADDR）
 * SC = 从帧头起逐字节累加取低 8 位，累加范围为帧头到 DATA 结束共 LEN+3 字节；
 * AC = 每累加一字节就把当时的 SC 再累加一次，取低 8 位。
 * 故 SC 位于下标 LEN+3、AC 位于 LEN+4；DATA 为小端（多字节字段低字节在前） */
#define VISION_FRAME_HEAD		(0xAAu)		//帧头（固定值）
#define VISION_FRAME_OVERHEAD	(5u)		//帧头 + 功能码 + 长度 + SC + AC，不含数据内容
#define VISION_DATA_MAX			(128u)		//单帧数据内容上限；调大须保证 +5 后小于 INPUT_RX_BUF
#define VISION_FRAME_MAX		(VISION_DATA_MAX + VISION_FRAME_OVERHEAD)	//单帧最大长度（含帧头与校验）
#define VISION_FRAME_TIMEOUT_MS	(500u)		//超过此时长没有完整帧即判离线

/*---- 视觉模块 1（USART1）：物块颜色 + 对准相对距离 ----------------*/

#define VISION1_ID_ALIGN		(0xF5u)		//功能码：物块颜色 + 相对距离（对准用）
#define VISION1_DATA_LEN		(5u)		//DATA 固定长度：颜色 1 字节 + 前后 int16 + 左右 int16，多字节小端

//颜色数字代号，取值与 DOC/07_智能搬运赛项命题要求.md 一致；0 = 未识别 / 未知
#define VISION1_COLOR_NONE		(0u)		//颜色代号：未识别
#define VISION1_COLOR_RED		(1u)		//颜色代号：红色
#define VISION1_COLOR_YELLOW	(2u)		//颜色代号：黄色
#define VISION1_COLOR_BLUE		(3u)		//颜色代号：蓝色
#define VISION1_COLOR_GREEN		(4u)		//颜色代号：绿色
#define VISION1_COLOR_BLACK		(5u)		//颜色代号：黑色
#define VISION1_COLOR_LIGHTBLUE	(6u)		//颜色代号：浅蓝（PLA 材料）

//视觉模块 1 槽位：物块颜色 + 以物料中心为原点的相对距离
typedef struct {
	uint8_t	 online;		//1 = 最后完整帧未超时
	uint32_t lastTick;		//最后收到完整帧的 tick
	uint16_t rxCnt;			//累计通过校验的完整帧数
	uint16_t errCnt;		//失步 / 长度越界 / 校验失败 / 未知功能码帧数
	uint8_t	 color;			//物块颜色代号 VISION1_COLOR_*；0 = 未识别
	int16_t	 front;			//前后相对距离(mm)：正 = 车需前进
	int16_t	 left;			//左右相对距离(mm)：正 = 车需左移
} Vision1Slot_t;

extern __IO Vision1Slot_t vision1Slot;	//视觉模块 1 槽位（USART1）

//周期处理：收帧 + 校验 + 刷新在线标志；须与 input_Step() 一样每周期调用
void vision1_Step(void);

//只读槽位（对准解算 / 诊断）；online = 0 时 color / front / left 不可信
const Vision1Slot_t *vision1_GetSlot(void);

/*---- 视觉模块 2（USART3）：避障 + 任务码 --------------------------*/

#define VISION_ID_OBST			(0xF3u)		//功能码：避障信息；数据内容未定，暂不解析
#define VISION_ID_TASK			(0xF4u)		//功能码：任务码（二维码），数据为 ASCII 原文
#define VISION_OBST_MAX			(8u)		//障碍物条数上限；数据内容未定，当前恒为 0 条
#define VISION_TASK_CODE_MAX	(16u)		//任务码 ASCII 原文上限；当前 4 组共 12 位
#define VISION_RANGE_MM			(500)		//有效识别距离(mm)，实测 50 cm；0 = 未知（哨兵）

//视场角(度) / 安装高度(mm) / 安装俯角(度)：⚠ 均留空，影响障碍物反投影与盲区判定
#define VISION_FOV_DEG			(0.0f)		//视场角：⚠ 留空，0 = 未知
#define VISION_MOUNT_H_MM		(0.0f)		//安装高度：⚠ 留空，0 = 未知
#define VISION_MOUNT_PITCH_DEG	(0.0f)		//安装俯角：⚠ 留空，0 = 未知

//障碍物：车体坐标；⚠ 字段全是待定值，避障层实现前不得使用
typedef struct {
	int16_t x;				//障碍中心横向(车体坐标, mm)：⚠ 待定
	int16_t y;				//障碍中心纵向(车体坐标, mm)：⚠ 待定
	int16_t width;			//障碍宽度(mm)：⚠ 待定
	uint8_t confidence;		//置信度 0~100：⚠ 待定
} VisionObst_t;

//任务码（二维码）：视觉模块 2 原样上报的数字串，如 "156123516231"（4 组各 3 位）
typedef struct {
	uint8_t len;						//有效字符数；0 = 无任务码
	char    code[VISION_TASK_CODE_MAX];	//ASCII 原文，末尾补 '\0'
} VisionTask_t;

//视觉模块 2 槽位
typedef struct {
	uint8_t  online;					//1 = 最后完整帧未超时
	uint32_t lastTick;					//最后收到完整帧的 tick
	uint16_t rxCnt;						//累计通过校验的完整帧数
	uint16_t errCnt;					//失步 / 长度越界 / 校验失败 / 未知功能码帧数

	uint8_t  obstFresh;					//1 = 有新避障帧待取走；数据内容未定，暂不会置位
	uint8_t  count;						//本帧障碍物条数
	VisionObst_t obst[VISION_OBST_MAX];	//障碍物列表

	uint8_t  taskFresh;					//1 = 有新任务码待取走
	VisionTask_t task;					//任务码（二维码）
} VisionSlot_t;

extern __IO VisionSlot_t visionSlot;	//视觉模块 2 槽位（USART3）

//初始化：复位两个视觉槽位与组帧状态；⚠ 须在 input_Init() 之后调用（槽位基准取 xTaskGetTickCount）
void vision_Init(void);

//周期处理：收帧 + 校验 + 刷新在线标志；⚠ 当前无调用点（未接入控制环）
void vision_Step(void);

//取走一帧避障数据：返回条数(0 = 无)，取后清 obstFresh
uint8_t vision_TakeFrame(VisionObst_t *dst, uint8_t max);

//取走任务码：返回 true = 取到，取后清 taskFresh（任务码本身留在槽位里）
bool vision_TakeTask(VisionTask_t *dst);

//只读槽位（诊断）
const VisionSlot_t *vision_GetSlot(void);

//视距查询：返回 mm；0 = 未知（未标定），须按"不可信"处理
int32_t vision_GetRangeMM(void);

#endif
