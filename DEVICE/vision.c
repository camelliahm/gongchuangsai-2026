#include "vision.h"
#include "input.h"
#include "FreeRTOS.h"
#include "task.h"

/*---- 槽位与组帧状态 ----------------------------------------------*/

__IO VisionSlot_t  visionSlot;		//视觉模块 2（USART3）
__IO Vision1Slot_t vision1Slot;		//视觉模块 1（USART1）

//组帧状态：两个视觉通道各一份（形状相同，只有功能码与槽位不同）
typedef struct {
	uint8_t  buf[VISION_FRAME_MAX];		//收帧缓冲，最长一帧（含帧头）
	uint16_t len;						//已收字节数（含帧头）
	uint16_t total;						//本帧总长 = LEN + 5；0 = 尚未收到长度字节
} VisionRx_t;

static VisionRx_t s_rx1;			//视觉模块 1
static VisionRx_t s_rx2;			//视觉模块 2

/*---- 初始化 ------------------------------------------------------*/

/**
 * @brief  视觉层初始化：复位两个槽位与组帧状态
 * @note   只清软件状态，两条串口的硬件由 input_Init() 负责；须在调度器启动前调用
 * @author camelliahm
 */
void vision_Init(void)
{
	vision1Slot.online   = 0u;
	vision1Slot.lastTick = xTaskGetTickCount();
	vision1Slot.rxCnt    = 0u;
	vision1Slot.errCnt   = 0u;
	vision1Slot.color    = VISION1_COLOR_NONE;
	vision1Slot.front    = 0;
	vision1Slot.left     = 0;

	visionSlot.online       = 0u;
	visionSlot.lastTick     = xTaskGetTickCount();
	visionSlot.rxCnt        = 0u;
	visionSlot.errCnt       = 0u;
	visionSlot.obstFresh    = 0u;
	visionSlot.count        = 0u;
	visionSlot.taskFresh    = 0u;
	visionSlot.task.len     = 0u;
	visionSlot.task.code[0] = '\0';

	s_rx1.len   = 0u;
	s_rx1.total = 0u;
	s_rx2.len   = 0u;
	s_rx2.total = 0u;
}

/*---- 公共组帧与校验（视觉模块 1 / 2 共用） ----------------------*/

/**
 * @brief  丢弃当前半帧，重新等帧头
 * @param  rx 组帧状态
 * @author camelliahm
 */
static void vision_RxReset(VisionRx_t *rx)
{
	rx->len   = 0u;
	rx->total = 0u;
}

/**
 * @brief  逐字节组帧
 * @param  rx 组帧状态
 * @param  b  新到达的字节
 * @retval 0 = 继续收；1 = 已收满一帧（在 rx->buf 里）；-1 = 失步或长度越界
 * @note   帧头前的杂散字节与长度越界的帧都返回 -1，且两种情况内部已把状态复位；
 *         调用方只需按返回值计数 / 取帧，不必再自行复位
 * @author camelliahm
 */
static int8_t vision_FeedByte(VisionRx_t *rx, uint8_t b)
{
	if (rx->len == 0u)
	{
		if (b != VISION_FRAME_HEAD)
		{
			return -1;
		}

		rx->buf[0] = b;
		rx->len    = 1u;
		return 0;
	}

	rx->buf[rx->len] = b;
	rx->len++;

	if (rx->len == 3u)
	{
		rx->total = (uint16_t)rx->buf[2] + VISION_FRAME_OVERHEAD;

		if (rx->total > VISION_FRAME_MAX)
		{
			vision_RxReset(rx);		//长度越界：丢弃本帧，等下一个帧头
			return -1;
		}
		return 0;
	}

	return ((rx->total != 0u) && (rx->len >= rx->total)) ? 1 : 0;
}

/**
 * @brief  校验一个完整帧（和校验 + 附加校验）
 * @param  f 完整帧，f[0] = 帧头，f[2] = 数据长度
 * @retval true = 两项校验都通过
 * @author camelliahm
 */
static bool vision_CheckSum(const uint8_t *f)
{
	uint8_t  len = f[2];
	uint8_t  sum = 0u;
	uint8_t  add = 0u;
	uint16_t i;

	for (i = 0u; i < ((uint16_t)len + 3u); i++)
	{
		sum = (uint8_t)(sum + f[i]);
		add = (uint8_t)(add + sum);
	}

	return ((sum == f[(uint16_t)len + 3u]) && (add == f[(uint16_t)len + 4u]));
}

/*---- 视觉模块 1（USART1）：物块颜色 + 对准相对距离 ----------------*/

/**
 * @brief  校验并把一帧颜色 + 相对距离存入槽位
 * @param  f 完整帧，长度 LEN+5
 * @note   校验不过 / 功能码不是 0xF5 / 数据长度不是 5 字节，都只计数，
 *         不更新 color / front / left（三者始终来自同一帧，不会新旧混用）
 * @author camelliahm
 */
static void vision1_OnFrame(const uint8_t *f)
{
	if ((vision_CheckSum(f) == false) ||
		(f[1] != VISION1_ID_ALIGN) ||
		(f[2] != VISION1_DATA_LEN))
	{
		vision1Slot.errCnt++;
		return;
	}

	vision1Slot.color = f[3];
	vision1Slot.front = (int16_t)((uint16_t)f[4] | ((uint16_t)f[5] << 8));
	vision1Slot.left  = (int16_t)((uint16_t)f[6] | ((uint16_t)f[7] << 8));

	vision1Slot.rxCnt++;
	vision1Slot.lastTick = xTaskGetTickCount();
}

/**
 * @brief  从视觉1 通道环形缓冲取字节并组帧
 * @author camelliahm
 */
static void vision1_Parse(void)
{
	uint8_t b;
	int8_t  r;

	while (input_Read(INPUT_VISION1, &b, 1u) == 1u)
	{
		r = vision_FeedByte(&s_rx1, b);

		if (r > 0)
		{
			vision1_OnFrame(s_rx1.buf);
			vision_RxReset(&s_rx1);
		}
		else if (r < 0)
		{
			vision1Slot.errCnt++;
			vision_RxReset(&s_rx1);
		}
	}
}

/**
 * @brief  刷新视觉1 槽位在线标志
 * @note   在线 = 最近收到过校验通过的完整帧；离线时 color / front / left 不可信
 * @author camelliahm
 */
static void vision1_Online(void)
{
	if ((xTaskGetTickCount() - vision1Slot.lastTick) <= pdMS_TO_TICKS(VISION_FRAME_TIMEOUT_MS))
	{
		vision1Slot.online = 1u;
	}
	else
	{
		vision1Slot.online = 0u;
	}
}

/**
 * @brief  视觉模块 1 周期处理：收帧 + 校验 + 刷新在线标志
 * @note   须与 input_Step() 一样每周期调用
 * @author camelliahm
 */
void vision1_Step(void)
{
	vision1_Parse();
	vision1_Online();
}

/**
 * @brief  只读视觉1 槽位（对准解算 / 诊断）
 * @retval 槽位指针；online = 0 时 color / front / left 不可信
 * @author camelliahm
 */
const Vision1Slot_t *vision1_GetSlot(void)
{
	return (const Vision1Slot_t *)&vision1Slot;
}

/*---- 视觉模块 2（USART3）：避障 + 任务码 ---------------------------*/

/**
 * @brief  把任务码原样存入槽位
 * @param  data 数据内容起始  len 数据内容长度
 * @note   存 ASCII 原文不做数值解析；超长截断但仍置 taskFresh
 * @author camelliahm
 */
static void vision_OnTask(const uint8_t *data, uint8_t len)
{
	uint8_t n = len;
	uint8_t i;

	if (n > (VISION_TASK_CODE_MAX - 1u))
	{
		n = (uint8_t)(VISION_TASK_CODE_MAX - 1u);
	}

	for (i = 0u; i < n; i++)
	{
		visionSlot.task.code[i] = (char)data[i];
	}
	visionSlot.task.code[n] = '\0';
	visionSlot.task.len     = n;
	visionSlot.taskFresh    = 1u;
}

/**
 * @brief  校验并把一帧视觉2 数据分发给对应槽位
 * @param  f 长度为 LEN+5 的完整帧，f[0] = 帧头
 * @note   校验不过只计数，不进槽位
 * @author camelliahm
 */
static void vision2_OnFrame(const uint8_t *f)
{
	uint8_t len = f[2];

	if (vision_CheckSum(f) == false)
	{
		visionSlot.errCnt++;
		return;
	}

	switch (f[1])
	{
	case VISION_ID_TASK:
		vision_OnTask(&f[3], len);
		break;

	case VISION_ID_OBST:
		break;					//数据内容未定：只计帧数，不解析

	default:
		visionSlot.errCnt++;	//功能码未定义
		return;
	}

	visionSlot.rxCnt++;
	visionSlot.lastTick = xTaskGetTickCount();
}

/**
 * @brief  从视觉2 通道环形缓冲取字节并组帧
 * @author camelliahm
 */
static void vision2_Parse(void)
{
	uint8_t b;
	int8_t  r;

	while (input_Read(INPUT_VISION2, &b, 1u) == 1u)
	{
		r = vision_FeedByte(&s_rx2, b);

		if (r > 0)
		{
			vision2_OnFrame(s_rx2.buf);
			vision_RxReset(&s_rx2);
		}
		else if (r < 0)
		{
			visionSlot.errCnt++;
			vision_RxReset(&s_rx2);
		}
	}
}

/**
 * @brief  刷新视觉2 槽位在线标志
 * @note   在线 = 最近收到过校验通过的完整帧
 * @author camelliahm
 */
static void vision2_Online(void)
{
	if ((xTaskGetTickCount() - visionSlot.lastTick) <= pdMS_TO_TICKS(VISION_FRAME_TIMEOUT_MS))
	{
		visionSlot.online = 1u;
	}
	else
	{
		visionSlot.online = 0u;
	}
}

/**
 * @brief  视觉模块 2 周期处理：收帧 + 校验 + 刷新在线标志
 * @note   须与 input_Step() 一样每周期调用；当前仍无调用点（未接入控制环）
 * @author camelliahm
 */
void vision_Step(void)
{
	vision2_Parse();
	vision2_Online();
}

/**
 * @brief  取走一帧避障数据
 * @param  dst 目标缓冲（须至少有 max 条容量）  max 缓冲区可容纳条数
 * @retval 实际拷贝的条数；0 = 无新帧
 * @note   避障帧的数据内容未定，当前不会置 obstFresh，故恒返回 0
 * @author camelliahm
 */
uint8_t vision_TakeFrame(VisionObst_t *dst, uint8_t max)
{
	uint8_t n;
	uint8_t i;

	if ((dst == NULL) || (max == 0u) || (visionSlot.obstFresh == 0u))
	{
		return 0u;
	}

	n = visionSlot.count;
	if (n > max)
	{
		n = max;
	}

	for (i = 0u; i < n; i++)
	{
		dst[i] = visionSlot.obst[i];
	}

	visionSlot.obstFresh = 0u;

	return n;
}

/**
 * @brief  取走任务码
 * @param  dst 目标结构
 * @retval true = 取到
 * @note   取走后清 taskFresh，因此同一个任务码只会被消费一次
 * @author camelliahm
 */
bool vision_TakeTask(VisionTask_t *dst)
{
	if ((dst == NULL) || (visionSlot.taskFresh == 0u))
	{
		return false;
	}

	*dst = visionSlot.task;
	visionSlot.taskFresh = 0u;

	return true;
}

/**
 * @brief  只读槽位
 * @author camelliahm
 */
const VisionSlot_t *vision_GetSlot(void)
{
	return (const VisionSlot_t *)&visionSlot;
}

/**
 * @brief  查询有效识别距离
 * @retval 距离(mm)；0 = 未知（未标定）
 * @note   0 是"不可信"哨兵而非"距离为零"，调用方不得据此放行速度
 * @author camelliahm
 */
int32_t vision_GetRangeMM(void)
{
	return (int32_t)VISION_RANGE_MM;
}
