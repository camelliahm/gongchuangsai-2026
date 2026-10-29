#include "screen.h"
#include "input.h"
#include "FreeRTOS.h"
#include "task.h"

/*---- 模块状态 ----------------------------------------------------*/

//屏上报事件的槽位；解析在任务上下文的 screen_Step() 内完成
__IO ScreenEvt_t screenEvt;			//屏上报事件槽位

#define SCREEN_RX_FRAME_MAX	(48u)	//单帧正文上限（不含三字节结束符）

//接收组帧状态：屏 → MCU 的帧同样以 0xFF 0xFF 0xFF 结尾
static uint8_t  s_scrBuf[SCREEN_RX_FRAME_MAX];	//收帧缓冲
static uint16_t s_scrLen;			//帧体已收长度
static uint8_t  s_scrEnd;			//已收连续 0xFF 个数

//发送组帧缓冲：正文 + 三字节结束符
typedef struct {
	uint8_t  buf[SCREEN_TX_CMD_MAX + SCREEN_END_RUN];	//指令正文 + 三字节结束符
	uint16_t len;					//正文长度
	uint8_t  over;					//1 = 正文超长，整条不发
} ScreenTx_t;

static void screen_Parse(void);
static void screen_Dispatch(const uint8_t *buf, uint16_t len);
static void screen_Online(void);
static void screen_TxPutCh(ScreenTx_t *tx, char c);
static void screen_TxPutStr(ScreenTx_t *tx, const char *s);
static void screen_TxPutInt(ScreenTx_t *tx, int32_t v);
static uint16_t screen_TxFlush(ScreenTx_t *tx);

/*---- 周期处理（任务上下文） ----------------------------------------*/

/**
 * @brief  把一个事件帧体解析进槽位
 * @param  buf 帧体（不含三字节结束符）  len 帧体长度
 * @note   帧长与 TJC 文档不符即计入 errCnt 并丢弃；起始码未列出的一律按非法帧处理
 * @author camelliahm
 */
static void screen_Dispatch(const uint8_t *buf, uint16_t len)
{
	volatile ScreenEvt_t *e = &screenEvt;

	switch (buf[0])
	{
	case SCREEN_EV_TOUCH:				//页面ID + 控件ID + 触摸事件
		if (len != 4u)
		{
			e->errCnt++;
			return;
		}
		e->page  = buf[1];
		e->comp  = buf[2];
		e->touch = buf[3];
		break;

	case SCREEN_EV_PAGE:				//页面ID
		if (len != 2u)
		{
			e->errCnt++;
			return;
		}
		e->page = buf[1];
		break;

	case SCREEN_EV_TOUCH_XY:			//X(高,低) + Y(高,低) + 触摸事件
		if (len != 6u)
		{
			e->errCnt++;
			return;
		}
		e->x     = (uint16_t)(((uint16_t)buf[1] << 8) | buf[2]);
		e->y     = (uint16_t)(((uint16_t)buf[3] << 8) | buf[4]);
		e->touch = buf[5];
		break;

	case SCREEN_EV_NUM:					//4 字节小端
		if (len != 5u)
		{
			e->errCnt++;
			return;
		}
		e->num = (int32_t)((uint32_t)buf[1]
						 | ((uint32_t)buf[2] << 8)
						 | ((uint32_t)buf[3] << 16)
						 | ((uint32_t)buf[4] << 24));
		break;

	case SCREEN_EV_TEXT:				//ASCII 内容，超长截断
		{
			uint16_t i;
			uint16_t n = (uint16_t)(len - 1u);

			if (n > (SCREEN_TEXT_MAX - 1u))
			{
				n = (uint16_t)(SCREEN_TEXT_MAX - 1u);
			}
			for (i = 0u; i < n; i++)
			{
				e->text[i] = (char)buf[1u + i];
			}
			e->text[n] = '\0';
			e->textLen = (uint8_t)n;
		}
		break;

	default:
		e->errCnt++;
		return;
	}

	e->type     = buf[0];
	e->fresh    = 1u;
	e->lastTick = xTaskGetTickCount();
	e->rxCnt++;
}

/**
 * @brief  从输入层的串口屏通道收齐一帧并分发
 * @note   帧边界靠三字节连续 0xFF；正文里允许出现单个 0xFF（被打断时补回帧体）
 * @author camelliahm
 */
static void screen_Parse(void)
{
	uint8_t b;

	while (input_Read(INPUT_SCREEN, &b, 1u) == 1u)
	{
		if (b == SCREEN_END_BYTE)
		{
			s_scrEnd++;
			if (s_scrEnd < SCREEN_END_RUN)
			{
				continue;			//结束符未收齐，先不入帧体
			}

			s_scrEnd = 0u;
			if (s_scrLen > 0u)
			{
				screen_Dispatch(s_scrBuf, s_scrLen);
			}
			s_scrLen = 0u;
			continue;
		}

		//结束符被打断：把已收到的 0xFF 补回帧体
		while ((s_scrEnd > 0u) && (s_scrLen < SCREEN_RX_FRAME_MAX))
		{
			s_scrBuf[s_scrLen] = SCREEN_END_BYTE;
			s_scrLen++;
			s_scrEnd--;
		}
		s_scrEnd = 0u;

		if (s_scrLen < SCREEN_RX_FRAME_MAX)
		{
			s_scrBuf[s_scrLen] = b;
			s_scrLen++;
		}
		else
		{
			s_scrLen = 0u;			//超长：整帧丢弃，等下一个结束符重新起头
			screenEvt.errCnt++;
		}
	}
}

/**
 * @brief  刷新串口屏槽位的在线标志
 * @note   屏只在被追问或有人操作时才上报，故 online = 0 不等于屏掉线
 * @author camelliahm
 */
static void screen_Online(void)
{
	volatile ScreenEvt_t *e = &screenEvt;

	if ((xTaskGetTickCount() - e->lastTick) <= pdMS_TO_TICKS(SCREEN_TIMEOUT_MS))
	{
		e->online = 1u;
	}
	else
	{
		e->online = 0u;
	}
}

/*---- 发送组帧 ----------------------------------------------------*/

/**
 * @brief  向发送缓冲追加一个字符
 * @note   超长只置溢出标志，不做截断 —— 半条指令会让屏一直等结束符
 * @author camelliahm
 */
static void screen_TxPutCh(ScreenTx_t *tx, char c)
{
	if (tx->len >= SCREEN_TX_CMD_MAX)
	{
		tx->over = 1u;
		return;
	}

	tx->buf[tx->len] = (uint8_t)c;
	tx->len++;
}

/**
 * @brief  向发送缓冲追加一个字符串
 * @author camelliahm
 */
static void screen_TxPutStr(ScreenTx_t *tx, const char *s)
{
	while (*s != '\0')
	{
		screen_TxPutCh(tx, *s);
		s++;
	}
}

/**
 * @brief  向发送缓冲追加一个十进制整数
 * @author camelliahm
 */
static void screen_TxPutInt(ScreenTx_t *tx, int32_t v)
{
	char     tmp[12];
	uint8_t  n = 0u;
	uint32_t u;

	if (v < 0)
	{
		screen_TxPutCh(tx, '-');
		u = (uint32_t)(~((uint32_t)v) + 1u);	//补码取绝对值
	}
	else
	{
		u = (uint32_t)v;
	}

	do
	{
		tmp[n] = (char)('0' + (char)(u % 10u));
		n++;
		u /= 10u;
	} while ((u != 0u) && (n < (uint8_t)sizeof(tmp)));

	while (n > 0u)
	{
		n--;
		screen_TxPutCh(tx, tmp[n]);
	}
}

/**
 * @brief  补三字节结束符并发出
 * @retval 实际发出的字节数；正文超长时返回 0
 * @note   阻塞式（忙等 TXE/TC），不可在中断或临界区调用
 * @author camelliahm
 */
static uint16_t screen_TxFlush(ScreenTx_t *tx)
{
	if (tx->over != 0u)
	{
		return 0u;
	}

	tx->buf[tx->len]      = SCREEN_END_BYTE;
	tx->buf[tx->len + 1u] = SCREEN_END_BYTE;
	tx->buf[tx->len + 2u] = SCREEN_END_BYTE;

	return input_Write(INPUT_SCREEN, tx->buf, (uint16_t)(tx->len + SCREEN_END_RUN));
}

/*---- 对外接口 ----------------------------------------------------*/

/**
 * @brief  串口屏模块初始化：复位上报事件槽位与收帧状态
 * @note   USART2 的时钟 / GPIO / DMA / NVIC 由 input_Init() 负责，本函数只清软件状态；
 *         须在调度器启动前调用（xTaskGetTickCount() 此时恒返回 0，仅作超时基准）
 * @author camelliahm
 */
void screen_Init(void)
{
	screenEvt.online   = 0u;
	screenEvt.fresh    = 0u;
	screenEvt.type     = 0u;
	screenEvt.rxCnt    = 0u;
	screenEvt.errCnt   = 0u;
	screenEvt.textLen  = 0u;
	screenEvt.lastTick = xTaskGetTickCount();

	s_scrLen = 0u;
	s_scrEnd = 0u;
}

/**
 * @brief  周期处理：收帧 + 刷新在线标志
 * @note   须与 input_Step() 在同一任务上下文调用；当前无调用点（未接入控制环）
 * @author camelliahm
 */
void screen_Step(void)
{
	screen_Parse();
	screen_Online();
}

/**
 * @brief  发送一条 TJC 指令
 * @param  cmd 以 '\0' 结尾的指令正文（不含结束符），如 "page 0"、"n0.val=12"
 * @retval 实际发出的字节数；参数非法或正文超长时返回 0
 * @author camelliahm
 */
uint16_t screen_Cmd(const char *cmd)
{
	ScreenTx_t tx;

	if (cmd == NULL)
	{
		return 0u;
	}

	tx.len  = 0u;
	tx.over = 0u;
	screen_TxPutStr(&tx, cmd);

	return screen_TxFlush(&tx);
}

/**
 * @brief  给文本控件赋值：obj.txt="txt"
 * @param  obj 控件名（区分大小写）  txt 内容（自动加双引号，内容里不要再带引号）
 * @retval 实际发出的字节数；参数非法或超长时返回 0
 * @author camelliahm
 */
uint16_t screen_Text(const char *obj, const char *txt)
{
	ScreenTx_t tx;

	if ((obj == NULL) || (txt == NULL))
	{
		return 0u;
	}

	tx.len  = 0u;
	tx.over = 0u;
	screen_TxPutStr(&tx, obj);
	screen_TxPutStr(&tx, ".txt=\"");
	screen_TxPutStr(&tx, txt);
	screen_TxPutCh(&tx, '"');

	return screen_TxFlush(&tx);
}

/**
 * @brief  给数值控件赋值：obj.val=val
 * @param  obj 控件名（区分大小写）  val 数值
 * @retval 实际发出的字节数；参数非法或超长时返回 0
 * @author camelliahm
 */
uint16_t screen_Num(const char *obj, int32_t val)
{
	ScreenTx_t tx;

	if (obj == NULL)
	{
		return 0u;
	}

	tx.len  = 0u;
	tx.over = 0u;
	screen_TxPutStr(&tx, obj);
	screen_TxPutStr(&tx, ".val=");
	screen_TxPutInt(&tx, val);

	return screen_TxFlush(&tx);
}

/**
 * @brief  切换页面：page <id>
 * @param  pageId 页面 ID（TJC 工程里的页面编号）
 * @retval 实际发出的字节数
 * @author camelliahm
 */
uint16_t screen_Page(uint8_t pageId)
{
	ScreenTx_t tx;

	tx.len  = 0u;
	tx.over = 0u;
	screen_TxPutStr(&tx, "page ");
	screen_TxPutInt(&tx, (int32_t)pageId);

	return screen_TxFlush(&tx);
}

/*---- 上屏内容 ----------------------------------------------------*/

/**
 * @brief  把无符号数按十进制追加到字符串缓冲
 * @param  buf 缓冲  len 当前长度（原地更新）  max 缓冲容量（含结尾 '\0'）
 * @param  v   待追加的数值
 * @note   缓冲不足即截断，不进位不报错
 * @author camelliahm
 */
static void screen_AppendU32(char *buf, uint8_t *len, uint8_t max, uint32_t v)
{
	char    tmp[11];
	uint8_t n = 0u;

	do
	{
		tmp[n] = (char)('0' + (char)(v % 10u));
		n++;
		v /= 10u;
	} while ((v != 0u) && (n < (uint8_t)sizeof(tmp)));

	while ((n > 0u) && (*len < (uint8_t)(max - 1u)))
	{
		n--;
		buf[*len] = tmp[n];
		(*len)++;
	}
}

/**
 * @brief  把"分子/分母"格式化进缓冲
 * @param  buf 缓冲  max 缓冲容量（含结尾 '\0'）  a 分子  b 分母
 * @note   缓冲不足即截断，不到 2 位就只写分母
 * @author camelliahm
 */
static void screen_FormatNumNum(char *buf, uint8_t max, uint32_t a, uint32_t b)
{
	uint8_t len = 0u;

	screen_AppendU32(buf, &len, max, a);
	if (len < (uint8_t)(max - 1u))
	{
		buf[len] = '/';
		len++;
	}
	screen_AppendU32(buf, &len, max, b);
	buf[len] = '\0';
}

/**
 * @brief  任务码上屏
 * @param  code 视觉上报的任务码原文（不校验内容，原样显示）；空指针按空串处理
 * @note   阻塞式发送，须在任务上下文调用
 * @author camelliahm
 */
void screen_ShowTask(const char *code)
{
	if (code == NULL)
	{
		code = "";
	}

	(void)screen_Text(SCREEN_OBJ_TASK, code);
}

/**
 * @brief  抓取次数上屏
 * @param  ok 抓取成功次数  tot 抓取动作总数（分母）
 * @note   阻塞式发送，须在任务上下文调用
 * @author camelliahm
 */
void screen_ShowGet(uint8_t ok, uint8_t tot)
{
	char buf[24];

	screen_FormatNumNum(buf, (uint8_t)sizeof(buf), ok, tot);
	(void)screen_Text(SCREEN_OBJ_GET, buf);
}

/**
 * @brief  放置次数上屏
 * @param  ok 放置成功次数  tot 放置动作总数（分母）
 * @note   阻塞式发送，须在任务上下文调用
 * @author camelliahm
 */
void screen_ShowPut(uint8_t ok, uint8_t tot)
{
	char buf[24];

	screen_FormatNumNum(buf, (uint8_t)sizeof(buf), ok, tot);
	(void)screen_Text(SCREEN_OBJ_PUT, buf);
}

/**
 * @brief  失败次数上屏
 * @param  errGet 抓取失败次数  errPut 放置失败次数
 * @note   阻塞式发送，须在任务上下文调用；SCREEN_SHOW_ERR 为 0 时只吞参数不发帧
 * @author camelliahm
 */
void screen_ShowErr(uint8_t errGet, uint8_t errPut)
{
#if (SCREEN_SHOW_ERR != 0u)
	char buf[24];

	screen_FormatNumNum(buf, (uint8_t)sizeof(buf), errGet, errPut);
	(void)screen_Text(SCREEN_OBJ_ERR, buf);
#else
	(void)errGet;
	(void)errPut;
#endif
}

/**
 * @brief  取走一个事件
 * @param  dst 目标结构
 * @retval true = 取到
 * @note   取走后清 fresh，因此同一个事件只会被消费一次
 * @author camelliahm
 */
bool screen_TakeEvent(ScreenEvt_t *dst)
{
	if ((dst == NULL) || (screenEvt.fresh == 0u))
	{
		return false;
	}

	*dst = screenEvt;
	screenEvt.fresh = 0u;

	return true;
}
