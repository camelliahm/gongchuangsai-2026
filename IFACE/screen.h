#ifndef __SCREEN_H
#define __SCREEN_H

#include "contract.h"

/* 串口屏（USART2，淘晶驰 TJC 系列 / Nextion 兼容协议）；分层与扩展指引见工程根目录 README.md
 * ⚠ USART2 的时钟 / GPIO / DMA / NVIC 与接收缓冲在 input.c，本模块只做协议组帧、解析与发送 */

/* 帧格式：MCU → 屏 = ASCII 指令正文 + 结束符；屏 → MCU = 起始码 + 数据 + 结束符。
 * ⚠ 字符串属性必须带双引号，控件名区分大小写；缺结束符屏不响应 */
#define SCREEN_END_BYTE		(0xFFu)		//结束符单字节
#define SCREEN_END_RUN		(3u)		//结束符连续字节数
#define SCREEN_TX_CMD_MAX	(64u)		//发送正文上限（不含结束符），超出整条不发
#define SCREEN_TEXT_MAX		(16u)		//屏上报字符串的缓冲容量（含 '\0'）
#define SCREEN_TIMEOUT_MS	(500u)		//超过此时长无事件帧，online 置 0

//初始化：复位上报事件槽位；由 ALL_Init() 在 input_Init() 配完三条串口后调用
void screen_Init(void);

//周期处理：收齐屏上报帧并刷新在线标志。⚠ 尚未接入控制环
void screen_Step(void);

//发送：返回实际发出的字节数；正文超长返回 0（不发半条，避免屏一直等结束符）
uint16_t screen_Cmd(const char *cmd);
uint16_t screen_Text(const char *obj, const char *txt);
uint16_t screen_Num(const char *obj, int32_t val);
uint16_t screen_Page(uint8_t pageId);

/*---- 上屏内容（任务码 / 任务统计）----*/
/* ⚠ 下面四个控件名是占位值：按 TJC 工程里的实际控件名改这里即可，代码不必动 */

#define SCREEN_OBJ_TASK		"t0"	//任务码文本框控件名
#define SCREEN_OBJ_GET		"t1"	//抓取次数文本框控件名，显示"成功/总数"
#define SCREEN_OBJ_PUT		"t2"	//放置次数文本框控件名，显示"成功/总数"
#define SCREEN_OBJ_ERR		"t3"	//失败次数文本框控件名，显示"抓错/放错"
#define SCREEN_SHOW_ERR		(1u)	//1 = 上屏失败次数；屏上没有该控件时置 0

/* ⚠ 下面四个函数各发一条屏指令（阻塞约 6 ms），需要摊开时由调用方一次调一个 */
//任务码上屏：原文照抄（如 "156123516231"）；空指针按空串处理
void screen_ShowTask(const char *code);

//抓取次数上屏：显示"成功/总数"
void screen_ShowGet(uint8_t ok, uint8_t tot);

//放置次数上屏：显示"成功/总数"
void screen_ShowPut(uint8_t ok, uint8_t tot);

//失败次数上屏：显示"抓错/放错"；SCREEN_SHOW_ERR 为 0 时本函数空转
void screen_ShowErr(uint8_t errGet, uint8_t errPut);

//屏上报的事件帧起始码
#define SCREEN_EV_TOUCH		(0x65u)		//控件点击：页面ID + 控件ID + 触摸事件
#define SCREEN_EV_PAGE		(0x66u)		//当前页面ID：发 sendme 后返回
#define SCREEN_EV_TOUCH_XY	(0x67u)		//触摸坐标：X(2,大端) + Y(2,大端) + 触摸事件
#define SCREEN_EV_TEXT		(0x70u)		//字符串变量上报：ASCII 内容
#define SCREEN_EV_NUM		(0x71u)		//数值变量上报：4 字节小端

//触摸事件状态（0x65 / 0x67 的末字节）
#define SCREEN_TOUCH_UP		(0x00u)		//触摸抬起
#define SCREEN_TOUCH_DOWN	(0x01u)		//触摸按下

//最近一个屏上报事件（只保留最新一个，字段按 type 取用）
typedef struct {
	uint8_t  online;				//⚠ 屏不主动上报时为 0，不代表屏掉线
	uint8_t  fresh;					//1 = 有事件待上层取走
	uint8_t  type;					//SCREEN_EV_*；0 = 无
	uint8_t  page;					//0x65 / 0x66：页面号
	uint8_t  comp;					//0x65：控件号
	uint8_t  touch;					//0x65 / 0x67：SCREEN_TOUCH_DOWN / _UP
	uint16_t x;						//0x67：X 坐标
	uint16_t y;						//0x67：Y 坐标
	int32_t  num;					//0x71：数值
	char     text[SCREEN_TEXT_MAX];	//0x70：字符串（'\0' 结尾，超长截断）
	uint8_t  textLen;				//0x70 字符串实际长度
	uint32_t lastTick;				//最后收到事件帧的 tick
	uint16_t rxCnt;					//累计有效事件帧数
	uint16_t errCnt;				//帧长不符 / 超长丢弃的帧数
} ScreenEvt_t;

extern __IO ScreenEvt_t screenEvt;

//取走一个事件：有未读事件时拷进 dst 并清 fresh，返回 true
bool screen_TakeEvent(ScreenEvt_t *dst);

#endif
