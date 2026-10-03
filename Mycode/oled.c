#include "oled.h"
#include "oled_font.h"
#include "chassis.h"
#include "chassis_drive.h"
#include "auxiliary.h"
#include "FreeRTOS.h"
#include "task.h"

/* 软件 I2C GPIO 操作（开漏 + 模块自带上拉；SCL/SDA 须在同一端口） */
#define OLED_SCL_H()	GPIO_SetBits(OLED_SCL_PORT, OLED_SCL_PIN)
#define OLED_SCL_L()	GPIO_ResetBits(OLED_SCL_PORT, OLED_SCL_PIN)
#define OLED_SDA_H()	GPIO_SetBits(OLED_SDA_PORT, OLED_SDA_PIN)
#define OLED_SDA_L()	GPIO_ResetBits(OLED_SDA_PORT, OLED_SDA_PIN)

/* 8x16 字体：每字符 8 列 × 2 页 */
#define OLED_CHAR_W			(8u)
#define OLED_CHAR_FIRST		(0x20u)
#define OLED_CHAR_LAST		(0x7Eu)

/* 显存：按页组织，与 SSD1306 的寻址方式一致 */
static uint8_t s_gram[OLED_PAGE_NUM][OLED_WIDTH];

/*----------------------------------------------------------*/
/*  软件 I2C 时序                                               */
/*----------------------------------------------------------*/

/**
 * @brief  I2C 起始条件
 * @author camelliahm
 */
static void oled_I2cStart(void)
{
	OLED_SDA_H();
	OLED_SCL_H();
	OLED_SDA_L();
	OLED_SCL_L();
}

/**
 * @brief  I2C 停止条件
 * @author camelliahm
 */
static void oled_I2cStop(void)
{
	OLED_SDA_L();
	OLED_SCL_H();
	OLED_SDA_H();
}

/**
 * @brief  发送一个字节（第 9 个时钟不读应答）
 * @note   SCL 全由主机驱动，被任务打断只会变慢不会出错，故不需要临界区
 * @author camelliahm
 */
static void oled_I2cSendByte(uint8_t b)
{
	uint8_t i;

	for (i = 0u; i < 8u; i++)
	{
		if ((b & 0x80u) != 0u) { OLED_SDA_H(); } else { OLED_SDA_L(); }
		b = (uint8_t)(b << 1);
		OLED_SCL_H();
		OLED_SCL_L();
	}

	OLED_SCL_H();
	OLED_SCL_L();
}

/**
 * @brief  写一条命令
 * @author camelliahm
 */
static void oled_WriteCmd(uint8_t cmd)
{
	oled_I2cStart();
	oled_I2cSendByte(OLED_I2C_ADDR);
	oled_I2cSendByte(0x00u);			/* 控制字节：命令 */
	oled_I2cSendByte(cmd);
	oled_I2cStop();
}

/**
 * @brief  设置写窗口（页 + 列起点）
 * @author camelliahm
 */
static void oled_SetWindow(uint8_t page, uint8_t x)
{
	oled_WriteCmd((uint8_t)(0xB0u | page));
	oled_WriteCmd((uint8_t)(0x10u | ((x & 0xF0u) >> 4)));
	oled_WriteCmd((uint8_t)(0x00u | (x & 0x0Fu)));
}

/*----------------------------------------------------------*/
/*  显存操作                                                    */
/*----------------------------------------------------------*/

/**
 * @brief  清显存（不立即刷面板）
 * @author camelliahm
 */
void oled_Clear(void)
{
	uint8_t page;
	uint8_t x;

	for (page = 0u; page < OLED_PAGE_NUM; page++)
	{
		for (x = 0u; x < OLED_WIDTH; x++)
		{
			s_gram[page][x] = 0x00u;
		}
	}
}

/**
 * @brief  把整块显存刷到面板
 * @note   每页一次 Start/Stop + 连续 128 字节，比"每字节一次事务"快数倍
 * @author camelliahm
 */
void oled_Refresh(void)
{
	uint8_t page;
	uint8_t x;

	for (page = 0u; page < OLED_PAGE_NUM; page++)
	{
		oled_SetWindow(page, 0u);

		oled_I2cStart();
		oled_I2cSendByte(OLED_I2C_ADDR);
		oled_I2cSendByte(0x40u);		/* 控制字节：数据 */
		for (x = 0u; x < OLED_WIDTH; x++)
		{
			oled_I2cSendByte(s_gram[page][x]);
		}
		oled_I2cStop();
	}
}

/**
 * @brief  把一个字符写进显存
 * @param  line 行 1~OLED_LINE_NUM；col 列 1~OLED_COL_NUM
 * @note   越界静默忽略；非可见 ASCII 按空格处理
 * @author camelliahm
 */
static void oled_ShowChar(uint8_t line, uint8_t col, char ch)
{
	const uint8_t *p;
	uint8_t i;
	uint8_t x;
	uint8_t page;

	if ((line < 1u) || (line > OLED_LINE_NUM) || (col < 1u) || (col > OLED_COL_NUM))
	{
		return;
	}

	if (((uint8_t)ch < OLED_CHAR_FIRST) || ((uint8_t)ch > OLED_CHAR_LAST))
	{
		ch = ' ';
	}

	p    = OLED_F8x16[(uint8_t)ch - OLED_CHAR_FIRST];
	x    = (uint8_t)((col - 1u) * OLED_CHAR_W);
	page = (uint8_t)((line - 1u) * 2u);		/* 8x16 → 每行占 2 页 */

	for (i = 0u; i < OLED_CHAR_W; i++)
	{
		s_gram[page][x + i]        = p[i];			/* 上半页 */
		s_gram[page + 1u][x + i]   = p[i + OLED_CHAR_W];	/* 下半页 */
	}
}

/**
 * @brief  在显存里写一个字符串
 * @author camelliahm
 */
void oled_ShowString(uint8_t line, uint8_t col, const char *str)
{
	uint8_t i = 0u;

	if (str == NULL)
	{
		return;
	}

	while ((str[i] != '\0') && ((uint16_t)(col + i) <= OLED_COL_NUM))
	{
		oled_ShowChar(line, (uint8_t)(col + i), str[i]);
		i++;
	}
}

/**
 * @brief  10 的 n 次幂
 * @author camelliahm
 */
static uint32_t oled_Pow10(uint8_t n)
{
	uint32_t r = 1u;
	uint8_t i;

	for (i = 0u; i < n; i++)
	{
		r *= 10u;
	}

	return r;
}

/**
 * @brief  写一个带符号十进制数（固定位数，不足补前导 0）
 * @param  len 数字位数（不含符号占位）
 * @note   用整数显示，避开 microLIB 下的浮点 printf
 * @author camelliahm
 */
void oled_ShowNum(uint8_t line, uint8_t col, int32_t num, uint8_t len)
{
	uint32_t v;
	uint8_t i;
	uint8_t c = col;

	if (num < 0)
	{
		oled_ShowChar(line, c, '-');
		c++;
		v = (uint32_t)(-num);
	}
	else
	{
		v = (uint32_t)num;
	}

	for (i = 0u; i < len; i++)
	{
		oled_ShowChar(line, (uint8_t)(c + i),
			(char)(v / oled_Pow10((uint8_t)(len - i - 1u)) % 10u + (uint32_t)'0'));
	}
}

/**
 * @brief  组装整车状态页并刷新
 * @note   数据源：chassis（状态与目标速度）、chassisDrive（里程计）、
 *         chassisMotor[]（在线数）、lift（升降高度）
 * @author camelliahm
 */
void oled_ShowStatus(void)
{
	char buf[8];
	uint8_t online = 0u;
	uint8_t i;

	for (i = 0u; i < CHASSIS_MOTOR_NUM; i++)
	{
		if (chassisMotor[i].online != 0u)
		{
			online++;
		}
	}

	oled_Clear();

	/* 行1：状态名 + 四轮在线数 */
	oled_ShowString(1u, 1u, chassis_StateName(chassis.state));
	buf[0] = (char)('0' + online);
	buf[1] = '/';
	buf[2] = (char)('0' + CHASSIS_MOTOR_NUM);
	buf[3] = '\0';
	oled_ShowString(1u, 14u, buf);

	/* 行2：目标底盘速度 vx / vy (mm/s) */
	oled_ShowString(2u, 1u, "V");
	oled_ShowNum(2u, 2u, (int32_t)chassis.vx, 4u);
	oled_ShowNum(2u, 9u, (int32_t)chassis.vy, 4u);

	/* 行3：里程计位移 X / Y (mm) */
	oled_ShowString(3u, 1u, "X");
	oled_ShowNum(3u, 2u, (int32_t)chassisDrive.odomX, 4u);
	oled_ShowString(3u, 9u, "Y");
	oled_ShowNum(3u, 10u, (int32_t)chassisDrive.odomY, 4u);

	/* 行4：航向 (度) + 升降高度 (mm) */
	oled_ShowString(4u, 1u, "A");
	oled_ShowNum(4u, 2u, (int32_t)chassisDrive.odomYaw, 4u);
	oled_ShowString(4u, 9u, "H");
	oled_ShowNum(4u, 10u, (int32_t)lift_GetHeightMM(), 4u);

	oled_Refresh();
}

/**
 * @brief  初始化（GPIO、上电时序、SSD1306 配置、清屏）
 * @note   上电延时用空循环，不依赖 SysTick，故调度器启动前后都能调用
 * @author camelliahm
 */
void oled_Init(void)
{
	GPIO_InitTypeDef g;
	uint32_t i;
	uint32_t j;

	/* 上电稳定延时（约 50ms） */
	for (i = 0u; i < 200u; i++)
	{
		for (j = 0u; j < 20000u; j++)
		{
			__NOP();
		}
	}

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	g.GPIO_Pin   = OLED_SCL_PIN | OLED_SDA_PIN;
	g.GPIO_Mode  = GPIO_Mode_OUT;
	g.GPIO_OType = GPIO_OType_OD;
	g.GPIO_PuPd  = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(OLED_SCL_PORT, &g);

	OLED_SCL_H();
	OLED_SDA_H();

	oled_WriteCmd(0xAE);		/* 关显示 */
	oled_WriteCmd(0xD5);		/* 时钟分频 / 振荡频率 */
	oled_WriteCmd(0x80);
	oled_WriteCmd(0xA8);		/* 多路复用率 */
	oled_WriteCmd(0x3F);
	oled_WriteCmd(0xD3);		/* 显示偏移 */
	oled_WriteCmd(0x00);
	oled_WriteCmd(0x40);		/* 起始行 */
	oled_WriteCmd(0xA1);		/* 左右方向：正常 */
	oled_WriteCmd(0xC8);		/* 上下方向：正常 */
	oled_WriteCmd(0xDA);		/* COM 引脚配置 */
	oled_WriteCmd(0x12);
	oled_WriteCmd(0x81);		/* 对比度 */
	oled_WriteCmd(0xCF);
	oled_WriteCmd(0xD9);		/* 预充电周期 */
	oled_WriteCmd(0xF1);
	oled_WriteCmd(0xDB);		/* VCOMH 电压 */
	oled_WriteCmd(0x30);
	oled_WriteCmd(0xA4);		/* 输出跟随显存 */
	oled_WriteCmd(0xA6);		/* 正常显示（非反色） */
	oled_WriteCmd(0x8D);		/* 电荷泵 */
	oled_WriteCmd(0x14);
	oled_WriteCmd(0xAF);		/* 开显示 */

	oled_Clear();
	oled_Refresh();
}

/**
 * @brief  周期调用：按 OLED_REFRESH_MS 限速刷新状态页
 * @note   暂未接入控制环；接入方式见 README.md
 * @author camelliahm
 */
void oled_Step(void)
{
	static uint32_t s_last;
	uint32_t now = xTaskGetTickCount();

	if ((now - s_last) >= pdMS_TO_TICKS(OLED_REFRESH_MS))
	{
		s_last = now;
		oled_ShowStatus();
	}
}
