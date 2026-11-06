#include "bmi088.h"

/* BMI088 软 SPI 驱动：SCK=PA5、MOSI=PA7、MISO=PA6（SDO1/SDO2 共用）、CSB1=PB0（ACC）、CSB2=PB1（GYR）。
 * 加速度计与陀螺仪靠 CS 分时选通，共用一根 MISO。 */

/*---- CS 控制（低有效）----*/
static void cs_acc(uint8_t en)
{
	if (en) { GPIO_ResetBits(BMI088_CSB1_PORT, BMI088_CSB1_PIN); }	//拉低选通
	else    { GPIO_SetBits(BMI088_CSB1_PORT, BMI088_CSB1_PIN); }
}

static void cs_gyr(uint8_t en)
{
	if (en) { GPIO_ResetBits(BMI088_CSB2_PORT, BMI088_CSB2_PIN); }
	else    { GPIO_SetBits(BMI088_CSB2_PORT, BMI088_CSB2_PIN); }
}

/*---- 软件 SPI 位收发（模式 0：空闲 SCK 低，上升沿采样）----*/
static void sck_h(void) { GPIO_SetBits(BMI088_SCK_PORT, BMI088_SCK_PIN); }
static void sck_l(void) { GPIO_ResetBits(BMI088_SCK_PORT, BMI088_SCK_PIN); }
static void mosi(uint8_t b) { (b) ? GPIO_SetBits(BMI088_SDI_PORT, BMI088_SDI_PIN) : GPIO_ResetBits(BMI088_SDI_PORT, BMI088_SDI_PIN); }
static uint8_t miso(void)  { return GPIO_ReadInputDataBit(BMI088_SDO_PORT, BMI088_SDO_PIN); }

static void spi_write_byte(uint8_t b)
{
	uint8_t i;
	for (i = 0u; i < 8u; i++)
	{
		mosi((b & 0x80u) ? 1u : 0u);
		b <<= 1;
		sck_h();
		sck_l();
	}
}

static uint8_t spi_read_byte(void)
{
	uint8_t i, r = 0u;
	for (i = 0u; i < 8u; i++)
	{
		r <<= 1;
		sck_h();
		if (miso()) { r |= 1u; }
		sck_l();
	}
	return r;
}

/*---- 寄存器读写（ACC / GYR 靠 CS 选通）----*/
static uint8_t acc_read_reg(uint8_t reg)
{
	uint8_t v;
	cs_acc(1);
	spi_write_byte(reg | 0x80u);		//读：最高位置 1
	(void)spi_read_byte();				//加速度计：地址后首字节无效，多读一个哑字节
	v = spi_read_byte();
	cs_acc(0);
	return v;
}

static void acc_write_reg(uint8_t reg, uint8_t val)
{
	cs_acc(1);
	spi_write_byte(reg & 0x7Fu);		//写：最高位清 0
	spi_write_byte(val);
	cs_acc(0);
}

static uint8_t gyr_read_reg(uint8_t reg)
{
	uint8_t v;
	cs_gyr(1);
	spi_write_byte(reg | 0x80u);
	v = spi_read_byte();				//陀螺仪：无哑字节，地址后直接返回
	cs_gyr(0);
	return v;
}

static void gyr_write_reg(uint8_t reg, uint8_t val)
{
	cs_gyr(1);
	spi_write_byte(reg & 0x7Fu);
	spi_write_byte(val);
	cs_gyr(0);
}

/**
 * @brief  BMI088 初始化
 * @note   软复位 → 量程（ACC ±3g、GYR ±2000°/s）→ ODR → 自检 ID
 * @author camelliahm
 */
bool bmi088_Init(void)
{
	GPIO_InitTypeDef g;
	volatile uint32_t i;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB, ENABLE);

	//SCK / MOSI / CSB1 / CSB2 推挽输出；MISO 上拉输入
	g.GPIO_Mode  = GPIO_Mode_OUT;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	g.GPIO_Speed = GPIO_Speed_50MHz;

	g.GPIO_Pin = BMI088_SCK_PIN;
	GPIO_Init(BMI088_SCK_PORT, &g);
	g.GPIO_Pin = BMI088_SDI_PIN;
	GPIO_Init(BMI088_SDI_PORT, &g);
	g.GPIO_Pin = BMI088_CSB1_PIN;
	GPIO_Init(BMI088_CSB1_PORT, &g);
	g.GPIO_Pin = BMI088_CSB2_PIN;
	GPIO_Init(BMI088_CSB2_PORT, &g);

	g.GPIO_Mode  = GPIO_Mode_IN;
	g.GPIO_PuPd  = GPIO_PuPd_UP;
	g.GPIO_Pin   = BMI088_SDO_PIN;
	GPIO_Init(BMI088_SDO_PORT, &g);

	//CS 置高（未选通）、SCK 空闲低
	cs_acc(0);
	cs_gyr(0);
	sck_l();

	//软复位（ACC 0x7E / GYR 0x14）
	acc_write_reg(0x7Eu, 0xB6u);
	gyr_write_reg(0x14u, 0xB6u);
	for (i = 0u; i < 100000u; i++) { __NOP(); }	//约 30ms 等复位生效

	//量程：ACC 0x41=0x00（±3g）、GYR 0x0F=0x00（±2000°/s）
	acc_write_reg(0x41u, 0x00u);
	gyr_write_reg(0x0Fu, 0x00u);

	//ODR / 带宽：ACC 0x40（1600Hz）、GYR 0x10（2000Hz / 230Hz）
	acc_write_reg(0x40u, 0xA7u);
	gyr_write_reg(0x10u, 0x02u);

	//模式使能：ACC 0x7D=0x04、GYR 0x11=0x00
	acc_write_reg(0x7Du, 0x04u);
	gyr_write_reg(0x11u, 0x00u);

	return (acc_read_reg(0x00u) == BMI088_ACC_ID) && (gyr_read_reg(0x00u) == BMI088_GYR_ID);
}

/**
 * @brief  读加速度计原始值
 * @param  acc 输出 int16[3]（X/Y/Z），单位 = ±3g / 32768
 * @note   从 0x12 起连读 6 字节，小端拼 int16
 * @author camelliahm
 */
void bmi088_ReadAccRaw(int16_t acc[3])
{
	uint8_t buf[6];
	uint8_t i;

	cs_acc(1);
	spi_write_byte(0x12u | 0x80u);	//起始寄存器 0x12，读
	for (i = 0u; i < 6u; i++) { buf[i] = spi_read_byte(); }
	cs_acc(0);

	acc[0] = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
	acc[1] = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
	acc[2] = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
}

/**
 * @brief  读陀螺仪原始值
 * @param  gyr 输出 int16[3]（X/Y/Z），单位 = ±2000°/s / 32768
 * @note   从 0x02 起连读 6 字节，小端拼 int16
 * @author camelliahm
 */
void bmi088_ReadGyrRaw(int16_t gyr[3])
{
	uint8_t buf[6];
	uint8_t i;

	cs_gyr(1);
	spi_write_byte(0x02u | 0x80u);	//起始寄存器 0x02，读
	for (i = 0u; i < 6u; i++) { buf[i] = spi_read_byte(); }
	cs_gyr(0);

	gyr[0] = (int16_t)((uint16_t)buf[1] << 8 | buf[0]);
	gyr[1] = (int16_t)((uint16_t)buf[3] << 8 | buf[2]);
	gyr[2] = (int16_t)((uint16_t)buf[5] << 8 | buf[4]);
}

/**
 * @brief  读加速度计（m/s²）
 * @param  acc 输出 float[3]
 * @author camelliahm
 */
void bmi088_ReadAccMss(float acc[3])
{
	int16_t raw[3];
	bmi088_ReadAccRaw(raw);
	acc[0] = (float)raw[0] / 32768.0f * BMI088_ACC_RANGE_G * BMI088_G_TO_MSS;
	acc[1] = (float)raw[1] / 32768.0f * BMI088_ACC_RANGE_G * BMI088_G_TO_MSS;
	acc[2] = (float)raw[2] / 32768.0f * BMI088_ACC_RANGE_G * BMI088_G_TO_MSS;
}

/**
 * @brief  读陀螺仪（rad/s）
 * @param  gyr 输出 float[3]
 * @author camelliahm
 */
void bmi088_ReadGyrRad(float gyr[3])
{
	int16_t raw[3];
	bmi088_ReadGyrRaw(raw);
	gyr[0] = (float)raw[0] / 32768.0f * BMI088_GYR_RANGE_DPS * 0.0174533f;
	gyr[1] = (float)raw[1] / 32768.0f * BMI088_GYR_RANGE_DPS * 0.0174533f;
	gyr[2] = (float)raw[2] / 32768.0f * BMI088_GYR_RANGE_DPS * 0.0174533f;
}
