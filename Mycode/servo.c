#include "servo.h"

//通道 → 复用源（PC6/PC7/PC8 依次对应 TIM3 CH1/CH2/CH3）
static const uint16_t s_src[SERVO_NUM] = { GPIO_PinSource6, GPIO_PinSource7, GPIO_PinSource8 };	//三路舵机的引脚源（CH1 / CH2 / CH3）

//每通道量程：CH1/CH2 是 180° 舵机，CH3 是 270° 舵机（车载物料盘）
static const float s_degMax[SERVO_NUM]   = { SERVO_ANGLE_MAX, SERVO_ANGLE_MAX, SERVO_ANGLE_MAX_CH3 };		//各通道角度量程(°)
static const uint16_t s_usMin[SERVO_NUM] = { SERVO_PULSE_MIN_US, SERVO_PULSE_MIN_US, SERVO_PULSE_MIN_US_CH3 };	//各通道最小脉宽(us)
static const uint16_t s_usMax[SERVO_NUM] = { SERVO_PULSE_MAX_US, SERVO_PULSE_MAX_US, SERVO_PULSE_MAX_US_CH3 };	//各通道最大脉宽(us)

static float s_angle[SERVO_NUM];	//当前设定角度（非实际反馈）

/**
 * @brief  三路舵机 PWM 初始化
 * @note   TIM3 CH1/CH2/CH3 = PC6/PC7/PC8；PSC 取 84-1 使计数 1MHz，故脉宽 us 可直接写 CCR
 * @author camelliahm
 */
void servo_Init(void)
{
	GPIO_InitTypeDef g;
	TIM_TimeBaseInitTypeDef t;
	TIM_OCInitTypeDef o;
	uint8_t i;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	//PC6/PC7/PC8 复用为 TIM3 推挽输出
	g.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	g.GPIO_Mode  = GPIO_Mode_AF;
	g.GPIO_OType = GPIO_OType_PP;
	g.GPIO_PuPd  = GPIO_PuPd_UP;
	g.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &g);

	for (i = 0; i < SERVO_NUM; i++)
	{
		GPIO_PinAFConfig(GPIOC, s_src[i], GPIO_AF_TIM3);
		s_angle[i] = 0.0f;
	}

	//时基：1MHz 计数，周期 20ms（50Hz）
	t.TIM_Prescaler     = SERVO_TIM_PSC;
	t.TIM_Period        = SERVO_TIM_ARR;
	t.TIM_ClockDivision = TIM_CKD_DIV1;
	t.TIM_CounterMode   = TIM_CounterMode_Up;
	t.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIM3, &t);

	//三通道 PWM1，初值取最小脉宽
	TIM_OCStructInit(&o);				//先填默认值，避免栈上未初始化成员被读到
	o.TIM_OCMode      = TIM_OCMode_PWM1;
	o.TIM_OutputState = TIM_OutputState_Enable;
	o.TIM_OCPolarity  = TIM_OCPolarity_High;
	o.TIM_Pulse       = SERVO_PULSE_MIN_US;
	TIM_OC1Init(TIM3, &o);
	TIM_OC2Init(TIM3, &o);
	TIM_OC3Init(TIM3, &o);

	TIM_Cmd(TIM3, ENABLE);
}

/**
 * @brief  设置通道脉宽
 * @param  ch 通道号 0~SERVO_NUM-1
 * @param  us 脉宽(us)，越界截断到标定范围
 * @retval true = 已设置；false = 通道号非法
 * @author camelliahm
 */
bool servo_SetPulse(uint8_t ch, uint16_t us)
{
	if (ch >= SERVO_NUM)
	{
		return false;
	}

	if (us < s_usMin[ch]) { us = s_usMin[ch]; }
	if (us > s_usMax[ch]) { us = s_usMax[ch]; }

	switch (ch)
	{
	case 0:  TIM_SetCompare1(TIM3, us); break;
	case 1:  TIM_SetCompare2(TIM3, us); break;
	default: TIM_SetCompare3(TIM3, us); break;
	}

	s_angle[ch] = (float)(us - s_usMin[ch]) * s_degMax[ch]
				/ (float)(s_usMax[ch] - s_usMin[ch]);
	return true;
}

/**
 * @brief  设置通道角度
 * @param  ch  通道号 0~SERVO_NUM-1
 * @param  deg 角度(度)，越界截断到 0~该通道量程（见 s_degMax）
 * @retval true = 已设置；false = 通道号非法
 * @note   脉宽与角度按该通道的 s_usMin/s_usMax 线性映射，换舵机只改 servo.h 的标定宏
 * @author camelliahm
 */
bool servo_SetAngle(uint8_t ch, float deg)
{
	uint16_t us;

	if (ch >= SERVO_NUM)
	{
		return false;
	}

	if (deg < 0.0f)           { deg = 0.0f; }
	if (deg > s_degMax[ch])   { deg = s_degMax[ch]; }

	us = (uint16_t)((float)s_usMin[ch]
		+ (float)(s_usMax[ch] - s_usMin[ch]) * (deg / s_degMax[ch]) + 0.5f);

	(void)servo_SetPulse(ch, us);
	s_angle[ch] = deg;
	return true;
}

/**
 * @brief  读回通道当前设定角度
 * @note   读的是设定值，不是舵机实际角度（无反馈线）
 * @author camelliahm
 */
float servo_GetAngle(uint8_t ch)
{
	return (ch < SERVO_NUM) ? s_angle[ch] : 0.0f;
}

/**
 * @brief  关闭某通道 PWM 输出
 * @note   只影响该通道，其它通道照常输出；重新 SetPulse/SetAngle 不会自动恢复输出
 * @author camelliahm
 */
void servo_Disable(uint8_t ch)
{
	if (ch >= SERVO_NUM)
	{
		return;
	}

	switch (ch)
	{
	case 0:  TIM_CCxCmd(TIM3, TIM_Channel_1, TIM_CCx_Disable); break;
	case 1:  TIM_CCxCmd(TIM3, TIM_Channel_2, TIM_CCx_Disable); break;
	default: TIM_CCxCmd(TIM3, TIM_Channel_3, TIM_CCx_Disable); break;
	}
}
