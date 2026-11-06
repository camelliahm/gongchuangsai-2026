#ifndef __BMI088_H
#define __BMI088_H

#include "stm32f4xx.h"

//BMI088 六轴 IMU 驱动（软件 SPI 模拟时序）

/*---- 软件 SPI 引脚----*/
#define BMI088_SCK_PORT		GPIOA			//SCK 所在端口
#define BMI088_SCK_PIN		GPIO_Pin_5		//SCK 引脚（PA5）
#define BMI088_SDI_PORT		GPIOA			//MOSI 所在端口
#define BMI088_SDI_PIN		GPIO_Pin_7		//MOSI 引脚（PA7，芯片 SDI/SDA）
#define BMI088_SDO_PORT		GPIOA			//MISO 所在端口（SDO1/SDO2 共用一根）
#define BMI088_SDO_PIN		GPIO_Pin_6		//MISO 引脚（PA6）
#define BMI088_CSB1_PORT	GPIOB			//加速度计片选所在端口
#define BMI088_CSB1_PIN		GPIO_Pin_0		//CSB1（加速度计，PB0）
#define BMI088_CSB2_PORT	GPIOB			//陀螺仪片选所在端口
#define BMI088_CSB2_PIN		GPIO_Pin_1		//CSB2（陀螺仪，PB1）

/*---- 芯片 ID（自检用）----*/
#define BMI088_ACC_ID		(0x1Eu)			//加速度计 CHIP_ID
#define BMI088_GYR_ID		(0x0Fu)			//陀螺仪 CHIP_ID

/*---- 量程标定：原始 int16 → 物理单位 ----*/
#define BMI088_ACC_RANGE_G		(3.0f)		//加速度计量程 ±3g
#define BMI088_GYR_RANGE_DPS	(2000.0f)	//陀螺仪量程 ±2000°/s
#define BMI088_G_TO_MSS			(9.80665f)	//g → m/s²

//初始化（GPIO、软复位、量程/ODR 配置），返回 true = 通过 ID 自检
bool bmi088_Init(void);

//读加速度计原始值（int16[3]，单位 = 量程/32768 g）
void bmi088_ReadAccRaw(int16_t acc[3]);

//读陀螺仪原始值（int16[3]，单位 = 量程/32768 °/s）
void bmi088_ReadGyrRaw(int16_t gyr[3]);

//读加速度计（m/s²）
void bmi088_ReadAccMss(float acc[3]);

//读陀螺仪（rad/s）
void bmi088_ReadGyrRad(float gyr[3]);

#endif
