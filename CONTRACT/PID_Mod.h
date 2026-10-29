#ifndef _PID_MOD_H
#define _PID_MOD_H

#include "stdint.h"
#include "Emm_V5.h"

/**
 * @brief PID改进算法位掩码枚举
 * @note  通过位或运算组合多个功能，如 Integral_Limit | Derivative_On_Measurement
 *        每个位对应一个独立的开关，不影响核心逻辑
 */
typedef enum pid_Improvement_e
{
    NONE = 0X00,                        // 0000 0000 纯基础位置式PID（不推荐单独使用）
    Integral_Limit = 0x01,              // 0000 0001 积分限幅+抗积分饱和（所有环必开）
    Derivative_On_Measurement = 0x02,   // 0000 0010 微分先行（所有环必开，解决目标突变冲击）
    Trapezoid_Intergral = 0x04,         // 0000 0100 梯形积分（仅速度环开，减少超调）
    Fallback_Diff = 0x08,               // 0000 1000 死区内误差微分阻尼（替代输出保持）
    OutputFilter = 0x10,                // 0001 0000 输出滤波（会增加延迟）
    ChangingIntegralRate = 0x20,        // 0010 0000 变速积分（位置环可选，大误差减小积分）
    DerivativeFilter = 0x40,            // 0100 0000 微分滤波（所有环必开，滤除传感器噪声）
    ErrorHandle = 0x80,                 // 1000 0000 电机堵转检测（可选，注意有除以0bug）
} PID_Improvement_e;

/**
 * @brief PID错误类型枚举
 */
typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,  // 无错误
    Motor_Blocked = 0x01U    // 电机堵转
} ErrorType_e;

/**
 * @brief PID错误处理结构体
 * @note  保存错误计数和错误类型，用于安全保护
 */
typedef struct
{
    uint64_t ERRORCount;     // 错误累计计数
    ErrorType_e ERRORType;   // 当前错误类型
} PID_ErrorHandler_t;

/**
 * @brief PID核心结构体
 * @note  每个PID实例独立拥有一个该结构体，完全隔离状态
 *        成员分为5类：配置参数、运行状态、中间变量、错误处理、函数指针
 */
typedef struct _PID_TypeDef
{
    // -------------------------- 配置参数（初始化后一般不变） --------------------------
    float Target;                          // 当前目标值
    float LastNoneZeroTarget;              // 上一个非零目标值（预留，本代码未使用）
    float Kp;                              // 比例系数
    float Ki;                              // 积分系数
    float Kd;                              // 微分系数

    // -------------------------- 运行状态（每次计算自动更新） --------------------------
    float Measure;                         // 当前测量值（编码器/传感器读数）
    float Last_Measure;                    // 上一次测量值
    float Err;                             // 当前误差 = 目标值 - 测量值
    float Last_Err;                        // 上一次误差

    // -------------------------- 中间计算变量 --------------------------
    float Pout;                            // 比例项输出
    float Iout;                            // 积分项累计输出（位置式PID核心）
    float Dout;                            // 微分项输出
    float ITerm;                           // 本次积分增量（每次计算的积分值）

    float Output;                          // PID最终输出
    float Last_Output;                     // 上一次最终输出
    float Last_Dout;                       // 上一次微分项输出

    // -------------------------- 限制与滤波参数 --------------------------
    float MaxOut;                          // 输出最大值（如PWM最大值1000）
    float IntegralLimit;                   // 积分项最大值（一般为MaxOut的10%-30%）
    float DeadBand;                        // 死区（误差小于该值时停止调节，减少抖动）
    float ControlPeriod;                   // 控制周期（预留，本代码未使用，需手动添加）
    float MaxErr;                          // 最大误差（预留，本代码未使用）
    float ScalarA;                         // 变速积分参数A
    float ScalarB;                         // 变速积分参数B
    float Output_Filtering_Coefficient;   // 输出滤波系数（1.0=不滤波）
    float Derivative_Filtering_Coefficient;// 微分滤波系数（0.2-0.7，越小滤波越强）

    // -------------------------- 时间相关（预留，需手动添加控制周期无关性） --------------------------
    uint32_t thistime;                     // 当前时间戳（ms）
    uint32_t lasttime;                     // 上一次计算时间戳（ms）
    uint8_t dtime;                         // 两次计算的时间差（ms）

    // -------------------------- 功能开关 --------------------------
    uint8_t Improve;                       // 改进算法位掩码（PID_Improvement_e组合）

    // -------------------------- 错误处理 --------------------------
    PID_ErrorHandler_t ERRORHandler;       // 错误处理结构体

    // -------------------------- 函数指针（预留扩展接口） --------------------------
    // PID参数初始化函数指针
    void (*PID_param_init)(
        struct _PID_TypeDef *pid,
        uint16_t maxOut,
        uint16_t integralLimit,
        float deadband,
        float Kp,
        float ki,
        float kd,
        float A,
        float B,
        float output_filtering_coefficient,
        float derivative_filtering_coefficient,
        uint8_t improve);

    // PID参数重置函数指针
    void (*PID_reset)(
        struct _PID_TypeDef *pid,
        float Kp,
        float ki,
        float kd);
} PID_TypeDef;

// -------------------------- 内部静态函数声明（仅在pid.c中调用） --------------------------
static void f_Trapezoid_Intergral(PID_TypeDef *pid);    // 梯形积分
static void f_Integral_Limit(PID_TypeDef *pid);         // 积分限幅+抗积分饱和
static void f_Derivative_On_Measurement(PID_TypeDef *pid); // 微分先行
static void f_Changing_Integral_Rate(PID_TypeDef *pid); // 变速积分
static void f_Output_Filter(PID_TypeDef *pid);          // 输出滤波
static void f_Derivative_Filter(PID_TypeDef *pid);      // 微分滤波
static void f_Output_Limit(PID_TypeDef *pid);           // 输出限幅
static void f_Proportion_Limit(PID_TypeDef *pid);       // 比例项限幅（仅用于调试图表）
static void f_PID_ErrorHandle(PID_TypeDef *pid);        // 电机堵转检测

// -------------------------- 对外接口函数声明 --------------------------
/**
 * @brief  PID初始化函数
 * @param  pid                  PID实例指针
 * @param  max_out              输出最大值
 * @param  intergral_limit      积分限幅值
 * @param  deadband             死区值
 * @param  kp                   比例系数
 * @param  ki                   积分系数
 * @param  kd                   微分系数
 * @param  A                    变速积分参数A
 * @param  B                    变速积分参数B
 * @param  output_filtering_coefficient 输出滤波系数
 * @param  derivative_filtering_coefficient 微分滤波系数
 * @param  improve              改进算法位掩码
 * @note   每个PID实例必须先调用此函数初始化才能使用
 */
void PID_Init(
    PID_TypeDef *pid,
    uint16_t max_out,
    uint16_t intergral_limit,
    float deadband,
    float kp,
    float ki,
    float kd,
    float A,
    float B,
    float output_filtering_coefficient,
    float derivative_filtering_coefficient,
    uint8_t improve);

/**
 * @brief  PID核心计算函数
 * @param  pid                  PID实例指针
 * @param  measure              当前测量值
 * @param  target               当前目标值
 * @return float                PID最终输出
 * @note   必须按固定周期调用（如1ms一次），双环串级先算外环再算内环
 */
float PID_Calculate(PID_TypeDef *pid, float measure, float target);
		
#endif
