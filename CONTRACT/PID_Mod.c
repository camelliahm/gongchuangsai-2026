#include "PID_Mod.h"          // 模块化PID头文件，包含结构体、枚举、函数声明

/**
 * @brief  PID参数初始化函数（内部静态函数，通过函数指针调用）
 * @param  pid    PID实例指针
 * @param  max_out            输出限幅值（绝对值）
 * @param  intergral_limit    积分限幅值（绝对值）
 * @param  deadband           死区大小
 * @param  kp                 比例增益
 * @param  Ki                 积分增益
 * @param  Kd                 微分增益
 * @param  Changing_Integral_A  变速积分参数A（过渡区宽度）
 * @param  Changing_Integral_B  变速积分参数B（全积分区宽度）
 * @param  output_filtering_coefficient      输出滤波系数 (0~1)
 * @param  derivative_filtering_coefficient  微分滤波系数 (0~1)
 * @param  improve            改进功能按位掩码组合
 * @note   该函数将PID所有参数装入结构体，并清零关键状态变量
 */
static void f_PID_param_init(
    PID_TypeDef *pid,
    uint16_t max_out,
    uint16_t intergral_limit,
    float deadband,
    float kp,
    float Ki,
    float Kd,
    float Changing_Integral_A,
    float Changing_Integral_B,
    float output_filtering_coefficient,
    float derivative_filtering_coefficient,
    uint8_t improve)
{
    // ---------- 限制与死区参数 ----------
    pid->DeadBand = deadband;                 // 死区：误差小于该值时不做PID计算，直接跳过
    pid->IntegralLimit = intergral_limit;     // 积分限幅：积分输出Iout的绝对值上限
    pid->MaxOut = max_out;                    // 输出限幅：最终Output的绝对值上限
    pid->MaxErr = max_out * 2;                // 最大误差（用于错误处理，如堵转检测）
    pid->Target = 0;                          // 目标值初始化为0，避免随机值

    // ---------- PID增益 ----------
    pid->Kp = kp;                             // 比例增益
    pid->Ki = Ki;                             // 积分增益
    pid->Kd = Kd;                             // 微分增益
    pid->ITerm = 0;                           // 本次积分增量清零

    // ---------- 变速积分参数 ----------
    pid->ScalarA = Changing_Integral_A;       // 变速积分过渡区宽度
    pid->ScalarB = Changing_Integral_B;       // 全积分区宽度

    // ---------- 滤波器系数 ----------
    pid->Output_Filtering_Coefficient = output_filtering_coefficient;           // 输出低通滤波系数
    pid->Derivative_Filtering_Coefficient = derivative_filtering_coefficient;   // 微分低通滤波系数

    // ---------- 改进功能开关 ----------
    pid->Improve = improve;                   // 按位组合的各种改进使能

    // ---------- 错误处理初始化 ----------
    pid->ERRORHandler.ERRORCount = 0;         // 错误计数器清零
    pid->ERRORHandler.ERRORType = PID_ERROR_NONE; // 错误类型设为无错误

    // ---------- 输出初始化 ----------
    pid->Output = 0;                          // 最终输出清零，确保首次安全
}

/**
 * @brief  PID参数在线重置函数（静态，运行时修改增益用）
 * @param  pid    PID实例指针
 * @param  Kp     新的比例系数
 * @param  Ki     新的积分系数
 * @param  Kd     新的微分系数
 * @note   修改Kp,Ki,Kd时调用，若Ki被设为0则自动清零积分输出Iout
 */
static void f_PID_reset(PID_TypeDef *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;                             // 更新比例增益
    pid->Ki = Ki;                             // 更新积分增益
    pid->Kd = Kd;                             // 更新微分增益

    // 若积分增益被设为0，说明不再需要积分控制，为避免残留积分造成影响，清零Iout
    if (pid->Ki == 0)
        pid->Iout = 0;
}

/**
 * @brief  PID核心计算函数（最重要的函数，执行顺序严格，不能乱）
 * @param  pid       PID实例指针
 * @param  measure   当前测量值
 * @param  target    期望目标值
 * @return float     计算后的控制输出
 * @note   整体流程：
 *         1. 错误检测（如堵转保护）
 *         2. 更新测量值、目标值，计算本次误差
 *         3. 死区判断：误差过小则跳过主计算，可选Fallback_Diff做微分阻尼
 *         4. 正常计算：比例、积分、微分基本值
 *         5. 按顺序执行各种改进算法（梯形积分→变积分→积分限幅→微分先行→微分滤波）
 *         6. 累加积分、合成输出，输出滤波、输出限幅
 *         7. 更新历史状态（Last_Measure, Last_Err等）
 */
float PID_Calculate(PID_TypeDef *pid, float measure, float target)
{
    // -------------------------- 第一步：错误处理前置 --------------------------
    // 检查 Improve 掩码中是否启用了 ErrorHandle（堵转保护）
    if (pid->Improve & ErrorHandle)
    {
        f_PID_ErrorHandle(pid);               // 执行错误判断（累计异常状态）
        // 若已判定为故障（如Motor_Blocked），强制输出0并返回
        if (pid->ERRORHandler.ERRORType != PID_ERROR_NONE)
        {
            pid->Output = 0;
            return 0;
        }
    }

    // -------------------------- 第二步：更新测量值、目标值，计算误差 --------------------------
    pid->Measure = measure;                   // 存入当前测量值
    pid->Target = target;                     // 存入当前目标值
    pid->Err = pid->Target - pid->Measure;    // 计算误差（目标 - 测量）

    // -------------------------- 第三步：死区判断 --------------------------
    // 若误差绝对值大于死区，进入正常PID计算；否则跳过，保持上一周期输出（或使用Fallback）
    if (ABS(pid->Err) > pid->DeadBand)
    {
        // -------------------------- 第四步：基础PID计算 --------------------------
        pid->Pout = pid->Kp * pid->Err;                    // 比例输出 = Kp * 误差
        pid->ITerm = pid->Ki * pid->Err;                   // 本次积分增量 = Ki * 误差（后续可能被改进算法修改）
        pid->Dout = pid->Kd * (pid->Err - pid->Last_Err);  // 微分输出 = Kd * 误差变化量

        // 以下按顺序执行积分相关改进，每一步都可能修改 ITerm 或 Iout
        // 顺序：梯形积分 → 变积分速率 → 积分限幅
        if (pid->Improve & Trapezoid_Intergral)
        {
            f_Trapezoid_Intergral(pid);        // 梯形积分重算 ITerm
        }
        
        if (pid->Improve & ChangingIntegralRate)
        {
            f_Changing_Integral_Rate(pid);     // 根据误差大小缩放或清零 ITerm
        }

        if (pid->Improve & Integral_Limit)
        {
            f_Integral_Limit(pid);             // 积分总量限幅，必要时清零 ITerm

        }
            
        // 以下按顺序执行微分相关改进，每一步可能修改 Dout
        // 顺序：微分先行 → 微分滤波
        if (pid->Improve & Derivative_On_Measurement)
        {
            f_Derivative_On_Measurement(pid);  // 改用测量值微分，消除设定值突变冲击

        }          	
		if (pid->Improve & DerivativeFilter)
		{
			f_Derivative_Filter(pid);
		}
        // 累加本次积分增量到积分总量 Iout
        pid->Iout += pid->ITerm;

        // 合成最终输出 = 比例 + 积分 + 微分
        pid->Output = pid->Pout + pid->Iout + pid->Dout;

        // 输出滤波（低通平滑）
		if (pid->Improve & OutputFilter)
		{
			f_Output_Filter(pid);
		}
        // 输出硬限幅
        f_Output_Limit(pid);

        // 比例项单独限幅
        f_Proportion_Limit(pid);
    }
    else  // 误差小于死区，进入此分支
    {
        // 检查是否启用了 Fallback_Diff 功能（死区内使用误差微分代替输出保持）
        if (pid->Improve & Fallback_Diff)
        {
            // 死区内积分清零，防止误差极小但积分残留
            pid->Iout   = 0.0f;
            pid->ITerm  = 0.0f;
            pid->Pout   = 0.0f;               // 比例项也清零

            // 计算误差的微分（基于原始误差变化量）
            pid->Dout   = pid->Kd * (pid->Err - pid->Last_Err);

            // 如果启用了微分滤波，对刚算出的误差微分进行滤波
			if (pid->Improve & DerivativeFilter)
			{
				f_Derivative_Filter(pid);
			}
            // 输出仅由微分项构成（提供微小阻尼）
            pid->Output = pid->Dout;

            // 如果启用了输出滤波，同样对微分输出进行平滑
			if (pid->Improve & OutputFilter)
			{
				f_Output_Filter(pid);
			}
                
            // 限幅，确保安全
            f_Output_Limit(pid);
            f_Proportion_Limit(pid);
        }
        // 如果未启用 Fallback_Diff，则完全不做计算，Output 保持上一周期的值（即“输出保持”）
    }

    // -------------------------- 第七步：更新所有历史状态 --------------------------
    pid->Last_Measure = pid->Measure;         // 保存本次测量值，供下次微分先行使用
    pid->Last_Output  = pid->Output;          // 保存本次输出，供输出滤波使用
    pid->Last_Dout    = pid->Dout;            // 保存本次微分输出，供微分滤波使用
    pid->Last_Err     = pid->Err;             // 保存本次误差，供下次微分计算

    return pid->Output;                       // 返回最终控制量
}

/**
 * @brief  梯形积分
 * @note   用本次误差和上次误差的平均值计算积分增量，抑制随机误差，使积分更平滑
 */
static void f_Trapezoid_Intergral(PID_TypeDef *pid)
{
    pid->ITerm = pid->Ki * ((pid->Err + pid->Last_Err) / 2);
}

/**
 * @brief  变积分速率
 * @note   根据误差大小动态调整积分速度：
 *         |Err| <= B     → 保持原 ITerm（全积分）
 *         B < |Err| <= A+B → 线性减弱积分 ITerm
 *         |Err| > A+B     → 积分清零 ITerm = 0
 *         仅当误差与积分输出同向时才调整，避免反向时错误削弱积分
 */
static void f_Changing_Integral_Rate(PID_TypeDef *pid)
{
    // 只有当前误差与积分输出同向（即积分正在增大超调方向）才进行削弱
    if (pid->Err * pid->Iout > 0)
    {
        // 误差在全积分区内，不修改 ITerm，直接返回
        if (ABS(pid->Err) <= pid->ScalarB)
            return;
        // 误差在过渡区，线性减弱积分
        if (ABS(pid->Err) <= (pid->ScalarA + pid->ScalarB))
            pid->ITerm *= (pid->ScalarA - ABS(pid->Err) + pid->ScalarB) / pid->ScalarA;
        else
            // 误差超过上限，完全关闭积分
            pid->ITerm = 0;
    }
}

/**
 * @brief  积分限幅
 * @note   两重保护：
 *         1. 若预测本次累加后总输出会超过 MaxOut，且积分方向与误差同向，则取消本次积分增量
 *         2. 积分总量 Iout 超过 IntegralLimit 时直接钳位，并清零 ITerm
 */
static void f_Integral_Limit(PID_TypeDef *pid)
{
    float temp_Output, temp_Iout;
    temp_Iout = pid->Iout + pid->ITerm;                     // 预估累加后的积分总量
    temp_Output = pid->Pout + pid->Iout + pid->Dout;       // 当前已有输出（未加本次积分增量）

    // 若已有输出绝对值已超限，且本次积分增量会使输出继续超限方向增加，则丢弃本次积分增量
    if (ABS(temp_Output) > pid->MaxOut)
    {
        if (pid->Err * pid->Iout > 0)   // 积分方向与误差同向（继续恶化）
        {
            pid->ITerm = 0;
        }
    }

    // 积分总量单独限幅
    if (temp_Iout > pid->IntegralLimit)
    {
        pid->ITerm = 0;                              // 本次增量清零
        pid->Iout = pid->IntegralLimit;              // 积分总量钳位至上限
    }
    if (temp_Iout < -pid->IntegralLimit)
    {
        pid->ITerm = 0;
        pid->Iout = -pid->IntegralLimit;
    }
}

/**
 * @brief  微分先行
 * @note   将微分从“误差变化率”改为“测量值变化率”，避免目标值突变引起微分冲击
 */
static void f_Derivative_On_Measurement(PID_TypeDef *pid)
{
    pid->Dout = pid->Kd * (pid->Last_Measure - pid->Measure);
}

/**
 * @brief  微分滤波
 * @note   对 Dout 进行一阶低通滤波，抑制高频噪声
 *         新Dout = 滤波系数 * 本次Dout + (1-系数) * 上次Dout
 */
static void f_Derivative_Filter(PID_TypeDef *pid)
{
    pid->Dout = pid->Dout * pid->Derivative_Filtering_Coefficient +
                pid->Last_Dout * (1 - pid->Derivative_Filtering_Coefficient);
}

/**
 * @brief  输出滤波
 * @note   对最终 Output 进行一阶低通滤波，平滑控制指令
 */
static void f_Output_Filter(PID_TypeDef *pid)
{
    pid->Output = pid->Output * pid->Output_Filtering_Coefficient +
                  pid->Last_Output * (1 - pid->Output_Filtering_Coefficient);
}

/**
 * @brief  输出限幅
 * @note   将 Output 限制在 [-MaxOut, MaxOut] 范围内
 */
static void f_Output_Limit(PID_TypeDef *pid)
{
    if (pid->Output > pid->MaxOut)   pid->Output = pid->MaxOut;
    if (pid->Output < -(pid->MaxOut)) pid->Output = -(pid->MaxOut);
}

/**
 * @brief  比例项限幅
 * @note   单独限制 Pout 到正负 MaxOut，防止显示或监控时曲线过冲（实际控制输出由 Output 决定）
 */
static void f_Proportion_Limit(PID_TypeDef *pid)
{
    if (pid->Pout > pid->MaxOut)   pid->Pout = pid->MaxOut;
    if (pid->Pout < -(pid->MaxOut)) pid->Pout = -(pid->MaxOut);
}

/**
 * @brief  错误处理（堵转保护）
 * @note   当输出高于 1% 最大输出，且长时间（连续1000次）测量值与目标值偏差超过90%，
 *         判定为电机堵转，设置 ERRORType = Motor_Blocked
 */
static void f_PID_ErrorHandle(PID_TypeDef *pid)
{
    // 输出太小，不进行堵转检测（可能正常待机）
    if (pid->Output < pid->MaxOut * 0.01f)
        return;

    // 偏差超过目标的90%，开始累计错误计数
    if ((ABS(pid->Target - pid->Measure) / pid->Target) > 0.9f)
    {
        pid->ERRORHandler.ERRORCount++;
    }
    else
    {
        pid->ERRORHandler.ERRORCount = 0;    // 偏差正常，重置计数
    }

    // 连续1000次以上异常，判定为堵转
    if (pid->ERRORHandler.ERRORCount > 1000)
    {
        pid->ERRORHandler.ERRORType = Motor_Blocked;
    }
}

/**
 * @brief  用户调用的PID初始化函数（绑定函数指针并执行初始化）
 * @param  pid      PID实例指针
 * @param  其余参数同 f_PID_param_init
 * @note   必须先调用此函数，才能使用 PID_Calculate
 */
void PID_Init(
    PID_TypeDef *pid,
    uint16_t max_out,
    uint16_t intergral_limit,
    float deadband,
    float kp,
    float Ki,
    float Kd,
    float A,
    float B,
    float output_filtering_coefficient,
    float derivative_filtering_coefficient,
    uint8_t improve)
{
    // 绑定内部静态函数到结构体的函数指针，实现“类方法”调用
    pid->PID_param_init = f_PID_param_init;
    pid->PID_reset      = f_PID_reset;

    // 立即调用参数初始化函数，将传入参数写入结构体
    pid->PID_param_init(pid, max_out, intergral_limit, deadband,
                        kp, Ki, Kd, A, B,
                        output_filtering_coefficient, derivative_filtering_coefficient,
                        improve);
}
