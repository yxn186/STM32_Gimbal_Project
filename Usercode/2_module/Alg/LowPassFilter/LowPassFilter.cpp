/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    LowPassFilter.cpp
  * @brief   低通滤波
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "LowPassFilter.h"

/**
 * @brief 用截止频率和采样周期配置滤波器
 * @param Cutoff_Hz 截止频率，单位 Hz
 * @param Dt 采样周期，单位 s
 * @note
 * 例如：
 * 1ms 调用一次 Update，则 Dt = 0.001f
 */
void Class_LowPassFilter::Configure(float Cutoff_Hz, float Dt)
{
    if (Cutoff_Hz <= 0.0f || Dt <= 0.0f)
    {
        Alpha = 1.0f;
        return;
    }
    const float PI = 3.14159265358979f;
    float RC = 1.0f / (2.0f * PI * Cutoff_Hz);
    Alpha = Dt / (RC + Dt);
    Alpha = Limit_Float(Alpha, 0.0f, 1.0f);
}

/**
 * @brief 输入一个新采样点，返回滤波结果
 * @param Input 当前输入值
 * @return float 当前滤波输出值
 */
float Class_LowPassFilter::Update(float Input)
{
    if (Initialized == false)
    {
        Y_Prev = Input;
        Initialized = true;
        return Y_Prev;
    }
    Y_Prev = Alpha * Input + (1.0f - Alpha) * Y_Prev;
    return Y_Prev;
}

/**
 * @brief 浮点限幅
 * @param Value 输入值
 * @param Low 下限
 * @param High 上限
 * @return float 限幅后的值
 */
float Class_LowPassFilter::Limit_Float(float Value, float Low, float High)
{
    if (Value > High) return High;
    if (Value < Low)  return Low;
    return Value;
}