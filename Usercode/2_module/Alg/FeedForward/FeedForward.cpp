/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    FeedForward.cpp
  * @brief   前馈库
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "FeedForward.h"
#include <math.h>


/**
 * @brief 限幅函数
 * 
 * @param value 
 * @param low 
 * @param high 
 * @return float 
 */
float Class_FeedForward::Limit_Float(float value, float low, float high)
{
    if(value > high) return high;
    if(value < low)  return low;
    return value;
}

/**
 * @brief 取绝对值函数
 * 
 * @param value 
 * @return float 
 */
float Class_FeedForward::Abs_Float(float value)
{
    return (value >= 0.0f) ? value : -value;
}

/**
 * @brief 阻力前馈补偿 简单版 线性拟合
 * @param Target 目标
 * @param Friction_FeedForward_Value 固定阻力补偿值
 * @param Zero_Point_Continuation_Range 零点连续化范围，例如 0.5f
 * @return float 前馈值
 */
float Class_FeedForward::Friction_Feedforward_Simple(float Target,
                                                     float Friction_FeedForward_Value,
                                                     float Zero_Point_Continuation_Range)
{
    if(Zero_Point_Continuation_Range <= 0.0f)
    {
        if(Target > 0.0f) return Friction_FeedForward_Value;
        if(Target < 0.0f) return -Friction_FeedForward_Value;
        return 0.0f;
    }

    if(Target > Zero_Point_Continuation_Range)
    {
        return Friction_FeedForward_Value;
    }
    else if(Target < -Zero_Point_Continuation_Range)
    {
        return -Friction_FeedForward_Value;
    }
    else
    {
        // 零点附近线性过渡，避免抖动
        return Friction_FeedForward_Value / Zero_Point_Continuation_Range * Target;
    }
}

/**
 * @brief 阻力前馈补偿 简单版 线性拟合 增加正负值区分
 * @param Target 目标
 * @param Friction_FeedForward_Value 固定阻力补偿值
 * @param Zero_Point_Continuation_Range 零点连续化范围，例如 0.5f
 * @return float 前馈值
 */
float Class_FeedForward::Friction_Feedforward_Simple_Plus(float Target,
                                                          float Friction_FeedForward_Value,
                                                          float Friction_FeedForward_Value_Negative,
                                                          float Zero_Point_Continuation_Range)
{
    if(Zero_Point_Continuation_Range <= 0.0f)
    {
        if(Target > 0.0f) return Friction_FeedForward_Value;
        if(Target < 0.0f) return -Friction_FeedForward_Value_Negative;
        return 0.0f;
    }

    if(Target > Zero_Point_Continuation_Range)
    {
        return Friction_FeedForward_Value;
    }
    else if(Target < -Zero_Point_Continuation_Range)
    {
        return Friction_FeedForward_Value_Negative;
    }
    else
    {
        // 零点附近线性过渡，避免抖动
        return Friction_FeedForward_Value / Zero_Point_Continuation_Range * Target;
    }
}


/**
 * @brief 阻力前馈补偿 高级版 二次拟合
 * @param Target 目标值
 * @param a0 二次拟合系数
 * @param a1 二次拟合系数
 * @param a2 二次拟合系数
 * @param Zero_Point_Continuation_Range 零点连续化范围，例如 0.5f
 * @param Limit_Target_Value 前馈生效的最高目标值，例如 6.0f
 * @return float 前馈值
 */
float Class_FeedForward::Friction_Feedforward_Advanced(float Target,
                                                       float a0,
                                                       float a1,
                                                       float a2,
                                                       float Zero_Point_Continuation_Range,
                                                       float Limit_Target_Value)
{
    float Abs_Target = Abs_Float(Target);

    // 高速限幅：超过这个目标后，按这个目标算前馈，不再继续增大
    Abs_Target = Limit_Float(Abs_Target, 0.0f, Limit_Target_Value);

    // 计算当前目标值下的前馈绝对值
    float Feedforward_Abs = a0 + a1 * Abs_Target + a2 * Abs_Target * Abs_Target;

    // 防止拟合出现负值
    if(Feedforward_Abs < 0.0f)
    {
        Feedforward_Abs = 0.0f;
    }

    // 零点连续化：在零点附近用直线过渡
    if(Zero_Point_Continuation_Range > 0.0f && Abs_Target < Zero_Point_Continuation_Range)
    {
        float Zero_Point_Continuation_Value_Abs = a0 + a1 * Zero_Point_Continuation_Range + a2 * Zero_Point_Continuation_Range * Zero_Point_Continuation_Range;

        if(Zero_Point_Continuation_Value_Abs < 0.0f)
        {
            Zero_Point_Continuation_Value_Abs = 0.0f;
        }

        return (Zero_Point_Continuation_Value_Abs / Zero_Point_Continuation_Range) * Target;
    }

    if(Target > 0.0f)
    {
        return Feedforward_Abs;
    }
    else if(Target < 0.0f)
    {
        return -Feedforward_Abs;
    }
    else
    {
        return 0.0f;
    }
}