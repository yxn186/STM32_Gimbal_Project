/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    FeedForward.h
  * @brief   This file contains all the function prototypes for
  *          the FeedForward.cpp file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FEEDFORWARD_H__
#define __FEEDFORWARD_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*YOUR CODE*/
class Class_FeedForward
{
    public:

    /**
    * @brief 阻力前馈补偿 简单版
    * @param Target 目标
    * @param Friction_FeedForward_Value 固定阻力补偿值
    * @param Zero_Point_Continuation_Range 零点连续化范围，例如 0.5f
    * @return float 前馈值
    */
    float Friction_Feedforward_Simple(float Target,float Friction_FeedForward_Value,float Zero_Point_Continuation_Range);

    /**
    * @brief 阻力前馈补偿 简单版 线性拟合 增加正负值区分
    * @param Target 目标
    * @param Friction_FeedForward_Value 固定阻力补偿值
    * @param Zero_Point_Continuation_Range 零点连续化范围，例如 0.5f
    * @return float 前馈值
    */
    float Friction_Feedforward_Simple_Plus(float Target,float Friction_FeedForward_Value,float Friction_FeedForward_Value_Negative,float Zero_Point_Continuation_Range);
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
    float Friction_Feedforward_Advanced(float Target,float a0,float a1,float a2,float Zero_Point_Continuation_Range,float Limit_Target_Value);
                                        
    protected:
    
    /**
    * @brief 限幅函数
    * 
    * @param value 
    * @param low 
    * @param high 
    * @return float 
    */
    float Limit_Float(float value, float low, float high);

    /**
    * @brief 取绝对值函数
    * 
    * @param value 
    * @return float 
    */
    float Abs_Float(float value);

};






#ifdef __cplusplus
}
#endif

#endif /* __FEEDFORWARD_H__ */
