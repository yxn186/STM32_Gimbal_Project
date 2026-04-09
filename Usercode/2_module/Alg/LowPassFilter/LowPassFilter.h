/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    LowPassFilter.h
  * @brief   This file contains all the function prototypes for
  *          the LowPassFilter.cpp file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LOWPASSFILTER_H__
#define __LOWPASSFILTER_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*YOUR CODE*/

/**
 * @brief 一阶 IIR 低通滤波器
 * @details
 * 滤波公式：
 *     y[k] = Alpha * x[k] + (1 - Alpha) * y[k-1]
 *
 * 特点：
 * - 实现简单
 * - 占用资源少
 * - 适合 STM32 实时控制
 * - 可用于传感器滤波、速度目标滤波等
 */
class Class_LowPassFilter
{
public:

    /**
     * @brief 构造函数
     */
    Class_LowPassFilter()
        : Alpha(1.0f),
          Y_Prev(0.0f),
          Initialized(false)
    {
    }

    /**
     * @brief 用截止频率和采样周期配置滤波器
     * @param Cutoff_Hz 截止频率，单位 Hz
     * @param Dt 采样周期，单位 s
     * @note
     * 例如：
     * 1ms 调用一次 Update，则 Dt = 0.001f
     */
    void Configure(float Cutoff_Hz, float Dt);

    /**
     * @brief 直接设置滤波系数 Alpha
     * @param alpha 滤波系数，范围建议 0~1
     * @note
     * - Alpha 越大：响应越快，滤波越弱
     * - Alpha 越小：响应越慢，滤波越强
     */
    void Set_Alpha(float alpha)
    {
        Alpha = Limit_Float(alpha, 0.0f, 1.0f);
    }

    /**
     * @brief 输入一个新采样点，返回滤波结果
     * @param Input 当前输入值
     * @return float 当前滤波输出值
     */
    float Update(float Input);

    /**
     * @brief 重置滤波器状态
     */
    void Reset(void)
    {
        Initialized = false;
        Y_Prev = 0.0f;
    }

    /**
     * @brief 重置滤波器状态并设置初始值
     * @param Value 初始值
     */
    void Reset(float Value)
    {
        Initialized = true;
        Y_Prev = Value;
    }

    /**
     * @brief 获取当前滤波值
     * @return float 当前滤波值
     */
    float Get_Value(void) const
    {
        return Y_Prev;
    }

    /**
     * @brief 获取当前 Alpha
     * @return float 当前 Alpha
     */
    float Get_Alpha(void) const
    {
        return Alpha;
    }

private:

    /**
     * @brief 浮点限幅
     * @param Value 输入值
     * @param Low 下限
     * @param High 上限
     * @return float 限幅后的值
     */
    float Limit_Float(float Value, float Low, float High);

private:
    float Alpha;       ///< 滤波系数
    float Y_Prev;      ///< 上一次输出值
    bool Initialized;  ///< 是否已初始化
};


#endif /* __LOWPASSFILTER_H__ */
