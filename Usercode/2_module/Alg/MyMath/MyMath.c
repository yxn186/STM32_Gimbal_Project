/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    MyMath.c
  * @brief   数学计算相关库
  *
  *          提供嵌入式开发中常用的数学工具函数，包括：
  *          - 角度 / 弧度归一化（Wrap）
  *          - 通用数值区间映射
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "MyMath.h"
#include <math.h>

/* 角度归一化 / Angle Wrapping Functions -----------------------------------*/
/**
 * @brief  将角度（度）归一化到 (-180°, 180°]
 * @param  angle_deg  待归一化角度（度）
 * @return float 归一化后的角度，范围 (-180, 180]
 *
 */
float MyMath_Wrap_To_180(float angle_deg)
{
  // 大于 180° 则反复减 360°
  while (angle_deg > 180.0f) angle_deg -= 360.0f;
  // 小于等于 -180° 则反复加 360°，使区间为 (-180, 180]
  while (angle_deg <= -180.0f) angle_deg += 360.0f;
  return angle_deg;
}

/**
 * @brief  将角度（弧度）归一化到 (-π, π]
 * @param  angle_rad  待归一化角度（弧度）
 * @return float 归一化后的角度，范围 (-π, π]
 *
 * MyMath_Wrap_To_180 的弧度版本，周期为 2π。
 */
float MyMath_Wrap_To_Pi(float angle_rad)
{
  const float two_pi = 2.0f * PI;
  while (angle_rad > PI) angle_rad -= two_pi;
  while (angle_rad <= -PI) angle_rad += two_pi;
  return angle_rad;
}

/**
 * @brief  将数值归一化到半开区间 [min, max)
 * @param  value  待归一化数值
 * @param  min    区间下界（闭合，等于 min 时不变）
 * @param  max    区间上界（开放，等于 max 时回绕到 min）
 * @return float 归一化后的数值，范围 [min, max)
 *
 * 基于 fmodf 的通用周期性映射，适用于角度 [0°, 360°)、相位、时间戳等。
 * @pre min < max
 */
float MyMath_Wrap_To_Range(float value, float min, float max)
{
  // range = 映射周期；将 min 平移到原点
  float range = max - min;
  float result = fmodf(value - min, range); // fmodf 结果在 (-range, range)
  // 负数时补 range，使结果落入 [0, range)
  if (result < 0.0f) result += range;
  // 回移：原点 → min
  return min + result;
}
