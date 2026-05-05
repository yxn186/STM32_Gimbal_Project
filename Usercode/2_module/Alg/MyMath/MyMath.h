/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    MyMath.h
  * @brief   This file contains all the function prototypes for
  *          the MyMath.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MYMATH_H__
#define __MYMATH_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*YOUR CODE*/

//常用数学函数和常量
#define PI 3.14159265358979323846f
#define RAD2DEG (57.295779513082320876f)  // 180°/3.1415926
#define DEG2RAD (0.01745329251994329577f) // 3.14/180°

#define SQRT2_OVER_2 0.70710678f //二分之根号二
#define SQRT2_OVER_4 0.35355339f //四分之根号二

/* 角度归一化 / Angle Wrapping Functions --------------------------------*/

/**
 * @brief  将角度（度）归一化到 (-180°, 180°]
 * @param  angle_deg  待归一化角度（度）
 * @return float 归一化后的角度，范围 (-180, 180]
 */
float MyMath_Wrap_To_180(float angle_deg);

/**
 * @brief  将角度（弧度）归一化到 (-π, π]
 * @param  angle_rad  待归一化角度（弧度）
 * @return float 归一化后的角度，范围 (-π, π]
 */
float MyMath_Wrap_To_Pi(float angle_rad);

/**
 * @brief  将数值归一化到半开区间 [min, max)
 * @param  value  待归一化数值
 * @param  min    区间下界（闭合）
 * @param  max    区间上界（开放）
 * @return float 归一化后的数值，范围 [min, max)
 */
float MyMath_Wrap_To_Range(float value, float min, float max);


#ifdef __cplusplus
}
#endif

#endif /* __MYMATH_H__ */
