/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bmi088_math.h
  * @brief   This file contains all the function prototypes for
  *          the app_bmi088_math.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BMI088_MATH_H__
#define __BMI088_MATH_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*YOUR CODE*/

uint32_t bmi088_getbiascalibration_target_samples(void);
uint32_t bmi088_getbiascalibration_current_samples(void);
uint32_t bmi088_getbiascalibration_current_samples_effective(void);
uint8_t bmi088_get_biascalibration_finish_flag(void);

float BMI088_GetRollDeg(void);
float BMI088_GetPitchDeg(void);
float BMI088_GetYawDeg(void);
float BMI088_GetRealPitchDeg(void);

uint8_t BMI088_IsBiasCalibrated(void);

/**
 * @brief BMI088 Mahony算法
 * 
 * @param gyro_raw_x 
 * @param gyro_raw_y 
 * @param gyro_raw_z 
 * @param acc_raw_x 
 * @param acc_raw_y 
 * @param acc_raw_z 
 */
void bmi088_mahony(int16_t gyro_raw_x,int16_t gyro_raw_y,int16_t gyro_raw_z,int16_t acc_raw_x,int16_t acc_raw_y,int16_t acc_raw_z);

/**
 * @brief BMI088 Mahony算法 输出外旋ZXY欧拉角
 * 
 * @param gyro_raw_x 
 * @param gyro_raw_y 
 * @param gyro_raw_z 
 * @param acc_raw_x 
 * @param acc_raw_y 
 * @param acc_raw_z 
 */
void bmi088_mahony_zxy(int16_t gyro_raw_x,int16_t gyro_raw_y,int16_t gyro_raw_z,int16_t acc_raw_x,int16_t acc_raw_y,int16_t acc_raw_z);


/**
 * @brief BMI088 Mahony算法 输出外旋ZYX欧拉角
 * 
 * @param gyro_raw_x 
 * @param gyro_raw_y 
 * @param gyro_raw_z 
 * @param acc_raw_x 
 * @param acc_raw_y 
 * @param acc_raw_z 
 */
void bmi088_mahony_zyx(int16_t gyro_raw_x,int16_t gyro_raw_y,int16_t gyro_raw_z,int16_t acc_raw_x,int16_t acc_raw_y,int16_t acc_raw_z);
/**
 * @brief bmi088开始零偏校准函数
 * 
 * @param biascalibration_target_samples 期望参与校准的目标采样数
 */
void bmi088_biascalibration_start(uint32_t biascalibration_target_samples);

 /**
  * @brief BMI088 零偏校准 传入样本数据进行校准数据计算
  * 
  * @param gyro_raw_x 
  * @param gyro_raw_y 
  * @param gyro_raw_z 
  * @param acc_raw_x 
  * @param acc_raw_y 
  * @param acc_raw_z 
  * @return uint8_t 零偏校准是否完成 完成返回1 未完成返回0
  */
uint8_t bmi088_biascalibration_pushsampletocalculate(int16_t gyro_raw_x,int16_t gyro_raw_y,int16_t gyro_raw_z,int16_t acc_raw_x,int16_t acc_raw_y,int16_t acc_raw_z);

void bmi088_complementaryfilter_1(int16_t gyro_raw_x,int16_t gyro_raw_y,int16_t gyro_raw_z,int16_t acc_raw_x,int16_t acc_raw_y,int16_t acc_raw_z,float dt_seconds);
void bmi088_complementaryfilter_2(int16_t gyro_raw_x,int16_t gyro_raw_y,int16_t gyro_raw_z,int16_t acc_raw_x,int16_t acc_raw_y,int16_t acc_raw_z,float dt_seconds);

/**
 * @brief 外旋ZYX欧拉角 + 自定义前向轴/上方向轴 -> yaw / pitch / roll
 *
 * @param ex_z_deg  外旋Z
 * @param ex_y_deg  外旋Y
 * @param ex_x_deg  外旋X
 * @param f_body_x  机体系前向轴 x 分量
 * @param f_body_y  机体系前向轴 y 分量
 * @param f_body_z  机体系前向轴 z 分量
 * @param u_body_x  机体系上方向轴 x 分量
 * @param u_body_y  机体系上方向轴 y 分量
 * @param u_body_z  机体系上方向轴 z 分量
 * @param yaw_deg   输出 yaw
 * @param pitch_deg 输出 pitch
 * @param roll_deg  输出 roll
 */
void euler_extrinsic_ZYX_body_axes_to_front_yaw_pitch_roll_deg(float ex_z_deg,
                                                               float ex_y_deg,
                                                               float ex_x_deg,
                                                               float f_body_x,
                                                               float f_body_y,
                                                               float f_body_z,
                                                               float u_body_x,
                                                               float u_body_y,
                                                               float u_body_z,
                                                               float *yaw_deg,
                                                               float *pitch_deg,
                                                               float *roll_deg);
                                                               
/**
 * @brief 外旋ZXY欧拉角 + 自定义前向轴/上方向轴 -> yaw / pitch / roll
 *
 * @param ex_z_deg  外旋Z
 * @param ex_x_deg  外旋X
 * @param ex_y_deg  外旋Y
 * @param f_body_x  机体系前向轴 x 分量
 * @param f_body_y  机体系前向轴 y 分量
 * @param f_body_z  机体系前向轴 z 分量
 * @param u_body_x  机体系上方向轴 x 分量
 * @param u_body_y  机体系上方向轴 y 分量
 * @param u_body_z  机体系上方向轴 z 分量
 * @param yaw_deg   输出 yaw
 * @param pitch_deg 输出 pitch
 * @param roll_deg  输出 roll
 */
void euler_extrinsic_ZXY_body_axes_to_front_yaw_pitch_roll_deg(float ex_z_deg,
                                                               float ex_x_deg,
                                                               float ex_y_deg,
                                                               float f_body_x,
                                                               float f_body_y,
                                                               float f_body_z,
                                                               float u_body_x,
                                                               float u_body_y,
                                                               float u_body_z,
                                                               float *yaw_deg,
                                                               float *pitch_deg,
                                                               float *roll_deg);

void euler_extrinsic_ZYX_to_intrinsic_ZYX_deg(float ex_z_deg, float ex_y_deg, float ex_x_deg,float *in_yaw_deg, float *in_pitch_deg, float *in_roll_deg);

void euler_extrinsic_ZXY_to_intrinsic_ZXY_deg(float ex_z_deg, float ex_x_deg, float ex_y_deg,float *in_z_deg, float *in_x_deg, float *in_y_deg);

void euler_extrinsic_ZYX_to_front_yaw_pitch_deg(float ex_z_deg, float ex_y_deg, float ex_x_deg,float *yaw_deg, float *pitch_deg);

void euler_extrinsic_ZXY_to_front_yaw_pitch_deg(float ex_z_deg, float ex_x_deg, float ex_y_deg,float *yaw_deg, float *pitch_deg);

#ifdef __cplusplus
}
#endif

#endif /* __BMI088_MATH_H__ */
