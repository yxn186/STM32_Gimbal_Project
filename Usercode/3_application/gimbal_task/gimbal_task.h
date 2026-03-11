/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gimbal_task.h
  * @brief   This file contains all the function prototypes for
  *          the gimbal_task.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bsp_usb.h"
#include "Serial.h"

/*YOUR CODE*/
extern bool init_finished;

/* Printf配置 */
#ifndef STM32_PRINTF_USE_USB
#define STM32_PRINTF_USE_USB 0
#endif

#if STM32_PRINTF_USE_USB
#define STM32_Printf(...) USB_Printf(__VA_ARGS__)
#else
#define STM32_Printf(...) Serial_Printf(__VA_ARGS__)
#endif

void gimbal_task_init(void);
uint8_t gimbal_task_init_loop(void);
void gimbal_pid_reset(void);
void gimbal_target_init(void);

//枚举

/**
 * @brief 云台状态枚举
 * 
 */
typedef enum
{
  gimbal_states_sentry_mode = 0,
  gimbal_states_aim_mode,
} gimtal_states_e;

/**
 * @brief 哨兵模式速度目标变换设置函数
 * 
 * @return float 速度目标值
 */
float Speed_Target_Sentry(void);

/**
 * @brief 哨兵模式角度目标变换设置函数
 * 
 * @return float 角度目标值
 */
float Angle_Target_Sentry(void);

/**
 * @brief 哨兵模式目标变换设置函数
 * 
 */
void Set_Yaw_and_Pitch_Motor_Target_Sentry();

/**
 * @brief Yaw电机设置角度和速度当前值
 * 
 * @param Angle 
 * @param Speed 
 */
void Set_Yaw_Current_Angle_and_Speed(float Angle,float Speed);

/**
 * @brief Pitch电机设置角度和速度当前值
 * 
 * @param Angle 
 * @param Speed 
 */
void Set_Pitch_Current_Angle_and_Speed(float Angle,float Speed);
#ifdef __cplusplus
}
#endif

#endif /* __GIMBAL_TASK_H__ */
