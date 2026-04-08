/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Gimbal_Task.h
  * @brief   This file contains all the function prototypes for
  *          the Gimbal_Task.c file
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
extern bool Global_Init_Finished;

/* Printf配置------------------------------------------------- */
#ifndef STM32_PRINTF_USE_USB
#define STM32_PRINTF_USE_USB 0
#endif

#if STM32_PRINTF_USE_USB
#define STM32_Printf(...) USB_Printf(__VA_ARGS__)
#else
#define STM32_Printf(...) Serial_Printf(__VA_ARGS__)
#endif

/* 函数---------------------------------------------------------*/

/**
 * @brief 云台初始化
 * 
 */
void gimbal_task_init(void);

/**
 * @brief 云台初始化（循环）
 * 
 * @return uint8_t 
 */
uint8_t gimbal_task_init_loop(void);

/**
 * @brief 云台PID重置信息
 * 
 */
void gimbal_pid_reset(void);

/**
 * @brief 云台目标初始化
 * 
 */
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
void Set_Yaw_and_Pitch_Motor_Target_Sentry(void);

/**
 * @brief 哨兵模式目标变换设置函数
 * 
 */
void Set_Yaw_and_Pitch_Motor_Speed_Target_Sentry();

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


/**
 * @brief 云台大疆电机初始化
 * 
 */
void Gimbal_DJI_Motor_Init(void);


/**
 * @brief 云台推送电机数据给PID类
 * 
 */
void Gimbal_Push_Motor_Current_Speed_To_PID(void);

/**
 * @brief 云台推送云台前向轴yaw和pitch给PID类
 * 
 * @param Pitch 
 * @param Yaw 
 */
void Gimbal_Push_Gimbal_Pitch_and_Yaw_To_PID(float Pitch,float Yaw);

/**
 * @brief 云台推送PID输出值至电机控制值
 * 
 */
void Gimbal_Push_PID_Out_To_Motor_Control(void);

/**
 * @brief 相机USB离线检测
 * 
 */
void Gimbal_Vision_Mode_Judge_1ms(void);

void Gimbal_Set_Front_Angle(float yaw,float roll,float pitch);

float Speed_Target_Sentry_Pitch(void);

float Angle_Target_Sentry_Pitch(void);

float Angle_Target_Sentry_Gimbal_Yaw(void);

float Angle_Target_Sentry_Gimbal_Pitch(void);
#ifdef __cplusplus
}
#endif

#endif /* __GIMBAL_TASK_H__ */
