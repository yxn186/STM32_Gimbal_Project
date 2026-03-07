/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    PID.h
  * @brief   This file contains all the function prototypes for
  *          the PID.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*YOUR CODE*/

typedef struct
{
    float Current_speed;
    float Current_Angle;
    float Error0;
    float Error1;
    float ErrorInt;
} PID_Status_t;

typedef struct
{
    float Speed_Target;
    float Kp_s;
    float Ki_s;
    float Kd_s;
    float Out;

    float ErrorInt_High_s;
    float ErrorInt_Low_s;

    float Out_High;
    float Out_Low;

    float Angle_Target;
    float Kp_a;
    float Ki_a;
    float Kd_a;

    float ErrorInt_High_a;
    float ErrorInt_Low_a;

    float Speed_Target_High;
    float Speed_Target_Low;

    PID_Status_t PID_Angle_Status;
    PID_Status_t PID_Speed_Status;
} PID_t;

/**
 * @brief PID控制 速度环
 * 
 * @param PID_Object PID控制对象指针，包含配置参数和状态
 */
void PID_Control_Speed(PID_t *PID_Object);

/**
 * @brief PID控制 角度环
 * 
 * @param PID_Object PID控制对象指针，包含配置参数和状态
 */
void PID_Control_Angle(PID_t *PID_Object);


#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
