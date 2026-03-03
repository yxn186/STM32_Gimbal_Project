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
    float Target;
    float Kp;
    float Ki;
    float Kd;
} PID_Cfg_t;

typedef struct
{
    float Current_speed;
    float Current_Angle;
    float Error0;
    float Error1;
    float ErrorInt;
    float Out;
} PID_Status_t;

/**
 * @brief PID控制 单电机
 * 
 * @param PID_Cfg_Data PID配置参数：Kp Ki Kd Target
 * @param PID_Status PID状态
 */
void PID_Control_Single(PID_Cfg_t *PID_Cfg_Data,PID_Status_t *PID_Status);


#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
