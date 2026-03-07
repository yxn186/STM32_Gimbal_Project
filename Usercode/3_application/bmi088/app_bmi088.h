/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_bmi088.h
  * @brief   This file contains all the function prototypes for
  *          the app_bmi088.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_BMI088_H__
#define __APP_BMI088_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "PID.h"

/*YOUR CODE*/

/**
 * @brief BMI088初始化
 * 
 */
void app_bmi088_init(void);
uint8_t app_bmi088_init_process_loop(void);
void app_bmi088_task1(void);
void app_bmi088_task2(void);

/**
 * @brief BMI088 20ms周期任务 函数
 * 
 */
void app_bmi088_20ms_task(void);

/**
 * @brief BMI088 FreeRTOS初始化
 * 
 */
void bmi088_freertos_init(void);

void bmi088_calculate_task(void *argument);

/**
 * @brief BMI088 1ms周期任务 函数
 * 
 */
void app_bmi088_1ms_task(PID_t *Pitch, PID_t *Yaw);

#ifdef __cplusplus
}
#endif

#endif /* __APP_BMI088_H__ */
