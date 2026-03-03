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

/*YOUR CODE*/

/**
 * @brief BMI088初始化
 * 
 */
void app_bmi088_init(void);

void app_bmi088_task1(void);
void app_bmi088_task2(void);

/**
 * @brief BMI088 循环函数
 * 
 */
void app_bmi088_loop(void);




#ifdef __cplusplus
}
#endif

#endif /* __APP_BMI088_H__ */
