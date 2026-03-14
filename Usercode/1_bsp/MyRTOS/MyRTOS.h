/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    MyRTOS.h
  * @brief   This file contains all the function prototypes for
  *          the MyRTOS.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MYRTOS_H__
#define __MYRTOS_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

/*YOUR CODE*/


/**
 * @brief 自定义RTOS封装函数 位通知模式 发送通知（中断版）
 * 
 * @param xTaskToNotify 
 * @param ulValue 
 */
void MyRTOS_TaskNotify_ISR_SetBits(TaskHandle_t xTaskToNotify,uint32_t ulValue);

/**
 * @brief 自定义RTOS封装函数 位通知模式 等待接收通知
 * 
 * @param NeedCheck_Value 
 * @return uint8_t 
 */
uint32_t MyRTOS_TaskNotifyWait();


#ifdef __cplusplus
}
#endif

#endif /* __MYRTOS_H__ */
