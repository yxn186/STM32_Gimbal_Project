/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gimbal_task.c
  * @brief   Task
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "gimbal_task.h"
#include "joled.h"
#include "app_bmi088.h"
#include "Serial.h"
#include "usart.h"

void gimbal_task_init(void)
{
    JOLED_Init();
    Serial_Init(&huart1);
    app_bmi088_init();
}

void gimbal_task_loop(void)
{
    app_bmi088_loop();
}