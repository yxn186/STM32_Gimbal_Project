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
#include "bsp_usb.h"
#include "joled.h"
#include "app_bmi088.h"
#include "Serial.h"
#include "usart.h"
#include <stdbool.h>
#include "usbd_cdc_if.h"
#include "bsp_usb.h"

/*  Task层全局变量 ------------------------------------------------------------*/
bool init_finished = false;

/*  Task层自定义回调函数类型 --------------------------------------------------*/



/*  Task层函数 ----------------------------------------------------------------*/

void gimbal_task_init(void)
{
    JOLED_Init();
    Serial_Init(&huart1);
    USB_Init();
    app_bmi088_init();
    init_finished = true;
}

void gimbal_task_loop(void)
{
    app_bmi088_loop();
}
