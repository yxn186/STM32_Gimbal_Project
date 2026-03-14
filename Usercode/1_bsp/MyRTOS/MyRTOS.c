/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    MyRTOS.c
  * @brief   RTOS库自定义封装函数
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "MyRTOS.h"

/**
 * @brief 自定义RTOS封装函数 位通知模式 发送通知（中断版）
 * 
 * @param xTaskToNotify 
 * @param ulValue 
 */
void MyRTOS_TaskNotify_ISR_SetBits(TaskHandle_t xTaskToNotify,uint32_t ulValue)
{
    if (xTaskToNotify == NULL)
    {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(xTaskToNotify,ulValue,eSetBits,&xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/**
 * @brief 自定义RTOS封装函数 位通知模式 等待接收通知
 * 
 * @param NeedCheck_Value 
 * @return uint8_t 
 */
uint32_t MyRTOS_TaskNotifyWait()
{
    uint32_t notify_value = 0;
    xTaskNotifyWait(0, 0xFFFFFFFF, &notify_value, portMAX_DELAY);
    return notify_value;
}
