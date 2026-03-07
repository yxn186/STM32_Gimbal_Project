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
void gimbal_pid_conrol(void);


#ifdef __cplusplus
}
#endif

#endif /* __GIMBAL_TASK_H__ */
