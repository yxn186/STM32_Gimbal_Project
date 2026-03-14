/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    mx_api.h
  * @brief   This file contains all the function prototypes for
  *          the mx_api.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MX_API_H__
#define __MX_API_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*YOUR CODE*/


void MX_FREERTOS_Init(void);//声明MX_FREERTOS_Init函数，供main.cpp调用 定义在freertos.c中




#ifdef __cplusplus
}
#endif

#endif /* __MX_API_H__ */
