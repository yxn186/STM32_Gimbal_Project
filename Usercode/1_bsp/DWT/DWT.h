/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    DWT.h
  * @brief   This file contains all the function prototypes for
  *          the DWT.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DWT_H__
#define __DWT_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdint.h>

/*YOUR CODE*/

/**
 * @brief 初始化DWT
 * 
 */
void DWT_Init(void);

/**
 * @brief 取得当前CYCCNT
 * 
 * @return uint32_t 
 */
uint32_t DWT_GetCYCCNT(void);

/**
 * @brief 取得当前微妙
 * 
 * @return uint64_t 
 */
uint64_t DWT_GetUs(void);

/**
 * @brief 取得当前毫秒
 * 
 * @return uint64_t 
 */
uint64_t DWT_GetMs(void);

#ifdef __cplusplus
}
#endif

#endif /* __DWT_H__ */
