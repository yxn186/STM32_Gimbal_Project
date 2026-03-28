/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bsp_usart.h
  * @brief   This file contains all the function prototypes for
  *          the bsp_usart.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BSP_USART_H__
#define __BSP_USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stdbool.h"
/*YOUR CODE*/
/* Exported macros -----------------------------------------------------------*/

// 缓冲区字节长度
#define UART_BUFFER_SIZE 512

/* Exported types ------------------------------------------------------------*/
extern bool Global_Init_Finished;

/**
 * @brief UART通信发送回调函数数据类型
 *
 */
typedef void (*UART_Tx_Call_Back)(UART_HandleTypeDef *huart);

/**
 * @brief UART通信接收回调函数数据类型
 *
 */
typedef void (*UART_Rx_Call_Back)(void *context, uint8_t *Buffer, uint16_t Length);

/**
 * @brief UART通信处理结构体
 */
typedef struct
{
    UART_HandleTypeDef *UART_Handler;
    uint8_t Tx_Buffer[UART_BUFFER_SIZE];
    uint8_t Rx_Buffer[UART_BUFFER_SIZE];
    uint16_t Rx_Buffer_Length;
    UART_Tx_Call_Back Tx_Callback_Function;
    UART_Rx_Call_Back Rx_Callback_Function;
    void *Rx_Callback_Context;
}Struct_UART_Manage_Object;


/* Exported variables --------------------------------------------------------*/


extern Struct_UART_Manage_Object UART1_Manage_Object;
extern Struct_UART_Manage_Object UART2_Manage_Object;
extern Struct_UART_Manage_Object UART3_Manage_Object;
extern Struct_UART_Manage_Object UART4_Manage_Object;
extern Struct_UART_Manage_Object UART5_Manage_Object;
extern Struct_UART_Manage_Object UART6_Manage_Object;

/* Exported function declarations --------------------------------------------*/

/**
 * @brief 初始化UART
 *
 * @param huart UART编号
 * @param Callback_Function 处理回调函数
 * @param Rx_Buffer_Length 接收缓冲区长度
 */
void UART_Init(UART_HandleTypeDef *huart, UART_Tx_Call_Back Tx_Callback_Function, UART_Rx_Call_Back Rx_Callback_Function, uint16_t Rx_Buffer_Length, void *Rx_Callback_Context);

/**
 * @brief 掉线重新初始化UART
 *
 * @param huart UART编号
 */
void UART_Reinit(UART_HandleTypeDef *huart);

/**
 * @brief 发送数据帧
 *
 * @param huart UART编号
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t UART_Transmit_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length);

/**
 * @brief UART的TIM定时器中断发送回调函数
 *
 */
void TIM_1ms_UART_PeriodElapsedCallback();






#ifdef __cplusplus
}
#endif

#endif /* __BSP_USART_H__ */
