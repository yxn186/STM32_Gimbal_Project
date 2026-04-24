/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Serial.h
  * @brief   This file contains all the function prototypes for
  *          the Serial.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SERIAL_H__
#define __SERIAL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*YOUR CODE*/

/**
 * @brief Serial接收回调函数数据类型
 *
 */
typedef void (*Serial_Rx_Call_Back)(void *context, uint8_t *Buffer, uint16_t Length);


/**
 * @brief 串口句柄
 * 
 */
typedef struct
{
  UART_HandleTypeDef *huart;//huartx
  uint8_t *TxBuffer;
  volatile uint16_t Tx_WriteIndex;
  volatile uint16_t Tx_ReadIndex;
  volatile uint8_t Tx_Flag;
  volatile uint16_t Rx_Current_Lenth;
  uint8_t *RxBuffer;
  volatile uint16_t Rx_WriteIndex;
  volatile uint16_t Rx_ReadIndex;
}Serial_handle_t;

/**
 * @brief 串口发送回调函数
 * 
 * @param huart 
 */
void Serial_Tx_Callback_Function(UART_HandleTypeDef *huart);

/**
 * @brief 串口接收数据回调函数
 * 
 * @param Buffer 
 * @param Length 
 */
// void Serial_Rx_Callback_Function(void *context, uint8_t *Buffer, uint16_t Length);

/**
 * @brief Serial 初始化
 * 
 * @param huart huartx
 * @param Rx_Callback_Function RX callback, NULL means no RX callback is registered
 * @param Rx_Callback_Context RX callback context, NULL means unused
 */
void Serial_Init(UART_HandleTypeDef *huart, Serial_Rx_Call_Back Rx_Callback_Function, void *Rx_Callback_Context);

/**
 * @brief Serial_Printf 串口打印函数
 * 
 * @param format 
 * @param ... 
 */
void Serial_Printf(const char *format, ...);

/**
 * @brief 串口发送数据函数
 * 
 * @param data 要发送的数据指针
 * @param length 要发送的数据长度
 */
void Serial_Send_Data(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* __SERIAL_H__ */
