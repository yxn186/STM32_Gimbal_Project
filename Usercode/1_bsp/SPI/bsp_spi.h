/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bsp_spi.h
  * @brief   This file contains all the function prototypes for
  *          the bsp_spi.c file
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __BSP_SPI_H__
#define __BSP_SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal_spi.h"
#include <string.h>
/*YOUR CODE*/

/**
 * @brief SPI通信接收回调函数数据类型
 *
 */
typedef void (*SPI_Callback)(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length);

/**
 * @brief SPI通信处理结构体
 *
 */
struct Struct_SPI_Manage_Object
{
    SPI_HandleTypeDef *SPI_Handler;
    SPI_Callback Callback_Function;

    // 片选信号的GPIO与电平
    GPIO_TypeDef *Activate_GPIOx;
    uint16_t Activate_GPIO_Pin;
    GPIO_PinState Activate_Level;

    // 一次收发对应的数据长度
    uint8_t *Tx_Buffer;
    uint8_t *Rx_Buffer;
    uint16_t Tx_Buffer_Length;
    uint16_t Rx_Buffer_Length;

    // 接收时间戳
    uint64_t Rx_Timestamp;
};

/**
 * @brief 初始化SPI
 *
 * @param hspi SPI编号
 * @param Callback_Function 处理回调函数
 */
void SPI_Init(SPI_HandleTypeDef *hspi, SPI_Callback Callback_Function);

/**
 * @brief 发送数据
 *
 * @param hspi SPI编号
 * @param GPIOx 片选GPIO引脚编组
 * @param GPIO_Pin 片选GPIO引脚号
 * @param Activate_Level 片选GPIO引脚电平
 * @param Tx_Length 长度
 * @return uint8_t 执行状态
 */
uint8_t SPI_Transmit_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState Activate_Level, uint8_t *Tx_Buffer, uint16_t Tx_Length);

/**
 * @brief 阻塞发送数据
 *
 * @param hspi SPI编号
 * @param GPIOx 片选GPIO引脚编组
 * @param GPIO_Pin 片选GPIO引脚号
 * @param Activate_Level 片选有效电平（一般CS低有效就传 GPIO_PIN_RESET）
 * @param Tx_Buffer 发送缓冲区
 * @param Tx_Length 发送长度
 * @param Timeout 超时（ms），比如 10/100，或 HAL_MAX_DELAY
 * @return uint8_t 执行状态（HAL_OK / HAL_BUSY / HAL_ERROR / HAL_TIMEOUT）
 */
uint8_t SPI_Transmit_Data_Blocking(SPI_HandleTypeDef *hspi,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin,GPIO_PinState Activate_Level,uint8_t *Tx_Buffer,uint16_t Tx_Length,uint32_t Timeout);

/**
 * @brief 交互数据帧
 * 
 * @param hspi SPI编号
 * @param GPIOx 片选GPIO引脚编组
 * @param GPIO_Pin 片选GPIO引脚号
 * @param Activate_Level 片选GPIO引脚电平
 * @param Tx_Length 发送数据长度
 * @param Rx_Length 接收数据长度
 * @return uint8_t 执行状态
 */
uint8_t SPI_Transmit_Receive_Data(SPI_HandleTypeDef *hspi, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState Activate_Level,uint8_t *Tx_Buffer,uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length);



#ifdef __cplusplus
}
#endif

#endif /* __BSP_SPI_H__ */
