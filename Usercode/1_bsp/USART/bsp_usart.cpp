/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bsp_usart.cpp
  * @brief   usart底层库
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "bsp_usart.h"

/*YOUR CODE*/

Struct_UART_Manage_Object UART1_Manage_Object = {0};
Struct_UART_Manage_Object UART2_Manage_Object = {0};
Struct_UART_Manage_Object UART3_Manage_Object = {0};
Struct_UART_Manage_Object UART4_Manage_Object = {0};
Struct_UART_Manage_Object UART5_Manage_Object = {0};
Struct_UART_Manage_Object UART6_Manage_Object = {0};


/**
 * @brief 初始化UART
 *
 * @param huart UART编号
 * @param Rx_Callback_Function 处理回调函数
 * @param Rx_Buffer_Length 接收缓冲区长度
 */
void UART_Init(UART_HandleTypeDef *huart, UART_Tx_Call_Back Tx_Callback_Function, UART_Rx_Call_Back Rx_Callback_Function, uint16_t Rx_Buffer_Length, void *Rx_Callback_Context)
{
    if (huart->Instance == USART1)
    {
        UART1_Manage_Object.UART_Handler = huart;
        UART1_Manage_Object.Tx_Callback_Function = Tx_Callback_Function;
        UART1_Manage_Object.Rx_Callback_Function = Rx_Callback_Function;
        UART1_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        UART1_Manage_Object.Rx_Callback_Context = Rx_Callback_Context;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART2)
    {
        UART2_Manage_Object.UART_Handler = huart;
        UART2_Manage_Object.Tx_Callback_Function = Tx_Callback_Function;  
        UART2_Manage_Object.Rx_Callback_Function = Rx_Callback_Function;
        UART2_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        UART2_Manage_Object.Rx_Callback_Context = Rx_Callback_Context;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_Manage_Object.Rx_Buffer, UART2_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART3)
    {
        UART3_Manage_Object.UART_Handler = huart;
        UART3_Manage_Object.Tx_Callback_Function = Tx_Callback_Function;
        UART3_Manage_Object.Rx_Callback_Function = Rx_Callback_Function;
        UART3_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        UART3_Manage_Object.Rx_Callback_Context = Rx_Callback_Context;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART4)
    {
        UART4_Manage_Object.UART_Handler = huart;
        UART4_Manage_Object.Tx_Callback_Function = Tx_Callback_Function;
        UART4_Manage_Object.Rx_Callback_Function = Rx_Callback_Function;
        UART4_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        UART4_Manage_Object.Rx_Callback_Context = Rx_Callback_Context;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART4_Manage_Object.Rx_Buffer, UART4_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART5)
    {
        UART5_Manage_Object.UART_Handler = huart;
        UART5_Manage_Object.Tx_Callback_Function = Tx_Callback_Function;
        UART5_Manage_Object.Rx_Callback_Function = Rx_Callback_Function;
        UART5_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        UART5_Manage_Object.Rx_Callback_Context = Rx_Callback_Context;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART6)
    {
        UART6_Manage_Object.UART_Handler = huart;
        UART6_Manage_Object.Tx_Callback_Function = Tx_Callback_Function;
        UART6_Manage_Object.Rx_Callback_Function = Rx_Callback_Function;
        UART6_Manage_Object.Rx_Buffer_Length = Rx_Buffer_Length;
        UART6_Manage_Object.Rx_Callback_Context = Rx_Callback_Context;
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Length);
    }
}

/**
 * @brief 掉线重新初始化UART
 *
 * @param huart UART编号
 */
void UART_Reinit(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART2)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_Manage_Object.Rx_Buffer, UART2_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART3)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART4)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART4_Manage_Object.Rx_Buffer, UART4_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART6)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Length);
    }
}

/**
 * @brief 发送数据帧
 *
 * @param huart UART编号
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return uint8_t 执行状态
 */
uint8_t UART_Transmit_Data(UART_HandleTypeDef *huart, uint8_t *Data, uint16_t Length)
{
    return (HAL_UART_Transmit_DMA(huart, Data, Length));
}

/**
 * @brief UART的TIM定时器中断发送回调函数
 *
 */
void TIM_1ms_UART_PeriodElapsedCallback()
{

}

/**
 * @brief HAL库UART发送完成回调函数
 *
 * @param huart UART编号
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // 选择回调函数
    if (huart->Instance == USART1)
    {
        if(UART1_Manage_Object.Tx_Callback_Function != nullptr)
        {
            UART1_Manage_Object.Tx_Callback_Function(UART1_Manage_Object.UART_Handler);
        }
    }
    else if (huart->Instance == USART2)
    {
        if(UART2_Manage_Object.Tx_Callback_Function != nullptr)
        {
            UART2_Manage_Object.Tx_Callback_Function(UART2_Manage_Object.UART_Handler);
        }
    }
    else if (huart->Instance == USART3)
    {
        if(UART3_Manage_Object.Tx_Callback_Function != nullptr)
        {
            UART3_Manage_Object.Tx_Callback_Function(UART3_Manage_Object.UART_Handler);
        }
    }
    else if (huart->Instance == UART4)
    {
        if(UART4_Manage_Object.Tx_Callback_Function != nullptr)
        {
            UART4_Manage_Object.Tx_Callback_Function(UART4_Manage_Object.UART_Handler);
        }
    }
    else if (huart->Instance == UART5)
    {
        if(UART5_Manage_Object.Tx_Callback_Function != nullptr)
        {
            UART5_Manage_Object.Tx_Callback_Function(UART5_Manage_Object.UART_Handler);
        }
    }
    else if (huart->Instance == USART6)
    {
        if(UART6_Manage_Object.Tx_Callback_Function != nullptr)
        {
            UART6_Manage_Object.Tx_Callback_Function(UART6_Manage_Object.UART_Handler);
        }
    }
}

/**
 * @brief HAL库UART接收DMA空闲中断回调函数
 *
 * @param huart UART编号
 * @param Size 长度
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // 判断程序初始化完成
    if (!init_finished)
    {
        // 重启接收
        if (huart->Instance == USART1)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART_BUFFER_SIZE);
        }
        else if (huart->Instance == USART2)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_Manage_Object.Rx_Buffer, UART_BUFFER_SIZE);
        }
        else if (huart->Instance == USART3)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(huart, UART3_Manage_Object.Rx_Buffer, UART_BUFFER_SIZE);
        }
        else if (huart->Instance == UART4)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(huart, UART4_Manage_Object.Rx_Buffer, UART_BUFFER_SIZE);
        }
        else if (huart->Instance == UART5)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_Manage_Object.Rx_Buffer, UART_BUFFER_SIZE);
        }
        else if (huart->Instance == USART6)
        {
            HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_Manage_Object.Rx_Buffer, UART_BUFFER_SIZE);
        }
        return;
    }
    
    // 选择回调函数
    if (huart->Instance == USART1)
    {
        if(UART1_Manage_Object.Rx_Callback_Function != nullptr)
        {
            UART1_Manage_Object.Rx_Callback_Function(UART1_Manage_Object.Rx_Callback_Context, UART1_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART2)
    {
        if(UART2_Manage_Object.Rx_Callback_Function != nullptr)
        {
            UART2_Manage_Object.Rx_Callback_Function(UART2_Manage_Object.Rx_Callback_Context, UART2_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_Manage_Object.Rx_Buffer, UART2_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART3)
    {
        if(UART3_Manage_Object.Rx_Callback_Function != nullptr)
        {
            UART3_Manage_Object.Rx_Callback_Function(UART3_Manage_Object.Rx_Callback_Context, UART3_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART4)
    {
        if(UART4_Manage_Object.Rx_Callback_Function != nullptr)
        {
            UART4_Manage_Object.Rx_Callback_Function(UART4_Manage_Object.Rx_Callback_Context, UART4_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART4_Manage_Object.Rx_Buffer, UART4_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART5)
    {
        if(UART5_Manage_Object.Rx_Callback_Function != nullptr)
        {
            UART5_Manage_Object.Rx_Callback_Function(UART5_Manage_Object.Rx_Callback_Context, UART5_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART6)
    {
        if(UART6_Manage_Object.Rx_Callback_Function != nullptr)
        {
            UART6_Manage_Object.Rx_Callback_Function(UART6_Manage_Object.Rx_Callback_Context,UART6_Manage_Object.Rx_Buffer, Size);
        }
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Length);
    }
}

/**
 * @brief HAL库UART错误中断回调函数
 *
 * @param huart UART编号
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART1_Manage_Object.Rx_Buffer, UART1_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART2)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART2_Manage_Object.Rx_Buffer, UART2_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART3)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART3_Manage_Object.Rx_Buffer, UART3_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART4)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART4_Manage_Object.Rx_Buffer, UART4_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == UART5)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART5_Manage_Object.Rx_Buffer, UART5_Manage_Object.Rx_Buffer_Length);
    }
    else if (huart->Instance == USART6)
    {
        HAL_UARTEx_ReceiveToIdle_DMA(huart, UART6_Manage_Object.Rx_Buffer, UART6_Manage_Object.Rx_Buffer_Length);
    }
}