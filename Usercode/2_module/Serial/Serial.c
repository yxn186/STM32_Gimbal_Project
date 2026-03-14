/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Serial.c
  * @brief   串口 库
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "Serial.h"
#include "bsp_usart.h"
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>

//参数配置
#define Serial_TxBuffer_Size                1024    //发送缓冲区大小
#define Serial_RxBuffer_Size                1024    //接收缓冲区大小

#define Serial_Printf_Temp_Buffer_Size      256     //Serial_Printf的临时缓冲区大小


/* ---------- Tx 环形缓冲区 ---------- */
static uint8_t Serial_TxBuffer_Data[Serial_TxBuffer_Size];
static uint8_t Serial_RxBuffer_Data[Serial_RxBuffer_Size];

Serial_handle_t Serial_handle_global =
{
    .TxBuffer = Serial_TxBuffer_Data,
    .RxBuffer = Serial_RxBuffer_Data,
};

//工具函数

/**
 * @brief 串口进入关键区 关中断 （防止中断里和主线程同时改 index）
 * 
 * @return uint32_t 返回进入前的中断状态
 */
static uint32_t Serial_EnterCriticalSection(void)
{
    uint32_t primaskValue = __get_PRIMASK();
    __disable_irq();
    return primaskValue;
}

/**
 * @brief 串口退出关键区 判断是否恢复中断
 * 
 * @param primaskValue 进入前的中断状态
 */
static void Serial_ExitCriticalSection(uint32_t primaskValue)
{
    if (primaskValue == 0)
    {
        __enable_irq();
    }
}

/**
 * @brief 计算环形缓冲区已用字节数函数
 *
 *@details 用“空一格”的方式区分满/空
 * 
 * @param writeIndex 写指针
 * @param readIndex 读指针
 * @param ringBufferSize 环形缓冲区大小
 * @return uint16_t 已用字节数函数
 */
static uint16_t Serial_CalculateRingBufferUsedCount(uint16_t writeIndex,uint16_t readIndex,uint16_t ringBufferSize)
{
    if (writeIndex >= readIndex)
    {
        return (uint16_t)(writeIndex - readIndex);
    }
    else
    {
        return (uint16_t)(ringBufferSize - readIndex + writeIndex);
    }
}

/**
 * @brief 计算环形缓冲区下一个索引函数
 * 
 * @param currentIndex 当前指针
 * @param ringBufferSize 环形缓冲区大小
 * @return uint16_t 下一个索引
 */
static uint16_t Serial_CalculateNextIndex(uint16_t currentIndex, uint16_t ringBufferSize)
{
    currentIndex++;
    if (currentIndex >= ringBufferSize)
    {
        currentIndex = 0;
    }
    return currentIndex;
}

/**
 * @brief Serial 初始化
 * 
 * @param huart huartx
 */
void Serial_Init(UART_HandleTypeDef *huart)
{
    Serial_handle_global.huart = huart;

    /* 清空 Tx/Rx 环形缓冲区指针 */
    {
        uint32_t primaskValue = Serial_EnterCriticalSection();

        Serial_handle_global.Tx_WriteIndex = 0;
        Serial_handle_global.Tx_ReadIndex  = 0;
        Serial_handle_global.Tx_Flag = 0;
        Serial_handle_global.Rx_Current_Lenth = 0;

        Serial_handle_global.Rx_WriteIndex = 0;
        Serial_handle_global.Rx_ReadIndex  = 0;

        Serial_ExitCriticalSection(primaskValue);
    }

    /* 关掉 DMA 半传输中断：否则数据量大时会频繁进中断 */
    if (Serial_handle_global.huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(Serial_handle_global.huart->hdmarx, DMA_IT_HT);
    }

    UART_Init(huart,Serial_Tx_Callback_Function,Serial_Rx_Callback_Function,Serial_RxBuffer_Size,NULL);
}

/**
 * @brief 把一段新数据塞进接收环形缓冲区
 * 
 * @details 溢出策略：接收环形缓冲区满了就丢新数据（不覆盖旧数据）
 *
 * @param data 新数据指针
 * @param length 数据长度
 */
static void Serial_Push_Data_To_RxBuffer(const uint8_t *data, uint16_t length)
{
    uint16_t i = 0;

    uint32_t primaskValue = Serial_EnterCriticalSection();

    for (i = 0; i < length; i++)
    {
        uint16_t nextWriteIndex = Serial_CalculateNextIndex(Serial_handle_global.Rx_WriteIndex,Serial_RxBuffer_Size);

        /* 满了：nextWriteIndex 追上 readIndex */
        if (nextWriteIndex == Serial_handle_global.Rx_ReadIndex)
        {
            break; /* 丢弃剩余数据 */
        }

        Serial_handle_global.RxBuffer[Serial_handle_global.Rx_WriteIndex] = data[i];
        Serial_handle_global.Rx_WriteIndex = nextWriteIndex;
    }

    Serial_ExitCriticalSection(primaskValue);
}

/**
 * @brief 尝试启动下一段 DMA 发送
 * 
 * @details DMA 空闲 -> 从 readIndex 开始发一段连续内存
 */
static void Serial_StartNext_Tx_DMA(void)
{
    uint16_t usedCount = 0;
    uint16_t Tx_Current_Length = 0;
    uint16_t startIndex = 0;

    if (Serial_handle_global.huart == NULL)
    {
        return;
    }

    /* 先在临界区里“拍板”：这次要发多少，从哪开始 */
    {
        uint32_t primaskValue = Serial_EnterCriticalSection();

        if (Serial_handle_global.Tx_Flag != 0)
        {
            Serial_ExitCriticalSection(primaskValue);
            return;
        }

        usedCount = Serial_CalculateRingBufferUsedCount(Serial_handle_global.Tx_WriteIndex,Serial_handle_global.Tx_ReadIndex,Serial_TxBuffer_Size);
        
        if (usedCount == 0)
        {
            Serial_ExitCriticalSection(primaskValue);
            return;
        }

        /* 这次 DMA 只能发“连续的一段”：
           - 如果 writeIndex 在 readIndex 前面：只能发到缓冲区末尾
           - 如果 writeIndex 在 readIndex 后面：可以直接发到 writeIndex */
        startIndex = Serial_handle_global.Tx_ReadIndex;

        if (Serial_handle_global.Tx_WriteIndex >= Serial_handle_global.Tx_ReadIndex)
        {
            Tx_Current_Length = (uint16_t)(Serial_handle_global.Tx_WriteIndex - Serial_handle_global.Tx_ReadIndex);
        }
        else
        {
            Tx_Current_Length = (uint16_t)(Serial_TxBuffer_Size - Serial_handle_global.Tx_ReadIndex);
        }

        Serial_handle_global.Tx_Flag = 1;
        Serial_handle_global.Rx_Current_Lenth = Tx_Current_Length;

        Serial_ExitCriticalSection(primaskValue);
    }

    /* 临界区外启动 DMA（避免长时间关中断） */
    if (Tx_Current_Length > 0)
    {
        HAL_StatusTypeDef status = UART_Transmit_Data(Serial_handle_global.huart,&Serial_handle_global.TxBuffer[startIndex],Tx_Current_Length);

        /* 如果启动失败，要把 Busy 状态回滚，否则会“卡死不再发” */
        if (status != HAL_OK)
        {
            uint32_t primaskValue = Serial_EnterCriticalSection();
            Serial_handle_global.Tx_Flag = 0u;
            Serial_handle_global.Rx_Current_Lenth = 0u;
            Serial_ExitCriticalSection(primaskValue);
        }
    }
}

/**
 * @brief 串口写数据到 Tx 环形缓冲区函数
 * 
 * @param data 要写入的数据指针
 * @param Tx_length 要写入的数据长度
 * @return uint16_t 实际写入的数据长度
 */
uint16_t Serial_Write_To_TxBuffer(const uint8_t *data, uint16_t Tx_length)
{
    uint16_t writtenCount = 0;

    if (data == NULL || Tx_length == 0u)
    {
        return 0;
    }

    /* 写入环形缓冲区：需要保护 index */
    {
        uint32_t primaskValue = Serial_EnterCriticalSection();

        for (writtenCount = 0; writtenCount < Tx_length; writtenCount++)
        {
            uint16_t nextWriteIndex = Serial_CalculateNextIndex(Serial_handle_global.Tx_WriteIndex,Serial_TxBuffer_Size);

            /* 满了：nextWriteIndex 追上 readIndex */
            if (nextWriteIndex == Serial_handle_global.Tx_ReadIndex)
            {
                break;
            }

            Serial_handle_global.TxBuffer[Serial_handle_global.Tx_WriteIndex] = data[writtenCount];
            Serial_handle_global.Tx_WriteIndex = nextWriteIndex;
        }

        Serial_ExitCriticalSection(primaskValue);
    }

    /* 写完后尝试启动 DMA（对应你原来的 Kick） */
    Serial_StartNext_Tx_DMA();

    return writtenCount;
}

/**
 * @brief Serial_Printf 串口打印函数
 * 
 * @param format 
 * @param ... 
 */
void Serial_Printf(const char *format, ...)
{
    char Printf_Temp_Buffer[Serial_Printf_Temp_Buffer_Size];

    va_list argumentList;
    va_start(argumentList, format);

    int length = vsnprintf(Printf_Temp_Buffer, sizeof(Printf_Temp_Buffer), format, argumentList);

    va_end(argumentList);

    if (length <= 0)
    {
        return;
    }

    if ((uint32_t)length > sizeof(Printf_Temp_Buffer))
    {
        length = (int)sizeof(Printf_Temp_Buffer);
    }

    (void)Serial_Write_To_TxBuffer((const uint8_t *)Printf_Temp_Buffer, (uint16_t)length);
}

/**
 * @brief 串口发送数据函数
 * 
 * @param data 要发送的数据指针
 * @param length 要发送的数据长度
 */
void Serial_Send_Data(const uint8_t *data, uint16_t length)
{
    Serial_Write_To_TxBuffer(data, length);
}

//RX环形缓冲区读取 接口
uint16_t Serial_GetReceiveRingBufferCount(void)
{
    uint16_t usedCount = 0;

    uint32_t primaskValue = Serial_EnterCriticalSection();

    usedCount = Serial_CalculateRingBufferUsedCount(Serial_handle_global.Rx_WriteIndex,Serial_handle_global.Rx_ReadIndex,Serial_RxBuffer_Size);

    Serial_ExitCriticalSection(primaskValue);

    return usedCount;
}

uint16_t Serial_ReadFromReceiveRingBuffer(uint8_t *destination, uint16_t destinationSize)
{
    uint16_t readCount = 0;

    if (destination == NULL || destinationSize == 0u)
    {
        return 0;
    }

    uint32_t primaskValue = Serial_EnterCriticalSection();

    while (readCount < destinationSize)
    {
        if (Serial_handle_global.Rx_ReadIndex == Serial_handle_global.Rx_WriteIndex)
        {
            break; /* 空了 */
        }

        destination[readCount] = Serial_handle_global.RxBuffer[Serial_handle_global.Rx_ReadIndex];
        Serial_handle_global.Rx_ReadIndex = Serial_CalculateNextIndex(Serial_handle_global.Rx_ReadIndex,Serial_RxBuffer_Size);
        readCount++;
    }

    Serial_ExitCriticalSection(primaskValue);

    return readCount;
}

/**
 * @brief 串口发送回调函数
 * 
 * @param huartx
 */
void Serial_Tx_Callback_Function(UART_HandleTypeDef *huart)
{
    if(Serial_handle_global.huart == NULL)
    {
        return;
    }

    /* 在中断里推进 readIndex，并继续发下一段 */
    {
        uint32_t primaskValue = Serial_EnterCriticalSection();

        Serial_handle_global.Tx_ReadIndex = (uint16_t)(Serial_handle_global.Tx_ReadIndex + Serial_handle_global.Rx_Current_Lenth);
        if (Serial_handle_global.Tx_ReadIndex >= Serial_TxBuffer_Size)
        {
            Serial_handle_global.Tx_ReadIndex = (uint16_t)(Serial_handle_global.Tx_ReadIndex - Serial_TxBuffer_Size);
        }

        Serial_handle_global.Tx_Flag = 0;
        Serial_handle_global.Rx_Current_Lenth = 0;

        Serial_ExitCriticalSection(primaskValue);
    }

    Serial_StartNext_Tx_DMA();
}

/**
 * @brief 串口接收数据回调函数
 * 
 * @param Buffer 
 * @param Length 
 */
void Serial_Rx_Callback_Function(void *context, uint8_t *Buffer, uint16_t Length)
{
    (void)context;

    if(Serial_handle_global.huart == NULL)
    {
        return;
    }

     if (Length > 0)
    {
        Serial_Push_Data_To_RxBuffer(Buffer, Length);
    }
}
