/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    CAN.h
  * @brief   This file contains all the function prototypes for
  *          the MyCAN.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MYCAN_H__
#define __MYCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*YOUR CODE*/

// 滤波器编号
#define CAN_FILTER(x) ((x) << 3)

// 接收队列
#define CAN_FIFO_0 (0 << 2)
#define CAN_FIFO_1 (1 << 2)

//标准帧或扩展帧
#define CAN_STDID (0 << 1)
#define CAN_EXTID (1 << 1)

// 数据帧或遥控帧
#define CAN_DATA_TYPE (0 << 0)
#define CAN_REMOTE_TYPE (1 << 0)

/**
 * @brief CAN接收的信息Buffer结构体
 *
 */
typedef struct
{
    CAN_RxHeaderTypeDef Header;
    uint8_t Data[8];
} CAN_Rx_Buffer_t;

/**
 * @brief CAN通信接收回调函数数据类型
 *
 */
typedef void (*CAN_RxCallBack_Function)(CAN_Rx_Buffer_t *);

/**
 * @brief CAN注册RX回调函数（FIFO0）
 * 
 * @param CallBack_Function 
 */
void CAN_Register_RxCallBack_FIFO0_Function(CAN_RxCallBack_Function CallBack_Function);

/**
 * @brief CAN注册RX回调函数（FIFO1）
 * 
 * @param CallBack_Function 
 */
void CAN_Register_RxCallBack_FIFO1_Function(CAN_RxCallBack_Function CallBack_Function);

/**
 * @brief 初始化CAN
 * 
 * @param hcan hcanx
 */
void CAN_Init(CAN_HandleTypeDef *hcan);

/**
 * @brief 配置CAN的滤波器
 *
 * @param hcan CAN编号（hcanx）
 * @param Object_Para 编号[3:] | FIFOx[2:2] | ID类型[1:1] | 帧类型[0:0]
 * @param ID ID
 * @param Mask_ID 屏蔽位(0x7ff, 0x1fffffff)
 */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID);

/**
 * @brief 发送CAN数据帧
 *
 * @param hcan CAN编号（hcanx）
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return HAL_StatusTypeDef 执行状态
 */
HAL_StatusTypeDef CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length);

/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan);


#ifdef __cplusplus
}
#endif

#endif /* __MYCAN_H__ */
