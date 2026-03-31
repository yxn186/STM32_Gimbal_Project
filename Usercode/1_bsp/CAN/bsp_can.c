/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    CAN.c
  * @brief   CAN使用
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "bsp_can.h"


static  CAN_RxCallBack_Function CAN_RxCallBack_FIFO0_Function = 0;
static  CAN_RxCallBack_Function CAN_RxCallBack_FIFO1_Function = 0;

/**
 * @brief CAN注册RX回调函数（FIFO0）
 * 
 * @param CallBack_Function 
 */
void CAN_Register_RxCallBack_FIFO0_Function(CAN_RxCallBack_Function CallBack_Function)
{
    CAN_RxCallBack_FIFO0_Function = CallBack_Function;
}

/**
 * @brief CAN注册RX回调函数（FIFO1）
 * 
 * @param CallBack_Function 
 */
void CAN_Register_RxCallBack_FIFO1_Function(CAN_RxCallBack_Function CallBack_Function)
{
    CAN_RxCallBack_FIFO1_Function = CallBack_Function;
}


/**
 * @brief 初始化CAN
 * 
 * @param hcan hcanx
 */
void CAN_Init(CAN_HandleTypeDef *hcan)
{
  HAL_CAN_Start(hcan);
  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
}

/**
 * @brief 配置CAN的滤波器
 *
 * @param hcan CAN编号（hcanx）
 * @param Object_Para 编号[3:] | FIFOx[2:2] | ID类型[1:1] | 帧类型[0:0]
 * @param ID ID
 * @param Mask_ID 屏蔽位(0x7ff, 0x1fffffff)
 */
void CAN_Filter_Mask_Config(CAN_HandleTypeDef *hcan, uint8_t Object_Para, uint32_t ID, uint32_t Mask_ID)
{
  CAN_FilterTypeDef Can_FilterInitstructure = {0};
  // 看第0位ID, 判断是数据帧还是遥控帧
  // 遥控帧暂不处理
  if (Object_Para & 0x01)
  {
    return;
  }

  // 看第1位ID, 判断是标准帧还是扩展帧
  // 扩展帧暂不处理
  if ((Object_Para & 0x02) >> 1)
  {
    return;
  }

  //过滤器编号配置 防止超
  uint8_t FilterBank = (Object_Para >> 3) & 0x1F;
  if (hcan->Instance == CAN1)
  {
      if (FilterBank >= 14)
      {
          return;
      }
  }
  else if (hcan->Instance == CAN2)
  {
      if (FilterBank < 14 || FilterBank > 27)
      {
          return;
      }
  }

  // 标准帧

  // ID配置, 标准帧的ID是11bit, 按规定放在高16bit中的[15:5]位
  // 掩码后ID的高16bit
  Can_FilterInitstructure.FilterIdHigh = (ID & 0x7FF) << 5;
  // 掩码后ID的低16bit
  Can_FilterInitstructure.FilterIdLow = 0x0000;
  // 掩码后屏蔽位的高16bit
  Can_FilterInitstructure.FilterMaskIdHigh = (Mask_ID & 0x7FF) << 5;
  // 掩码后屏蔽位的低16bit
  Can_FilterInitstructure.FilterMaskIdLow = 0x0000;

  // 滤波器配置
  // 滤波器序号0-27, 共28个滤波器, can1是0~13, can2是14~27
  Can_FilterInitstructure.FilterBank = FilterBank;
  // 滤波器模式, 设置ID掩码模式
  Can_FilterInitstructure.FilterMode = CAN_FILTERMODE_IDMASK;
  // 32位滤波
  Can_FilterInitstructure.FilterScale = CAN_FILTERSCALE_32BIT;
  // 使能滤波器
  Can_FilterInitstructure.FilterActivation = ENABLE;
  
  // 从机模式配置
  // 从机模式选择开始单元, 一般均分14个单元给CAN1和CAN2
  Can_FilterInitstructure.SlaveStartFilterBank = 14;

  // 滤波器绑定FIFOx, 只能绑定一个
  Can_FilterInitstructure.FilterFIFOAssignment = ((Object_Para >> 2) & 0x01) ? CAN_FILTER_FIFO1 : CAN_FILTER_FIFO0;

  HAL_CAN_ConfigFilter(hcan, &Can_FilterInitstructure);
}

/**
 * @brief 发送CAN数据帧
 *
 * @param hcan CAN编号（hcanx）
 * @param ID ID
 * @param Data 被发送的数据指针
 * @param Length 长度
 * @return HAL_StatusTypeDef 执行状态
 */
HAL_StatusTypeDef CAN_Send_Data(CAN_HandleTypeDef *hcan, uint16_t ID, uint8_t *Data, uint16_t Length)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t used_mailbox;

  // 检测关键传参
  assert_param(hcan != NULL);

  TxHeader.StdId = ID;
  TxHeader.ExtId = 0;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = Length;

  return (HAL_CAN_AddTxMessage(hcan, &TxHeader, Data, &used_mailbox));
}

/**
 * @brief HAL库CAN接收FIFO0中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    //接收缓冲区
    static CAN_Rx_Buffer_t CAN_Rx_Buffer_FIFO0;

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_Rx_Buffer_FIFO0.Header, CAN_Rx_Buffer_FIFO0.Data);

    if(CAN_RxCallBack_FIFO0_Function)
    {
      CAN_RxCallBack_FIFO0_Function(&CAN_Rx_Buffer_FIFO0);
    }
}

/**
 * @brief HAL库CAN接收FIFO1中断
 *
 * @param hcan CAN编号
 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  //接收缓冲区
  static CAN_Rx_Buffer_t CAN_Rx_Buffer_FIFO1;
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN_Rx_Buffer_FIFO1.Header, CAN_Rx_Buffer_FIFO1.Data);
  if(CAN_RxCallBack_FIFO1_Function)
  {
    CAN_RxCallBack_FIFO1_Function(&CAN_Rx_Buffer_FIFO1);
  }
}

