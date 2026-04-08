/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Vision.cpp
  * @brief   Vision USB communication
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "Vision.h"
#include "bsp_usb.h"

extern uint32_t Task_Time;

Class_Vision Vision;

void Vision_USB_CallBack(uint8_t *Buffer, uint16_t Length)
{
    if (Length == 0) return;

    if (Global_Init_Finished == false) return;

    //回显
    //USB_Transmit_Data(Buffer, Length);

    //长度限制
    if (Length != sizeof(Class_Vision::USB_RX_Frame_u)) return;

    memcpy(Vision.Receive_Union.Raw, Buffer, sizeof(Vision.Receive_Union.Raw));

    //判断包头包尾
    if (Vision.Receive_Union.Data.Frame_Header != 0xAA || Vision.Receive_Union.Data.Frame_Tail != 0x55)
    {
        return;
    }

    //在线处理
    Vision.Online_Time = Task_Time;
    Vision.USB_Rx_Flag = true;
    Vision.Online_State = true;

    if (Vision.Receive_Union.Data.Mode == 1)
    {
        Vision.Detected_State = true;
        Vision.Delta_Yaw = Vision.Receive_Union.Data.Delta_Yaw_10 * 0.1f;
        Vision.Delta_Pitch = Vision.Receive_Union.Data.Delta_Pitch_10 * 0.1f;
    }
    else
    {
        Vision.Detected_State = false;
        Vision.Delta_Yaw = 0.0f;
        Vision.Delta_Pitch = 0.0f;
    }
}

void Class_Vision::Init(void)
{
    Online_Time = 0;
    USB_Rx_Flag = false;
    Online_State = false;
    Detected_State = false;
    Delta_Yaw = 0.0f;
    Delta_Pitch = 0.0f;

    USB_Init(Vision_USB_CallBack);
}

void Class_Vision::USB_Transmit_Angle(float Yaw,float Pitch)
{
    Transmit_Union.Data.Frame_Header = 0xAA;
    Transmit_Union.Data.Mode = 1;
    Transmit_Union.Data.Yaw_10 = (int16_t)(Yaw * 10.0f);
    Transmit_Union.Data.Pitch_10 = (int16_t)(Pitch * 10.0f);
    Transmit_Union.Data.Frame_Tail = 0x55;

    USB_Transmit_Data(Transmit_Union.Raw, sizeof(Transmit_Union.Raw));
}

void Class_Vision::USB_Offline_Detection_1ms(uint32_t Task_Time)
{
    //等待初始化完成
    if (Global_Init_Finished == false)
    {
        Online_Time = 0U;
        USB_Rx_Flag = false;
        Online_State = false;
        Detected_State = false;
        Delta_Yaw = 0.0f;
        Delta_Pitch = 0.0f;
        return;
    }

    //判断是否接收到数据
    if (USB_Rx_Flag == false)
    {
        Online_State = false;
        Detected_State = false;
        Delta_Yaw = 0.0f;
        Delta_Pitch = 0.0f;
        return;
    }

    //判断在线状态
    if (Task_Time - Online_Time <= 100U)
    {
        Online_State = true;
        return;
    }

    USB_Rx_Flag = false;
    Online_State = false;
    Detected_State = false;
    Delta_Yaw = 0.0f;
    Delta_Pitch = 0.0f;
}
