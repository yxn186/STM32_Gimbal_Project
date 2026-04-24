/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Vision.h
  * @brief   This file contains all the function prototypes for
  *          the Vision.cpp file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __VISION_H__
#define __VISION_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*YOUR CODE*/
class Class_Vision
{
    protected:

    //视觉USB接收
    friend void Vision_USB_CallBack(uint8_t *Buffer, uint16_t Length);

    #pragma pack(push, 1)
    typedef struct
    {
        uint8_t Frame_Header;
        uint8_t Mode;
        float Yaw;
        float Pitch;
        uint8_t Frame_Tail;
    } USB_RX_Frame_t;
    #pragma pack(pop)

    typedef union
    {
        USB_RX_Frame_t Data;
        uint8_t Raw[sizeof(USB_RX_Frame_t)];
    } USB_RX_Frame_u;

    USB_RX_Frame_u Receive_Union;

    //视觉USB发送
    #pragma pack(push, 1)
    typedef struct
    {
        uint8_t Frame_Header;
        uint8_t Mode;
        float Yaw;
        float Pitch;
        uint8_t Frame_Tail;
    } USB_TX_Frame_t;
    #pragma pack(pop)

    typedef union
    {
        USB_TX_Frame_t Data;
        uint8_t Raw[sizeof(USB_TX_Frame_t)];
    } USB_TX_Frame_u;

    USB_TX_Frame_u Transmit_Union;


    uint32_t Online_Time = 0;
    bool USB_Rx_Flag = false;
    bool Online_State = false;
    bool Detected_State = false;

    float Delta_Yaw = 0.0f;
    float Delta_Pitch = 0.0f;

    float Yaw = 0.0f;
    float Pitch = 0.0f;

    // 模式去抖动相关
    uint8_t Confirmed_Mode = 0;          // 当前已确认生效的模式
    uint8_t Pending_Mode = 0;            // 正在累计计数的候选模式
    uint8_t Mode_Pending_Count = 0;      // 候选模式连续帧计数
    uint8_t Mode_Confirm_Threshold = 5; // 连续N帧才确认模式切换（可调）
    float Last_Valid_Yaw = 0.0f;   // mode=1时最后一次有效Yaw偏差
    float Last_Valid_Pitch = 0.0f; // mode=1时最后一次有效Pitch偏差

    void Mode_Debounce_Filter(uint8_t Raw_Mode, float Raw_Yaw, float Raw_Pitch);

    public:

    void Init(void);

    void USB_Transmit_Angle(float Yaw,float Pitch);

    void USB_Offline_Detection_1ms(uint32_t Task_Time);

    /**
     * @brief 设置模式去抖动确认阈值
     * @param n 连续收到n帧新模式才确认切换，默认10
     */
    void Set_Mode_Confirm_Threshold(uint8_t n) { Mode_Confirm_Threshold = (n > 0U) ? n : 1U; }

    bool Get_Online_State(void) const { return Online_State; }

    bool Get_Detected_State(void) const { return Detected_State; }

    float Get_Yaw(void) const { return Yaw; }
    float Get_Pitch(void) const { return Pitch; }
};

extern Class_Vision Vision;

#endif /* __VISION_H__ */
