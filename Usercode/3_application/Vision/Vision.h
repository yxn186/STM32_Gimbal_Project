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
        int16_t Delta_Yaw_10;
        int16_t Delta_Pitch_10;
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
        int16_t Yaw_10;
        int16_t Pitch_10;
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

    public:

    void Init(void);

    void USB_Transmit_Angle(float Yaw,float Pitch);

    void USB_Offline_Detection_1ms(uint32_t Task_Time);

    bool Get_Online_State(void) const { return Online_State; }

    bool Get_Detected_State(void) const { return Detected_State; }

    float Get_Delta_Yaw(void) const { return Delta_Yaw; }
    float Get_Delta_Pitch(void) const { return Delta_Pitch; }
};

extern Class_Vision Vision;

#endif /* __VISION_H__ */
