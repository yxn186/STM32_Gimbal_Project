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
extern volatile bool Gimbal_Vision_Ready;
extern volatile bool Gimbal_Auto_Mode_Ready;

Class_Vision Vision;

void Vision_USB_CallBack(uint8_t *Buffer, uint16_t Length)
{
    if (Length == 0) return;

    if (Global_Init_Finished == false) return;
    if (Gimbal_Vision_Ready == false) return;
    if (Gimbal_Auto_Mode_Ready == false) return;

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
    Vision.Rx_Count++;

    Vision.Mode_Debounce_Filter(
        Vision.Receive_Union.Data.Mode,
        Vision.Receive_Union.Data.Yaw,
        Vision.Receive_Union.Data.Pitch
    );
}

void Class_Vision::Init(void)
{
    Online_Time = 0;
    USB_Rx_Flag = false;
    Online_State = false;
    Detected_State = false;
    Delta_Yaw = 0.0f;
    Delta_Pitch = 0.0f;

    Confirmed_Mode = 0;
    Pending_Mode = 0;
    Mode_Pending_Count = 0;
    Last_Valid_Yaw = 0.0f;
    Last_Valid_Pitch = 0.0f;

    USB_Init(Vision_USB_CallBack);
}

/**
 * @brief 模式去抖动滤波器
 * @details
 * mode 0->1 和 mode 1->0 使用独立阈值确认切换。
 * 突发跳变帧数不足时忽略，保持当前确认模式不变。
 * mode=1 切换为 0 时，Delta_Yaw/Pitch 保留最后一次有效值（不清零）。
 *
 * @param Raw_Mode     本帧原始模式字段
 * @param Raw_Yaw   本帧原始Yaw偏差
 * @param Raw_Pitch 本帧原始Pitch偏差
 */
void Class_Vision::Mode_Debounce_Filter(uint8_t Raw_Mode, float Raw_Yaw, float Raw_Pitch)
{
    // 每次收到 mode=1 的帧，实时记录当时的有效角度
    if (Raw_Mode == 1)
    {
        Last_Valid_Yaw   = Raw_Yaw;
        Last_Valid_Pitch = Raw_Pitch;
    }

    // 连续帧计数：和候选模式相同则累加，否则重置候选
    if (Raw_Mode == Pending_Mode)
    {
        if (Mode_Pending_Count < 255U)
        {
            Mode_Pending_Count++;
        }
    }
    else
    {
        Pending_Mode = Raw_Mode;
        Mode_Pending_Count = 1U;
    }

    uint8_t Mode_confirm_threshold = 1U;
    if (Confirmed_Mode == 0U && Pending_Mode == 1U)
    {
        Mode_confirm_threshold = Mode_Confirm_Threshold_0_To_1;
    }
    else if (Confirmed_Mode == 1U && Pending_Mode == 0U)
    {
        Mode_confirm_threshold = Mode_Confirm_Threshold_1_To_0;
    }

    // 达到对应方向的阈值才真正切换已确认模式
    if (Mode_Pending_Count >= Mode_confirm_threshold)
    {
        Confirmed_Mode = Pending_Mode;
    }

    // 根据已确认模式更新对外输出
    if (Confirmed_Mode == 1)
    {
        Detected_State = true;
        Yaw   = Raw_Yaw;
        Pitch = Raw_Pitch;
    }
    else
    {
        Detected_State = false;
        // mode1→0 确认切换后，保留最后一次有效角度，不清零
        Yaw   = Last_Valid_Yaw;
        Pitch = Last_Valid_Pitch;
    }
}

void Class_Vision::USB_Transmit_Angle(float Yaw,float Pitch)
{
    Transmit_Union.Data.Frame_Header = 0xAA;
    Transmit_Union.Data.Mode = 1;
    Transmit_Union.Data.Yaw = Yaw;
    Transmit_Union.Data.Pitch = Pitch;
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
        Rx_Count = 0;
        Rx_Freq = 0.0f;
        Freq_Sample_Timer = 0;
        return;
    }

    // 接收频率采样（每1000ms统计一次）
    Freq_Sample_Timer++;
    if (Freq_Sample_Timer >= 1000U)
    {
        Rx_Freq = (float)Rx_Count;  // 单位: Hz
        Rx_Count = 0;
        Freq_Sample_Timer = 0;
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
