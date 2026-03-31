/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Gimbal.cpp
  * @brief   云台库
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "Gimbal.h"

/**
 * @brief 复位前向轴角度相关状态
 *
 */
void Class_Gimbal::Reset_Front_Angle_State(void)
{
    Front_Yaw = 0.0f;
    Front_Pitch = 0.0f;
    Front_Roll = 0.0f;

    Last_Front_Yaw = 0.0f;
    Last_Front_Pitch = 0.0f;
    Last_Front_Roll = 0.0f;

    Delta_Front_Yaw = 0.0f;
    Delta_Front_Pitch = 0.0f;
    Delta_Front_Roll = 0.0f;

    Front_Yaw_Count = 0;
    Front_Pitch_Count = 0;
    Front_Roll_Count = 0;

    Front_Continuous_Yaw = 0.0f;
    Front_Continuous_Pitch = 0.0f;
    Front_Continuous_Roll = 0.0f;

    Target_Front_Continuous_Yaw = 0.0f;
    Target_Front_Continuous_Pitch = 0.0f;
    Target_Front_Continuous_Roll = 0.0f;

    Front_Angle_Initialized = 0;
}

/**
 * @brief 设置云台前向轴角度（Yaw + Pitch）
 *
 * @param Yaw   当前前向轴 Yaw 角（包角）
 * @param Pitch 当前前向轴 Pitch 角（包角）
 */
void Class_Gimbal::Set_Front_Angle(float Yaw, float Pitch)
{
    Set_Front_Yaw(Yaw);
    Set_Front_Pitch(Pitch);
}

/**
 * @brief 设置云台前向轴角度（Yaw + Pitch + Roll）
 *
 * @param Yaw   当前前向轴 Yaw 角（包角）
 * @param Pitch 当前前向轴 Pitch 角（包角）
 * @param Roll  当前前向轴 Roll 角（包角）
 */
void Class_Gimbal::Set_Front_Angle(float Yaw, float Pitch, float Roll)
{
    Set_Front_Yaw(Yaw);
    Set_Front_Pitch(Pitch);
    Set_Front_Roll(Roll);
}

/**
 * @brief 计算前向轴角度差值，并自动处理跨圈计数
 *
 */
void Class_Gimbal::Set_Delta_Front_Angle(void)
{
    if(Front_Angle_Initialized == 0U)
    {
        Last_Front_Yaw = Front_Yaw;
        Last_Front_Pitch = Front_Pitch;
        Last_Front_Roll = Front_Roll;

        Delta_Front_Yaw = 0.0f;
        Delta_Front_Pitch = 0.0f;
        Delta_Front_Roll = 0.0f;

        Front_Yaw_Count = 0;
        Front_Pitch_Count = 0;
        Front_Roll_Count = 0;

        Front_Angle_Initialized = 1U;
        return;
    }

    Delta_Front_Yaw = Front_Yaw - Last_Front_Yaw;
    Delta_Front_Pitch = Front_Pitch - Last_Front_Pitch;
    Delta_Front_Roll = Front_Roll - Last_Front_Roll;

    if(Delta_Front_Yaw < -180.0f)
    {
        Front_Yaw_Count++;
    }
    else if(Delta_Front_Yaw > 180.0f)
    {
        Front_Yaw_Count--;
    }

    if(Delta_Front_Pitch < -180.0f)
    {
        Front_Pitch_Count++;
    }
    else if(Delta_Front_Pitch > 180.0f)
    {
        Front_Pitch_Count--;
    }

    if(Delta_Front_Roll < -180.0f)
    {
        Front_Roll_Count++;
    }
    else if(Delta_Front_Roll > 180.0f)
    {
        Front_Roll_Count--;
    }

    Last_Front_Yaw = Front_Yaw;
    Last_Front_Pitch = Front_Pitch;
    Last_Front_Roll = Front_Roll;
}

/**
 * @brief 根据当前包角和圈数计数，计算前向轴连续角度
 *
 */
void Class_Gimbal::Set_Front_Continuous_Angle(void)
{
    Front_Continuous_Yaw = Front_Yaw + (float)Front_Yaw_Count * 360.0f;
    Front_Continuous_Pitch = Front_Pitch + (float)Front_Pitch_Count * 360.0f;
    Front_Continuous_Roll = Front_Roll + (float)Front_Roll_Count * 360.0f;
}

/**
 * @brief 更新前向轴角度（Yaw + Pitch），并自动完成差值与连续角计算
 *
 * @param Yaw   当前前向轴 Yaw 角（包角）
 * @param Pitch 当前前向轴 Pitch 角（包角）
 */
void Class_Gimbal::Update_Front_Angle(float Yaw, float Pitch)
{
    Set_Front_Angle(Yaw, Pitch);
    Set_Delta_Front_Angle();
    Set_Front_Continuous_Angle();
}

/**
 * @brief 更新前向轴角度（Yaw + Pitch + Roll），并自动完成差值与连续角计算
 *
 * @param Yaw   当前前向轴 Yaw 角（包角）
 * @param Pitch 当前前向轴 Pitch 角（包角）
 * @param Roll  当前前向轴 Roll 角（包角）
 */
void Class_Gimbal::Update_Front_Angle(float Yaw, float Pitch, float Roll)
{
    Set_Front_Angle(Yaw, Pitch, Roll);
    Set_Delta_Front_Angle();
    Set_Front_Continuous_Angle();
}