/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Gimbal.h
  * @brief   This file contains all the function prototypes for
  *          the Gimbal.cpp file
  ******************************************************************************
  */
/* USER CODE END Header */
#ifndef __GIMBAL_H__
#define __GIMBAL_H__

/* Includes ------------------------------------------------------------------*/
#include "main.h"

#ifdef __cplusplus

class Class_Gimbal
{
protected:
    /**
     * @brief 云台 IMU 反馈实际角度
     */
    float Imu_Yaw = 0.0f;
    float Imu_Pitch = 0.0f;
    float Imu_Roll = 0.0f;

    /**
     * @brief 云台前向轴当前角度（原始包角）
     */
    float Front_Yaw = 0.0f;
    float Front_Pitch = 0.0f;
    float Front_Roll = 0.0f;

    /**
     * @brief 云台前向轴上一次角度（原始包角）
     */
    float Last_Front_Yaw = 0.0f;
    float Last_Front_Pitch = 0.0f;
    float Last_Front_Roll = 0.0f;

    /**
     * @brief 云台前向轴角度差值
     */
    float Delta_Front_Yaw = 0.0f;
    float Delta_Front_Pitch = 0.0f;
    float Delta_Front_Roll = 0.0f;

    /**
     * @brief 云台前向轴连续角度圈数计数
     */
    int32_t Front_Yaw_Count = 0;
    int32_t Front_Pitch_Count = 0;
    int32_t Front_Roll_Count = 0;

    /**
     * @brief 云台前向轴连续实际角度
     */
    float Front_Continuous_Yaw = 0.0f;
    float Front_Continuous_Pitch = 0.0f;
    float Front_Continuous_Roll = 0.0f;

    /**
     * @brief 云台前向连续角下的目标角度
     */
    float Target_Front_Continuous_Yaw = 0.0f;
    float Target_Front_Continuous_Pitch = 0.0f;
    float Target_Front_Continuous_Roll = 0.0f;

    /**
     * @brief 前向轴角度是否已经完成首次初始化
     */
    uint8_t Front_Angle_Initialized = 0;

public:
    /* ========================= 多行函数：放 .cpp ========================= */

    /**
     * @brief 复位前向轴角度相关状态
     */
    void Reset_Front_Angle_State(void);

    /**
     * @brief 设置云台前向轴角度（Yaw + Pitch）
     *
     * @param Yaw   当前前向轴 Yaw 角（包角）
     * @param Pitch 当前前向轴 Pitch 角（包角）
     */
    void Set_Front_Angle(float Yaw, float Pitch);

    /**
     * @brief 设置云台前向轴角度（Yaw + Pitch + Roll）
     *
     * @param Yaw   当前前向轴 Yaw 角（包角）
     * @param Pitch 当前前向轴 Pitch 角（包角）
     * @param Roll  当前前向轴 Roll 角（包角）
     */
    void Set_Front_Angle(float Yaw, float Pitch, float Roll);

    /**
     * @brief 计算前向轴角度差值，并自动处理跨圈计数
     */
    void Set_Delta_Front_Angle(void);

    /**
     * @brief 根据当前包角和圈数计数，计算前向轴连续角度
     */
    void Set_Front_Continuous_Angle(void);

    /**
     * @brief 更新前向轴角度（Yaw + Pitch），并自动完成差值与连续角计算
     *
     * @param Yaw   当前前向轴 Yaw 角（包角）
     * @param Pitch 当前前向轴 Pitch 角（包角）
     */
    void Update_Front_Angle(float Yaw, float Pitch);

    /**
     * @brief 更新前向轴角度（Yaw + Pitch + Roll），并自动完成差值与连续角计算
     *
     * @param Yaw   当前前向轴 Yaw 角（包角）
     * @param Pitch 当前前向轴 Pitch 角（包角）
     * @param Roll  当前前向轴 Roll 角（包角）
     */
    void Update_Front_Angle(float Yaw, float Pitch, float Roll);

    /* ========================= 一行函数：直接放 .h ========================= */

    /**
     * @brief 设置云台前向轴 Yaw 角
     *
     * @param Yaw 当前前向轴 Yaw 角（包角）
     */
    void Set_Front_Yaw(float Yaw) { Front_Yaw = Yaw; }

    /**
     * @brief 设置云台前向轴 Pitch 角
     *
     * @param Pitch 当前前向轴 Pitch 角（包角）
     */
    void Set_Front_Pitch(float Pitch) { Front_Pitch = Pitch; }

    /**
     * @brief 设置云台前向轴 Roll 角
     *
     * @param Roll 当前前向轴 Roll 角（包角）
     */
    void Set_Front_Roll(float Roll) { Front_Roll = Roll; }

    /**
     * @brief 设置云台前向连续角下的目标 Yaw 角（相对于当前实际连续角）
     *
     * @param Delta_Yaw 相对于当前连续 Yaw 的增量
     */
    void Set_Target_Front_Continuous_Yaw(float Delta_Yaw) { Target_Front_Continuous_Yaw = Front_Continuous_Yaw + Delta_Yaw; }

    /**
     * @brief 设置云台前向连续角下的目标 Pitch 角（相对于当前实际连续角）
     *
     * @param Delta_Pitch 相对于当前连续 Pitch 的增量
     */
    void Set_Target_Front_Continuous_Pitch(float Delta_Pitch) { Target_Front_Continuous_Pitch = Front_Continuous_Pitch + Delta_Pitch; }

    /**
     * @brief 设置云台前向连续角下的目标 Roll 角（相对于当前实际连续角）
     *
     * @param Delta_Roll 相对于当前连续 Roll 的增量
     */
    void Set_Target_Front_Continuous_Roll(float Delta_Roll) { Target_Front_Continuous_Roll = Front_Continuous_Roll + Delta_Roll; }

    /**
     * @brief 直接设置前向连续目标 Yaw 角（绝对值）
     *
     * @param Target_Yaw 目标连续 Yaw 角
     */
    void Set_Target_Front_Continuous_Yaw_Absolute(float Target_Yaw) { Target_Front_Continuous_Yaw = Target_Yaw; }

    /**
     * @brief 直接设置前向连续目标 Pitch 角（绝对值）
     *
     * @param Target_Pitch 目标连续 Pitch 角
     */
    void Set_Target_Front_Continuous_Pitch_Absolute(float Target_Pitch) { Target_Front_Continuous_Pitch = Target_Pitch; }

    /**
     * @brief 直接设置前向连续目标 Roll 角（绝对值）
     *
     * @param Target_Roll 目标连续 Roll 角
     */
    void Set_Target_Front_Continuous_Roll_Absolute(float Target_Roll) { Target_Front_Continuous_Roll = Target_Roll; }

    /**
     * @brief 获取当前前向轴 Yaw 角（包角）
     *
     * @return float
     */
    float Get_Front_Yaw(void) const { return Front_Yaw; }

    /**
     * @brief 获取当前前向轴 Pitch 角（包角）
     *
     * @return float
     */
    float Get_Front_Pitch(void) const { return Front_Pitch; }

    /**
     * @brief 获取当前前向轴 Roll 角（包角）
     *
     * @return float
     */
    float Get_Front_Roll(void) const { return Front_Roll; }

    /**
     * @brief 获取上一次前向轴 Yaw 角（包角）
     *
     * @return float
     */
    float Get_Last_Front_Yaw(void) const { return Last_Front_Yaw; }

    /**
     * @brief 获取上一次前向轴 Pitch 角（包角）
     *
     * @return float
     */
    float Get_Last_Front_Pitch(void) const { return Last_Front_Pitch; }

    /**
     * @brief 获取上一次前向轴 Roll 角（包角）
     *
     * @return float
     */
    float Get_Last_Front_Roll(void) const { return Last_Front_Roll; }

    /**
     * @brief 获取前向轴 Yaw 角度差值
     *
     * @return float
     */
    float Get_Delta_Front_Yaw(void) const { return Delta_Front_Yaw; }

    /**
     * @brief 获取前向轴 Pitch 角度差值
     *
     * @return float
     */
    float Get_Delta_Front_Pitch(void) const { return Delta_Front_Pitch; }

    /**
     * @brief 获取前向轴 Roll 角度差值
     *
     * @return float
     */
    float Get_Delta_Front_Roll(void) const { return Delta_Front_Roll; }

    /**
     * @brief 获取前向轴 Yaw 连续圈数计数
     *
     * @return int32_t
     */
    int32_t Get_Front_Yaw_Count(void) const { return Front_Yaw_Count; }

    /**
     * @brief 获取前向轴 Pitch 连续圈数计数
     *
     * @return int32_t
     */
    int32_t Get_Front_Pitch_Count(void) const { return Front_Pitch_Count; }

    /**
     * @brief 获取前向轴 Roll 连续圈数计数
     *
     * @return int32_t
     */
    int32_t Get_Front_Roll_Count(void) const { return Front_Roll_Count; }

    /**
     * @brief 获取前向轴连续 Yaw 角
     *
     * @return float
     */
    float Get_Front_Continuous_Yaw(void) const { return Front_Continuous_Yaw; }

    /**
     * @brief 获取前向轴连续 Pitch 角
     *
     * @return float
     */
    float Get_Front_Continuous_Pitch(void) const { return Front_Continuous_Pitch; }

    /**
     * @brief 获取前向轴连续 Roll 角
     *
     * @return float
     */
    float Get_Front_Continuous_Roll(void) const { return Front_Continuous_Roll; }

    /**
     * @brief 获取前向轴连续目标 Yaw 角
     *
     * @return float
     */
    float Get_Target_Front_Continuous_Yaw(void) const { return Target_Front_Continuous_Yaw; }

    /**
     * @brief 获取前向轴连续目标 Pitch 角
     *
     * @return float
     */
    float Get_Target_Front_Continuous_Pitch(void) const { return Target_Front_Continuous_Pitch; }

    /**
     * @brief 获取前向轴连续目标 Roll 角
     *
     * @return float
     */
    float Get_Target_Front_Continuous_Roll(void) const { return Target_Front_Continuous_Roll; }
};

#endif /* __cplusplus */

#endif /* __GIMBAL_H__ */