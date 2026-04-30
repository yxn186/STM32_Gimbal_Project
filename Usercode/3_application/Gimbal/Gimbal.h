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

    /* ---矩阵--- */

    /**
     * @brief 云台通用角度状态结构体
     * 
     */
    typedef struct
    {
        float Raw_Angle = 0.0f;
        float Last_Raw_Angle = 0.0f;
        float Delta_Angle = 0.0f;
        float Continuous_Angle = 0.0f;
        float Startup_Zero_Angle = 0.0f;
        int32_t Angle_Count = 0;
        uint8_t Initialized = 0U;
    } Struct_Gimbal_Angle_State_t;

    /**
     * @brief Yaw电机角度状态
     * 
     */
    Struct_Gimbal_Angle_State_t Yaw_Motor_Angle_State;

    /**
     * @brief Pitch电机角度状态
     * 
     */
    Struct_Gimbal_Angle_State_t Pitch_Motor_Angle_State;

    /**
     * @brief IMU相对于启动时基座的Yaw角度状态
     * 
     */
    Struct_Gimbal_Angle_State_t Imu_Relative_BaseStart_Yaw_State;

    /**
     * @brief IMU相对于启动时基座的Pitch角度状态
     * 
     */
    Struct_Gimbal_Angle_State_t Imu_Relative_BaseStart_Pitch_State;

    /**
     * @brief IMU相对于启动时基座的Roll角度状态
     * 
     */
    Struct_Gimbal_Angle_State_t Imu_Relative_BaseStart_Roll_State;

    /**
     * @brief IMU相对于世界系的Yaw角度状态
     * 
     */
    Struct_Gimbal_Angle_State_t Imu_Relative_World_Yaw_State;
    float Imu_Relative_World_Yaw_Zero_Offset = 0.0f;
    uint8_t Imu_Relative_World_Yaw_Zero_Enabled = 0U;

    /**
     * @brief IMU相对于世界系的Pitch角度状态
     * 
     */
    Struct_Gimbal_Angle_State_t Imu_Relative_World_Pitch_State;

    /**
     * @brief IMU相对于世界系的Roll角度状态
     * 
     */
    Struct_Gimbal_Angle_State_t Imu_Relative_World_Roll_State;

    /**
     * @brief 电机模型参考角度（包角）
     * 
     */
    float Model_Relative_BaseStart_Yaw = 0.0f;
    float Model_Relative_BaseStart_Pitch = 0.0f;
    float Model_Relative_BaseStart_Roll = 0.0f;

    /**
     * @brief 电机模型参考连续角度
     * 
     */
    float Model_Relative_BaseStart_Continuous_Yaw = 0.0f;
    float Model_Relative_BaseStart_Continuous_Pitch = 0.0f;
    float Model_Relative_BaseStart_Continuous_Roll = 0.0f;

    /**
     * @brief IMU相对于启动时基座目标连续角度
     * 
     */
    float Target_Imu_Relative_BaseStart_Continuous_Yaw = 0.0f;
    float Target_Imu_Relative_BaseStart_Continuous_Pitch = 0.0f;
    float Target_Imu_Relative_BaseStart_Continuous_Roll = 0.0f;

    /**
     * @brief 启动时基座在世界系中的矩阵是否已建立
     * 
     */
    uint8_t BaseStart_World_Initialized = 0U;

    /**
     * @brief Yaw电机方向修正系数
     * 
     */
    float Yaw_Motor_Direction = 1.0f;

    /**
     * @brief Pitch电机方向修正系数
     * 
     */
    float Pitch_Motor_Direction = 1.0f;

    /**
    * @brief Pitch机械零位角
    * 
    * 约定：
    * 当云台Pitch物理上水平时，对应的电机包角就是这个值
    */
    float Pitch_Mechanical_Zero_Angle = 0.0f;

    /**
     * @brief IMU原始坐标系在世界系中的实测旋转矩阵
     * 
     * v_world = R_World_From_ImuRaw_Measurement * v_imuRaw
     */
    float R_World_From_ImuRaw_Measurement[3][3] =
    {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    /**
     * @brief IMU虚拟坐标系在世界系中的实测旋转矩阵
     * 
     * v_world = R_World_From_ImuVirtual_Measurement * v_imuVirtual
     */
    float R_World_From_ImuVirtual_Measurement[3][3] =
    {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    /**
     * @brief 启动时基座在世界系中的旋转矩阵
     * 
     * v_world = R_World_From_BaseStart * v_baseStart
     */
    float R_World_From_BaseStart[3][3] =
    {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    /**
     * @brief IMU虚拟坐标系相对于启动时基座的真实旋转矩阵（由IMU实测得到）
     * 
     * v_baseStart = R_BaseStart_From_ImuVirtual_True * v_imuVirtual
     */
    float R_BaseStart_From_ImuVirtual_True[3][3] =
    {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    /**
     * @brief IMU原始坐标系相对于启动时基座的真实旋转矩阵（由IMU实测得到）
     * 
     * v_baseStart = R_BaseStart_From_ImuRaw_True * v_imuRaw
     */
    float R_BaseStart_From_ImuRaw_True[3][3] =
    {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    /**
     * @brief IMU虚拟坐标系相对于启动时基座的模型参考旋转矩阵（由电机机构链得到）
     * 
     * v_baseStart = R_BaseStart_From_ImuVirtual_Model * v_imuVirtual
     */
    float R_BaseStart_From_ImuVirtual_Model[3][3] =
    {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    /**
     * @brief IMU原始坐标系相对于启动时基座的模型参考旋转矩阵（由电机机构链得到）
     * 
     * v_baseStart = R_BaseStart_From_ImuRaw_Model * v_imuRaw
     */
    float R_BaseStart_From_ImuRaw_Model[3][3] =
    {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    /**
    * @brief IMU相对于当前时刻基座的模型角度（包角）
    * 
    */
    float Imu_Relative_BaseCurrent_Model_Yaw = 0.0f;
    float Imu_Relative_BaseCurrent_Model_Pitch = 0.0f;
    float Imu_Relative_BaseCurrent_Model_Roll = 0.0f;

    /**
    * @brief IMU相对于当前时刻基座的模型连续角度
    * 
    */
    float Imu_Relative_BaseCurrent_Model_Continuous_Yaw = 0.0f;
    float Imu_Relative_BaseCurrent_Model_Continuous_Pitch = 0.0f;
    float Imu_Relative_BaseCurrent_Model_Continuous_Roll = 0.0f;

    /**
    * @brief IMU虚拟坐标系相对于当前时刻基座的模型矩阵
    * 
    * v_baseCurrent = R_BaseCurrent_From_ImuVirtual_Model * v_imuVirtual
    */
    float R_BaseCurrent_From_ImuVirtual_Model[3][3] =
    {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

    /**
    * @brief IMU原始坐标系相对于当前时刻基座的模型矩阵
    * 
    * v_baseCurrent = R_BaseCurrent_From_ImuRaw_Model * v_imuRaw
    */
    float R_BaseCurrent_From_ImuRaw_Model[3][3] =
    {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}
    };

public:

    /**
     * @brief 复位“IMU相对于启动时基座”相关状态
     * 
     */
    void Reset_Imu_Relative_BaseStart_State(void);

    void Set_Imu_Relative_World_Yaw_Zero(void)
    {
        Imu_Relative_World_Yaw_Zero_Offset = Imu_Relative_World_Yaw_State.Continuous_Angle;
        Imu_Relative_World_Yaw_Zero_Enabled = 1U;
        Imu_Relative_World_Yaw_State.Raw_Angle = 0.0f;
        Imu_Relative_World_Yaw_State.Continuous_Angle = 0.0f;
    }

    /**
     * @brief 设置IMU角度（Yaw + Pitch + Roll）
     * 
     * @param Yaw
     * @param Pitch
     * @param Roll
     */
    void Set_Imu_Angle(float Yaw, float Pitch, float Roll);

    /**
     * @brief 计算IMU角度差值，并自动处理跨圈计数
     * 
     */
    void Set_Delta_Imu_Angle(void);

    /**
     * @brief 根据当前包角和圈数计数，计算IMU连续角度
     * 
     */
    void Set_Imu_Continuous_Angle(void);

    /**
     * @brief 更新IMU角度（Yaw + Pitch + Roll），并自动完成差值与连续角计算
     * 
     * @param Yaw
     * @param Pitch
     * @param Roll
     */
    void Update_Imu_Angle(float Yaw, float Pitch, float Roll);

    /**
     * @brief 更新单个角度状态（连续角）
     * 
     * @param Angle_State
     * @param Raw_Angle
     */
    void Update_Angle_State(Struct_Gimbal_Angle_State_t *Angle_State, float Raw_Angle);

    /**
     * @brief 四元数转旋转矩阵
     * 
     * @param Q0
     * @param Q1
     * @param Q2
     * @param Q3
     * @param R
     */
    void Quaternion_To_RotationMatrix(float Q0, float Q1, float Q2, float Q3, float R[3][3]);

    /**
     * @brief 旋转矩阵解算Yaw / Pitch / Roll
     * 
     * 约定：
     * - 坐标系：X前 Y左 Z上
     * - Yaw 正方向：向左转为正
     * - Pitch 正方向：抬头为正
     * - Roll 正方向：绕前向轴右手定则为正
     * 
     * @param R
     * @param Yaw
     * @param Pitch
     * @param Roll
     */
    void RotationMatrix_To_YawPitchRoll(const float R[3][3], float *Yaw, float *Pitch, float *Roll);

    /**
     * @brief 计算IMU原始坐标系在世界系中的实测旋转矩阵
     * 
     * @param Q0
     * @param Q1
     * @param Q2
     * @param Q3
     */
    void Set_R_World_From_ImuRaw_Measurement(float Q0, float Q1, float Q2, float Q3);

    /**
     * @brief 计算IMU虚拟坐标系在世界系中的实测旋转矩阵
     * 
     */
    void Set_R_World_From_ImuVirtual_Measurement(void);

    /**
     * @brief 计算电机模型参考角度
     * 
     */
    void Set_Model_Relative_BaseStart_Angle(void);

    /**
     * @brief 计算IMU虚拟坐标系相对于启动时基座的模型参考矩阵
     * 
     */
    void Set_R_BaseStart_From_ImuVirtual_Model(void);

    /**
     * @brief 计算IMU原始坐标系相对于启动时基座的模型参考矩阵
     * 
     */
    void Set_R_BaseStart_From_ImuRaw_Model(void);

    /**
    * @brief 计算IMU相对于当前时刻基座的模型角度
    * 
    */
    void Set_Imu_Relative_BaseCurrent_Model_Angle(void);

    /**
    * @brief 计算IMU虚拟坐标系相对于当前时刻基座的模型矩阵
    * 
    */
    void Set_R_BaseCurrent_From_ImuVirtual_Model(void);

    /**
    * @brief 计算IMU原始坐标系相对于当前时刻基座的模型矩阵
    * 
    */
    void Set_R_BaseCurrent_From_ImuRaw_Model(void);

    /**
     * @brief 尝试建立“启动时基座在世界系中的矩阵”
     * 
     * @param Now_Time
     * @param Delay_Time
     */
    void Try_Init_BaseStart_World(uint32_t Now_Time, uint32_t Delay_Time = 100U);

    /**
     * @brief 计算IMU虚拟坐标系相对于启动时基座的真实矩阵
     * 
     */
    void Set_R_BaseStart_From_ImuVirtual_True(void);

    /**
     * @brief 计算IMU原始坐标系相对于启动时基座的真实矩阵
     * 
     */
    void Set_R_BaseStart_From_ImuRaw_True(void);

    /**
     * @brief 从“真实相对矩阵”中解算IMU相对于启动时基座的角度
     * 
     */
    void Set_Imu_Relative_BaseStart_Angle(void);

    /**
     * @brief 从“世界系实测矩阵”中解算IMU相对于世界系的角度
     * 
     */
    void Set_Imu_Relative_World_Angle(void);

    /**
     * @brief 总更新函数：只更新IMU虚拟坐标系相对于世界系的姿态角
     * 
     * @param Q0
     * @param Q1
     * @param Q2
     * @param Q3
     */
    void Update_Imu_Pose_Relative_World(float Q0,
                                        float Q1,
                                        float Q2,
                                        float Q3);

    /**
     * @brief 总更新函数：同时更新IMU实测链和电机模型链
     * 
     * @param Q0
     * @param Q1
     * @param Q2
     * @param Q3
     * @param Yaw_Motor_Raw_Angle
     * @param Pitch_Motor_Raw_Angle
     * @param Now_Time
     * @param Delay_Time
     */
    void Update_Imu_Pose_Relative_BaseStart(float Q0,
                                            float Q1,
                                            float Q2,
                                            float Q3,
                                            float Yaw_Motor_Raw_Angle,
                                            float Pitch_Motor_Raw_Angle,
                                            uint32_t Now_Time,
                                            uint32_t Delay_Time = 100U);

    /* ========================= 多行函数：放 .cpp ========================= */


    /* ========================= 一行函数：直接放 .h ========================= */

        /**
     * @brief 设置Yaw电机方向修正系数
     * 
     * @param Direction +1或-1
     */
    void Set_Yaw_Motor_Direction(float Direction) { Yaw_Motor_Direction = Direction; }

    /**
     * @brief 设置Pitch电机方向修正系数
     * 
     * @param Direction +1或-1
     */
    void Set_Pitch_Motor_Direction(float Direction) { Pitch_Motor_Direction = Direction; }

    /**
     * @brief 获取启动时基座在世界系中的矩阵是否已初始化
     * 
     * @return uint8_t
     */
    uint8_t Get_BaseStart_World_Initialized(void) const { return BaseStart_World_Initialized; }

    /**
     * @brief 设置IMU相对于启动时基座目标Yaw连续角（相对当前角增量）
     * 
     * @param Delta_Yaw
     */
    void Set_Target_Imu_Relative_BaseStart_Continuous_Yaw(float Delta_Yaw)
    {
        Target_Imu_Relative_BaseStart_Continuous_Yaw = Imu_Relative_BaseStart_Yaw_State.Continuous_Angle + Delta_Yaw;
    }

    /**
     * @brief 设置IMU相对于启动时基座目标Pitch连续角（相对当前角增量）
     * 
     * @param Delta_Pitch
     */
    void Set_Target_Imu_Relative_BaseStart_Continuous_Pitch(float Delta_Pitch)
    {
        Target_Imu_Relative_BaseStart_Continuous_Pitch = Imu_Relative_BaseStart_Pitch_State.Continuous_Angle + Delta_Pitch;
    }

    /**
     * @brief 设置IMU相对于启动时基座目标Roll连续角（相对当前角增量）
     * 
     * @param Delta_Roll
     */
    void Set_Target_Imu_Relative_BaseStart_Continuous_Roll(float Delta_Roll)
    {
        Target_Imu_Relative_BaseStart_Continuous_Roll = Imu_Relative_BaseStart_Roll_State.Continuous_Angle + Delta_Roll;
    }

    /**
     * @brief 直接设置IMU相对于启动时基座目标Yaw连续角
     * 
     * @param Target_Yaw
     */
    void Set_Target_Imu_Relative_BaseStart_Continuous_Yaw_Absolute(float Target_Yaw)
    {
        Target_Imu_Relative_BaseStart_Continuous_Yaw = Target_Yaw;
    }

    /**
     * @brief 直接设置IMU相对于启动时基座目标Pitch连续角
     * 
     * @param Target_Pitch
     */
    void Set_Target_Imu_Relative_BaseStart_Continuous_Pitch_Absolute(float Target_Pitch)
    {
        Target_Imu_Relative_BaseStart_Continuous_Pitch = Target_Pitch;
    }

    /**
     * @brief 直接设置IMU相对于启动时基座目标Roll连续角
     * 
     * @param Target_Roll
     */
    void Set_Target_Imu_Relative_BaseStart_Continuous_Roll_Absolute(float Target_Roll)
    {
        Target_Imu_Relative_BaseStart_Continuous_Roll = Target_Roll;
    }

    /**
    * @brief 设置Pitch机械零位角
    * 
    * @param Angle
    */
    void Set_Pitch_Mechanical_Zero_Angle(float Angle) 
    { 
        Pitch_Mechanical_Zero_Angle = Angle; 
    }

    /**
    * @brief 获取Pitch机械零位角
    * 
    * @return float
    */
    float Get_Pitch_Mechanical_Zero_Angle(void) const 
    { 
        return Pitch_Mechanical_Zero_Angle; 
    }

    /**
     * @brief 获取IMU相对于启动时基座Yaw角（包角）
     * 
     * @return float
     */
    float Get_Imu_Relative_BaseStart_Yaw(void) const { return Imu_Relative_BaseStart_Yaw_State.Raw_Angle; }

    /**
     * @brief 获取IMU相对于启动时基座Pitch角（包角）
     * 
     * @return float
     */
    float Get_Imu_Relative_BaseStart_Pitch(void) const { return Imu_Relative_BaseStart_Pitch_State.Raw_Angle; }

    /**
     * @brief 获取IMU相对于启动时基座Roll角（包角）
     * 
     * @return float
     */
    float Get_Imu_Relative_BaseStart_Roll(void) const { return Imu_Relative_BaseStart_Roll_State.Raw_Angle; }

    /**
     * @brief 获取IMU相对于启动时基座Yaw连续角
     * 
     * @return float
     */
    float Get_Imu_Relative_BaseStart_Continuous_Yaw(void) const { return Imu_Relative_BaseStart_Yaw_State.Continuous_Angle; }

    /**
     * @brief 获取IMU相对于启动时基座Pitch连续角
     * 
     * @return float
     */
    float Get_Imu_Relative_BaseStart_Continuous_Pitch(void) const { return Imu_Relative_BaseStart_Pitch_State.Continuous_Angle; }

    /**
     * @brief 获取IMU相对于启动时基座Roll连续角
     * 
     * @return float
     */
    float Get_Imu_Relative_BaseStart_Continuous_Roll(void) const { return Imu_Relative_BaseStart_Roll_State.Continuous_Angle; }

    /**
     * @brief 获取模型参考Yaw角（包角）
     * 
     * @return float
     */
    float Get_Model_Relative_BaseStart_Yaw(void) const { return Model_Relative_BaseStart_Yaw; }

    /**
     * @brief 获取模型参考Pitch角（包角）
     * 
     * @return float
     */
    float Get_Model_Relative_BaseStart_Pitch(void) const { return Model_Relative_BaseStart_Pitch; }

    /**
     * @brief 获取模型参考Roll角（包角）
     * 
     * @return float
     */
    float Get_Model_Relative_BaseStart_Roll(void) const { return Model_Relative_BaseStart_Roll; }

    /**
     * @brief 获取模型参考Yaw连续角
     * 
     * @return float
     */
    float Get_Model_Relative_BaseStart_Continuous_Yaw(void) const { return Model_Relative_BaseStart_Continuous_Yaw; }

    /**
     * @brief 获取模型参考Pitch连续角
     * 
     * @return float
     */
    float Get_Model_Relative_BaseStart_Continuous_Pitch(void) const { return Model_Relative_BaseStart_Continuous_Pitch; }

    /**
     * @brief 获取模型参考Roll连续角
     * 
     * @return float
     */
    float Get_Model_Relative_BaseStart_Continuous_Roll(void) const { return Model_Relative_BaseStart_Continuous_Roll; }

    /**
     * @brief 获取目标Yaw连续角
     * 
     * @return float
     */
    float Get_Target_Imu_Relative_BaseStart_Continuous_Yaw(void) const
    {
        return Target_Imu_Relative_BaseStart_Continuous_Yaw;
    }

    /**
     * @brief 获取目标Pitch连续角
     * 
     * @return float
     */
    float Get_Target_Imu_Relative_BaseStart_Continuous_Pitch(void) const
    {
        return Target_Imu_Relative_BaseStart_Continuous_Pitch;
    }

    /**
     * @brief 获取目标Roll连续角
     * 
     * @return float
     */
    float Get_Target_Imu_Relative_BaseStart_Continuous_Roll(void) const
    {
        return Target_Imu_Relative_BaseStart_Continuous_Roll;
    }

    /**
     * @brief 获取IMU相对于世界系Yaw角（包角）
     * 
     * @return float
     */
    float Get_Imu_Relative_World_Yaw(void) const { return Imu_Relative_World_Yaw_State.Raw_Angle; }

    /**
     * @brief 获取IMU相对于世界系Pitch角（包角）
     * 
     * @return float
     */
    float Get_Imu_Relative_World_Pitch(void) const { return Imu_Relative_World_Pitch_State.Raw_Angle; }

    /**
     * @brief 获取IMU相对于世界系Roll角（包角）
     * 
     * @return float
     */
    float Get_Imu_Relative_World_Roll(void) const { return Imu_Relative_World_Roll_State.Raw_Angle; }

    /**
     * @brief 获取IMU相对于世界系Yaw连续角
     * 
     * @return float
     */
    float Get_Imu_Relative_World_Continuous_Yaw(void) const { return Imu_Relative_World_Yaw_State.Continuous_Angle; }

    /**
     * @brief 获取IMU相对于世界系Pitch连续角
     * 
     * @return float
     */
    float Get_Imu_Relative_World_Continuous_Pitch(void) const { return Imu_Relative_World_Pitch_State.Continuous_Angle; }

    /**
     * @brief 获取IMU相对于世界系Roll连续角
     * 
     * @return float
     */
    float Get_Imu_Relative_World_Continuous_Roll(void) const { return Imu_Relative_World_Roll_State.Continuous_Angle; }

    /**
     * @brief 获取IMU原始坐标系在世界系中的实测矩阵
     * 
     * @return const float (*)[3]
     */
    const float (*Get_R_World_From_ImuRaw_Measurement(void) const)[3] 
    { 
        return R_World_From_ImuRaw_Measurement; 
    }

    /**
     * @brief 获取IMU虚拟坐标系在世界系中的实测矩阵
     * 
     * @return const float (*)[3]
     */
    const float (*Get_R_World_From_ImuVirtual_Measurement(void) const)[3] 
    { 
        return R_World_From_ImuVirtual_Measurement; 
    }

    /**
     * @brief 获取启动时基座在世界系中的矩阵
     * 
     * @return const float (*)[3]
     */
    const float (*Get_R_World_From_BaseStart(void) const)[3] 
    { 
        return R_World_From_BaseStart; 
    }

    /**
     * @brief 获取IMU虚拟坐标系相对于启动时基座的真实矩阵
     * 
     * @return const float (*)[3]
     */
    const float (*Get_R_BaseStart_From_ImuVirtual_True(void) const)[3] 
    { 
        return R_BaseStart_From_ImuVirtual_True; 
    }

    /**
     * @brief 获取IMU原始坐标系相对于启动时基座的真实矩阵
     * 
     * @return const float (*)[3]
     */
    const float (*Get_R_BaseStart_From_ImuRaw_True(void) const)[3] 
    { 
        return R_BaseStart_From_ImuRaw_True; 
    }

    /**
     * @brief 获取IMU虚拟坐标系相对于启动时基座的模型参考矩阵
     * 
     * @return const float (*)[3]
     */
    const float (*Get_R_BaseStart_From_ImuVirtual_Model(void) const)[3] 
    { 
        return R_BaseStart_From_ImuVirtual_Model; 
    }

    /**
     * @brief 获取IMU原始坐标系相对于启动时基座的模型参考矩阵
     * 
     * @return const float (*)[3]
     */
    const float (*Get_R_BaseStart_From_ImuRaw_Model(void) const)[3] 
    { 
        return R_BaseStart_From_ImuRaw_Model; 
    }

    /**
    * @brief 获取IMU相对于当前时刻基座的模型Yaw角（包角）
    * 
    * @return float
    */
    float Get_Imu_Relative_BaseCurrent_Model_Yaw(void) const
    {
        return Imu_Relative_BaseCurrent_Model_Yaw;
    }

    /**
    * @brief 获取IMU相对于当前时刻基座的模型Pitch角（包角）
    * 
    * @return float
    */
    float Get_Imu_Relative_BaseCurrent_Model_Pitch(void) const
    {
        return Imu_Relative_BaseCurrent_Model_Pitch;
    }

    /**
    * @brief 获取IMU相对于当前时刻基座的模型Roll角（包角）
    * 
    * @return float
    */
    float Get_Imu_Relative_BaseCurrent_Model_Roll(void) const
    {
        return Imu_Relative_BaseCurrent_Model_Roll;
    }

    /**
    * @brief 获取IMU相对于当前时刻基座的模型Yaw连续角
    * 
    * @return float
    */
    float Get_Imu_Relative_BaseCurrent_Model_Continuous_Yaw(void) const
    {
        return Imu_Relative_BaseCurrent_Model_Continuous_Yaw;
    }

    /**
    * @brief 获取IMU相对于当前时刻基座的模型Pitch连续角
    * 
    * @return float
    */
    float Get_Imu_Relative_BaseCurrent_Model_Continuous_Pitch(void) const
    {
        return Imu_Relative_BaseCurrent_Model_Continuous_Pitch;
    }

    /**
    * @brief 获取IMU相对于当前时刻基座的模型Roll连续角
    * 
    * @return float
    */
    float Get_Imu_Relative_BaseCurrent_Model_Continuous_Roll(void) const
    {
        return Imu_Relative_BaseCurrent_Model_Continuous_Roll;
    }

    /**
    * @brief 获取IMU虚拟坐标系相对于当前时刻基座的模型矩阵
    * 
    * @return const float (*)[3]
    */
    const float (*Get_R_BaseCurrent_From_ImuVirtual_Model(void) const)[3]
    {
        return R_BaseCurrent_From_ImuVirtual_Model;
    }

    /**
    * @brief 获取IMU原始坐标系相对于当前时刻基座的模型矩阵
    * 
    * @return const float (*)[3]
    */
    const float (*Get_R_BaseCurrent_From_ImuRaw_Model(void) const)[3]
    {
        return R_BaseCurrent_From_ImuRaw_Model;
    }
};

#endif /* __cplusplus */

#endif /* __GIMBAL_H__ */
