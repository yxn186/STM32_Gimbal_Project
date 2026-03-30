/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    PID.h
  * @brief   This file contains all the function prototypes for
  *          the PID.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*YOUR CODE*/

//cpp
typedef struct
{
    float Current;
    float Error0;
    float Error1;
    float ErrorInt;
} PID_States_t;

class Class_PID
{
    public:
    // 目标值
    float Speed_Target = 0.0f;
    float Angle_Target = 0.0f;

    // 最终输出
    float Out = 0.0f;

    // 速度环参数
    float Kp_s = 0.0f;
    float Ki_s = 0.0f;
    float Kd_s = 0.0f;
    float ErrorInt_High_s = 0.0f;
    float ErrorInt_Low_s = 0.0f;
    float Out_High = 0.0f;
    float Out_Low = 0.0f;

    // 角度环参数
    float Kp_a = 0.0f;
    float Ki_a = 0.0f;
    float Kd_a = 0.0f;
    float ErrorInt_High_a = 0.0f;
    float ErrorInt_Low_a = 0.0f;
    float Speed_Target_High = 0.0f;
    float Speed_Target_Low = 0.0f;

    // 状态
    PID_States_t Angle_States{};
    PID_States_t Speed_States{};

    public:

    /**
     * @brief 设置速度环目标值
     * @param target 速度目标值
     */
    void Set_Speed_Target(float target)
    {
        Speed_Target = target;
    }

    /**
     * @brief 设置角度环目标值
     * @param target 角度目标值
     */
    void Set_Angle_Target(float target)
    {
        Angle_Target = target;
    }

    /**
     * @brief 设置速度环当前值
     * @param current 速度当前值
     */
    void Set_Current_Speed(float current)
    {
        Speed_States.Current = current;
    }

    /**
     * @brief 设置角度环当前值
     * @param current 速度当前值
     */
    void Set_Current_Angle(float current)
    {
        Angle_States.Current = current;
    }

    /**
     * @brief 获取输出值
     * 
     * @return float 输出值
     */
    float Get_Out(void) const
    {
        return Out;
    }

    /**
     * @brief 获取速度目标值
     * 
     * @return float 速度目标值
     */
    float Get_Speed_Target(void) const
    {
        return Speed_Target;
    }

    /**
    * @brief PID控制 速度环
    * @details 使用类内部的速度目标、速度反馈、PID参数和状态量进行计算
    */
    void Control_Speed_To_Out(void);

    /**
    * @brief PID控制 角度环-->速度
    * @details 使用类内部的角度目标、角度反馈、PID参数和状态量进行计算，
    *          输出结果作为速度目标值 Speed_Target
    */
    void Control_Angle_To_Speed(void);

    /**
    * @brief PID控制 串级环
    * @details 先执行角度环得到速度目标，再执行速度环得到最终输出
    */
    void Control_Cascade(void);   

   /**
    * @brief PID重置 目前只有重置积分项
    * 
    */
    void Reset(void);

    /**
    * @brief 限幅函数
    * 
    * @param value 传入值
    * @param low 最低
    * @param high 最高
    * @return float 限幅后值
    */
    float Limit(float value, float low, float high);
};



typedef struct
{
    float Current_speed;
    float Current_Angle;
    float Error0;
    float Error1;
    float ErrorInt;
} PID_Status_t;

typedef struct
{
    float Speed_Target;
    float Kp_s;
    float Ki_s;
    float Kd_s;
    float Out;

    float ErrorInt_High_s;
    float ErrorInt_Low_s;

    float Out_High;
    float Out_Low;

    float Angle_Target;
    float Kp_a;
    float Ki_a;
    float Kd_a;

    float ErrorInt_High_a;
    float ErrorInt_Low_a;

    float Speed_Target_High;
    float Speed_Target_Low;

    PID_Status_t PID_Angle_Status;
    PID_Status_t PID_Speed_Status;
} PID_Object_t;

/**
 * @brief PID控制 速度环
 * 
 * @param PID_Object PID控制对象指针，包含配置参数和状态
 */
void PID_Control_Speed(PID_Object_t *PID_Object);

/**
 * @brief PID控制 角度环
 * 
 * @param PID_Object PID控制对象指针，包含配置参数和状态
 */
void PID_Control_Angle(PID_Object_t *PID_Object);

/**
 * @brief PID设置速度目标
 * 
 * @param PID_Object 
 * @param Speed_Target 
 */
void PID_Set_Speed_Target(PID_Object_t *PID_Object, float Speed_Target);

/**
 * @brief PID设置角度目标
 * 
 * @param PID_Object 
 * @param Angle_Target 
 */
void PID_Set_Angle_Target(PID_Object_t *PID_Object, float Angle_Target);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H__ */
