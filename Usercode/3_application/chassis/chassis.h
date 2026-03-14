/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    chassis.h
  * @brief   This file contains all the function prototypes for
  *          the chassis.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CHASSIS_H__
#define __CHASSIS_H__

#include <cstdint>
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "PID.h"
/*YOUR CODE*/

/**
 * @brief 底盘类
 * 
 */
class Class_Chassis
{
  public:
  //底盘数据
  float wheel_s = 0.0815;                       //轮半径
  float wheel_s_inv = 12.269938650f;            //轮半径倒数
  float r = 0.1125;                             //底盘半径
  float r_inv = 8.8888888888888f;                         //底盘半径倒数
  float motor_gear_ratio = 15.764705882453f;    //减速比
  float motor_gear_ratio_inv = 0.06346153846f;  //减速比倒数

  //存储四个电机ID
  uint8_t Motor_ID[4];

  //底盘目标值 Vx Vy ω （总数据）
  float Target_Speed_X = 0;
  float Target_Speed_Y = 0;
  float Target_AngleSpeed_w = 0;

  //底盘实际值 Vx Vy ω （总数据）
  float Current_Speed_X = 0;
  float Current_Speed_Y = 0;
  float Current_AngleSpeed_w = 0;

  //底盘牵引力（总牵引力）
  float Target_Force_X = 0;
  float Target_Force_Y = 0;
  float Target_Force_T = 0;

  //各轮实际角速度
  float Motor_Current_AngleSpeed[4];

  //电机力矩
  float Motor_Target_Torque[4];

  //电机控制值（电压/电流）
  float Motor_Control_Value[4];
  float Motor_SpeedControl_Value[4];
  float Motor_ForceControl_Value[4];

  //各电机数据
  //包括目标速度 当前速度 PID参数 输出Out
  Class_PID PID_Motor[4];
  //好像写错了？不需要四个PID类 而是一个底盘大类？
  //好像写错了？不需要四个PID类 而是一个底盘大类？

  Class_PID PID_X;
  Class_PID PID_Y;
  Class_PID PID_W;

  float MotorCurrent_Out_K_Torque_to_Current;//经验性比例系数

  /**
  * @brief 底盘电机目标角速度计算
  * @details 根据底盘各轴目标值进行解算到各电机PID目标值上
  */
  void Motor_Target_AngleSpeed_Calculate(void);

  /**
 * @brief 底盘PID 输入速度 输出力
 * 
 */
  void Speed_PID_To_Force_Calculate(void);

  /**
  * @brief 底盘PID 输入速度 输出Out
  * 
  */
  void Speed_PID_To_Out_Calculate(void);

  /**
  * @brief 解算 输入力 输出电机实际输入参数
  * 
  */
  void Motor_Torque_calculate_to_Motor_Control_Value(void);

  /**
  * @brief 速控底盘
  * 
  */
  void Speed_Control(void);

  /**
  * @brief 底盘速度闭环直接输出四轮电流的控制方式 
  * 
  */
  void Speed_To_Force_Control(void);

  /**
  * @brief 力控底盘
  * 
  */
  void Force_Control(void);

  /**
  * @brief 设置底盘目标速度 XYZ
  * 
  * @param X X方向速度
  * @param Y Y方向速度
  * @param Z 旋转角速度
  */
  void Set_Target_Speed_XYZ(float X,float Y,float Z);

  /**
  * @brief 底盘推送输出值给电机
  * 
  */
  void Push_Control_Value_To_Motor(void);

  /**
   * @brief 获取电机输出值
   * 
   * @param Num 0/1/2/3号电机
   * @return float 输出值
   */
  float Get_Motor_Control_Value(uint8_t Num)
  {
    if(Num < 0 || Num > 3)
    {
      return 0;
    }

   return Motor_Control_Value[Num];
  }
};





#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_H__ */
