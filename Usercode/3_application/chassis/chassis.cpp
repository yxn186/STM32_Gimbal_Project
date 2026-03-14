/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    chassis.c
  * @brief   底盘库
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "chassis.h"
#include "dr16.h"
#include "MyMath.h"
#include "PID.h"
#include "DJI_Motor.h"
#include <cstdint>

/**
 * @brief 底盘类
 * 
 */
Class_Chassis Chassis;

/**
 * @brief 底盘参数初始化
 * 
 */
void Chassis_Init(void)
{
  //ID设置
  Chassis.Motor_ID[0] = 0;
  Chassis.Motor_ID[1] = 0;
  Chassis.Motor_ID[2] = 0;
  Chassis.Motor_ID[3] = 0;

  //电机PID参数
  for(uint8_t i = 0;i < 4;i++)
  {
    Chassis.PID_Motor[i].Kp_s = 0;
    Chassis.PID_Motor[i].Ki_s = 0;
    Chassis.PID_Motor[i].Kd_s = 0;

    Chassis.PID_Motor[i].ErrorInt_High_s = 0;
    Chassis.PID_Motor[i].ErrorInt_Low_s = 0;

    Chassis.PID_Motor[i].Speed_Target_High = 0;
    Chassis.PID_Motor[i].Speed_Target_Low = 0;

    Chassis.PID_Motor[i].Out_High = 0;
    Chassis.PID_Motor[i].Out_Low  = 0;
  }

  //PID参数 底盘

  //X
  Chassis.PID_X.Kp_s = 0;
  Chassis.PID_X.Ki_s = 0;
  Chassis.PID_X.Kd_s = 0;

  Chassis.PID_X.ErrorInt_High_s = 0;
  Chassis.PID_X.ErrorInt_Low_s = 0;

  Chassis.PID_X.Speed_Target_High = 0;
  Chassis.PID_X.Speed_Target_Low = 0;

  Chassis.PID_X.Out_High = 0;
  Chassis.PID_X.Out_Low  = 0;

  //Y
  Chassis.PID_Y.Kp_s = 0;
  Chassis.PID_Y.Ki_s = 0;
  Chassis.PID_Y.Kd_s = 0;

  Chassis.PID_Y.ErrorInt_High_s = 0;
  Chassis.PID_Y.ErrorInt_Low_s = 0;

  Chassis.PID_Y.Speed_Target_High = 0;
  Chassis.PID_Y.Speed_Target_Low = 0;

  Chassis.PID_Y.Out_High = 0;
  Chassis.PID_Y.Out_Low  = 0;

  //Z or W
  Chassis.PID_W.Kp_s = 0;
  Chassis.PID_W.Ki_s = 0;
  Chassis.PID_W.Kd_s = 0;

  Chassis.PID_W.ErrorInt_High_s = 0;
  Chassis.PID_W.ErrorInt_Low_s = 0;

  Chassis.PID_W.Speed_Target_High = 0;
  Chassis.PID_W.Speed_Target_Low = 0;

  Chassis.PID_W.Out_High = 0;
  Chassis.PID_W.Out_Low  = 0;

  //经验性比例系数
  Chassis.MotorCurrent_Out_K_Torque_to_Current = 0;
}


/**
 * @brief 底盘电机目标角速度计算
 * 
 */
void Class_Chassis::Motor_Target_AngleSpeed_Calculate(void)
{
  float temp_data_x = (float)(Chassis.Target_Speed_X * SQRT2_OVER_2) * wheel_s_inv;
  float temp_data_y = (float)(Chassis.Target_Speed_Y * SQRT2_OVER_2) * wheel_s_inv;
  float temp_data_w = (float)(Chassis.Target_AngleSpeed_w * r) * wheel_s_inv;

  //计算各电机目标角速度
  Chassis.PID_Motor[0].Set_Speed_Target(-temp_data_x + temp_data_y + temp_data_w);
  Chassis.PID_Motor[1].Set_Speed_Target(-temp_data_x - temp_data_y + temp_data_w);
  Chassis.PID_Motor[2].Set_Speed_Target(temp_data_x - temp_data_y + temp_data_w);
  Chassis.PID_Motor[3].Set_Speed_Target(temp_data_x + temp_data_y + temp_data_w);

  //之后交给pid 输出Out
}

/**
 * @brief 底盘PID 输入速度 输出Out
 * 
 */
void Class_Chassis::Speed_PID_To_Out_Calculate(void)
{
    for(uint8_t i = 0; i < 4; i++)
  {
    //获取当前速度 存入PID当前值
    //大疆系电机返回的速度到底是“电机转子速度”还是“减速后输出轴速度”？思考
    //思考
    //思考
    Chassis.PID_Motor[i].Set_Current_Speed(DJI_Motor_Get_AngleSpeed(Chassis.Motor_ID[i]));
  
    //PID
    Chassis.PID_Motor[i].Control_Speed_To_Out();

    Chassis.Motor_SpeedControl_Value[i] = Chassis.PID_Motor[i].Get_Out();
  }
}


/**
 * @brief 底盘PID 输入速度 输出力
 * 
 */
void Class_Chassis::Speed_PID_To_Force_Calculate(void)
{
    for(uint8_t i = 0; i < 4; i++)
  {
    //大疆系电机返回的速度到底是“电机转子速度”还是“减速后输出轴速度”？思考
    //思考
    //思考

    //获取当前速度 存入PID当前值
    //Chassis.PID_Motor[i].Set_Current_Speed(DJI_Motor_Get_AngleSpeed(Chassis.Motor_ID[i]));
    Chassis.Motor_Current_AngleSpeed[i] = DJI_Motor_Get_AngleSpeed(Chassis.Motor_ID[i]);
  }

  //运动学正解算！！
  //vx = (-ω0 - ω1 + ω2 + ω3) * √2 * s / 4
  //vy = (ω0 - ω1 - ω2 + ω3) * √2 * s / 4
  //ω = (ω0 + ω1 + ω2 + ω3) / r * s / 4

  float w0 =  Chassis.Motor_Current_AngleSpeed[0];
  float w1 =  Chassis.Motor_Current_AngleSpeed[1];
  float w2 =  Chassis.Motor_Current_AngleSpeed[2];
  float w3 =  Chassis.Motor_Current_AngleSpeed[3];

  //解算底盘实际速度XYZ
  Chassis.Current_Speed_X = (-w0 - w1 + w2 + w3)  * wheel_s * SQRT2_OVER_4;
  Chassis.Current_Speed_Y = (w0 - w1 - w2 + w3) * wheel_s * SQRT2_OVER_4;
  Chassis.Current_AngleSpeed_w = (w0 + w1 + w2 + w3) * wheel_s * r_inv * 0.25;

  Chassis.PID_X.Set_Speed_Target(Chassis.Target_Speed_X);
  Chassis.PID_Y.Set_Speed_Target(Chassis.Target_Speed_Y);
  Chassis.PID_W.Set_Speed_Target(Chassis.Target_AngleSpeed_w);

  Chassis.PID_X.Set_Current_Speed(Chassis.Current_Speed_X);
  Chassis.PID_Y.Set_Current_Speed(Chassis.Current_Speed_Y);
  Chassis.PID_W.Set_Current_Speed(Chassis.Current_AngleSpeed_w);
 

  Chassis.PID_X.Control_Speed_To_Out();
  Chassis.PID_Y.Control_Speed_To_Out();
  Chassis.PID_W.Control_Speed_To_Out();


  //输出为力
  Chassis.Target_Force_X = Chassis.PID_X.Get_Out();
  Chassis.Target_Force_Y = Chassis.PID_Y.Get_Out();
  Chassis.Target_Force_T = Chassis.PID_W.Get_Out();
}

/**
 * @brief 解算 输入力 输出电机实际输入参数
 * 
 */
void Class_Chassis::Motor_Torque_calculate_to_Motor_Control_Value(void)
{
  float temp_data_x = (float)(Chassis.Target_Force_X * SQRT2_OVER_4 * wheel_s);
  float temp_data_y = (float)(Chassis.Target_Force_Y * SQRT2_OVER_4 * wheel_s);
  float temp_data_w = (float)(Chassis.Target_Force_T * r_inv) * 0.25f * wheel_s;

  //计算电机力矩 等于轮边扭矩再除以减速比
  Chassis.Motor_Target_Torque[0] = (float)(-temp_data_x + temp_data_y + temp_data_w) * motor_gear_ratio_inv;
  Chassis.Motor_Target_Torque[1] = (float)(-temp_data_x - temp_data_y + temp_data_w) * motor_gear_ratio_inv;
  Chassis.Motor_Target_Torque[2] = (float)(temp_data_x - temp_data_y + temp_data_w) * motor_gear_ratio_inv;
  Chassis.Motor_Target_Torque[3] = (float)(temp_data_x + temp_data_y + temp_data_w) * motor_gear_ratio_inv;

  //计算电机输出电流 等于力矩乘以比例系数
  for (int i = 0; i < 4; i++)
  {
    Chassis.Motor_ForceControl_Value[i] = Chassis.Motor_Target_Torque[i] * Chassis.MotorCurrent_Out_K_Torque_to_Current;
  }
}

/**
 * @brief 简单速控底盘
 * 
 */
void Class_Chassis::Speed_Control(void)
{
  Motor_Target_AngleSpeed_Calculate();
  Speed_PID_To_Out_Calculate();
  
  for(uint8_t i = 0;i < 4;i++)
  {
    Chassis.Motor_Control_Value[i] =  Chassis.Motor_SpeedControl_Value[i];
  }
}

/**
 * @brief 底盘速度闭环直接输出四轮电流的控制方式 
 * 
 */
void Class_Chassis::Speed_To_Force_Control(void)
{
  Speed_PID_To_Force_Calculate();
  Motor_Torque_calculate_to_Motor_Control_Value();

  for(uint8_t i = 0;i < 4;i++)
  {
    Chassis.Motor_Control_Value[i] =  Chassis.Motor_ForceControl_Value[i];
  }
}

/**
 * @brief 轮速内环 + 底盘速度外环 + 力矩分配
 * 
 */
void Class_Chassis::Force_Control(void)
{
  Motor_Target_AngleSpeed_Calculate();
  Speed_PID_To_Out_Calculate();
  Speed_PID_To_Force_Calculate();
  Motor_Torque_calculate_to_Motor_Control_Value();

  for(uint8_t i = 0;i < 4;i++)
  {
    Chassis.Motor_Control_Value[i] =  Chassis.Motor_SpeedControl_Value[i] + Chassis.Motor_ForceControl_Value[i];
  }
}

/**
 * @brief 设置底盘目标速度 XYZ
 * 
 * @param X X方向速度
 * @param Y Y方向速度
 * @param Z 旋转角速度
 */
void Class_Chassis::Set_Target_Speed_XYZ(float X,float Y,float Z)
{
  Target_Speed_X = X;
  Target_Speed_Y = Y;
  Target_AngleSpeed_w = Z; 
}
