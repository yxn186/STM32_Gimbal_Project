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

#define wheel_s   0.0815                          //轮半径
#define chassis_r 0.2125                          //底盘半径
#define chassis_motor_gear_ratio 15.764705882453f //减速比 268：17
#define chassis_motor_gear_ratio_inv 0.06346153846f //减速比倒数

typedef struct
{
  //Vx Vy ω —目标值
  float Target_Speed_X;
  float Target_Speed_Y;
  float Target_AngleSpeed_w;

  //电机角速度
  float Motor_Target_AngleSppeed[4];

  //电机牵引力
  float Motor_Target_Force_X;
  float Motor_Target_Force_Y;
  float Motor_Target_Force_T;

  //电机力矩
  float Motor_Target_Torque[4];

  float Motor_Current_Out[4];

  float MotorCurrent_Out_K_Torque_to_Current;//比例系数经验性

  float Motor_ID[4];
  float Motor_Speed[4];

  //Vx Vy ω —实际值
  float Motor_Speed_X;
  float Motor_Speed_Y;
  float Motor_AngleSpeed_w;

  PID_Object_t Motor_PID_X;
  PID_Object_t Motor_PID_Y;
  PID_Object_t Motor_PID_w;

} chassis_t;

void chassis_init(chassis_t *chassis)
{
  //设置ID
  chassis->Motor_ID[0] = 1;
  chassis->Motor_ID[1] = 2;
  chassis->Motor_ID[2] = 3;
  chassis->Motor_ID[3] = 4;

  chassis->Target_Speed_X = 0.0f;
  chassis->Target_Speed_Y = 0.0f;
  chassis->Target_AngleSpeed_w = 0.0f;

  for (int i = 0; i < 4; i++)
  {
    chassis->Motor_Target_AngleSppeed[i] = 0.0f;
    chassis->Motor_Target_Torque[i] = 0.0f;
    chassis->Motor_Current_Out[i] = 0.0f;
  }

  chassis->Motor_Target_Force_X = 0.0f;
  chassis->Motor_Target_Force_Y = 0.0f;
  chassis->Motor_Target_Force_T = 0.0f;

  chassis->MotorCurrent_Out_K_Torque_to_Current = 0.0f;//最重要设置这个！！
}

//从DR16获取X、Y轴目标速度
// //获取目标速度
  // chassis->Target_Speed_X = DR16->Get_Right_X();
  // chassis->Target_Speed_Y = DR16->Get_Right_Y();
  // chassis->Target_AngleSpeed_w = DR16->Get_Left_X();

/**
 * @brief 底盘电机角速度解算
 * 
 * @param chassis 
 */
void chassis_motor_anglespeed_calculate(chassis_t *chassis)
{
  float temp_data_x = (float)(chassis->Target_Speed_X * SQRT2_OVER_2) / (float)wheel_s;
  float temp_data_y = (float)(chassis->Target_Speed_Y * SQRT2_OVER_2) / (float)wheel_s;
  float temp_data_w = (float)(chassis->Target_AngleSpeed_w * chassis_r) / (float)wheel_s;

  //计算电机角速度
  chassis->Motor_Target_AngleSppeed[0] = -temp_data_x + temp_data_y + temp_data_w;
  chassis->Motor_Target_AngleSppeed[1] = -temp_data_x - temp_data_y + temp_data_w;
  chassis->Motor_Target_AngleSppeed[2] = temp_data_x - temp_data_y + temp_data_w;
  chassis->Motor_Target_AngleSppeed[3] = temp_data_x + temp_data_y + temp_data_w;
  //之后交给pid 输出力矩
}


void chassis_anglespeed_pid_to_force_calculate(chassis_t *chassis)
{
  

  for(uint8_t i = 0; i < 4; i++)
  {
    //获取当前速度
    chassis->Motor_Speed[i] = DJI_Motor_Get_AngleSpeed(chassis->Motor_ID[i]);

    

   // PID_Set_Speed_Target(&chassis->Motor_PID[i], chassis->Motor_Target_AngleSppeed[i]);
    //获取数据环节

    

    //chassis->Motor_PID[i].Out 
  }

  //运动学正解算！！
  //vx = (-ω0 - ω1 + ω2 + ω3) * √2 * s / 4
  //vy = (ω0 - ω1 - ω2 + ω3) * √2 * s / 4
  //ω = (ω0 + ω1 + ω2 + ω3) / r * s / 4
  chassis->Motor_Speed_X = (-chassis->Motor_Speed[0] - chassis->Motor_Speed[1] + chassis->Motor_Speed[2] + chassis->Motor_Speed[3]) * SQRT2_OVER_4 * wheel_s;
  chassis->Motor_Speed_Y = (chassis->Motor_Speed[0] - chassis->Motor_Speed[1] - chassis->Motor_Speed[2] + chassis->Motor_Speed[3]) * SQRT2_OVER_4 * wheel_s;
  chassis->Motor_AngleSpeed_w = (float)((chassis->Motor_Speed[0] + chassis->Motor_Speed[1] + chassis->Motor_Speed[2] + chassis->Motor_Speed[3]) * wheel_s * 0.25) / chassis_r;


  //数据push！ 待写！！！！！！！！！！！！！

    PID_Control_Speed(&chassis->Motor_PID_X);
    PID_Control_Speed(&chassis->Motor_PID_Y);
    PID_Control_Speed(&chassis->Motor_PID_w);

  //输出为力
  chassis->Motor_Target_Force_X = chassis->Motor_PID_X.Out;
  chassis->Motor_Target_Force_Y = chassis->Motor_PID_Y.Out;
  chassis->Motor_Target_Force_T = chassis->Motor_PID_w.Out;
}


  //之后交给力矩转电流的函数计算电流输出


/**
 * @brief 底盘电机力矩解算到电流输出值
 * 
 * @param chassis 
 */
void chassis_motor_torque_calculate_to_current_out(chassis_t *chassis)
{
  float temp_data_x = (float)(chassis->Motor_Target_Force_X * SQRT2_OVER_4 * wheel_s);
  float temp_data_y = (float)(chassis->Motor_Target_Force_Y * SQRT2_OVER_2 * wheel_s);
  float temp_data_w = (float)(chassis->Motor_Target_Force_T / chassis_r) * 0.25f * wheel_s;

  //计算电机力矩 等于轮边扭矩再除以减速比
  chassis->Motor_Target_Torque[0] = (float)(-temp_data_x + temp_data_y + temp_data_w) * chassis_motor_gear_ratio_inv;
  chassis->Motor_Target_Torque[1] = (float)(-temp_data_x - temp_data_y + temp_data_w) * chassis_motor_gear_ratio_inv;
  chassis->Motor_Target_Torque[2] = (float)(temp_data_x - temp_data_y + temp_data_w) * chassis_motor_gear_ratio_inv;
  chassis->Motor_Target_Torque[3] = (float)(temp_data_x + temp_data_y + temp_data_w) * chassis_motor_gear_ratio_inv;

  //计算电机输出电流 等于力矩乘以比例系数
  for (int i = 0; i < 4; i++)
  {
    chassis->Motor_Current_Out[i] = chassis->Motor_Target_Torque[i] * chassis->MotorCurrent_Out_K_Torque_to_Current;
  }
}