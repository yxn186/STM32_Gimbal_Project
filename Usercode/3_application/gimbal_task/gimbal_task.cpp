/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gimbal_task.c
  * @brief   Task
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "gimbal_task.h"
#include "bsp_usb.h"
#include "joled.h"
#include "app_bmi088.h"
#include "Serial.h"
#include "usart.h"
#include <stdbool.h>
#include "usbd_cdc_if.h"
#include "bsp_usb.h"
#include "cmsis_os2.h"
#include "usb_device.h"
#include "PID.h"
#include "DJI_Motor.h"
#include "can.h"
#include "MyMath.h"
#include <math.h>

/*  Task层全局变量 ------------------------------------------------------------*/
bool init_finished = false;

/*  Task层数据    ------------------------------------------------------------*/

/**
 * @brief DJI_Mmotr类
 * 
 */
Class_DJI_Motor_Group Gimbal_DJI_Motor_Group;
Class_DJI_Motor DJI_Motor_Pitch;
Class_DJI_Motor DJI_Motor_Yaw;

/**
 * @brief PID类
 * 
 */
Class_PID PID_Gimbal_Motor_Yaw;
Class_PID PID_Gimbal_Motor_Pitch;

/**
 * @brief 云台状态枚举
 * 
 */
gimtal_states_e gimtal_states = gimbal_states_aim_mode;

/**
 * @brief 云台哨兵模式时间
 * 
 */
uint32_t target_set_time;

/**
 * @brief 云台是否需要切换模式（哨兵/自瞄）
 * 
 */
bool need_change_mode = false;

/**
 * @brief 自瞄目标参数
 * 
 */
float Aim_Pitch;
float Aim_Yaw;

/*  Task层初始化函数    ------------------------------------------------------*/

/**
 * @brief PID参数初始化 Yaw电机
 * 
 */
void Gimbal_Yaw_Motor_PID_Init(void)
{
  PID_Gimbal_Motor_Yaw.Kp_s = 0;
  PID_Gimbal_Motor_Yaw.Ki_s = 0;
  PID_Gimbal_Motor_Yaw.Kd_s = 0;
  PID_Gimbal_Motor_Yaw.Kp_a = 0;
  PID_Gimbal_Motor_Yaw.Ki_a = 0;
  PID_Gimbal_Motor_Yaw.Kd_a = 0;

  PID_Gimbal_Motor_Yaw.ErrorInt_High_s = 0;
  PID_Gimbal_Motor_Yaw.ErrorInt_Low_s  = 0;
  PID_Gimbal_Motor_Yaw.ErrorInt_High_a = 0;
  PID_Gimbal_Motor_Yaw.ErrorInt_Low_a  = 0;

  PID_Gimbal_Motor_Yaw.Speed_Target_High = 0;
  PID_Gimbal_Motor_Yaw.Speed_Target_Low = 0;

  PID_Gimbal_Motor_Yaw.Out_High = 0;
  PID_Gimbal_Motor_Yaw.Out_Low  = 0;
}

/**
 * @brief PID参数初始化 Pitch电机
 * 
 */
void Gimbal_Pitch_Motor_PID_Init(void)
{
  PID_Gimbal_Motor_Pitch.Kp_s = 0;
  PID_Gimbal_Motor_Pitch.Ki_s = 0;
  PID_Gimbal_Motor_Pitch.Kd_s = 0;
  PID_Gimbal_Motor_Pitch.Kp_a = 0;
  PID_Gimbal_Motor_Pitch.Ki_a = 0;
  PID_Gimbal_Motor_Pitch.Kd_a = 0;

  PID_Gimbal_Motor_Pitch.ErrorInt_High_s = 0;
  PID_Gimbal_Motor_Pitch.ErrorInt_Low_s  = 0;
  PID_Gimbal_Motor_Pitch.ErrorInt_High_a = 0;
  PID_Gimbal_Motor_Pitch.ErrorInt_Low_a  = 0;

  PID_Gimbal_Motor_Pitch.Speed_Target_High = 0;
  PID_Gimbal_Motor_Pitch.Speed_Target_Low = 0;

  PID_Gimbal_Motor_Pitch.Out_High = 0;
  PID_Gimbal_Motor_Pitch.Out_Low  = 0;
}

/**
 * @brief 云台大疆电机初始化
 * 
 */
void Gimbal_DJI_Motor_Init(void)
{
  Gimbal_DJI_Motor_Group.Init(&hcan1, DJI_Motor_6020);
  DJI_Motor_Pitch.Init(DJI_Motor_6020, 0, &Gimbal_DJI_Motor_Group);
  DJI_Motor_Yaw.Init(DJI_Motor_6020, 0, &Gimbal_DJI_Motor_Group);
}

/*  Task层自定义回调函数类型 --------------------------------------------------*/

void USB_CallBack(uint8_t *Buffer, uint16_t Length)
{
  if(Length == 0)
  {
    return;
  }
  
  //数据处理

  //通信约定是？

  //设置为一直通信？

  //不发信息代表视觉掉线
  //发信息就用开头告知是否检测到目标

  //加一个每次切换都触发need_change_mode!

  if(Buffer[0] == 'a')
  {
    gimtal_states = gimbal_states_aim_mode;
   // 把解算目标存入target
   //待加拿取数据相关
   
    //Aim_Pitch = ?
    //Aim_Yaw = ?
  }
  else
  {
    gimtal_states = gimbal_states_sentry_mode;
  }
  

}

/*  Task层FreeRTOS函数 任务函数 -----------------------------------------------*/

void StartInitTask(void *argument)
{
 /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartInitTask */
  gimbal_task_init();
  /* Infinite loop */
  for(;;)
  {
    if(gimbal_task_init_loop())
    {
      init_finished = true;
      osThreadTerminate(osThreadGetId());
    }
  }
  /* USER CODE END StartInitTask */
}

void Data_ptintf_task(void *argument)
{
  /* USER CODE BEGIN Data_ptintf_task */
  /* Infinite loop */
  for(;;)
  {
    app_bmi088_20ms_task();
    osDelay(20);
  }
  /* USER CODE END Data_ptintf_task */
}

void main_Task_1ms(void *argument)
{
  /* USER CODE BEGIN main_Task_1ms */
  /* Infinite loop */
  for(;;)
  {
    //模式判断
    if(need_change_mode)
    {
      gimbal_pid_reset();
      //待补充
    }
    //设置target
    if(gimtal_states == gimbal_states_aim_mode)//瞄准装甲板模式 接收视觉信息
    {
      //USB信息获取在回调中

      //获取到的信息存入PID目标（可能需要先做数据处理！！！！！！！！！）
      PID_Gimbal_Motor_Pitch.Set_Angle_Target(Aim_Pitch);
      PID_Gimbal_Motor_Yaw.Set_Angle_Target(Aim_Yaw);

    }
    else if(gimtal_states == gimbal_states_sentry_mode)//哨兵模式 自己乱转
    {
      target_set_time++;
      Set_Yaw_and_Pitch_Motor_Target_Sentry();
    }

    float Yaw,Pitch;
    //得到前向角 pitch和yaw 传给PID控制对象（增加个速度？）
    app_bmi088_1ms_task_get_now_pitch_and_yaw(&Yaw,&Pitch);

    //当前速度赋值 待确认赋值正确
    Gimbal_Push_Motor_Current_Speed_To_PID();

    //角度赋值
    Gimbal_Push_Gimbal_Pitch_and_Yaw_To_PID(Pitch,Yaw);

    //单环PID 先跑通
    PID_Gimbal_Motor_Yaw.Control_Speed_To_Out();
    PID_Gimbal_Motor_Pitch.Control_Speed_To_Out();

    //双环PID控制 跑通后设置
    // PID_Gimbal_Motor_Yaw.Control_Cascade();
    // PID_Gimbal_Motor_Pitch.Control_Cascade();

    //将PID输出值推送至电机输出值
    Gimbal_Push_PID_Out_To_Motor_Control();

    //将电机输出值进行CAN通信发送
    Gimbal_DJI_Motor_Group.Push_Data();

    osDelay(1);
  }
  /* USER CODE END main_Task_1ms */
}

/*  Task层函数 ----------------------------------------------------------------*/

/**
 * @brief 云台初始化
 * 
 */
void gimbal_task_init(void)
{
    //JOLED_Init();
    Serial_Init(&huart1);
    USB_Init(USB_CallBack);
    Gimbal_DJI_Motor_Init();
    app_bmi088_init();
    Gimbal_Yaw_Motor_PID_Init();
    Gimbal_Pitch_Motor_PID_Init();
    gimbal_target_init();
}

/**
 * @brief 云台初始化（循环）
 * 
 * @return uint8_t 
 */
uint8_t gimbal_task_init_loop(void)
{
  return app_bmi088_init_process_loop();
}

/**
 * @brief 云台PID重置信息
 * 
 */
void gimbal_pid_reset(void)
{
  PID_Gimbal_Motor_Yaw.Reset();
  PID_Gimbal_Motor_Pitch.Reset();
}

/* 哨兵模式 Target设置相关 ---------------------------------------------------*/

/**
 * @brief 云台目标初始化
 * 
 */
void gimbal_target_init(void)
{
  target_set_time = 0;
}

/**
 * @brief 哨兵模式速度目标变换设置函数
 * 
 * @return float 速度目标值
 */
float Speed_Target_Sentry(void)
{
  float f = 0.00025;//频率 = 周期倒数
  return 5.0f * sinf(2.0f * PI * f * target_set_time);//返回
}

/**
 * @brief 哨兵模式角度目标变换设置函数
 * 
 * @return float 角度目标值
 */
float Angle_Target_Sentry(void)
{
  float f = 0.00025;//频率 = 周期倒数
  return 60.0f * sinf(2.0f * PI * f * target_set_time);//返回-60到+60
}

/**
 * @brief 哨兵模式目标变换设置函数
 * 
 */
void Set_Yaw_and_Pitch_Motor_Target_Sentry()
{
  PID_Gimbal_Motor_Yaw.Set_Angle_Target(Angle_Target_Sentry());
  PID_Gimbal_Motor_Yaw.Set_Speed_Target(Speed_Target_Sentry());

  PID_Gimbal_Motor_Pitch.Set_Angle_Target(Angle_Target_Sentry());
  PID_Gimbal_Motor_Pitch.Set_Speed_Target(Speed_Target_Sentry());
}

/* PID 当前参数传入相关 ------------------------------------------------------*/

/**
 * @brief Yaw电机设置角度和速度当前值
 * 
 * @param Angle 
 * @param Speed 
 */
void Set_Yaw_Current_Angle_and_Speed(float Angle,float Speed)
{
  PID_Gimbal_Motor_Yaw.Set_Current_Angle(Angle);
  PID_Gimbal_Motor_Yaw.Set_Current_Speed(Speed);
}

/**
 * @brief Pitch电机设置角度和速度当前值
 * 
 * @param Angle 
 * @param Speed 
 */
void Set_Pitch_Current_Angle_and_Speed(float Angle,float Speed)
{
  PID_Gimbal_Motor_Pitch.Set_Current_Angle(Angle);
  PID_Gimbal_Motor_Pitch.Set_Current_Speed(Speed);
}

/**
 * @brief 云台推送电机数据给PID类
 * 
 */
void Gimbal_Push_Motor_Current_Speed_To_PID(void)
{
  PID_Gimbal_Motor_Yaw.Set_Current_Speed(DJI_Motor_Yaw.Get_AngleSpeed());
  PID_Gimbal_Motor_Pitch.Set_Current_Speed(DJI_Motor_Pitch.Get_AngleSpeed());
}

/**
 * @brief 云台推送云台前向轴yaw和pitch给PID类
 * 
 * @param Pitch 
 * @param Yaw 
 */
void Gimbal_Push_Gimbal_Pitch_and_Yaw_To_PID(float Pitch,float Yaw)
{
  PID_Gimbal_Motor_Pitch.Set_Current_Angle(Pitch);
  PID_Gimbal_Motor_Yaw.Set_Current_Angle(Yaw);
}

/**
 * @brief 云台推送PID输出值至电机控制值
 * 
 */
void Gimbal_Push_PID_Out_To_Motor_Control(void)
{
  DJI_Motor_Pitch.Set_Out(PID_Gimbal_Motor_Pitch.Get_Out());
  DJI_Motor_Yaw.Set_Out(PID_Gimbal_Motor_Yaw.Get_Out());
}