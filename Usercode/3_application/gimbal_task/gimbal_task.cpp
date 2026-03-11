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
uint8_t Pitch_PID_Times;
uint8_t Yaw_PID_Times;

PID_Object_t Pitch_Motor_PID = {0};
PID_Object_t Yaw_Motor_PID = {0};

uint8_t ID1 = 1;
uint8_t ID2 = 2;

DJI_Motor_Data_t DJI_Motors_Data[8] = {0};

gimtal_states_e gimtal_states = gimbal_states_aim_mode;

uint32_t target_set_time;
float target;

Class_PID PID_Gimbal_Motor_Yaw;
Class_PID PID_Gimbal_Motor_Pitch;
/*  Task层函数    ------------------------------------------------------------*/
//PID参数设置！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！

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

/*  Task层自定义回调函数类型 --------------------------------------------------*/



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
    //设置target
    if(gimtal_states == gimbal_states_aim_mode)//瞄准装甲板模式 接收视觉信息
    {
      gimbal_pid_reset();
      //Target获取 转成前向角
      //设置Target
    }
    else if(gimtal_states == gimbal_states_sentry_mode)//哨兵模式 自己乱转
    {
      target_set_time++;
      gimbal_pid_reset();
      Set_Yaw_and_Pitch_Motor_Target_Sentry();
    }

    float Yaw,Pitch;
    //得到前向角 pitch和yaw 传给PID控制对象（增加个速度？）
    app_bmi088_1ms_task_get_now_pitch_and_yaw(&Yaw,&Pitch);

    //赋值current
    //DJI_Motor_Get_AngleSpeed
    //DJI_Motor_Get_AngleSpeed

    //Set_Yaw_Current_Angle_and_Speed(Yaw,)
    //Set_Pitch_Current_Angle_and_Speed(Pitch,)

    //单环PID 先跑通
    PID_Gimbal_Motor_Yaw.Control_Speed_To_Out();
    PID_Gimbal_Motor_Pitch.Control_Speed_To_Out();\

    //双环PID控制 跑通后设置
    // PID_Gimbal_Motor_Yaw.Control_Cascade();
    // PID_Gimbal_Motor_Pitch.Control_Cascade();


    //CAN数据发送函数 这个要不要单拎？
    //DJI_Motor_Control_Double(&hcan1,DJI_Motor_6020,ID1,Pitch_Motor_PID.Out,ID2,Yaw_Motor_PID.Out);


    //CAN数据接收 处理函数 中断发信号 好像会在can库中自己做好

    osDelay(1);
  }
  /* USER CODE END main_Task_1ms */
}

/*  Task层函数 ----------------------------------------------------------------*/

void gimbal_task_init(void)
{
    JOLED_Init();
    Serial_Init(&huart1);
    USB_Init();
    //DJI_Motor_Init(&hcan1,DJI_Motors_Data);
    app_bmi088_init();
    Gimbal_Yaw_Motor_PID_Init();
    Gimbal_Pitch_Motor_PID_Init();
    gimbal_target_init();
}

uint8_t gimbal_task_init_loop(void)
{
  return app_bmi088_init_process_loop();
}

void gimbal_pid_reset(void)
{
  PID_Gimbal_Motor_Yaw.Reset();
  PID_Gimbal_Motor_Pitch.Reset();
}

/* 哨兵模式 Target设置相关 ---------------------------------------------------*/
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


