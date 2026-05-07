/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    Gimbal_Task.c
  * @brief   Task
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "Gimbal_Task.h"
#include "MahonyAHRS.h"
#include "app_bmi088.h"
#include "Serial.h"
#include "gimbal_task.h"
#include "usart.h"
#include <cstdint>
#include <stdbool.h>
#include "usbd_cdc_if.h"
#include "bsp_usb.h"
#include "cmsis_os2.h"
#include "usb_device.h"
#include "Vision.h"
#include "PID.h"
#include "DJI_Motor.h"
#include "can.h"
#include "MyMath.h"
#include <math.h>
#include "Gimbal.h"
#include "FeedForward.h"
#include "LowPassFilter.h"
#include "DWT.h"

/* 重力补偿采集模式开关 */
bool is_pitch_gravity_collect_mode = false;

/* Ozone观察用变量 */
float Gravity_Test_Target_Pitch = 0.0f;      // 当前给定目标角
float Gravity_Test_Current_Pitch = 0.0f;     // 当前实际pitch角
float Gravity_Test_Current_Speed = 0.0f;     // 当前pitch角速度
float Gravity_Test_Current_Torque = 0.0f;    // 当前pitch电机反馈电流/力矩电流
uint8_t Gravity_Test_Direction = 1;          // +1: 上扫  -1: 下扫
bool  Gravity_Test_Stable_Window = false;    // 1表示当前处于可采样稳定窗口
volatile uint32_t Main_Task_Period_Us = 0;   // 相邻两次任务入口间隔，含调度抖动
volatile uint32_t Main_Task_Exec_Us = 0;     // 单次循环执行耗时，不含osDelay后的阻塞时间
volatile uint32_t Main_Task_Period_Cycle = 0;
volatile uint32_t Main_Task_Exec_Cycle = 0;

/**
 * @brief 控制参数结构体
 * 
 */
typedef struct
{
  float speed_amplitude;                      //速度环--正弦目标振幅
  uint32_t speed_period_ms;                   //速度环--正弦目标周期               
  float angle_amplitude;                      //角度环--正弦目标振幅
  uint32_t angle_period_ms;                   //角度环--正弦目标周期
  float Yaw_f;                                //Yaw环--正弦目标频率
  float Yaw_a;                                //Yaw环--正弦目标振幅
  bool Yaw_Is_Trigonometric_Target_Mode;      //Yaw环--是否为正弦目标模式
  float Yaw_Step_Target;                      //Yaw环--步进目标
  float Pitch_f;                              //Pitch环--正弦目标频率
  float Pitch_a;                              //Pitch环--正弦目标振幅
  bool Pitch_Is_Trigonometric_Target_Mode;    //Pitch环--是否为正弦目标模式
  float Pitch_Step_Target;                    //Pitch环--阶跃目标
  bool Is_Go_To_Zero_To_Start_Sentry;         //是否回零以开始哨兵模式
}Control_Config_t;

Control_Config_t Control_Config_Data = {0.5f,3000,38.0f,
                                        2000,0.8f,50.0f,
                                        true,0.0f,0.4f,
                                        38.0f,true,
                                        0.0f,false};


/*  Task层全局变量 ------------------------------------------------------------*/
//全局初始化变量
bool Global_Init_Finished = false;
volatile bool Gimbal_Vision_Ready = false;
volatile bool Gimbal_Auto_Mode_Ready = false;

//自定义是否开启云台模式
//不开启则纯读BMI数据
bool Is_Gimbal_Mode = true;

bool is_gimbal_target_mode = true;

bool is_feedforward_mode = false;

bool is_lpf_mode = true;

bool is_g_feedback_mode = true;

float Gimbal_Normal_Kf_a_Yaw = 19.0f;
float Gimbal_Normal_Kf_a_Pitch = 15.0f;
float Gimbal_Aim_Kf_a_Yaw = 3.0f;
float Gimbal_Aim_Kf_a_Pitch = 3.0f;

// Yaw 限位配置保持包角语义，连续角目标需要先展开到当前圈数。
float Gimbal_Yaw_Target_High = 180.0f;
float Gimbal_Yaw_Target_Low = -180.0f;
float Gimbal_Pitch_Target_High = 40.0f;
float Gimbal_Pitch_Target_Low = -40.0f;

//任务时间
uint32_t Task_Time;

//哨兵模式目标过渡相关
static const uint32_t Sentry_Target_Transition_Time_Ms = 1400U;
static uint32_t Sentry_Target_Entry_Time = 0U;
static float Sentry_Target_Entry_Yaw = 0.0f;
static float Sentry_Target_Entry_Pitch = 0.0f;


//工具函数
static uint32_t DWT_Cycles_To_Us(uint32_t cycles)
{
  return (uint32_t)(((uint64_t)cycles * 1000000ULL) / SystemCoreClock);
}

static float Gimbal_Task_Get_Turn_Base(float continuous_angle)
{
  return continuous_angle - MyMath_Wrap_To_180(continuous_angle);
}

static float Gimbal_Task_Unwrap_Vision_Angle(float wrapped_target, float current_continuous_angle)
{
  float current_wrapped_angle = MyMath_Wrap_To_180(current_continuous_angle);
  float angle_delta = MyMath_Wrap_To_180(wrapped_target - current_wrapped_angle);

  return current_continuous_angle + angle_delta;
}
 


/*  Task层数据    ------------------------------------------------------------*/

/**
 * @brief 云台状态枚举
 * 
 */
gimtal_states_e gimtal_states = gimbal_states_aim_mode;
gimtal_states_e last_gimtal_states = gimbal_states_aim_mode;

/**
 * @brief 云台哨兵模式时间
 * 
 */
uint32_t target_set_time;

/**
 * @brief 云台是否需要切换模式（哨兵/自瞄）
 * 
 */
bool Need_Change_Mode = false;


/**
 * @brief 云台自瞄目标参数
 * 
 */
float Aim_Pitch;
float Aim_Yaw;

/**
 * @brief 云台低通滤波器输出
 * 
 */
float Gimbal_Yaw_LPF_Out;
float Gimbal_Pitch_LPF_Out;
/*  Task层类    --------------------------------------------------------------*/

/**
 * @brief 云台低通滤波器
 * 
 */
Class_LowPassFilter Gimbal_Yaw_LPF;
Class_LowPassFilter Gimbal_Pitch_LPF;

/**
 * @brief 云台前馈类
 * 
 */
Class_FeedForward Gimbal_FeedForward;

/**
 * @brief 云台类
 * 
 */
Class_Gimbal Gimbal;

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

//云台机械角参数
static const float Gimbal_Pitch_Mechanical_Zero = 59.0f;
static const float Gimbal_Yaw_Mechanical_Zero = 304.0f;
float Gimbal_Yaw_Mechanical_Relative_Angle = 0.0f;
float Gimbal_Pitch_Mechanical_Relative_Angle = 0.0f;


//云台初始化Yaw归位目标
static const float Gimbal_Yaw_Startup_Home_Target = 304.0f;

//归位完成条件
static const float Gimbal_Yaw_Startup_Home_Error_Threshold = 1.0f;
static const float Gimbal_Yaw_Startup_Home_Speed_Threshold = 0.10f;
static const float Gimbal_Yaw_Startup_Home_Speed_Limit = 20.0f;
static const float Gimbal_Yaw_Startup_Home_Imu_Target_Step = 20.0f;
static const float Gimbal_Yaw_Startup_Home_Imu_Direction = 1.0f;
static const float Gimbal_Yaw_Startup_Home_Zero_Check_Threshold = 0.05f;
static const uint32_t Gimbal_Yaw_Startup_Home_Stable_Count_Threshold = 200U;
static const uint32_t Gimbal_Startup_Post_Zero_Sync_Time_Ms = 300U;

//云台初始化归位相关静态变量
static bool Gimbal_Yaw_Startup_Home_Done = false;
static uint32_t Gimbal_Yaw_Startup_Home_Stable_Count = 0U;
static uint32_t Gimbal_Startup_Post_Zero_Sync_Start_Time = 0U;

/**
 * @brief 开始云台自动模式的条件检查和准备工作
 * 
 * @return true 
 * @return false 
 */
static bool Gimbal_Startup_Auto_Mode_Update(void)
{
  if(Gimbal_Auto_Mode_Ready)
  {
    return true;
  }

  //等待Yaw轴归位完成和视觉系统可以准备好
  if((Gimbal_Yaw_Startup_Home_Done == false) || (Gimbal_Vision_Ready == false))
  {
    return false;
  }

  //归位完成后，等待一段时间让系统稳定，确保视觉系统的目标数据可靠，然后再进入自动模式
  if((Task_Time - Gimbal_Startup_Post_Zero_Sync_Start_Time) < Gimbal_Startup_Post_Zero_Sync_Time_Ms)
  {
    return false;
  }

  //进入自动模式，置位Flag
  Gimbal_Auto_Mode_Ready = true;
  Need_Change_Mode = true;

  return true;
}

/**
 * @brief 云台Yaw轴归位更新 Pitch强制输出0
 * @details 回中目标是Yaw电机机械角 304度
 *
 * @return true 归位完成
 * @return false 归位未完成
 */
static bool Gimbal_Yaw_Startup_Home_Update(void)
{
  if(Gimbal_Yaw_Startup_Home_Done)
  {
    return true;
  }

  //等待全局初始化完成，确保IMU数据有效且已建立世界坐标系
  if(Global_Init_Finished == false)
  {
    return false;
  }

  //数据获取
  float yaw_motor_current = DJI_Motor_Yaw.Get_Angle();
  float yaw_imu_current = Gimbal.Get_Imu_Relative_World_Continuous_Yaw();
  float yaw_speed = DJI_Motor_Yaw.Get_AngleSpeed();

  //用电机当前包角 DJI_Motor_Yaw.Get_Angle() 算机械误差。
  float yaw_error = MyMath_Wrap_To_180(Gimbal_Yaw_Startup_Home_Target - yaw_motor_current);

  //把这个误差限幅到 ±20°，形成 IMU yaw 目标增量 但不过大
  float yaw_imu_target_delta =PID_Gimbal_Motor_Yaw.Limit(yaw_error * Gimbal_Yaw_Startup_Home_Imu_Direction,
                                                         -Gimbal_Yaw_Startup_Home_Imu_Target_Step,
                                                         Gimbal_Yaw_Startup_Home_Imu_Target_Step);

  float yaw_imu_target = yaw_imu_current + yaw_imu_target_delta;

  PID_Gimbal_Motor_Yaw.Set_Angle_Target(yaw_imu_target);
  PID_Gimbal_Motor_Yaw.Angle_Target_Last = yaw_imu_target;
  PID_Gimbal_Motor_Yaw.Set_Current_Angle(yaw_imu_current);
  PID_Gimbal_Motor_Yaw.Set_Current_Speed(yaw_speed);

  PID_Gimbal_Motor_Yaw.Control_Angle_To_Speed();

  //限幅速度输出，推送给电机
  PID_Gimbal_Motor_Yaw.Set_Speed_Target(PID_Gimbal_Motor_Yaw.Limit(PID_Gimbal_Motor_Yaw.Get_Speed_Target(),
                                        -Gimbal_Yaw_Startup_Home_Speed_Limit,
                                        Gimbal_Yaw_Startup_Home_Speed_Limit));

  PID_Gimbal_Motor_Yaw.Control_Speed_To_Out();

  DJI_Motor_Yaw.Set_Out(PID_Gimbal_Motor_Yaw.Get_Out());
  DJI_Motor_Pitch.Set_Out(0);

  Gimbal_DJI_Motor_Group.Push_Data();

  //判断是否完成回中以及满足回中条件
  //误差小于1度，速度小于0.1度/s，并且持续稳定一定时间（200帧），则认为回中完成
  if((fabsf(yaw_error) < Gimbal_Yaw_Startup_Home_Error_Threshold) && (fabsf(yaw_speed) < Gimbal_Yaw_Startup_Home_Speed_Threshold))
  {
    Gimbal_Yaw_Startup_Home_Stable_Count++;
  }
  else
  {
    Gimbal_Yaw_Startup_Home_Stable_Count = 0U;
  }

  //如果稳定计数达到阈值，认为回中完成，进行后续处理
  if(Gimbal_Yaw_Startup_Home_Stable_Count >= Gimbal_Yaw_Startup_Home_Stable_Count_Threshold)
  {
    //回中完成，建立IMU相对于启动时基座的坐标系，并设置模型参考角度
    Gimbal_Yaw_Startup_Home_Done = true;

    //reset
    Gimbal_PID_Reset();
    Gimbal_Yaw_LPF.Reset(0.0f);
    Gimbal_Pitch_LPF.Reset(0.0f);

    //设置IMU相对于世界系的Yaw零点，建立IMU相对于启动时基座的坐标系
    Gimbal.Set_Imu_Relative_World_Yaw_Zero();

    //判断IMU相对于世界系的Yaw是否已经非常接近零，如果不接近零，说明世界坐标系的建立可能存在较大误差，暂不进入自动模式，等待视觉系统准备好后再进入自动模式
    //检查清零后 yaw 是否小于 0.05°
    if(fabsf(Gimbal.Get_Imu_Relative_World_Continuous_Yaw()) > Gimbal_Yaw_Startup_Home_Zero_Check_Threshold)
    {
      Gimbal_Yaw_Startup_Home_Done = false;
      Gimbal_Yaw_Startup_Home_Stable_Count = 0U;
      return false;
    }

    //回中完成，置位Flag
    Gimbal_Vision_Ready = true;
    Gimbal_Auto_Mode_Ready = false;
    Gimbal_Startup_Post_Zero_Sync_Start_Time = Task_Time;

    //把当前 yaw/pitch 写成 PID 目标和 Angle_Target_Last
    PID_Gimbal_Motor_Yaw.Set_Angle_Target(Gimbal.Get_Imu_Relative_World_Continuous_Yaw());
    PID_Gimbal_Motor_Yaw.Angle_Target_Last = Gimbal.Get_Imu_Relative_World_Continuous_Yaw();
    PID_Gimbal_Motor_Pitch.Set_Angle_Target(Gimbal.Get_Imu_Relative_World_Continuous_Pitch());
    PID_Gimbal_Motor_Pitch.Angle_Target_Last = Gimbal.Get_Imu_Relative_World_Continuous_Pitch();
    Gimbal_Target_Init();

    return true;
  }

  return false;
}

/**
 * @brief 根据模式更新云台前馈系数Kf_a
 * 
 */
static void Gimbal_Update_Kf_a_By_Mode(void)
{
  float Yaw_Target_Kf_a = Gimbal_Normal_Kf_a_Yaw;
  float Pitch_Target_Kf_a = Gimbal_Normal_Kf_a_Pitch;
  bool is_sentry_smooth_transition = (gimtal_states == gimbal_states_sentry_mode) && (Need_Change_Mode || Sentry_Target_Is_In_Transition());

  if((is_pitch_gravity_collect_mode == false) && ((gimtal_states == gimbal_states_aim_mode) || is_sentry_smooth_transition))
  {
    Yaw_Target_Kf_a = Gimbal_Aim_Kf_a_Yaw;
    Pitch_Target_Kf_a = Gimbal_Aim_Kf_a_Pitch;
  }

  PID_Gimbal_Motor_Yaw.Kf_a = Yaw_Target_Kf_a;
  PID_Gimbal_Motor_Pitch.Kf_a = Pitch_Target_Kf_a;
}

/*  Task层初始化函数    ------------------------------------------------------*/

//PID参数在这里~~~
/**
 * @brief PID参数初始化 Yaw电机
 * 
 */
void Gimbal_Yaw_Motor_PID_Init(void)
{
  if(is_feedforward_mode)
  {
    PID_Gimbal_Motor_Yaw.Kp_s = 600;
    PID_Gimbal_Motor_Yaw.Ki_s = 30;
    PID_Gimbal_Motor_Yaw.Kd_s = 0;
    PID_Gimbal_Motor_Yaw.Kp_a = 0.18;
    PID_Gimbal_Motor_Yaw.Ki_a = 0;
    PID_Gimbal_Motor_Yaw.Kd_a = 0;

    PID_Gimbal_Motor_Yaw.ErrorInt_High_s = 45;
    PID_Gimbal_Motor_Yaw.ErrorInt_Low_s  = -45;
    PID_Gimbal_Motor_Yaw.ErrorInt_High_a = 30;
    PID_Gimbal_Motor_Yaw.ErrorInt_Low_a  = -30;

    PID_Gimbal_Motor_Yaw.Speed_Target_High = 20;
    PID_Gimbal_Motor_Yaw.Speed_Target_Low = -20;

    PID_Gimbal_Motor_Yaw.Out_High = 6000;
    PID_Gimbal_Motor_Yaw.Out_Low  = -6000;
  }
  else
  {
    PID_Gimbal_Motor_Yaw.Kp_s = 1300;
    PID_Gimbal_Motor_Yaw.Ki_s = 70;
    PID_Gimbal_Motor_Yaw.Kd_s = 0;
    PID_Gimbal_Motor_Yaw.Kp_a = 0.35;
    PID_Gimbal_Motor_Yaw.Ki_a = 0.0001;
    PID_Gimbal_Motor_Yaw.Kd_a = 0;

    PID_Gimbal_Motor_Yaw.ErrorInt_High_s = 45;
    PID_Gimbal_Motor_Yaw.ErrorInt_Low_s  = -45;
    PID_Gimbal_Motor_Yaw.ErrorInt_High_a = 1800;
    PID_Gimbal_Motor_Yaw.ErrorInt_Low_a  = -1800;

    PID_Gimbal_Motor_Yaw.Integral_Stop_Near_Zero_Enable_a = 1;
    PID_Gimbal_Motor_Yaw.Integral_Stop_Target_Abs_Threshold_a = 2.0f;
    PID_Gimbal_Motor_Yaw.Integral_Stop_Error_Abs_Threshold_a = 2.0f;

    PID_Gimbal_Motor_Yaw.Kf_a = Gimbal_Normal_Kf_a_Yaw;
    PID_Gimbal_Motor_Yaw.FeedForward_Enable_a = 1;
    PID_Gimbal_Motor_Yaw.FeedForward_High_a = 20;
    PID_Gimbal_Motor_Yaw.FeedForward_Low_a = -20;

    PID_Gimbal_Motor_Yaw.Speed_Target_High = 20;
    PID_Gimbal_Motor_Yaw.Speed_Target_Low = -20;

    PID_Gimbal_Motor_Yaw.Out_High = 10000;
    PID_Gimbal_Motor_Yaw.Out_Low  = -10000;
  }
}

/**
 * @brief PID参数初始化 Pitch电机
 * 
 */
void Gimbal_Pitch_Motor_PID_Init(void)
{
  PID_Gimbal_Motor_Pitch.Kp_s = 900;
  PID_Gimbal_Motor_Pitch.Ki_s = 24;
  PID_Gimbal_Motor_Pitch.Kd_s = 0;
  PID_Gimbal_Motor_Pitch.Kp_a = 0.6;
  PID_Gimbal_Motor_Pitch.Ki_a = 0.00005;
  PID_Gimbal_Motor_Pitch.Kd_a = 0;

  PID_Gimbal_Motor_Pitch.ErrorInt_High_s = 200;
  PID_Gimbal_Motor_Pitch.ErrorInt_Low_s  = -200;
  PID_Gimbal_Motor_Pitch.ErrorInt_High_a = 6000;
  PID_Gimbal_Motor_Pitch.ErrorInt_Low_a  = -6000;

  PID_Gimbal_Motor_Pitch.Integral_Stop_Near_Zero_Enable_a = 1;
  PID_Gimbal_Motor_Pitch.Integral_Stop_Target_Abs_Threshold_a = 3.0f;
  PID_Gimbal_Motor_Pitch.Integral_Stop_Error_Abs_Threshold_a = 3.0f;

  PID_Gimbal_Motor_Pitch.Kf_a = Gimbal_Normal_Kf_a_Pitch;
  PID_Gimbal_Motor_Pitch.FeedForward_Enable_a = 1;
  PID_Gimbal_Motor_Pitch.FeedForward_High_a = 20;
  PID_Gimbal_Motor_Pitch.FeedForward_Low_a = -20;

  PID_Gimbal_Motor_Pitch.Speed_Target_High = 10;
  PID_Gimbal_Motor_Pitch.Speed_Target_Low = -10;

  PID_Gimbal_Motor_Pitch.Out_High = 8000;
  PID_Gimbal_Motor_Pitch.Out_Low  = -8000;
}

/**
 * @brief 云台大疆电机初始化
 * 
 */
void Gimbal_DJI_Motor_Init(void)
{
  Gimbal_DJI_Motor_Group.Init(&hcan2, DJI_Motor_6020);
  DJI_Motor_Pitch.Init(DJI_Motor_6020, 4, &Gimbal_DJI_Motor_Group);
  DJI_Motor_Yaw.Init(DJI_Motor_6020, 2, &Gimbal_DJI_Motor_Group);
}

/*  Task层FreeRTOS函数 任务函数 -----------------------------------------------*/

extern "C" void StartInitTask(void *argument)
{
 /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartInitTask */

  //云台相关全局初始化
  Gimbal_Task_Global_Init();

  /* Infinite loop */
  for(;;)
  {
    ////云台相关全局初始化循环
    if(Gimbal_Task_Global_Init_Loop())
    {
      Global_Init_Finished = true;
      osThreadTerminate(osThreadGetId());
    }
  }
  /* USER CODE END StartInitTask */
}

/**
 * @brief 数据打印任务
 * 
 * @param argument 
 */
extern "C" void Data_ptintf_task(void *argument)
{
  /* USER CODE BEGIN Data_ptintf_task */
  /* Infinite loop */
  for(;;)
  {
    if(Gimbal_Vision_Ready)
    {
      Vision.USB_Transmit_Angle(Gimbal.Get_Imu_Relative_World_Continuous_Yaw(),Gimbal.Get_Imu_Relative_World_Continuous_Pitch());
    }
    osDelay(5);
  }
  /* USER CODE END Data_ptintf_task */
}

/**
 * @brief 时间计数任务
 * 
 */
extern "C" void TimeCountTask(void *argument)
{
  /* USER CODE BEGIN TimeCountTask */
  /* Infinite loop */
  for(;;)
  {
    if(Global_Init_Finished)
    {
      Task_Time++;
    }
    
    osDelay(1);
  }
  /* USER CODE END TimeCountTask */
}

/**
 * @brief 云台主控制任务 1ms
 * 
 * @param argument 
 */
extern "C" void main_Task_1ms(void *argument)
{
  /* USER CODE BEGIN main_Task_1ms */
  /* Infinite loop */
  static uint32_t last_loop_start_cyccnt = 0;

  const uint32_t period_ticks = 1U;              // 1 tick，前提是你的RTOS tick就是1ms
  uint32_t next_tick = osKernelGetTickCount();   // 记录下一次唤醒基准
  for(;;)
  {
    uint32_t loop_start_cyccnt = DWT_GetCYCCNT();

    if(last_loop_start_cyccnt != 0U)
    {
      Main_Task_Period_Cycle = loop_start_cyccnt - last_loop_start_cyccnt;
      Main_Task_Period_Us = DWT_Cycles_To_Us(Main_Task_Period_Cycle);
    }

    last_loop_start_cyccnt = loop_start_cyccnt;


    Gimbal.Update_Imu_Pose_Relative_World(q0,q1,q2,q3);

    //云台Yaw轴归位和自动模式准备更新
    bool Gimbal_Ready_To_Run = true;
    bool Gimbal_Auto_Mode_Ready = false;
    if(Is_Gimbal_Mode)
    {
      //云台归位
      Gimbal_Ready_To_Run = Gimbal_Yaw_Startup_Home_Update();
      if(Gimbal_Ready_To_Run)
      {
        //云台自动模式准备
        Gimbal_Auto_Mode_Ready = Gimbal_Startup_Auto_Mode_Update();
      }
    }

    //等待一切就绪后才开始视觉相关检测与状态切换
    if(Gimbal_Ready_To_Run && Gimbal_Auto_Mode_Ready)
    {
      //等待允许运行才判断视觉相关的模式和更新模式切换前馈系数
      Gimbal_Vision_Mode_Judge_1ms();
      Gimbal_Update_Kf_a_By_Mode();
    }

    //真正的云台控制逻辑 先判断是否可以开始云台控制链路
    if(Is_Gimbal_Mode && Gimbal_Ready_To_Run)
    {
      //模式切换标志位，主要用于哨兵模式和自瞄模式切换时的处理
      bool Is_Mode_Changed_This_Loop = false;

      //模式切换判断
      if(Need_Change_Mode)
      {
        Gimbal_PID_Reset();
        Gimbal_Yaw_LPF.Reset( PID_Gimbal_Motor_Yaw.Get_Speed_Target());
        Gimbal_Pitch_LPF.Reset(PID_Gimbal_Motor_Pitch.Get_Speed_Target());

        //如果是切换到哨兵模式，记录进入哨兵模式的时间和角度，用于后续实现平滑过渡
        if(gimtal_states == gimbal_states_sentry_mode)
        {
          Gimbal_Target_Init();
        }

        Need_Change_Mode = false;
        Is_Mode_Changed_This_Loop = true;
      }

      //判断模式 执行对应控制链路
      if(Gimbal_Auto_Mode_Ready == false)
      {
        float current_pitch = Gimbal.Get_Imu_Relative_World_Continuous_Pitch();
        float current_yaw = Gimbal.Get_Imu_Relative_World_Continuous_Yaw();

        PID_Gimbal_Motor_Pitch.Set_Angle_Target(current_pitch);
        PID_Gimbal_Motor_Pitch.Angle_Target_Last = current_pitch;
        PID_Gimbal_Motor_Yaw.Set_Angle_Target(current_yaw);
        PID_Gimbal_Motor_Yaw.Angle_Target_Last = current_yaw;
      }
      else if(is_pitch_gravity_collect_mode)//重力补偿采集模式 只设置pitch target 让云台上下扫
      {
        Set_Pitch_Motor_Target_Gravity_Collect();
      }
      //判断当前模式 执行对应逻辑
      else if(gimtal_states == gimbal_states_aim_mode)//自瞄模式--瞄准装甲板模式 接收视觉信息
      {
        //待优化
        float Current_Continuous_Pitch = Gimbal.Get_Imu_Relative_World_Continuous_Pitch();
        float Current_Continuous_Yaw = Gimbal.Get_Imu_Relative_World_Continuous_Yaw();

        float Pitch_Target = Gimbal_Task_Unwrap_Vision_Angle(Vision.Get_Pitch(), Current_Continuous_Pitch);
        float Yaw_Target = Gimbal_Task_Unwrap_Vision_Angle(Vision.Get_Yaw(), Current_Continuous_Yaw);

        float yaw_turn_base = Gimbal_Task_Get_Turn_Base(Current_Continuous_Yaw);
        float yaw_target_low = yaw_turn_base + Gimbal_Yaw_Target_Low;
        float yaw_target_high = yaw_turn_base + Gimbal_Yaw_Target_High;

        //获取到的信息存入PID目标
        PID_Gimbal_Motor_Pitch.Set_Angle_Target(PID_Gimbal_Motor_Pitch.Limit(Pitch_Target, Gimbal_Pitch_Target_Low, Gimbal_Pitch_Target_High));
        PID_Gimbal_Motor_Yaw.Set_Angle_Target(PID_Gimbal_Motor_Yaw.Limit(Yaw_Target, yaw_target_low, yaw_target_high));
        
      }
      else if(gimtal_states == gimbal_states_sentry_mode)//哨兵模式--自己乱转
      {
        target_set_time++;
        if(is_gimbal_target_mode)
        {
          Set_Yaw_and_Pitch_Motor_Target_Sentry();//角度
        }
        else
        {
          Set_Yaw_and_Pitch_Motor_Speed_Target_Sentry();//速度
        }
      }
      
      //当前这 1ms 循环刚刚处理过模式切换 在本轮 PID 计算前同步 Angle_Target_Last 避免角度前馈突然打一脚。
      if(Is_Mode_Changed_This_Loop)
      {
        PID_Gimbal_Motor_Yaw.Angle_Target_Last = PID_Gimbal_Motor_Yaw.Angle_Target;
        PID_Gimbal_Motor_Pitch.Angle_Target_Last = PID_Gimbal_Motor_Pitch.Angle_Target;
      }

      //上面根据不同模式设置完Target 开始进行PID
      
      //当前速度赋值 获取电机当前速度作为PID的当前速度输入
      PID_Gimbal_Motor_Yaw.Set_Current_Speed(DJI_Motor_Yaw.Get_AngleSpeed());
      PID_Gimbal_Motor_Pitch.Set_Current_Speed(DJI_Motor_Pitch.Get_AngleSpeed());

      //当前角度赋值 获取IMU相对于世界系的连续角度作为PID的当前角度输入
      PID_Gimbal_Motor_Pitch.Set_Current_Angle(Gimbal.Get_Imu_Relative_World_Continuous_Pitch());
      PID_Gimbal_Motor_Yaw.Set_Current_Angle(Gimbal.Get_Imu_Relative_World_Continuous_Yaw());

      //PID计算
      if(!is_lpf_mode)
      {
        //单环PID 先跑通
        //PID_Gimbal_Motor_Yaw.Control_Speed_To_Out();
        //PID_Gimbal_Motor_Pitch.Control_Speed_To_Out();

        //双环PID控制 跑通后设置
        PID_Gimbal_Motor_Yaw.Control_Cascade();
        PID_Gimbal_Motor_Pitch.Control_Cascade();
      }
      else
      {
        PID_Gimbal_Motor_Yaw.Control_Angle_To_Speed();
        PID_Gimbal_Motor_Pitch.Control_Angle_To_Speed();
        
        Gimbal_Yaw_LPF_Out = Gimbal_Yaw_LPF.Update(PID_Gimbal_Motor_Yaw.Get_Speed_Target());
        PID_Gimbal_Motor_Yaw.Set_Speed_Target(Gimbal_Yaw_LPF_Out);

        Gimbal_Pitch_LPF_Out = Gimbal_Pitch_LPF.Update(PID_Gimbal_Motor_Pitch.Get_Speed_Target());
        PID_Gimbal_Motor_Pitch.Set_Speed_Target(Gimbal_Pitch_LPF_Out);

        PID_Gimbal_Motor_Yaw.Control_Speed_To_Out();
        PID_Gimbal_Motor_Pitch.Control_Speed_To_Out();
      }
      
      //机械限位
      Gimbal_Yaw_Mechanical_Relative_Angle = MyMath_Wrap_To_180(DJI_Motor_Yaw.Get_Angle() - Gimbal_Yaw_Mechanical_Zero);
      Gimbal_Pitch_Mechanical_Relative_Angle = MyMath_Wrap_To_180(DJI_Motor_Pitch.Get_Angle() - Gimbal_Pitch_Mechanical_Zero);

      if(Gimbal_Yaw_Mechanical_Relative_Angle <= Gimbal_Yaw_Target_Low)
      {
        if(PID_Gimbal_Motor_Yaw.Get_Out() < 0)
        {
          PID_Gimbal_Motor_Yaw.Set_Out(0);
        }
      }
      else if(Gimbal_Yaw_Mechanical_Relative_Angle >= Gimbal_Yaw_Target_High)
      {
        if(PID_Gimbal_Motor_Yaw.Get_Out() > 0)
        {
          PID_Gimbal_Motor_Yaw.Set_Out(0);
        }
      }

      //  if(Gimbal_Pitch_Mechanical_Relative_Angle <= Gimbal_Pitch_Target_Low)
      //   {
      //     if(PID_Gimbal_Motor_Pitch.Get_Out() > 0)
      //     {
      //       PID_Gimbal_Motor_Pitch.Set_Out(0);
      //     }
      //   }
      //   else if(Gimbal_Pitch_Mechanical_Relative_Angle >= Gimbal_Pitch_Target_High)
      //   {
      //     if(PID_Gimbal_Motor_Pitch.Get_Out() < 0)
      //     {
      //       PID_Gimbal_Motor_Pitch.Set_Out(0);
      //     }
      //   }

      //将PID输出值（Out值）推送至电机输出值
      Gimbal_Push_PID_Out_To_Motor_Control();
      Gimbal_DJI_Motor_Group.Push_Data();

      //重力补偿采集模式下记录数据
      Gravity_Test_Current_Pitch = Gimbal.Get_Imu_Relative_World_Continuous_Pitch();
      Gravity_Test_Current_Speed = DJI_Motor_Pitch.Get_AngleSpeed();
      Gravity_Test_Current_Torque = DJI_Motor_Pitch.Get_Torque_Current();
    }

    uint32_t loop_end_cyccnt = DWT_GetCYCCNT();
    Main_Task_Exec_Cycle = loop_end_cyccnt - loop_start_cyccnt;
    Main_Task_Exec_Us = DWT_Cycles_To_Us(Main_Task_Exec_Cycle);

    next_tick += period_ticks;
    osDelayUntil(next_tick);
  }
  /* USER CODE END main_Task_1ms */
}

/*  Task层函数 ----------------------------------------------------------------*/

/**
 * @brief 云台初始化
 * 
 */
void Gimbal_Task_Global_Init(void)
{
  //回中相关初始化
  Gimbal_Vision_Ready = false;
  Gimbal_Auto_Mode_Ready = false;
  Gimbal_Startup_Post_Zero_Sync_Start_Time = 0U;

  //串口初始化
  Serial_Init(&huart1, NULL, NULL);

  //BMI088初始化
  app_bmi088_init();

  //视觉初始化
  Vision.Init();

  //云台模式才初始化
  if(Is_Gimbal_Mode)
  {
    //云台大疆电机初始化
    Gimbal_DJI_Motor_Init();

    //云台大疆电机PID初始化
    Gimbal_Yaw_Motor_PID_Init();
    Gimbal_Pitch_Motor_PID_Init();

    //云台目标初始化
    Gimbal_Target_Init();

    //云台IMU角度初始化
    Gimbal.Reset_Imu_Relative_BaseStart_State();

    //初始化低通滤波器
    Gimbal_Yaw_LPF.Configure(25.0f, 0.001f);
    Gimbal_Pitch_LPF.Configure(8.0f, 0.001f);
  }
}

/**
 * @brief 云台初始化（循环）
 * 
 * @return uint8_t 
 */
uint8_t Gimbal_Task_Global_Init_Loop(void)
{
  return app_bmi088_init_process_loop();
}

/**
 * @brief 云台PID重置信息
 * 
 */
void Gimbal_PID_Reset(void)
{
  PID_Gimbal_Motor_Yaw.Reset();
  PID_Gimbal_Motor_Pitch.Reset();
}

/* 哨兵模式 Target设置相关 ---------------------------------------------------*/
/**
 * @brief 云台目标初始化
 * 
 */
void Gimbal_Target_Init(void)
{
  target_set_time = 0;
  Sentry_Target_Entry_Time = Task_Time;
  Sentry_Target_Entry_Yaw = Gimbal.Get_Imu_Relative_World_Continuous_Yaw();
  Sentry_Target_Entry_Pitch = Gimbal.Get_Imu_Relative_World_Continuous_Pitch();
}

/**
 * @brief 判断哨兵模式目标是否处于过渡状态
 * 
 * @return true 处于过渡状态
 * @return false 不处于过渡状态
 */
static bool Sentry_Target_Is_In_Transition(void)
{
  return (Task_Time - Sentry_Target_Entry_Time) < Sentry_Target_Transition_Time_Ms;
}

/**
 * @brief 获取哨兵模式目标过渡结束后的时间（秒）
 * 
 * @return float 过渡结束后的时间（秒）
 */
static float Sentry_Target_Get_Post_Transition_Time_S(void)
{
  if(Sentry_Target_Is_In_Transition())
  {
    return 0.0f;
  }

  return (float)(Task_Time - Sentry_Target_Entry_Time - Sentry_Target_Transition_Time_Ms) * 0.001f;
}

/**
 * @brief 平滑过渡哨兵模式目标角度
 * 
 * @param Entry_Angle 初始角度
 * @param Sentry_Target 目标角度
 * @return float 平滑过渡后的角度
 */
static float Sentry_Target_Smooth_Transition(float Entry_Angle, float Sentry_Target)
{
  //计算过渡比例 Transition_Ratio
  //Transition_Ratio = (当前时间 - 进入时间) / 过渡时间
  float Transition_Ratio = (float)(Task_Time - Sentry_Target_Entry_Time) / (float)Sentry_Target_Transition_Time_Ms;

  if(Transition_Ratio > 1.0f)
  {
    Transition_Ratio = 1.0f;
  }

  //本质是线性插值：
  //最终目标 = 进入时角度 + (哨兵目标 - 进入时角度) * 过渡比例

  //输出平滑过渡后的角度
  return Entry_Angle + (Sentry_Target - Entry_Angle) * Transition_Ratio;
}

/**
 * @brief 哨兵模式 Yaw 角度目标函数
 *        以 Task_Time 为时间基准 左右平滑扫描
 * 
 * @return float Yaw目标角度（度）
 */
float Angle_Target_Sentry_Gimbal_Yaw(void)
{
  //阶跃控制模式
  if(Control_Config_Data.Yaw_Is_Trigonometric_Target_Mode == false)
  {
    //返回阶跃模式目标值
    return Control_Config_Data.Yaw_Step_Target;
  }

  float Yaw_A = Control_Config_Data.Yaw_a;
  float Yaw_f = Control_Config_Data.Yaw_f;

  //Yaw_Is_Trigonometric_Target_Mode是True
  //正常哨兵模式
  if(Control_Config_Data.Is_Go_To_Zero_To_Start_Sentry)
  {
    //Yaw 不会一进哨兵就开始扫，而是先归到哨兵扫描中心 0°。
    if(Sentry_Target_Is_In_Transition())
    {
      return Sentry_Target_Smooth_Transition(Sentry_Target_Entry_Yaw, 0.0f);
    }

    // 过渡结束后再从0点开始计时
    float Target_Time = Sentry_Target_Get_Post_Transition_Time_S();   // 过渡结束后再从0点开始计时

    //返回正弦函数模式目标值
    float Sentry_Target_Yaw = Yaw_A * sinf(2.0f * PI * Yaw_f * Target_Time);
    return Sentry_Target_Yaw;
  }
  else
  {
    //从当前开始计时 直接从当前位置进入哨兵
    float Target_Time = (Task_Time - Sentry_Target_Entry_Time) * 0.001f;   // Task_Time单位是1ms，这里转成秒
    float sentry_entry_yaw_wrapped = MyMath_Wrap_To_180(Sentry_Target_Entry_Yaw);

    //返回正弦函数模式目标值
    //Yaw 扫描幅值 = min(原始幅值, 距离左右限位的剩余空间)
    if(Gimbal_Yaw_Target_High - sentry_entry_yaw_wrapped < Yaw_A)
    {
      Yaw_A = Gimbal_Yaw_Target_High - sentry_entry_yaw_wrapped;
    }

    if(sentry_entry_yaw_wrapped - Gimbal_Yaw_Target_Low  < Yaw_A)
    {
      Yaw_A = sentry_entry_yaw_wrapped - Gimbal_Yaw_Target_Low;
    }

    if(Yaw_A < 0.0f)
    {
      Yaw_A = 0.0f;
    }
    //加上进入哨兵模式时的角度，保证以进入时角度为中心进行扫描
    float Sentry_Target_Yaw = Yaw_A * sinf(2.0f * PI * Yaw_f * Target_Time) + Sentry_Target_Entry_Yaw;
    return Sentry_Target_Yaw;
  }
}

/**
 * @brief 哨兵模式 Pitch 角度目标函数
 *        以 Task_Time 为时间基准，上下平滑扫描，范围 -38° ~ +38°
 * 
 * @return float Pitch目标角度（度）
 */
float Angle_Target_Sentry_Gimbal_Pitch(void)
{
  //阶跃控制模式
  if(Control_Config_Data.Pitch_Is_Trigonometric_Target_Mode == false)
  {
    //返回阶跃模式目标值
    return Control_Config_Data.Pitch_Step_Target;
  }

  //Pitch_Is_Trigonometric_Target_Mode是True
  //正常哨兵模式
  if(Control_Config_Data.Is_Go_To_Zero_To_Start_Sentry)
  {
    //Pitch 不会一进哨兵就开始扫，而是先归到哨兵扫描中心 0°。
    if(Sentry_Target_Is_In_Transition())
    {
      return Sentry_Target_Smooth_Transition(Sentry_Target_Entry_Pitch, 0.0f);
    }
  }

  float Target_Time = (Task_Time - Sentry_Target_Entry_Time) * 0.001f;   // Task_Time单位是1ms，这里转成秒

  float Pitch_A = Control_Config_Data.Pitch_a;             // 扫描幅值：±38°
  float Pitch_f = Control_Config_Data.Pitch_f;             // 频率 0.12Hz，对应周期约 8.33s
  float phase = PI / 2.0f;             // 加一个相位差，避免和Yaw完全同步

  float sentry_target = Pitch_A * sinf(2.0f * PI * Pitch_f * Target_Time + phase);

  //因为Pitch是sin形式目标且加入相位差 所以Pitch加入平滑插值来避免一进哨兵模式就从当前角度直接跳到±38°的目标，造成大幅度抖动
  //这个插值只在进入哨兵模式的前 Sentry_Target_Transition_Time_Ms 时间内生效
  // 也就是每次进入哨兵模式时都会有一次平滑过渡，过渡完成后就完全切换到sin目标
  return Sentry_Target_Smooth_Transition(Sentry_Target_Entry_Pitch, sentry_target);
} 

/**
 * @brief 哨兵模式速度目标变换设置函数
 * 
 * @return float 速度目标值
 */
float Speed_Target_Sentry(void)
{
  // float f = 0.00025;//频率 = 周期倒数
  // return 5.0f * sinf(2.0f * PI * f * target_set_time);//返回sin变化的目标值
  uint32_t time_in_period = target_set_time % Control_Config_Data.speed_period_ms;

  if (time_in_period < (Control_Config_Data.speed_period_ms / 2U))
  {
      return Control_Config_Data.speed_amplitude;   // 前半周期输出 +5
  }
  else
  {
     return -Control_Config_Data.speed_amplitude;  // 后半周期输出 -5
  }
}

/**
 * @brief 哨兵模式速度目标变换设置函数Pitch
 * 
 * @return float 速度目标值
 */
float Speed_Target_Sentry_Pitch(void)
{
  static float temp = 0.5f;   // 初始先给一个方向，也可以写成 Control_Config_Data.speed_amplitude
  float current_angle = PID_Gimbal_Motor_Pitch.Angle_States.Current;

  if (current_angle < -38.0f)
  {
    temp = Control_Config_Data.speed_amplitude;
  }
  else if (current_angle > 38.0f)
  {
    temp = -Control_Config_Data.speed_amplitude;
  }

  return temp;
}

/**
 * @brief 哨兵模式角度目标变换设置函数
 * 
 * @return float 角度目标值
 */
float Angle_Target_Sentry(void)
{
  // float f = 0.00025;//频率 = 周期倒数
  // return 60.0f * sinf(2.0f * PI * f * target_set_time);//返回-60到+60

  uint32_t time_in_period = target_set_time % Control_Config_Data.angle_period_ms;

  if (time_in_period < (Control_Config_Data.angle_period_ms / 2U))
  {
      return Control_Config_Data.angle_amplitude;   // 前半周期输出 +5
  }
  else
  {
     return -Control_Config_Data.angle_amplitude;  // 后半周期输出 -5
  }
}

/**
 * @brief 哨兵模式角度目标变换设置函数
 * 
 * @return float 角度目标值
 */
float Angle_Target_Sentry_Pitch(void)
{
  static float temp = 38; 
  float current_angle = PID_Gimbal_Motor_Pitch.Angle_States.Current;

  if (current_angle < -38.0f)
  {
    temp = Control_Config_Data.angle_amplitude;
  }
  else if (current_angle > 38.0f)
  {
    temp = -Control_Config_Data.angle_amplitude;
  }

  return temp;
}

/**
 * @brief 哨兵模式目标变换设置函数（角度模式）
 * 
 */
void Set_Yaw_and_Pitch_Motor_Target_Sentry(void)
{
  float current_continuous_yaw = Gimbal.Get_Imu_Relative_World_Continuous_Yaw();
  float yaw_turn_base = Gimbal_Task_Get_Turn_Base(current_continuous_yaw);
  float yaw_target_low = yaw_turn_base + Gimbal_Yaw_Target_Low;
  float yaw_target_high = yaw_turn_base + Gimbal_Yaw_Target_High;

  PID_Gimbal_Motor_Yaw.Set_Angle_Target(PID_Gimbal_Motor_Yaw.Limit(Angle_Target_Sentry_Gimbal_Yaw(),
                                                                   yaw_target_low,
                                                                   yaw_target_high));
  //PID_Gimbal_Motor_Yaw.Set_Angle_Target(Angle_Target_Sentry());
  //PID_Gimbal_Motor_Yaw.Set_Speed_Target(Speed_Target_Sentry());

  PID_Gimbal_Motor_Pitch.Set_Angle_Target(PID_Gimbal_Motor_Pitch.Limit(Angle_Target_Sentry_Gimbal_Pitch(), 
                                                                              Gimbal_Pitch_Target_Low, Gimbal_Pitch_Target_High));
  //PID_Gimbal_Motor_Pitch.Set_Angle_Target(Angle_Target_Sentry_Pitch());
  //PID_Gimbal_Motor_Pitch.Set_Speed_Target(Speed_Target_Sentry());
}

/**
 * @brief 哨兵模式速度目标变换设置函数
 * 
 */
void Set_Yaw_and_Pitch_Motor_Speed_Target_Sentry(void)
{
  PID_Gimbal_Motor_Yaw.Set_Speed_Target(Speed_Target_Sentry());

  PID_Gimbal_Motor_Pitch.Set_Speed_Target(Speed_Target_Sentry_Pitch());
}

/* PID 当前参数传入相关函数 ------------------------------------------------------*/
/**
 * @brief 云台推送PID输出值至电机控制值
 * 
 */
void Gimbal_Push_PID_Out_To_Motor_Control(void)
{
  float yaw_out;
  if(is_feedforward_mode)
  {
    float yaw_feedforward = Gimbal_FeedForward.Friction_Feedforward_Advanced(
    PID_Gimbal_Motor_Yaw.Get_Speed_Target(),
    0.0f,
    1290.0f,
    -212.0f,
    0.5f,
    3.0f
  );
    yaw_out = PID_Gimbal_Motor_Yaw.Get_Out() + yaw_feedforward;
  }
  else
  {
    yaw_out = PID_Gimbal_Motor_Yaw.Get_Out();
  }

  float pitch_out;

  if(is_g_feedback_mode)
  {
    pitch_out = PID_Gimbal_Motor_Pitch.Get_Out()
     + 0.6 * Pitch_Gravity_Compensation(Gimbal.Get_Imu_Relative_World_Continuous_Pitch());//重补力度
  }
  else
  {
    pitch_out = PID_Gimbal_Motor_Pitch.Get_Out();
  }
  
  DJI_Motor_Pitch.Set_Out(pitch_out);
  DJI_Motor_Yaw.Set_Out(yaw_out);
}

/* Vision 相关函数 ----------------------------------------------------------*/

/**
 * @brief 相机USB离线检测
 * 
 */
void Gimbal_Vision_Mode_Judge_1ms(void)
{
  Vision.USB_Offline_Detection_1ms(Task_Time);

  if(Vision.Get_Online_State() == true && Vision.Get_Detected_State() == true)
  {
    gimtal_states = gimbal_states_aim_mode;
    if(last_gimtal_states != gimbal_states_aim_mode)
    {
      last_gimtal_states = gimbal_states_aim_mode;
      Need_Change_Mode = true;
    }
  }
  else
  {
    gimtal_states = gimbal_states_sentry_mode;
    if(last_gimtal_states != gimbal_states_sentry_mode)
    {
      last_gimtal_states = gimbal_states_sentry_mode;
      Need_Change_Mode = true;
    }
  }
}

/* Pitch重力补偿获取数据相关函数 ----------------------------------------------------------*/
float Pitch_Gravity_Compensation(float pitch_deg)
{
    float theta = pitch_deg * PI / 180.0f;
    return 1419.6f * sinf(theta + 0.661f) - 1032.5f;
}

/**
 * @brief Pitch重力补偿数据采集目标函数
 *        采用“阶梯式目标角 + 停留”的方式，方便在每个角度稳定后读取电流
 *
 * @return float 当前Pitch目标角（度）
 */
float Pitch_Target_Gravity_Collect(void)
{
  const float min_angle = -40.0f;        // 采集下限
  const float max_angle =  40.0f;        // 采集上限
  const float step_angle =  5.0f;        // 每次步进5度
  const uint32_t hold_ms = 3500U;        // 每个角度停留2秒
  const uint32_t stable_window_ms = 500U;// 最后0.5秒视为稳定采样窗口

  static bool initialized = false;
  static float target_angle = min_angle;
  static int direction = 1;              // +1上扫，-1下扫
  static uint32_t last_change_time = 0U;

  if(initialized == false)
  {
    initialized = true;
    target_angle = min_angle;
    direction = 1;
    last_change_time = Task_Time;
  }

  uint32_t elapsed = Task_Time - last_change_time;

  /* 最后 stable_window_ms 时间作为稳定采样窗口 */
  Gravity_Test_Stable_Window = (elapsed >= (hold_ms - stable_window_ms));

  if(elapsed >= hold_ms)
  {
    last_change_time = Task_Time;
    target_angle += direction * step_angle;

    if(target_angle >= max_angle)
    {
      target_angle = max_angle;
      direction = -1;
    }
    else if(target_angle <= min_angle)
    {
      target_angle = min_angle;
      direction = 1;
    }
  }

  Gravity_Test_Target_Pitch = target_angle;
  Gravity_Test_Direction = direction;

  return target_angle;
}

/**
 * @brief Pitch重力补偿数据采集目标设置函数
 *        Yaw锁定在当前角度，Pitch按阶梯目标运动
 */
void Set_Pitch_Motor_Target_Gravity_Collect(void)
{
  /* Yaw保持当前值不动，避免跟着乱跑 */
  PID_Gimbal_Motor_Yaw.Set_Angle_Target(Gimbal.Get_Imu_Relative_World_Continuous_Yaw());

  /* Pitch执行阶梯扫描 */
  PID_Gimbal_Motor_Pitch.Set_Angle_Target(Pitch_Target_Gravity_Collect());
}