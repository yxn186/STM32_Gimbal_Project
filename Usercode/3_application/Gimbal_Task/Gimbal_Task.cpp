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


float temp1,temp2,temp3;

/* 重力补偿采集模式开关 */
bool is_pitch_gravity_collect_mode = false;

/* Ozone观察用变量 */
float Gravity_Test_Target_Pitch = 0.0f;      // 当前给定目标角
float Gravity_Test_Current_Pitch = 0.0f;     // 当前实际pitch角
float Gravity_Test_Current_Speed = 0.0f;     // 当前pitch角速度
float Gravity_Test_Current_Torque = 0.0f;    // 当前pitch电机反馈电流/力矩电流
int   Gravity_Test_Direction = 1;            // +1: 上扫  -1: 下扫
bool  Gravity_Test_Stable_Window = false;    // 1表示当前处于可采样稳定窗口
volatile uint32_t Main_Task_Period_Us = 0;   // 相邻两次任务入口间隔，含调度抖动
volatile uint32_t Main_Task_Exec_Us = 0;     // 单次循环执行耗时，不含osDelay后的阻塞时间
volatile uint32_t Main_Task_Period_Cycle = 0;
volatile uint32_t Main_Task_Exec_Cycle = 0;

typedef struct
{
  float speed_amplitude;
  uint32_t speed_period_ms;
  float angle_amplitude;
  uint32_t angle_period_ms;
  float Yaw_f;
  float Yaw_a;
  float Pitch_f;
  float Pitch_a;

}Temp_Data;

Temp_Data Temp_Control_Data ={0.5,3000,38,2000,0.8,50,0.4,38};


/*  Task层全局变量 ------------------------------------------------------------*/
//全局初始化变量
bool Global_Init_Finished = false;

//自定义是否开启云台模式
//不开启则纯读BMI数据
bool is_gimbal_mode = true;

bool is_gimbal_target_mode = true;

bool is_feedforward_mode = false;

bool is_lpf_mode = true;

bool is_g_feedback_mode = true;

float Gimbal_Normal_Kf_a_Yaw = 15.0f;
float Gimbal_Normal_Kf_a_Pitch = 15.0f;
float Gimbal_Aim_Kf_a_Yaw = 3.0f;
float Gimbal_Aim_Kf_a_Pitch = 3.0f;

//任务时间
uint32_t Task_Time;

//哨兵模式目标过渡相关
static const uint32_t Sentry_Target_Transition_Time_Ms = 2000U;
static uint32_t Sentry_Target_Entry_Time = 0U;
static float Sentry_Target_Entry_Yaw = 0.0f;
static float Sentry_Target_Entry_Pitch = 0.0f;

/*  Task层数据    ------------------------------------------------------------*/

static uint32_t DWT_Cycles_To_Us(uint32_t cycles)
{
  return (uint32_t)(((uint64_t)cycles * 1000000ULL) / SystemCoreClock);
}

static float Sentry_Target_Get_Transition_Ratio(void)
{
  float transition_ratio =
      (float)(Task_Time - Sentry_Target_Entry_Time) / (float)Sentry_Target_Transition_Time_Ms;

  if(transition_ratio > 1.0f)
  {
    transition_ratio = 1.0f;
  }

  return transition_ratio;
}

static bool Sentry_Target_Is_In_Transition(void)
{
  return (Task_Time - Sentry_Target_Entry_Time) < Sentry_Target_Transition_Time_Ms;
}

static float Sentry_Target_Smooth_Transition(float entry_angle, float sentry_target)
{
  float transition_ratio = Sentry_Target_Get_Transition_Ratio();

  return entry_angle + (sentry_target - entry_angle) * transition_ratio;
}

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
bool need_change_mode = false;


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

/**
 * @brief 根据模式更新云台前馈系数Kf_a
 * 
 */
static void Gimbal_Update_Kf_a_By_Mode(void)
{
  float Yaw_Target_Kf_a = Gimbal_Normal_Kf_a_Yaw;
  float Pitch_Target_Kf_a = Gimbal_Normal_Kf_a_Pitch;
  bool is_sentry_smooth_transition =
      (gimtal_states == gimbal_states_sentry_mode) &&
      (need_change_mode || Sentry_Target_Is_In_Transition());

  if((is_pitch_gravity_collect_mode == false) &&
     ((gimtal_states == gimbal_states_aim_mode) ||
      is_sentry_smooth_transition))
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
    PID_Gimbal_Motor_Yaw.Ki_s = 60;
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

    PID_Gimbal_Motor_Yaw.Out_High = 8000;
    PID_Gimbal_Motor_Yaw.Out_Low  = -8000;
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
  gimbal_task_init();
  /* Infinite loop */
  for(;;)
  {
    if(gimbal_task_init_loop())
    {
      Global_Init_Finished = true;
      osThreadTerminate(osThreadGetId());
    }
  }
  /* USER CODE END StartInitTask */
}

/**
 * @brief 数据打印任务 20ms
 * 
 * @param argument 
 */
extern "C" void Data_ptintf_task(void *argument)
{
  /* USER CODE BEGIN Data_ptintf_task */
  /* Infinite loop */
  for(;;)
  {
    Vision.USB_Transmit_Angle(Gimbal.Get_Imu_Relative_World_Continuous_Yaw(),Gimbal.Get_Imu_Relative_World_Continuous_Pitch());
    //app_bmi088_20ms_task();
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

    // Gimbal.Update_Imu_Pose_Relative_BaseStart(q0,
    //                                       q1,
    //                                       q2,
    //                                       q3,
    //                                       DJI_Motor_Yaw.Get_Angle(),
    //                                       DJI_Motor_Pitch.Get_Angle(),
    //                                       Task_Time,
    //                                       100);

    Gimbal.Update_Imu_Pose_Relative_World(q0,
                                          q1,
                                          q2,
                                          q3);


    //视觉模式判断
    Gimbal_Vision_Mode_Judge_1ms();
    Gimbal_Update_Kf_a_By_Mode();

    if(is_gimbal_mode)
    {
      //模式切换判断
      if(need_change_mode)
      {
        gimbal_pid_reset();
        Gimbal_Yaw_LPF.Reset();
        Gimbal_Pitch_LPF.Reset();
        if(gimtal_states == gimbal_states_sentry_mode)
        {
          gimbal_target_init();
        }
        need_change_mode = false;
      }

      //判断模式 设置target
      if(is_pitch_gravity_collect_mode)
      {
        Set_Pitch_Motor_Target_Gravity_Collect();
      }
      //判断模式 设置target 弄成函数？
      else if(gimtal_states == gimbal_states_aim_mode)//瞄准装甲板模式 接收视觉信息
      {
        // float Pitch_Target = Gimbal.Get_Imu_Relative_World_Continuous_Pitch() + Vision.Get_Pitch();
        // float Yaw_Target = Gimbal.Get_Imu_Relative_World_Continuous_Yaw() + Vision.Get_Yaw();

        float Pitch_Target = Vision.Get_Pitch();
        float Yaw_Target = Vision.Get_Yaw();

        // float Pitch_Target = Gimbal.Get_Imu_Relative_BaseCurrent_Model_Continuous_Pitch() + Vision.Get_Delta_Pitch();
        // float Yaw_Target = Gimbal.Get_Imu_Relative_BaseCurrent_Model_Continuous_Yaw() + Vision.Get_Delta_Yaw();

        //获取到的信息存入PID目标（可能需要先做数据处理！！！！！！！！！）
        PID_Gimbal_Motor_Pitch.Set_Angle_Target(PID_Gimbal_Motor_Pitch.Limit(Pitch_Target, -40.0f, 40.0f));
        PID_Gimbal_Motor_Yaw.Set_Angle_Target(PID_Gimbal_Motor_Yaw.Limit(Yaw_Target, -180.0f, 180.0f));
        
      }
      else if(gimtal_states == gimbal_states_sentry_mode)//哨兵模式 自己乱转
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

      temp1 = DJI_Motor_Yaw.Get_Angle();
      temp2 = DJI_Motor_Pitch.Get_Angle();
      temp3 = DJI_Motor_Yaw.Get_Torque_Current();

      //上面设置完target 开始pid
      
      //当前速度赋值 待确认赋值正确
      Gimbal_Push_Motor_Current_Speed_To_PID();

      //当前角度赋值
      //Gimbal_Push_Gimbal_Pitch_and_Yaw_To_PID(Gimbal.Get_Front_Continuous_Pitch(),Gimbal.Get_Front_Continuous_Yaw());

      

      Gimbal_Push_Gimbal_Pitch_and_Yaw_To_PID(Gimbal.Get_Imu_Relative_World_Continuous_Pitch(),
                                              Gimbal.Get_Imu_Relative_World_Continuous_Yaw()); 

      // Gimbal_Push_Gimbal_Pitch_and_Yaw_To_PID(Gimbal.Get_Imu_Relative_BaseCurrent_Model_Continuous_Pitch(),
      //                                             Gimbal.Get_Imu_Relative_BaseCurrent_Model_Continuous_Yaw());                                     
      

      //PID计算
      if(is_lpf_mode)
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
      else
      {
        //单环PID 先跑通
        //PID_Gimbal_Motor_Yaw.Control_Speed_To_Out();
        //PID_Gimbal_Motor_Pitch.Control_Speed_To_Out();

        //双环PID控制 跑通后设置
        PID_Gimbal_Motor_Yaw.Control_Cascade();
        PID_Gimbal_Motor_Pitch.Control_Cascade();
      }
      
      //将PID输出值推送至电机输出值
      Gimbal_Push_PID_Out_To_Motor_Control();

      //将电机输出值进行CAN通信发送
      Gimbal_DJI_Motor_Group.Push_Data();

      Gravity_Test_Current_Pitch = Gimbal.Get_Imu_Relative_World_Continuous_Pitch();
      Gravity_Test_Current_Speed = DJI_Motor_Pitch.Get_AngleSpeed();
      Gravity_Test_Current_Torque = DJI_Motor_Pitch.Get_Torque_Current();
    }

    uint32_t loop_end_cyccnt = DWT_GetCYCCNT();
    Main_Task_Exec_Cycle = loop_end_cyccnt - loop_start_cyccnt;
    Main_Task_Exec_Us = DWT_Cycles_To_Us(Main_Task_Exec_Cycle);

    next_tick += period_ticks;
    osDelayUntil(next_tick);
    //osDelay(1);
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
  //串口初始化
  Serial_Init(&huart1, NULL, NULL);

  //BMI088初始化
  app_bmi088_init();

  //视觉初始化
  Vision.Init();

  if(is_gimbal_mode)
  {
    //云台大疆电机初始化
    Gimbal_DJI_Motor_Init();

    //云台大疆电机PID初始化
    Gimbal_Yaw_Motor_PID_Init();
    Gimbal_Pitch_Motor_PID_Init();

    //云台目标初始化
    gimbal_target_init();

    //云台IMU角度初始化
    Gimbal.Reset_Imu_Relative_BaseStart_State();
    //Gimbal.Set_Pitch_Mechanical_Zero_Angle(64.0f);

    //初始化低通滤波器
    Gimbal_Yaw_LPF.Configure(100.0f, 0.001f);
    Gimbal_Pitch_LPF.Configure(8.0f, 0.001f);
  }
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

/**
 * @brief 云台目标初始化
 * 
 */
void gimbal_target_init(void)
{
  target_set_time = 0;
  Sentry_Target_Entry_Time = Task_Time;
  Sentry_Target_Entry_Yaw = Gimbal.Get_Imu_Relative_World_Continuous_Yaw();
  Sentry_Target_Entry_Pitch = Gimbal.Get_Imu_Relative_World_Continuous_Pitch();
}

/**
 * @brief 哨兵模式 Yaw 角度目标函数
 *        以 Task_Time 为时间基准，左右平滑扫描，范围 -70° ~ +70°
 * 
 * @return float Yaw目标角度（度）
 */
float Angle_Target_Sentry_Gimbal_Yaw(void)
{
  float time_s = (Task_Time - Sentry_Target_Entry_Time) * 0.001f;   // Task_Time单位是1ms，这里转成秒

  float amplitude = Temp_Control_Data.Yaw_a;             // 扫描幅值：±70°
  float frequency = Temp_Control_Data.Yaw_f;             // 频率 0.20Hz，对应周期 5s

  float sentry_target = amplitude * sinf(2.0f * PI * frequency * time_s);

  return Sentry_Target_Smooth_Transition(Sentry_Target_Entry_Yaw, sentry_target);
}

/**
 * @brief 哨兵模式 Pitch 角度目标函数
 *        以 Task_Time 为时间基准，上下平滑扫描，范围 -38° ~ +38°
 * 
 * @return float Pitch目标角度（度）
 */
float Angle_Target_Sentry_Gimbal_Pitch(void)
{
  float time_s = (Task_Time - Sentry_Target_Entry_Time) * 0.001f;   // Task_Time单位是1ms，这里转成秒

  float amplitude = Temp_Control_Data.Pitch_a;             // 扫描幅值：±38°
  float frequency = Temp_Control_Data.Pitch_f;             // 频率 0.12Hz，对应周期约 8.33s
  float phase = PI / 2.0f;             // 加一个相位差，避免和Yaw完全同步

  float sentry_target = amplitude * sinf(2.0f * PI * frequency * time_s + phase);

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
  uint32_t time_in_period = target_set_time % Temp_Control_Data.speed_period_ms;

  if (time_in_period < (Temp_Control_Data.speed_period_ms / 2U))
  {
      return Temp_Control_Data.speed_amplitude;   // 前半周期输出 +5
  }
  else
  {
     return -Temp_Control_Data.speed_amplitude;  // 后半周期输出 -5
  }
}

/**
 * @brief 哨兵模式速度目标变换设置函数Pitch
 * 
 * @return float 速度目标值
 */
float Speed_Target_Sentry_Pitch(void)
{
  static float temp = 0.5f;   // 初始先给一个方向，也可以写成 Temp_Control_Data.speed_amplitude
  float current_angle = PID_Gimbal_Motor_Pitch.Angle_States.Current;

  if (current_angle < -38.0f)
  {
    temp = Temp_Control_Data.speed_amplitude;
  }
  else if (current_angle > 38.0f)
  {
    temp = -Temp_Control_Data.speed_amplitude;
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

  uint32_t time_in_period = target_set_time % Temp_Control_Data.angle_period_ms;

  if (time_in_period < (Temp_Control_Data.angle_period_ms / 2U))
  {
      return Temp_Control_Data.angle_amplitude;   // 前半周期输出 +5
  }
  else
  {
     return -Temp_Control_Data.angle_amplitude;  // 后半周期输出 -5
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
    temp = Temp_Control_Data.angle_amplitude;
  }
  else if (current_angle > 38.0f)
  {
    temp = -Temp_Control_Data.angle_amplitude;
  }

  return temp;
}

/**
 * @brief 哨兵模式目标变换设置函数
 * 
 */
void Set_Yaw_and_Pitch_Motor_Target_Sentry(void)
{
  PID_Gimbal_Motor_Yaw.Set_Angle_Target(Angle_Target_Sentry_Gimbal_Yaw());
  //PID_Gimbal_Motor_Yaw.Set_Angle_Target(Angle_Target_Sentry());
  //PID_Gimbal_Motor_Yaw.Set_Speed_Target(Speed_Target_Sentry());

  PID_Gimbal_Motor_Pitch.Set_Angle_Target(Angle_Target_Sentry_Gimbal_Pitch());
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
                  + 2 * Pitch_Gravity_Compensation(Gimbal.Get_Imu_Relative_World_Continuous_Pitch());
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
      need_change_mode = true;
    }
  }
  else
  {
    gimtal_states = gimbal_states_sentry_mode;
    if(last_gimtal_states != gimbal_states_sentry_mode)
    {
      last_gimtal_states = gimbal_states_sentry_mode;
      need_change_mode = true;
    }
  }
}
