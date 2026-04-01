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
#include "app_bmi088.h"
#include "Serial.h"
#include "gimbal_task.h"
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
#include "Gimbal.h"
#include <stdint.h>

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

Temp_Data Temp_Control_Data ={0.5,2000,38,2000,0.8,40,0.4,38};


/*  Task层全局变量 ------------------------------------------------------------*/
//全局初始化变量
bool Global_Init_Finished = false;

//自定义是否开启云台模式
//不开启则纯读BMI数据
bool is_gimbal_mode = true;

bool is_gimbal_target_mode = true;

/**
 * @brief 相机USB在线状态
 * 
 */
bool Camera_USB_Online = false;

//任务时间
uint32_t Task_Time;

//USB在线时间 判断超时用
uint32_t Camera_USB_Online_Time;
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
bool need_change_mode = false;

/**
 * @brief 云台实际角度
 * 
 */
float Yaw,Pitch,Roll;

/**
 * @brief 云台Imu反馈实际角度
 * 
 */
float Imu_Yaw,Imu_Pitch,Imu_Roll;


/**
 * @brief 云台自瞄目标参数
 * 
 */
float Aim_Pitch;
float Aim_Yaw;

/*  Task层类    --------------------------------------------------------------*/

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

/*  Task层初始化函数    ------------------------------------------------------*/

//PID参数在这里~~~
/**
 * @brief PID参数初始化 Yaw电机
 * 
 */
void Gimbal_Yaw_Motor_PID_Init(void)
{
  PID_Gimbal_Motor_Yaw.Kp_s = 1550;
  PID_Gimbal_Motor_Yaw.Ki_s = 105;
  PID_Gimbal_Motor_Yaw.Kd_s = 100;
  PID_Gimbal_Motor_Yaw.Kp_a = 0.3;
  PID_Gimbal_Motor_Yaw.Ki_a = 0;
  PID_Gimbal_Motor_Yaw.Kd_a = 0;

  PID_Gimbal_Motor_Yaw.ErrorInt_High_s = 30;
  PID_Gimbal_Motor_Yaw.ErrorInt_Low_s  = -30;
  PID_Gimbal_Motor_Yaw.ErrorInt_High_a = 0;
  PID_Gimbal_Motor_Yaw.ErrorInt_Low_a  = 0;

  PID_Gimbal_Motor_Yaw.Speed_Target_High = 20;
  PID_Gimbal_Motor_Yaw.Speed_Target_Low = -20;

  PID_Gimbal_Motor_Yaw.Out_High = 4000;
  PID_Gimbal_Motor_Yaw.Out_Low  = -4000;
}

/**
 * @brief PID参数初始化 Pitch电机
 * 
 */
void Gimbal_Pitch_Motor_PID_Init(void)
{
  PID_Gimbal_Motor_Pitch.Kp_s = 820;
  PID_Gimbal_Motor_Pitch.Ki_s = 34;
  PID_Gimbal_Motor_Pitch.Kd_s = 10;
  PID_Gimbal_Motor_Pitch.Kp_a = 0.22;
  PID_Gimbal_Motor_Pitch.Ki_a = 0;
  PID_Gimbal_Motor_Pitch.Kd_a = 0;

  PID_Gimbal_Motor_Pitch.ErrorInt_High_s = 80;
  PID_Gimbal_Motor_Pitch.ErrorInt_Low_s  = -80;
  PID_Gimbal_Motor_Pitch.ErrorInt_High_a = 0;
  PID_Gimbal_Motor_Pitch.ErrorInt_Low_a  = -0;

  PID_Gimbal_Motor_Pitch.Speed_Target_High = 10;
  PID_Gimbal_Motor_Pitch.Speed_Target_Low = -10;

  PID_Gimbal_Motor_Pitch.Out_High = 4000;
  PID_Gimbal_Motor_Pitch.Out_Low  = -4000;
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

/*  Task层自定义回调函数类型 --------------------------------------------------*/

#pragma pack(push, 1)
/**
 * @brief 相机USB接收数据帧
 *
 * Delta_Yaw_10   : Yaw角度增量，单位 0.1度
 * Delta_Pitch_10 : Pitch角度增量，单位 0.1度
 *
 * 例如：
 *  12.3°  -> 123
 *  -8.7°  -> -87
 */
typedef struct
{
    uint8_t Frame_Header;      // 0xAA
    uint8_t Mode;              // 0: 哨兵模式, 1: 自瞄模式
    int16_t Delta_Yaw_10;      // 单位 0.1°
    int16_t Delta_Pitch_10;    // 单位 0.1°
    uint8_t Frame_Tail;        // 0x55
} Struct_Camera_USB_Frame_t;
#pragma pack(pop)

/**
 * @brief USB接收联合体
 *
 * Raw  用来接收原始字节
 * Data 用来按结构体字段解析
 */
typedef union
{
    Struct_Camera_USB_Frame_t Data;
    uint8_t Raw[sizeof(Struct_Camera_USB_Frame_t)];
} Union_Camera_USB_Frame;


void Camera_USB_CallBack(uint8_t *Buffer, uint16_t Length)
{
  if(Length == 0)  return;
  
  if(Global_Init_Finished == false) return;

  //回显
  USB_Transmit_Data(Buffer, Length);
  STM32_Printf("USB收到数据长度：%d\r\n",Length);
  //USB_Printf("USB收到数据长度：%d\r\n",Length);
  //Serial_Send_Data(Buffer,Length);

  // 长度必须和协议长度一致
  if (Length != sizeof(Union_Camera_USB_Frame)) return;
  
  //拷贝数据
  Union_Camera_USB_Frame Receive_Frame;
  memcpy(Receive_Frame.Raw, Buffer, sizeof(Receive_Frame.Raw));

  // 判断包头包尾
  if (Receive_Frame.Data.Frame_Header != 0xAA || Receive_Frame.Data.Frame_Tail != 0x55)
  {
    return;
  }

  //判断包头包尾
  // if(Buffer[0] != 0xAA || Buffer[12] != 0x55) return;

  //在线处理
  Camera_USB_Online_Time = Task_Time;
  Camera_USB_Online = true;
  //不发信息代表视觉掉线-->自动切换哨兵模式
  //发信息就用开头告知是否检测到目标

  //-------------解算数据--------------
  // 模式处理
  if (Receive_Frame.Data.Mode == 1)
  {
    gimtal_states = gimbal_states_aim_mode;

    if (last_gimtal_states != gimbal_states_aim_mode)
    {
        last_gimtal_states = gimbal_states_aim_mode;
        need_change_mode = true;
    }

    float Delta_Yaw = Receive_Frame.Data.Delta_Yaw_10 * 0.1f;
    float Delta_Pitch = Receive_Frame.Data.Delta_Pitch_10 * 0.1f;

    Gimbal.Set_Target_Front_Continuous_Pitch(Delta_Pitch);
    Gimbal.Set_Target_Front_Continuous_Yaw(Delta_Yaw);

    Serial_Printf("Delta_Pitch:%f Delta_Yaw:%f\r\n", Delta_Pitch, Delta_Yaw);
  }
  else
  {
    gimtal_states = gimbal_states_sentry_mode;

    if (last_gimtal_states != gimbal_states_sentry_mode)
    {
      last_gimtal_states = gimbal_states_sentry_mode;
      need_change_mode = true;
    }
  }

  //包头后的第一帧 决定模式
  // 0：哨兵模式
  // 1：自瞄模式
  // if(Buffer[1] == 1)
  // {
  //   //把解算目标存入Aim
    
  //   float Delta_Yaw = Buffer[3]*100 + Buffer[4]*10 + Buffer[5] + Buffer[6] *0.1;
  //   float Delta_Pitch = Buffer[8]*100 + Buffer[9]*10 + Buffer[10] + Buffer[11] *0.1;

  //   if(Buffer[2] == 0)
  //   {
  //     Delta_Yaw = -Delta_Yaw;
  //   }

  //   if(Buffer[7] == 0)
  //   {
  //     Delta_Pitch = -Delta_Pitch;
  //   }
    
  //   Gimbal.Set_Target_Front_Continuous_Pitch(Delta_Pitch);
  //   Gimbal.Set_Target_Front_Continuous_Yaw(Delta_Yaw);

  //   Serial_Printf("Delta_Pitch:%f Delta_Yaw:%f\r\n",Delta_Pitch,Delta_Yaw);
  //   //USB_Printf("Delta_Pitch:%f Delta_Yaw:%f\r\n",Delta_Pitch,Delta_Yaw);
  //   gimtal_states = gimbal_states_aim_mode;
  //   if(last_gimtal_states != gimbal_states_aim_mode)
  //   {
  //     last_gimtal_states = gimbal_states_aim_mode;
  //     need_change_mode = true;
  //   }
  // }
  // else//哨兵模式
  // {
  //   gimtal_states = gimbal_states_sentry_mode;
  //   if(last_gimtal_states != gimbal_states_sentry_mode)
  //   {
  //     last_gimtal_states = gimbal_states_sentry_mode;
  //     need_change_mode = true;
  //   }
  // }
}

/*  Task层FreeRTOS函数 任务函数 -----------------------------------------------*/

extern "C" void StartInitTask(void *argument)
{
 /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartInitTask */
  gimbal_task_init();
  Gimbal.Reset_Front_Angle_State();
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
    //app_bmi088_20ms_task();
    osDelay(20);
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
  for(;;)
  {
    //相机USB离线检测
    Camera_USB_Offline_Check();

    float y,r,p;
    app_bmi088_1ms_task_get_now_pitch_yaw_roll(&y,&p,&r);
    Gimbal_Set_Front_Angle(y,r,p);
    Gimbal.Update_Front_Angle(y,p);

    if(is_gimbal_mode)
    {
      //模式切换判断
      if(need_change_mode)
      {
        gimbal_pid_reset();
        need_change_mode = false;
      }
      //判断模式 设置target 弄成函数？
      if(gimtal_states == gimbal_states_aim_mode)//瞄准装甲板模式 接收视觉信息
      {
        //USB信息获取在回调中

        //获取到的信息存入PID目标（可能需要先做数据处理！！！！！！！！！）
        PID_Gimbal_Motor_Pitch.Set_Angle_Target(PID_Gimbal_Motor_Pitch.Limit(Gimbal.Get_Target_Front_Continuous_Pitch(), -40.0f, 40.0f));
        PID_Gimbal_Motor_Yaw.Set_Angle_Target(Gimbal.Get_Target_Front_Continuous_Yaw());
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

      //上面设置完target 开始pid
      
      //当前速度赋值 待确认赋值正确
      Gimbal_Push_Motor_Current_Speed_To_PID();

      //当前角度赋值
      Gimbal_Push_Gimbal_Pitch_and_Yaw_To_PID(Gimbal.Get_Front_Continuous_Pitch(),
                                                  Gimbal.Get_Front_Continuous_Yaw());

      //单环PID 先跑通
      //PID_Gimbal_Motor_Yaw.Control_Speed_To_Out();
      //PID_Gimbal_Motor_Pitch.Control_Speed_To_Out();

      //双环PID控制 跑通后设置
      PID_Gimbal_Motor_Yaw.Control_Cascade();
      PID_Gimbal_Motor_Pitch.Control_Cascade();

      //将PID输出值推送至电机输出值
      Gimbal_Push_PID_Out_To_Motor_Control();

      //将电机输出值进行CAN通信发送
      Gimbal_DJI_Motor_Group.Push_Data();
    } 
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
  //串口初始化
  Serial_Init(&huart1);

  //BMI088初始化
  app_bmi088_init();

  //USB初始化
  USB_Init(Camera_USB_CallBack);

  if(is_gimbal_mode)
  {
    //云台大疆电机初始化
    Gimbal_DJI_Motor_Init();

    //云台大疆电机PID初始化
    Gimbal_Yaw_Motor_PID_Init();
    Gimbal_Pitch_Motor_PID_Init();

    //云台目标初始化
    gimbal_target_init();
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

void Gimbal_Set_Front_Angle(float yaw,float roll,float pitch)
{
  Gimbal.Set_Front_Yaw(yaw);
  Gimbal.Set_Front_Pitch(pitch);
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
 * @brief 哨兵模式 Yaw 角度目标函数
 *        以 Task_Time 为时间基准，左右平滑扫描，范围 -70° ~ +70°
 * 
 * @return float Yaw目标角度（度）
 */
float Angle_Target_Sentry_Gimbal_Yaw(void)
{
  float time_s = Task_Time * 0.001f;   // Task_Time单位是1ms，这里转成秒

  float amplitude = Temp_Control_Data.Yaw_a;             // 扫描幅值：±70°
  float frequency = Temp_Control_Data.Yaw_f;             // 频率 0.20Hz，对应周期 5s

  return amplitude * sinf(2.0f * PI * frequency * time_s);
}

/**
 * @brief 哨兵模式 Pitch 角度目标函数
 *        以 Task_Time 为时间基准，上下平滑扫描，范围 -38° ~ +38°
 * 
 * @return float Pitch目标角度（度）
 */
float Angle_Target_Sentry_Gimbal_Pitch(void)
{
  float time_s = Task_Time * 0.001f;   // Task_Time单位是1ms，这里转成秒

  float amplitude = Temp_Control_Data.Pitch_a;             // 扫描幅值：±38°
  float frequency = Temp_Control_Data.Pitch_f;             // 频率 0.12Hz，对应周期约 8.33s
  float phase = PI / 2.0f;             // 加一个相位差，避免和Yaw完全同步

  return amplitude * sinf(2.0f * PI * frequency * time_s + phase);
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
  DJI_Motor_Pitch.Set_Out(PID_Gimbal_Motor_Pitch.Get_Out());
  DJI_Motor_Yaw.Set_Out(PID_Gimbal_Motor_Yaw.Get_Out());
}

/* USB 相关函数 ----------------------------------------------------------*/

/**
 * @brief 相机USB离线检测
 * 
 */
void Camera_USB_Offline_Check(void)
{
  if(Global_Init_Finished == false)
  {
    return;
  }

  if(Task_Time - Camera_USB_Online_Time <= 100)
  {
    return;
  }

  Camera_USB_Online = false;

  gimtal_states = gimbal_states_sentry_mode;
  if(last_gimtal_states != gimbal_states_sentry_mode)
  {
    last_gimtal_states = gimbal_states_sentry_mode;
    need_change_mode = true;
  }
}
