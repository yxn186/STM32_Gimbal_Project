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

/*  Task层全局变量 ------------------------------------------------------------*/
bool init_finished = false;

/*  Task层数据    ------------------------------------------------------------*/
uint8_t Pitch_PID_Times;
uint8_t Yaw_PID_Times;

PID_t Pitch_Motor_PID ={0};
PID_t Yaw_Motor_PID = {0};

uint8_t ID1 = 1;
uint8_t ID2 = 2;

DJI_Motor_Data_t DJI_Motors_Data[8] = {0};

//PID参数设置！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！

/*  Task层自定义回调函数类型 --------------------------------------------------*/



/*  Task层FreeRTOS函数 --------------------------------------------------------*/

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
    //接收target函数

    app_bmi088_1ms_task(&Pitch_Motor_PID,&Yaw_Motor_PID);

    //gimbal_pid_conrol();


    //CAN数据发送函数
    //DJI_Motor_Control_Double(&hcan1,DJI_Motor_6020,ID1,Pitch_Motor_PID.Out,ID2,Yaw_Motor_PID.Out);

    //CAN数据接收 处理函数

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
    
}

uint8_t gimbal_task_init_loop(void)
{
  return app_bmi088_init_process_loop();
}

void gimbal_pid_conrol(void)
{
  PID_Control_Speed(&Pitch_Motor_PID);
  Pitch_PID_Times++;
  if(Pitch_PID_Times >= 10)
  {
    PID_Control_Angle(&Pitch_Motor_PID);
  }
  PID_Control_Speed(&Yaw_Motor_PID);
  Yaw_PID_Times++;
  if(Yaw_PID_Times >= 10)
  {
    PID_Control_Angle(&Yaw_Motor_PID);
  }
}
