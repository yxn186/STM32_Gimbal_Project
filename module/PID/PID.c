/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    PID.c
  * @brief   PID调控库
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "PID.h"
#include <stdint.h>

#define PID_High 10000
#define PID_Low 0

/**
 * @brief PID控制 单电机
 * 
 * @param PID_Cfg_Data PID配置参数：Kp Ki Kd Target
 * @param PID_Status PID状态
 */
void PID_Control_Single(PID_Cfg_t *PID_Cfg_Data,PID_Status_t *PID_Status)
{
	//获取当前角速度
	float Actual = PID_Status->Current_speed;
	
	//获取误差
	PID_Status->Error1 = PID_Status->Error0;
	PID_Status->Error0 = PID_Cfg_Data->Target - Actual;
	
	//误差积分
	PID_Status->ErrorInt = PID_Status->Error0 + PID_Status->ErrorInt;
	
	//积分限幅
	if(PID_Status->ErrorInt >= 5000)
	{
		PID_Status->ErrorInt = 5000;
	}
	if(PID_Status->ErrorInt <= -1000)
	{
		PID_Status->ErrorInt = -1000;
	}
	
	//执行控制
	PID_Status->Out = PID_Cfg_Data->Kp * PID_Status->Error0 + PID_Cfg_Data->Ki * PID_Status->ErrorInt + PID_Cfg_Data->Kd * (PID_Status->Error0 - PID_Status->Error1);
	
	if(PID_Status->Out >= PID_High)
	{
		PID_Status->Out = PID_High;
	}
	if(PID_Status->Out <= PID_Low)
	{
		PID_Status->Out = PID_Low;
	}
}