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


/**
 * @brief PID控制 速度环
 * 
 * @param PID_Object PID控制对象指针，包含配置参数和状态
 */
void PID_Control_Speed(PID_t *PID_Object)
{
	//获取误差
	PID_Object->PID_Speed_Status.Error1 = PID_Object->PID_Speed_Status.Error0;
	PID_Object->PID_Speed_Status.Error0 = PID_Object->Speed_Target - PID_Object->PID_Speed_Status.Current_speed;;
	
	//误差积分
	PID_Object->PID_Speed_Status.ErrorInt = PID_Object->PID_Speed_Status.Error0 + PID_Object->PID_Speed_Status.ErrorInt;
	
	//积分限幅
	if(PID_Object->PID_Speed_Status.ErrorInt >= PID_Object->ErrorInt_High_s)
	{
		PID_Object->PID_Speed_Status.ErrorInt = PID_Object->ErrorInt_High_s;
	}
	if(PID_Object->PID_Speed_Status.ErrorInt <= PID_Object->ErrorInt_Low_s)
	{
		PID_Object->PID_Speed_Status.ErrorInt = PID_Object->ErrorInt_Low_s;
	}
	
	//执行控制
	PID_Object->Out = PID_Object->Kp_s * PID_Object->PID_Speed_Status.Error0 + PID_Object->Ki_s * PID_Object->PID_Speed_Status.ErrorInt + PID_Object->Kd_s * (PID_Object->PID_Speed_Status.Error0 - PID_Object->PID_Speed_Status.Error1);
	
	if(PID_Object->Out >= PID_Object->Out_High)
	{
		PID_Object->Out = PID_Object->Out_High;
	}
	if(PID_Object->Out <= PID_Object->Out_Low)
	{
		PID_Object->Out = PID_Object->Out_Low;
	}
}

/**
 * @brief PID控制 角度环
 * 
 * @param PID_Object PID控制对象指针，包含配置参数和状态
 */
void PID_Control_Angle(PID_t *PID_Object)
{
	//获取误差
	PID_Object->PID_Angle_Status.Error1 = PID_Object->PID_Angle_Status.Error0;
	PID_Object->PID_Angle_Status.Error0 = PID_Object->Angle_Target - PID_Object->PID_Angle_Status.Current_Angle;
	
	//误差积分
	PID_Object->PID_Angle_Status.ErrorInt = PID_Object->PID_Angle_Status.Error0 + PID_Object->PID_Angle_Status.ErrorInt;
	
	//积分限幅
	if(PID_Object->PID_Angle_Status.ErrorInt >= PID_Object->ErrorInt_High_a)
	{
		PID_Object->PID_Angle_Status.ErrorInt = PID_Object->ErrorInt_High_a;
	}
	if(PID_Object->PID_Angle_Status.ErrorInt <= PID_Object->ErrorInt_Low_a)
	{
		PID_Object->PID_Angle_Status.ErrorInt = PID_Object->ErrorInt_Low_a;
	}
	
	//执行控制
	PID_Object->Speed_Target = PID_Object->Kp_a * PID_Object->PID_Angle_Status.Error0 + PID_Object->Ki_a * PID_Object->PID_Angle_Status.ErrorInt + PID_Object->Kd_a * (PID_Object->PID_Angle_Status.Error0 - PID_Object->PID_Angle_Status.Error1);
	
	if(PID_Object->Speed_Target >= PID_Object->Speed_Target_High)
	{
		PID_Object->Speed_Target = PID_Object->Speed_Target_High;
	}
	if(PID_Object->Speed_Target <= PID_Object->Speed_Target_Low)
	{
		PID_Object->Speed_Target = PID_Object->Speed_Target_Low;
	}
}