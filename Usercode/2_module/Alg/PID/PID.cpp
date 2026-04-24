/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    PID.cpp
  * @brief   PID调控库
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "PID.h"
#include <math.h>

/**
 * @brief PID控制 速度环-->输出值
 * @details 使用类内部的速度目标、速度反馈、PID参数和状态量进行计算
 */
void Class_PID::Control_Speed_To_Out(void)
{
	//获取误差
	Speed_States.Error1 = Speed_States.Error0;
	Speed_States.Error0 = Speed_Target - Speed_States.Current;
	
	//目标接近0且误差很小时停积分
	uint8_t stop_integral_s = 0;
	if (Integral_Stop_Near_Zero_Enable_s != 0)
	{
		if ((fabsf(Speed_Target) <= Integral_Stop_Target_Abs_Threshold_s) &&
			(fabsf(Speed_States.Error0) <= Integral_Stop_Error_Abs_Threshold_s))
		{
			stop_integral_s = 1;
		}
	}

	//误差积分
	if (stop_integral_s == 0)
	{
		Speed_States.ErrorInt += Speed_States.Error0;
	}
	
	//积分限幅
	Speed_States.ErrorInt = Limit(Speed_States.ErrorInt, ErrorInt_Low_s, ErrorInt_High_s);
	
	//执行控制
	Out = Kp_s * Speed_States.Error0 + Ki_s * Speed_States.ErrorInt + Kd_s * (Speed_States.Error0 - Speed_States.Error1);
	
	Out = Limit(Out, Out_Low, Out_High);
}


/**
 * @brief PID控制 角度环-->速度
 * @details 使用类内部的角度目标、角度反馈、PID参数和状态量进行计算，
 *          输出结果作为速度目标值 Speed_Target
 */
void Class_PID::Control_Angle_To_Speed(void)
{
	//获取误差
	Angle_States.Error1 = Angle_States.Error0;
	Angle_States.Error0 = Angle_Target - Angle_States.Current;
	
	//目标接近0且误差很小时停积分
	uint8_t stop_integral_a = 0;
	if (Integral_Stop_Near_Zero_Enable_a != 0)
	{
		if ((fabsf(Angle_Target) <= Integral_Stop_Target_Abs_Threshold_a) &&
			(fabsf(Angle_States.Error0) <= Integral_Stop_Error_Abs_Threshold_a))
		{
			stop_integral_a = 1;
		}
	}

	//误差积分
	if (stop_integral_a == 0)
	{
		Angle_States.ErrorInt += Angle_States.Error0;
	}
	
	//积分限幅
	Angle_States.ErrorInt = Limit(Angle_States.ErrorInt, ErrorInt_Low_a, ErrorInt_High_a);
	
	//执行控制
	Speed_Target = Kp_a * Angle_States.Error0 + Ki_a * Angle_States.ErrorInt + Kd_a * (Angle_States.Error0 - Angle_States.Error1);
	
	Speed_Target = Limit(Speed_Target, Speed_Target_Low, Speed_Target_High);
}

/**
 * @brief PID控制 串级控制
 * @details 使用类内部的角度目标、角度反馈、PID参数和状态量进行计算
 */
void Class_PID::Control_Cascade(void)
{
	Control_Angle_To_Speed();
	Control_Speed_To_Out();
}

/**
 * @brief PID重置 目前只有重置积分项
 * 
 */
void Class_PID::Reset(void)
{
	Set_Angle_Target(0);
	Angle_States.ErrorInt = 0;
	Angle_States.Error0 = 0;
	Angle_States.Error1 = 0;

	Set_Speed_Target(0);
	Speed_States.ErrorInt = 0;
	Speed_States.Error0 = 0;
	Speed_States.Error1 = 0;
}

//--------------------------------------------------------------------


/**
 * @brief PID控制 速度环
 * 
 * @param PID_Object PID控制对象指针，包含配置参数和状态
 */
void PID_Control_Speed(PID_Object_t *PID_Object)
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
void PID_Control_Angle(PID_Object_t *PID_Object)
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


/**
 * @brief PID设置角度目标
 * 
 * @param PID_Object 
 * @param Angle_Target 
 */
void PID_Set_Angle_Target(PID_Object_t *PID_Object, float Angle_Target)
{
	PID_Object->Angle_Target = Angle_Target;
}

/**
 * @brief PID设置速度目标
 * 
 * @param PID_Object 
 * @param Speed_Target 
 */
void PID_Set_Speed_Target(PID_Object_t *PID_Object, float Speed_Target)
{
	PID_Object->Speed_Target = Speed_Target;
}


/**
 * @brief 限幅函数
 * 
 * @param value 传入值
 * @param low 最低
 * @param high 最高
 * @return float 限幅后值
 */
float Class_PID::Limit(float value, float low, float high)
{
    if (value > high)
    {
        return high;
    }

    if (value < low)
    {
        return low;
    }

    return value;
}
