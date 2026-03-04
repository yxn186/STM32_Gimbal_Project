/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    DJI_Motor.h
  * @brief   This file contains all the function prototypes for
  *          the DJI_Motor.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DJI_MOTOR_H__
#define __DJI_MOTOR_H__

#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bsp_can.h"
/*YOUR CODE*/

/* DJI电机ID宏定义 */
//目前似乎没什么用
#define ID1_3508 0x201
#define ID2_3508 0x202
#define ID3_3508 0x203
#define ID4_3508 0x204
#define ID5_3508 0x205
#define ID6_3508 0x206
#define ID7_3508 0x207
#define ID8_3508 0x208

#define ID1_6020 0x205
#define ID2_6020 0x206
#define ID3_6020 0x207
#define ID4_6020 0x208
#define ID5_6020 0x209
#define ID6_6020 0x20A
#define ID7_6020 0x20B

/**
 * @brief 大疆系电机型号枚举
 * 
 */
typedef enum
{
    DJI_Motor_6020,
    DJI_Motor_3508
}DJI_Motor_Type_Typedef;

/**
 * @brief 大疆系电机数据结构体
 * 
 */
typedef struct
{
    uint16_t RawAngle;       //机械角度（0~8191）
    int16_t speed_rpm;      //转速（RPM）
    int16_t Torque_Current; //转矩电流
    uint8_t Temperature;    //电机温度
}DJI_Motor_Data_t;


/**
 * @brief 大疆电机初始化函数
 * 
 * @param hcan hcanx
 * @param DJI_Motors_Data 电机数据结构体数组[8]（一定要是8！！）
 */
void DJI_Motor_Init(CAN_HandleTypeDef *hcan,DJI_Motor_Data_t DJI_Motors_Data[8]);

/**
 * @brief 大疆单电机控制函数
 * 
 * @param hcan hcanx
 * @param DJI_Motor_Type 电机型号 DJI_Motor_6020 / DJI_Motor_3508
 * @param DJI_Motor_ID 电机CAN-ID  6020：1~7   3508：1~8
 * @param Out 输出值 6020电压控制：-25000~0~25000   3508电流控制：-16384~0~16384
 */
void DJI_Motor_Control_Single(CAN_HandleTypeDef *hcan,DJI_Motor_Type_Typedef DJI_Motor_Type,uint16_t DJI_Motor_ID,int16_t Out);

/**
 * @brief 大疆电机获得角度函数
 * 
 * @param DJI_Motor_ID 电机CAN-ID  6020：1~7   3508：1~8
 * @return float 角度（360度）单位：Deg
 */
float DJI_Motor_Get_Angle(uint8_t DJI_Motor_ID);

/**
 * @brief 大疆电机获得角速度函数
 * 
 * @param DJI_Motor_ID 电机CAN-ID  6020：1~7   3508：1~8
 * @return float 角速度 rad/s
 */
float DJI_Motor_Get_AngleSpeed(uint8_t DJI_Motor_ID);

#ifdef __cplusplus
}
#endif

#endif /* __DJI_MOTOR_H__ */
