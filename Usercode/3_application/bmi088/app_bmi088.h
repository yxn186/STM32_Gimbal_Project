/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_bmi088.h
  * @brief   This file contains all the function prototypes for
  *          the app_bmi088.c file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_BMI088_H__
#define __APP_BMI088_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "PID.h"

/*YOUR CODE*/

/**
 * @brief bmi088初始化进程枚举
 * 
 */
typedef enum
{
    init_state_start = 0,
    init_state_accsoftrest,
    init_state_gyrosoftrest,
    init_state_acc_dummyread,
    init_state_readaccidtocheck,
    init_state_readgyroidtocheck,
    init_state_finishidcheck,
    init_state_startconfigreg,
    init_state_check_data,
    init_state_wait_check_data,
    init_state_finish
} bmi088_init_state_e;

/**
 * @brief bmi088计算任务状态枚举
 * 
 */
typedef enum
{
    BMI088_Start_Get_Data_State = 0,
    BMI088_Wait_Get_Acc_Data_State,
    BMI088_Wait_Get_Gyro_Data_State,
} bmi088_calculate_task_state_e;

/**
 * @brief bmi088数据结构体
 * 
 */
typedef struct
{
    int16_t gyro_raw_x;
    int16_t gyro_raw_y;
    int16_t gyro_raw_z;

    int16_t acc_raw_x;
    int16_t acc_raw_y;
    int16_t acc_raw_z;

}bmi088_data_t;

/**
 * @brief BMI088初始化
 * 
 */
void app_bmi088_init(void);

/**
 * @brief BMI088 初始化过程循环函数
 * 
 * @return uint8_t 完成标志
 */
uint8_t app_bmi088_init_process_loop(void);


/**
 * @brief BMI088 20ms周期任务 函数
 * 
 */
void app_bmi088_20ms_task(void);

/**
 * @brief BMI088 FreeRTOS初始化
 * 
 */
void bmi088_freertos_init(void);

/**
 * @brief bmi088 freertos任务函数 计算 
 * 
 * @param argument 
 */
void bmi088_calculate_task(void *argument);

/**
 * @brief BMI088 1ms周期任务 函数
 * 
 */
void app_bmi088_1ms_task_get_now_pitch_and_yaw(float *Yaw,float *Picth);

#ifdef __cplusplus
}
#endif

#endif /* __APP_BMI088_H__ */
