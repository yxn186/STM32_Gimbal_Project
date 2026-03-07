/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_bmi088.c
  * @brief   bmi088 app层
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "app_bmi088.h"
#include "bmi088.h"
#include "bmi088reg.h"
#include "joled.h"
#include "spi.h"
#include <math.h>
#include <stdint.h>
#include "app_bmi088_math.h"
#include "Serial.h"
#include "gimbal_task.h"
#include "MahonyAHRS.h"
#include "MyMath.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "MyRTOS.h"

osThreadId_t imu_calculate_ThreadIdHandle = NULL;
TaskHandle_t imu_calculate_TaskHandle = NULL;

bmi088_handle_t bmi088_handle;

float roll = 0,pitch = 0,yaw = 0;

static uint32_t bmi088_init_process_time;


static uint8_t bmi088_readid_acc_flag = 0;
static uint8_t read_accid = 0;
static uint8_t bmi088_readid_gyro_flag = 0;
static uint8_t read_gyroid = 0;

static uint8_t bmi088_gyro_get_raw_data_finished_flag;
static uint8_t bmi088_acc_get_raw_data_finished_flag;
static uint8_t bmi088_acc_get_raw_data_flag;
static uint8_t bmi088_gyro_get_raw_data_flag;
float gimbal_pitch = 0,gimbal_yaw = 0;

//检查glag
uint8_t checkid_flag = 0;
uint8_t writereg_flag = 0;


uint32_t last_time1 = 0;
uint32_t last_time2 = 0;
uint32_t last_time3 = 0;
uint32_t last_time4 = 0;

//位通知用
#define BMI088_Start_Get_Data   (1U << 0)
#define BMI088_Get_Acc_OK       (1U << 1)
#define BMI088_Get_Gyro_OK      (1U << 2)

typedef enum
{
    BMI088_Start_Get_Data_State = 0,
    BMI088_Wait_Get_Acc_Data_State,
    BMI088_Wait_Get_Gyro_Data_State,
} bmi088_calculate_task_state_e;

bmi088_calculate_task_state_e bmi088_calculate_task_state = BMI088_Start_Get_Data_State;
 
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

bmi088_init_state_e bmi088_init_state = init_state_start;

typedef struct
{
    int16_t gyro_raw_x;
    int16_t gyro_raw_y;
    int16_t gyro_raw_z;

    int16_t acc_raw_x;
    int16_t acc_raw_y;
    int16_t acc_raw_z;

}bmi088_data_t;

bmi088_data_t bmi088_data;

/* ---回调函数---------------------------------------------------------------------------------------- */
void bmi088_acc_get_raw_data_finished_init(int16_t acc_raw_x,int16_t acc_raw_y,int16_t acc_raw_z)
{
    bmi088_data.acc_raw_x = acc_raw_x;
    bmi088_data.acc_raw_y = acc_raw_y;
    bmi088_data.acc_raw_z = acc_raw_z;

    bmi088_acc_get_raw_data_finished_flag = 1;
    bmi088_gyro_get_raw_data_flag = 1;
}

void bmi088_gyro_get_raw_data_finished_init(int16_t gyro_raw_x,int16_t gyro_raw_y,int16_t gyro_raw_z)
{
    bmi088_data.gyro_raw_x = gyro_raw_x;
    bmi088_data.gyro_raw_y = gyro_raw_y;
    bmi088_data.gyro_raw_z = gyro_raw_z;

    bmi088_gyro_get_raw_data_finished_flag = 1;
}

void bmi088_acc_get_raw_data_finished(int16_t acc_raw_x,int16_t acc_raw_y,int16_t acc_raw_z)
{
    bmi088_data.acc_raw_x = acc_raw_x;
    bmi088_data.acc_raw_y = acc_raw_y;
    bmi088_data.acc_raw_z = acc_raw_z;
    //任务通知
    MyRTOS_TaskNotify_ISR_SetBits(imu_calculate_TaskHandle, BMI088_Get_Acc_OK);
}

void bmi088_gyro_get_raw_data_finished(int16_t gyro_raw_x,int16_t gyro_raw_y,int16_t gyro_raw_z)
{
    bmi088_data.gyro_raw_x = gyro_raw_x;
    bmi088_data.gyro_raw_y = gyro_raw_y;
    bmi088_data.gyro_raw_z = gyro_raw_z;

    //任务通知
    MyRTOS_TaskNotify_ISR_SetBits(imu_calculate_TaskHandle, BMI088_Get_Gyro_OK);
}

static void bmi088_readid_acc_finished(uint8_t accid)
{
    read_accid = accid;
    bmi088_readid_acc_flag = 1;
}

static void bmi088_readid_gyro_finished(uint8_t gyroid)
{
    read_gyroid = gyroid;
    bmi088_readid_gyro_flag = 1;
}
/* ---回调函数---------------------------------------------------------------------------------------- */


/**
 * @brief BMI088初始化
 * 
 */
void app_bmi088_init(void)
{
    bmi088_init(&bmi088_handle, &hspi1, GPIOC, GPIO_PIN_4, GPIO_PIN_RESET, GPIOB, GPIO_PIN_1, GPIO_PIN_RESET, GPIOB, GPIO_PIN_0, GPIOC, GPIO_PIN_5);
    
    
    bmi088_init_state = init_state_accsoftrest;
    
    STM32_Printf("start_init_bmi088\r\n");
}

/**
 * @brief BMI088读accid任务
 * 
 */
void app_bmi088_read_accid_task(void)
{
    bmi088_readid_acc(&bmi088_handle, bmi088_readid_acc_finished);
}

/**
 * @brief BMI088读gyroid任务
 * 
 */
void app_bmi088_read_gyroid_task(void)
{
    bmi088_readid_gyro(&bmi088_handle, bmi088_readid_gyro_finished);
}

/**
 * @brief BMI088得到触发标志 获取一次acc原始数据
 * 
 */
void app_bmi088_getflag_to_read_acc_raw_data(void)
{
    bmi088_acc_get_raw_data_flag = 0;
    bmi088_get_acc_raw_data(&bmi088_handle, bmi088_acc_get_raw_data_finished_init);
}

/**
 * @brief BMI088得到触发标志 获取一次gyro原始数据
 * 
 */
void app_bmi088_getflag_to_read_gyro_raw_data(void)
{
    bmi088_gyro_get_raw_data_flag = 0;
    bmi088_get_gyro_raw_data(&bmi088_handle, bmi088_gyro_get_raw_data_finished_init);
}

/**
 * @brief BMI088 初始化过程循环函数
 * 
 * @return uint8_t 完成标志
 */
uint8_t app_bmi088_init_process_loop(void)
{
    if(bmi088_init_state == init_state_finish) return 1;

    if(bmi088_init_state == init_state_accsoftrest)//acc软复位
    {
        STM32_Printf("accsoftrest ing...\r\n");

        bmi088_acc_softreset(&bmi088_handle,NULL);

        bmi088_init_process_time = HAL_GetTick();
        bmi088_init_state = init_state_gyrosoftrest;
    }
    else if(bmi088_init_state == init_state_gyrosoftrest)//gyro软复位
    {
        if(HAL_GetTick() - bmi088_init_process_time >= 10)//等待1ms以上软复位完成
        {
            STM32_Printf("gyrosoftrest ing...\r\n");

            bmi088_gyro_softreset(&bmi088_handle,NULL);
            bmi088_init_process_time = HAL_GetTick();
            bmi088_init_state = init_state_acc_dummyread;
        }
    }
    else if(bmi088_init_state == init_state_acc_dummyread)//acc舍去一次无效读写
    {
        if(HAL_GetTick() - bmi088_init_process_time >= 40)//等待30ms以上软复位完成
        {
            STM32_Printf("acc舍去一次无效读写 ing...\r\n");

            app_bmi088_read_accid_task();
            bmi088_init_state = init_state_readaccidtocheck;
        }
    }
    else if(bmi088_init_state == init_state_readaccidtocheck)//读accid
    {
        if(bmi088_readid_acc_flag)
        {
            STM32_Printf("read acc id ing...\r\n");

            bmi088_readid_acc_flag = 0;

            bmi088_readid_acc(&bmi088_handle, bmi088_readid_acc_finished);
            bmi088_init_state = init_state_readgyroidtocheck;
        }
    }
    else if(bmi088_init_state == init_state_readgyroidtocheck)//读gyroid
    {
        if(bmi088_readid_acc_flag)
        {
            bmi088_readid_acc_flag = 0;
            STM32_Printf("read gyro id ing...\r\n");

            app_bmi088_read_gyroid_task();
            bmi088_init_state = init_state_finishidcheck;
        }
    }
    else if(bmi088_init_state == init_state_finishidcheck)//检查读取的id
    {
        if(bmi088_readid_gyro_flag)
        {
            bmi088_readid_gyro_flag = 0;
            if(read_accid == BMI088_ACC_CHIP_ID_VALUE && read_gyroid == BMI088_GYRO_CHIP_ID_VALUE)
            {
                STM32_Printf("Check acc and gyro ID is correct!\r\n");
                checkid_flag = 1;
            }
            else 
            {
                STM32_Printf("Check acc and gyro ID Error!\r\n");
            }
            STM32_Printf("Start write reg!\r\n");
            bmi088_init_state = init_state_startconfigreg;
        }
    }
    else if(bmi088_init_state ==  init_state_startconfigreg)//配置寄存器
    {
        bmi088_start(&bmi088_handle);//内部阻塞式配置寄存器，直到配置完成才返回

        STM32_Printf("write reg finish!\r\n");

        writereg_flag = 1;
        bmi088_init_state = init_state_check_data;

        if(writereg_flag && checkid_flag)
        {
            JOLED_Clear();
            JOLED_ShowString(4, 1, "BOK");
            STM32_Printf("BMI088 init all OK!\r\n");
        }
        else 
        {   
            JOLED_Clear();
            JOLED_ShowString(4, 1, "BERR");
            STM32_Printf("BMI088 init Error!\r\n");
        }
    }
    else if(bmi088_init_state == init_state_check_data)//开始零偏校准
    {
        JOLED_ShowString(1, 1, "wait");
        STM32_Printf("wait for bias calibration!\r\n");

        bmi088_biascalibration_start(100);
        bmi088_init_state = init_state_wait_check_data;

        last_time4 = HAL_GetTick();
    }
    else if(bmi088_init_state == init_state_wait_check_data)//等待零偏校准完成
    {
        if(bmi088_acc_get_raw_data_flag)
        {
            app_bmi088_getflag_to_read_acc_raw_data();
        }
        if(bmi088_gyro_get_raw_data_flag)
        {
            app_bmi088_getflag_to_read_gyro_raw_data();
        }

        //二者均读取完成 推送一次数据进行零偏计算
        if(bmi088_gyro_get_raw_data_finished_flag)
        {
            if(bmi088_acc_get_raw_data_finished_flag)
            {
                bmi088_gyro_get_raw_data_finished_flag = 0;
                bmi088_acc_get_raw_data_finished_flag = 0;
                bmi088_biascalibration_pushsampletocalculate(bmi088_data.gyro_raw_x,bmi088_data.gyro_raw_y,bmi088_data.gyro_raw_z,bmi088_data.acc_raw_x,bmi088_data.acc_raw_y,bmi088_data.acc_raw_z);
            }
        }
        
        //处理进度通知
        if(HAL_GetTick() - last_time3 >= 15)
        {
            STM32_Printf("now get %d effective samples\r\n", bmi088_getbiascalibration_current_samples_effective());
           
            STM32_Printf("speed: %.2f samples/s\r\n", (float)(bmi088_getbiascalibration_current_samples()  / ((float)(HAL_GetTick() - last_time4)/1000.0f)));   

            STM32_Printf("processing... %.2f %%\r\n", (float)bmi088_getbiascalibration_current_samples_effective()/(float)bmi088_getbiascalibration_target_samples()*100.0f);

            last_time3 = HAL_GetTick();
        }

        //检测零偏校准完成标志
        if(bmi088_get_biascalibration_finish_flag())
        {
            if(imu_calculate_TaskHandle == NULL)
            {
                bmi088_freertos_init();
            }

            bmi088_init_state = init_state_finish;
            JOLED_ShowString(1, 1, "ok get:              ");
            STM32_Printf("bias calibration finish!\r\n");

            STM32_Printf("Get %d effective samples use %.2f s\r\n", bmi088_getbiascalibration_current_samples_effective(), (float)(HAL_GetTick() - last_time4)/1000.0f);
            
            JOLED_ShowNum(1, 8, bmi088_getbiascalibration_current_samples_effective(), 5);
            return 1;
        }
    }
    return 0;
}

/**
 * @brief BMI088 1ms周期任务 函数
 * 
 */
void app_bmi088_1ms_task(PID_t *Pitch, PID_t *Yaw)
{   
    if(bmi088_init_state == init_state_finish)//完成则进入任务循环
    {
        //ZXY欧拉角转换为云台前向的pitch和yaw
        euler_extrinsic_ZXY_to_front_yaw_pitch_deg(yaw,roll,pitch,&gimbal_yaw,&gimbal_pitch);
        Pitch->PID_Angle_Status.Current_Angle = gimbal_pitch;
        Yaw->PID_Angle_Status.Current_Angle = gimbal_yaw;
    }
}

/**
 * @brief BMI088 20ms周期任务 函数
 * 
 */
void app_bmi088_20ms_task(void)
{   
    if(bmi088_init_state == init_state_finish)//完成则进入任务循环
    {
        STM32_Printf("%.8f,%.8f,%.8f,%.8f,%.8f\r\n",roll,pitch,yaw,gimbal_pitch,gimbal_yaw);
    }
}



/* BMI088 FreeRTOS相关----------------------------------------------------------------*/


const osThreadAttr_t imu_calculate_attributes = {
  .name = "imu_calculate",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime5,
};

/**
 * @brief BMI088 FreeRTOS初始化
 * 
 */
void bmi088_freertos_init(void)
{
    imu_calculate_ThreadIdHandle = osThreadNew(bmi088_calculate_task, NULL, &imu_calculate_attributes);
    imu_calculate_TaskHandle = (TaskHandle_t)imu_calculate_ThreadIdHandle;
}

void bmi088_calculate_task(void *argument)
{
    for(;;)
    {
        if(bmi088_init_state == init_state_finish)
        {
            uint32_t notify_value = MyRTOS_TaskNotifyWait();
            //等待通知
            if((notify_value & BMI088_Start_Get_Data) && (bmi088_calculate_task_state == BMI088_Start_Get_Data_State))
            {
                //等到通知 读一次acc数据
                if(bmi088_get_acc_raw_data(&bmi088_handle, bmi088_acc_get_raw_data_finished))
                {
                    bmi088_calculate_task_state = BMI088_Wait_Get_Acc_Data_State;
                }
            }
            //等待通知
            if((notify_value & BMI088_Get_Acc_OK) && (bmi088_calculate_task_state == BMI088_Wait_Get_Acc_Data_State))
            {
                //等到通知 读一次gyro数据 同时也
                if(bmi088_get_gyro_raw_data(&bmi088_handle, bmi088_gyro_get_raw_data_finished))
                {
                    bmi088_calculate_task_state = BMI088_Wait_Get_Gyro_Data_State;
                }
                else 
                {
                    bmi088_calculate_task_state = BMI088_Start_Get_Data_State;
                }
            } 
            //等待通知
            if((notify_value & BMI088_Get_Gyro_OK) && (bmi088_calculate_task_state == BMI088_Wait_Get_Gyro_Data_State))
            {
                //得到zxy欧拉角 配合vofa显示
                bmi088_mahony_zxy(bmi088_data.gyro_raw_x,bmi088_data.gyro_raw_y,bmi088_data.gyro_raw_z,bmi088_data.acc_raw_x,bmi088_data.acc_raw_y,bmi088_data.acc_raw_z);
                roll = BMI088_GetRollDeg();
                pitch = BMI088_GetPitchDeg();
                yaw = BMI088_GetYawDeg();
                bmi088_calculate_task_state = BMI088_Start_Get_Data_State;
            } 
        } 
    }
}

/**
 * @brief GPIO外部中断回调函数 由BMI088 INT1引脚触发 ACC数据准备好中断
 * 
 * @param GPIO_Pin 
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_0)//acc数据准备好中断
    {
        if(bmi088_init_state == init_state_finish)
        {
            if(imu_calculate_TaskHandle != NULL)
            {
                //任务通知
                MyRTOS_TaskNotify_ISR_SetBits(imu_calculate_TaskHandle, BMI088_Start_Get_Data);
            }
        }
        if(bmi088_init_state == init_state_check_data || bmi088_init_state == init_state_wait_check_data)
        {
            bmi088_acc_get_raw_data_flag = 1;
        }
    }
}
