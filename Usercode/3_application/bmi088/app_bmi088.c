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
#include "MahonyAHRS.h"
#include "MyMath.h"


bmi088_handle_t bmi088_handle;

float roll = 0,pitch = 0,yaw = 0;
float x,y;
float pitch_x,pitch_y,pitch_z,roll_x,roll_y,roll_z;
static uint32_t last_time;
uint32_t last_time4 = 0;

static uint8_t bmi088_readid_acc_flag = 0;
static uint8_t read_accid = 0;
static uint8_t bmi088_readid_gyro_flag = 0;
static uint8_t read_gyroid = 0;

static uint8_t bmi088_gyro_get_raw_data_finished_flag;
static uint8_t bmi088_acc_get_raw_data_finished_flag;
static uint8_t bmi088_acc_get_raw_data_flag;
static uint8_t bmi088_gyro_get_raw_data_flag;

//检查glag
uint8_t checkid_flag = 0;
uint8_t writereg_flag = 0;

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

void bmi088_acc_get_raw_data_finished(int16_t acc_raw_x,int16_t acc_raw_y,int16_t acc_raw_z)
{
    bmi088_data.acc_raw_x = acc_raw_x;
    bmi088_data.acc_raw_y = acc_raw_y;
    bmi088_data.acc_raw_z = acc_raw_z;
    bmi088_gyro_get_raw_data_flag = 1;
    bmi088_acc_get_raw_data_finished_flag = 1;
}

void bmi088_gyro_get_raw_data_finished(int16_t gyro_raw_x,int16_t gyro_raw_y,int16_t gyro_raw_z)
{
    bmi088_data.gyro_raw_x = gyro_raw_x;
    bmi088_data.gyro_raw_y = gyro_raw_y;
    bmi088_data.gyro_raw_z = gyro_raw_z;

    bmi088_gyro_get_raw_data_finished_flag = 1;
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

/**
 * @brief BMI088初始化
 * 
 */
void app_bmi088_init(void)
{
    bmi088_init(&bmi088_handle, &hspi1, GPIOC, GPIO_PIN_4, GPIO_PIN_RESET, GPIOB, GPIO_PIN_1, GPIO_PIN_RESET, GPIOB, GPIO_PIN_0, GPIOC, GPIO_PIN_5);
    bmi088_init_state = init_state_accsoftrest;
    JOLED_ShowString(2, 1, "start_init_bmi088");
    Serial_Printf("start_init_bmi088\r\n");
}

void app_bmi088_task1(void)
{
    bmi088_readid_acc(&bmi088_handle, bmi088_readid_acc_finished);
}

void app_bmi088_task2(void)
{
    bmi088_readid_gyro(&bmi088_handle, bmi088_readid_gyro_finished);
}

void app_bmi088_init_process_loop(void)
{
    if(bmi088_init_state == init_state_finish) return;

    if(bmi088_init_state == init_state_accsoftrest)//acc软复位
    {
        JOLED_ShowString(2, 1, "start_init_1        ");
        Serial_Printf("accsoftrest ing...\r\n");

        bmi088_acc_softreset(&bmi088_handle,NULL);
        last_time = HAL_GetTick();
        bmi088_init_state = init_state_gyrosoftrest;
    }
    else if(bmi088_init_state == init_state_gyrosoftrest)//gyro软复位
    {
        if(HAL_GetTick() - last_time >= 20)//等待1ms以上软复位完成
        {
            Serial_Printf("gyrosoftrest ing...\r\n");

            JOLED_ShowString(2, 1, "start_init_2      ");
            bmi088_gyro_softreset(&bmi088_handle,NULL);
            last_time = HAL_GetTick();
            bmi088_init_state = init_state_acc_dummyread;
        }
    }
    else if(bmi088_init_state == init_state_acc_dummyread)//acc舍去一次无效读写
    {
        if(HAL_GetTick() - last_time >= 40)//等待30ms以上软复位完成
        {
            Serial_Printf("acc舍去一次无效读写 ing...\r\n");

            JOLED_ShowString(2, 1, "start_init_3      ");
            bmi088_readid_acc(&bmi088_handle, bmi088_readid_acc_finished);
            last_time = HAL_GetTick();
            bmi088_init_state = init_state_readaccidtocheck;
        }
    }
    else if(bmi088_init_state == init_state_readaccidtocheck)//读accid
    {
        if(bmi088_readid_acc_flag)
        {
            Serial_Printf("read acc id ing...\r\n");

            JOLED_ShowString(2, 1, "start_init_4        ");
            bmi088_readid_acc(&bmi088_handle, bmi088_readid_acc_finished);
            bmi088_init_state = init_state_readgyroidtocheck;
            bmi088_readid_acc_flag = 0;
        }
    }
    else if(bmi088_init_state == init_state_readgyroidtocheck)//读gyroid
    {
        if(bmi088_readid_acc_flag)
        {
            Serial_Printf("read gyro id ing...\r\n");

            JOLED_ShowHexNum(1, 1, read_accid, 2);
            bmi088_readid_gyro(&bmi088_handle, bmi088_readid_gyro_finished);
            bmi088_init_state = init_state_finishidcheck;
            bmi088_readid_acc_flag = 0;
        }
    }
    else if(bmi088_init_state == init_state_finishidcheck)//检查读取的id
    {
        if(bmi088_readid_gyro_flag)
        {
            JOLED_ShowHexNum(1, 4, read_gyroid, 2);
            bmi088_readid_gyro_flag = 0;
            if(read_accid == BMI088_ACC_CHIP_ID_VALUE && read_gyroid == BMI088_GYRO_CHIP_ID_VALUE)
            {
                Serial_Printf("Check acc and gyro ID OK!\r\n");
                
                JOLED_ShowString(1, 1, "  Check ID OK!   ");
                checkid_flag = 1;
            }
            else 
            {
                Serial_Printf("Check acc and gyro ID Error!\r\n");

                JOLED_ShowString(1, 1, "!Check ID Error!");
            }
            JOLED_ShowString(2, 1, "Start write reg");
            Serial_Printf("Start write reg!\r\n");
            bmi088_init_state = init_state_startconfigreg;
        }
    }
    else if(bmi088_init_state ==  init_state_startconfigreg)//配置寄存器
    {
        bmi088_start(&bmi088_handle);
        JOLED_ShowString(2, 1, "  write reg ok ");
        Serial_Printf("write reg finish!\r\n");

        writereg_flag = 1;
        bmi088_init_state = init_state_check_data;
        if(writereg_flag && checkid_flag)
        {
            JOLED_Clear();
            JOLED_ShowString(4, 1, "BOK");
            Serial_Printf("BMI088 init OK!\r\n");
        }
    }
    else if(bmi088_init_state == init_state_check_data)
    {
        JOLED_ShowString(1, 1, "wait");
        Serial_Printf("wait for bias calibration!\r\n");

        bmi088_biascalibration_start(1000);
        bmi088_init_state = init_state_wait_check_data;
        last_time4 = HAL_GetTick();
    }
}



uint32_t last_time1 = 0;
uint32_t last_time2 = 0;
uint32_t last_time3 = 0;


/**
 * @brief BMI088 循环函数
 * 
 */
void app_bmi088_loop(void)
{
    app_bmi088_init_process_loop();

    if(bmi088_init_state == init_state_wait_check_data)
    {
        if(bmi088_acc_get_raw_data_flag)
        {
            bmi088_acc_get_raw_data_flag = 0;
            bmi088_get_acc_raw_data(&bmi088_handle, bmi088_acc_get_raw_data_finished);
        }
        if(bmi088_gyro_get_raw_data_flag)
        {
            bmi088_gyro_get_raw_data_flag = 0;
            bmi088_get_gyro_raw_data(&bmi088_handle, bmi088_gyro_get_raw_data_finished);
        }
        if(bmi088_gyro_get_raw_data_finished_flag)
        {
            if(bmi088_acc_get_raw_data_finished_flag)
            {
                bmi088_gyro_get_raw_data_finished_flag = 0;
                bmi088_acc_get_raw_data_finished_flag = 0;
                bmi088_biascalibration_pushsampletocalculate(bmi088_data.gyro_raw_x,bmi088_data.gyro_raw_y,bmi088_data.gyro_raw_z,bmi088_data.acc_raw_x,bmi088_data.acc_raw_y,bmi088_data.acc_raw_z);
            }
        }
        
        if(HAL_GetTick() - last_time3 >= 15)
        {
            Serial_Printf("now get %d effective samples\r\n", bmi088_getbiascalibration_current_samples_effective());
           
            Serial_Printf("speed: %.2f samples/s\r\n", (float)(bmi088_getbiascalibration_current_samples()  / ((float)(HAL_GetTick() - last_time4)/1000.0f)));   

            Serial_Printf("processing... %.2f %%\r\n", (float)bmi088_getbiascalibration_current_samples_effective()/(float)bmi088_getbiascalibration_target_samples()*100.0f);
            
            //JOLED_ShowNum(1, 10, bmi088_getbiascalibration_current_samples_effective(), 3);
            //JOLED_ShowNum(1, 6, bmi088_getbiascalibration_current_samples(), 3);
            last_time3 = HAL_GetTick();
        }

        if(bmi088_get_biascalibration_finish_flag())
        {
            bmi088_init_state = init_state_finish;
            JOLED_ShowString(1, 1, "ok get:              ");
            Serial_Printf("bias calibration finish!\r\n");


            Serial_Printf("Get %d effective samples use %.2f s\r\n", bmi088_getbiascalibration_current_samples_effective(), (float)(HAL_GetTick() - last_time4)/1000.0f);
            
            JOLED_ShowNum(1, 8, bmi088_getbiascalibration_current_samples_effective(), 5);
            
        }
        
    }
    if(bmi088_init_state == init_state_finish)
    {
        if(bmi088_acc_get_raw_data_flag)
        {
            bmi088_acc_get_raw_data_flag = 0;
            bmi088_get_acc_raw_data(&bmi088_handle, bmi088_acc_get_raw_data_finished);
        }
        if(bmi088_gyro_get_raw_data_flag)
        {
            bmi088_gyro_get_raw_data_flag = 0;
            bmi088_get_gyro_raw_data(&bmi088_handle, bmi088_gyro_get_raw_data_finished);
        }
        if(HAL_GetTick() - last_time1 >= 2)
        {
            last_time1 += 2;
            //bmi088_complementaryfilter_1(bmi088_data.gyro_raw_x,bmi088_data.gyro_raw_y,bmi088_data.gyro_raw_z,bmi088_data.acc_raw_x,bmi088_data.acc_raw_y,bmi088_data.acc_raw_z,0.005);
            bmi088_mahony_zxy(bmi088_data.gyro_raw_x,bmi088_data.gyro_raw_y,bmi088_data.gyro_raw_z,bmi088_data.acc_raw_x,bmi088_data.acc_raw_y,bmi088_data.acc_raw_z);
            roll = BMI088_GetRollDeg();
            pitch = BMI088_GetPitchDeg();
            yaw = BMI088_GetYawDeg();
        }
        if(HAL_GetTick() - last_time2 >= 15)
        {

            last_time2 += 15; 
            float gimbal_pitch,gimbal_yaw;

            euler_extrinsic_ZXY_to_front_yaw_pitch_deg(yaw,roll,pitch,&gimbal_yaw,&gimbal_pitch);
            Serial_Printf("%.8f,%.8f,%.8f,%.8f,%.8f\r\n",roll,pitch,yaw,gimbal_pitch,gimbal_yaw);

        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_0)
    {
        if(bmi088_init_state == init_state_finish || bmi088_init_state == init_state_check_data || bmi088_init_state == init_state_wait_check_data)
        {
            bmi088_acc_get_raw_data_flag = 1;  
        }
    }
}