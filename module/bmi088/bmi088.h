/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bmi088.h
  * @brief   This file contains all the function prototypes for
  *          the bmi088.cpp file
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BMI088_H__
#define __BMI088_H__


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "bsp_spi.h"
#include "bmi088reg.h"
/*YOUR CODE*/

/* 回调函数 */
typedef void (*bmi088_readid_acc_finishedfunction)(uint8_t acc_id);
typedef void (*bmi088_readid_gyro_finishedfunction)(uint8_t gyro_id);

typedef void (*bmi088_acc_read_reg_finishedfunction)(uint8_t read_reg_address, uint8_t *rx_data, uint16_t rx_length);
typedef void (*bmi088_gyro_read_reg_finishedfunction)(uint8_t read_reg_address, uint8_t *rx_data, uint16_t rx_length);

typedef void (*bmi088_acc_write_reg_finishedfunction)(uint8_t write_reg_address,uint8_t *tx_data,uint16_t tx_length);
typedef void (*bmi088_gyro_write_reg_finishedfunction)(uint8_t write_reg_address,uint8_t *tx_data,uint16_t tx_length);

/**
 * @brief GYRO 原始角速度回调函数（返回三轴 int16 原始值）
 * 
 * @param gyro_raw_x 原始X轴角速度
 * @param gyro_raw_y 原始Y轴角速度
 * @param gyro_raw_z 原始Z轴角速度
 */
typedef void (*bmi088_gyro_get_raw_data_finishedfunction)(int16_t gyro_raw_x,int16_t gyro_raw_y,int16_t gyro_raw_z);

/**
 * @brief ACC 原始加速度回调函数（返回三轴 int16 原始值）
 * 
 * @param acc_raw_x 原始X轴加速度
 * @param acc_raw_y 原始Y轴加速度
 * @param acc_raw_z 原始Z轴加速度
 */
typedef void (*bmi088_acc_get_raw_data_finishedfunction)(int16_t acc_raw_x,int16_t acc_raw_y,int16_t acc_raw_z);


/**
 * @brief BMI088运行状态
 * 
 */
typedef enum
{
    bmi088_no_operation = 0,

    bmi088_reading_acc_reg,
    bmi088_reading_gyro_reg,

    bmi088_writing_acc_reg,
    bmi088_writing_gyro_reg,

} bmi088_current_operation;

/**
 * @brief bmi088句柄
 * 
 */
typedef struct
{
    SPI_HandleTypeDef *hspi;                    //hspix

    GPIO_TypeDef *csb1_acc_gpiox;                   //CSB1-GPIOx
    uint16_t csb1_acc_pin;                          //CSB1-Pin
    GPIO_PinState csb1_acc_gpio_pinstate;           //CSB1-引脚状态

    GPIO_TypeDef *csb2_gyro_gpiox;                   //CSB2-GPIOx
    uint16_t csb2_gyro_pin;                          //CSB2-Pin
    GPIO_PinState csb2_gyro_gpio_pinstate;           //CSB2-引脚状态

    GPIO_TypeDef *int1_acc_gpiox;                   //INT1-GPIOx
    uint16_t int1_acc_pin;                          //INT1-Pin

    GPIO_TypeDef *int3_gyro_gpiox;                   //INT3-GPIOx
    uint16_t int3_gyro_pin;                          //INT3-Pin

    bmi088_current_operation current_operation;

    uint8_t tx_buffer[260];
    uint8_t rx_buffer[260];

    uint8_t acc_id;
    uint8_t gyro_id;

    uint8_t  *acc_read_reg_rx_buffer;
    uint16_t  acc_read_reg_rx_length;
    uint8_t   acc_read_reg_address;

    uint8_t  *gyro_read_reg_rx_buffer;
    uint16_t  gyro_read_reg_rx_length;
    uint8_t   gyro_read_reg_address;

    uint8_t   acc_write_reg_address;
    uint16_t  acc_write_reg_tx_length;
    bmi088_acc_write_reg_finishedfunction acc_write_reg_finishedfunction;

    uint8_t   gyro_write_reg_address;
    uint16_t  gyro_write_reg_tx_length;
    bmi088_gyro_write_reg_finishedfunction gyro_write_reg_finishedfunction;

    bmi088_acc_read_reg_finishedfunction acc_read_reg_finishedfunction;
    bmi088_gyro_read_reg_finishedfunction gyro_read_reg_finishedfunction;

    bmi088_readid_acc_finishedfunction readid_acc_finishedfunction;
    bmi088_readid_gyro_finishedfunction readid_gyro_finishedfunction;

    //GET DATA  
    bmi088_gyro_get_raw_data_finishedfunction gyro_get_raw_data_finishedfunction;
    uint8_t gyro_raw_data_rx_buffer[6];

    bmi088_acc_get_raw_data_finishedfunction acc_get_raw_data_finishedfunction;
    uint8_t acc_raw_data_rx_buffer[6];

}bmi088_handle_t;

/**
 * @brief BMI088初始化
 * 
 * @param handle BMI088句柄指针
 * @param hspi hspix
 * @param csb1_acc_gpiox acc片选-CSB1-GPIOx
 * @param csb1_acc_pin acc片选-CSB1-GPIO-pinx
 * @param csb1_acc_gpio_pinstate acc片选-CSB1-GPIO状态
 * @param csb2_gyro_gpiox gyro片选-CSB2-GPIOx
 * @param csb2_gyro_pin gyro片选-CSB2-GPIO-pinx
 * @param csb2_gyro_gpio_pinstate gyro片选-CSB2-GPIO状态
 * @param int1_acc_gpiox acc中断-INT1-GPIOx
 * @param int1_acc_pin acc中断-INT1-GPIO-pinx
 * @param int3_gyro_gpiox gyro中断-INT3-GPIOx
 * @param int3_gyro_pin gyro中断-INT3-GPIO-pinx
 */
void bmi088_init(bmi088_handle_t *handle,SPI_HandleTypeDef *hspi, GPIO_TypeDef *csb1_acc_gpiox,uint16_t csb1_acc_pin,GPIO_PinState csb1_acc_gpio_pinstate,
                                                                  GPIO_TypeDef *csb2_gyro_gpiox,uint16_t csb2_gyro_pin,GPIO_PinState csb2_gyro_gpio_pinstate,
                                                                  GPIO_TypeDef *int1_acc_gpiox,uint16_t int1_acc_pin,
                                                                  GPIO_TypeDef *int3_gyro_gpiox,uint16_t int3_gyro_pin
);

/**
 * @brief BMI088开始配置参数函数
 * 
 * @param handle BMI088句柄指针
 */
void bmi088_start(bmi088_handle_t *handle);

/**
 * @brief bmi088_get_acc_raw_data 获取acc原始加速度（X/Y/Z，共6字节）
 * 
 * @param handle BMI088句柄指针
 * @param acc_get_raw_data_finishedfunction 读取完成回调（三轴int16原始值）
 */
void bmi088_get_acc_raw_data(bmi088_handle_t *handle,bmi088_acc_get_raw_data_finishedfunction acc_get_raw_data_finishedfunction);

/**
 * @brief bmi088_get_gyro_raw_data 获取gyro原始角速度（X/Y/Z，共6字节）
 * 
 * @param handle BMI088句柄指针
 * @param gyro_raw_data_finishedfunction 读取完成回调（三轴int16原始值）
 */
void bmi088_get_gyro_raw_data(bmi088_handle_t *handle,bmi088_gyro_get_raw_data_finishedfunction gyro_get_raw_data_finishedfunction);

/**
 * @brief BMI088 acc 写寄存器阻塞版
 * 
 * @param handle BMI088句柄指针
 * @param Tx_Buffer 发送缓冲区
 * @param Tx_Length 发送长度
 */
void bmi088_acc_write_reg_blocking(bmi088_handle_t *handle,uint8_t *Tx_Buffer,uint16_t Tx_Length);

/**
 * @brief BMI088 gyro 写寄存器阻塞版
 * 
 * @param handle BMI088句柄指针
 * @param Tx_Buffer 发送缓冲区
 * @param Tx_Length  发送长度
 */
void bmi088_gyro_write_reg_blocking(bmi088_handle_t *handle,uint8_t *Tx_Buffer,uint16_t Tx_Length);

/**
 * @brief 读ACC寄存器ID 期望返回值0x1E
 * 
 * @param handle BMI088句柄指针
 * @param readid_acc_finishedfunction 读acc的id的回调函数
 */
void bmi088_readid_acc(bmi088_handle_t *handle,bmi088_readid_acc_finishedfunction readid_acc_finishedfunction);

/**
 * @brief 读取GYRO寄存器ID 期望返回值 0x0F
 * 
 * @param handle BMI088句柄指针
 * @param readid_gyro_finishedfunction 读取gyro的id的回调函数
 */
void bmi088_readid_gyro(bmi088_handle_t *handle,bmi088_readid_gyro_finishedfunction readid_gyro_finishedfunction);

/**
 * @brief 读acc寄存器
 * 
 * @param handle BMI088句柄指针
 * @param read_reg_address 读acc寄存器的地址
 * @param rx_data 接收数据地址
 * @param rx_length 接收数据长度
 * @param acc_read_reg_finishedfunction 接收数据回调函数
 */
void bmi088_acc_read_reg(bmi088_handle_t *handle,uint8_t read_reg_address,uint8_t *rx_data,uint16_t rx_length,bmi088_acc_read_reg_finishedfunction acc_read_reg_finishedfunction);

/**
 * @brief 读gyro寄存器
 * 
 * @param handle BMI088句柄指针
 * @param read_reg_address 读gyro寄存器的地址
 * @param rx_data 接收数据地址
 * @param rx_length 接收数据长度
 * @param gyro_read_reg_finishedfunction 接收数据回调函数
 */
void bmi088_gyro_read_reg(bmi088_handle_t *handle,uint8_t read_reg_address,uint8_t *rx_data,uint16_t rx_length,bmi088_gyro_read_reg_finishedfunction gyro_read_reg_finishedfunction);

/**
 * @brief 写acc寄存器
 *
 * @param handle BMI088句柄指针
 * @param write_reg_address 写acc寄存器的地址
 * @param tx_data 要写入的数据地址
 * @param tx_length 写入数据长度
 * @param acc_write_reg_finishedfunction 写完成回调函数（可为NULL）
 */
void bmi088_acc_write_reg(bmi088_handle_t *handle,uint8_t write_reg_address,uint8_t *tx_data,uint16_t tx_length,bmi088_acc_write_reg_finishedfunction acc_write_reg_finishedfunction);

/**
 * @brief 写gyro寄存器
 *
 * @param handle BMI088句柄指针
 * @param write_reg_address 写gyro寄存器的地址
 * @param tx_data 要写入的数据地址
 * @param tx_length 写入数据长度
 * @param gyro_write_reg_finishedfunction 写完成回调函数（可为NULL）
 */
void bmi088_gyro_write_reg(bmi088_handle_t *handle,uint8_t write_reg_address,uint8_t *tx_data,uint16_t tx_length,bmi088_gyro_write_reg_finishedfunction gyro_write_reg_finishedfunction);

/**
 * @brief acc软复位
 * 
 * @param handle BMI088句柄指针
 * @param acc_write_reg_finishedfunction 写完成回调函数（可为NULL）
 */
void bmi088_acc_softreset(bmi088_handle_t *handle,bmi088_acc_write_reg_finishedfunction acc_write_reg_finishedfunction);

/**
 * @brief gyro软复位
 * 
 * @param handle BMI088句柄指针
 * @param acc_write_reg_finishedfunction 写完成回调函数（可为NULL）
 */
void bmi088_gyro_softreset(bmi088_handle_t *handle,bmi088_gyro_write_reg_finishedfunction gyro_write_reg_finishedfunction);

/**
 * @brief BMI088 TxRx回调函数
 * 
 * @param Tx_Buffer 发送缓冲区
 * @param Rx_Buffer 接收缓冲区
 * @param Tx_Length 发送长度
 * @param Rx_Length 接受长度
 */
void bmi088_spi_txrxcallback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length);

#ifdef __cplusplus
}
#endif

#endif /* __BMI088_H__ */
