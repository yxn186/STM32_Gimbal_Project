/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bmi088.cpp
  * @brief   bmi088库
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "bmi088.h"
#include "bmi088reg.h"
#include "bsp_spi.h"
#include <cstdint>
#include <string.h>

static bmi088_handle_t *bmi088_handle_global = NULL;

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
)
{
  if (handle == NULL) return;
  handle->hspi = hspi;

  handle->csb1_acc_gpiox = csb1_acc_gpiox;
  handle->csb1_acc_pin = csb1_acc_pin;
  handle->csb1_acc_gpio_pinstate = csb1_acc_gpio_pinstate;

  handle->csb2_gyro_gpiox = csb2_gyro_gpiox;
  handle->csb2_gyro_pin = csb2_gyro_pin;
  handle->csb2_gyro_gpio_pinstate = csb2_gyro_gpio_pinstate;

  handle->int1_acc_gpiox = int1_acc_gpiox;
  handle->int1_acc_pin = int1_acc_pin;

  handle->int3_gyro_gpiox = int3_gyro_gpiox;
  handle->int3_gyro_pin = int3_gyro_pin;

  bmi088_handle_global = handle;

  SPI_Init(handle->hspi, bmi088_spi_txrxcallback);
}

/**
 * @brief BMI088 acc 写寄存器阻塞版
 * 
 * @param handle BMI088句柄指针
 * @param Tx_Buffer 发送缓冲区
 * @param Tx_Length 发送长度
 */
void bmi088_acc_write_reg_blocking(bmi088_handle_t *handle,uint8_t *Tx_Buffer,uint16_t Tx_Length)
{
  SPI_Transmit_Data_Blocking(handle->hspi,handle->csb1_acc_gpiox,handle->csb1_acc_pin,handle->csb1_acc_gpio_pinstate,Tx_Buffer,Tx_Length,HAL_MAX_DELAY);
}

/**
 * @brief BMI088 gyro 写寄存器阻塞版
 * 
 * @param handle BMI088句柄指针
 * @param Tx_Buffer 发送缓冲区
 * @param Tx_Length  发送长度
 */
void bmi088_gyro_write_reg_blocking(bmi088_handle_t *handle,uint8_t *Tx_Buffer,uint16_t Tx_Length)
{
  SPI_Transmit_Data_Blocking(handle->hspi,handle->csb2_gyro_gpiox,handle->csb2_gyro_pin,handle->csb2_gyro_gpio_pinstate,Tx_Buffer,Tx_Length,HAL_MAX_DELAY);
}

/**
 * @brief BMI088开始配置参数函数
 * 
 * @param handle BMI088句柄指针
 */
void bmi088_start(bmi088_handle_t *handle)
{
  //1-把 ACC 从 suspend 拉到 normal
  uint8_t Temp1[2] = {BMI088_ACC_PWR_CONF & BMI088_WRITE , BMI088_ACC_PWR_ACTIVE_MODE};
  bmi088_acc_write_reg_blocking(handle,Temp1, 2);

  HAL_Delay(2);
  uint8_t Temp2[2] = {BMI088_ACC_PWR_CTRL & BMI088_WRITE , BMI088_ACC_ENABLE_ACC_ON};
  bmi088_acc_write_reg_blocking(handle,Temp2, 2);
  HAL_Delay(5);

  //2-配置 ACC 的“输出频率 + 滤波 + 量程”
  uint8_t Temp3[2] = {BMI088_ACC_CONF & BMI088_WRITE , BMI088_ACC_NORMAL | BMI088_ACC_200_HZ | BMI088_ACC_CONF_MUST_Set};
  bmi088_acc_write_reg_blocking(handle,Temp3, 2);

  uint8_t Temp4[2] = {BMI088_ACC_RANGE & BMI088_WRITE , BMI088_ACC_RANGE_6G};
  bmi088_acc_write_reg_blocking(handle,Temp4, 2);

  //3-配置 GYRO 的“电源模式 + 量程 + 带宽/ODR
  uint8_t Temp5[2] = {BMI088_GYRO_LPM1 & BMI088_WRITE , BMI088_GYRO_NORMAL_MODE};
  bmi088_gyro_write_reg_blocking(handle,Temp5, 2);

  uint8_t Temp6[2] = {BMI088_GYRO_RANGE & BMI088_WRITE , BMI088_GYRO_2000};
  bmi088_gyro_write_reg_blocking(handle,Temp6, 2);

  uint8_t Temp7[2] = {BMI088_GYRO_BANDWIDTH & BMI088_WRITE , BMI088_GYRO_200_23_HZ | BMI088_GYRO_BANDWIDTH_MUST_Set};
  bmi088_gyro_write_reg_blocking(handle,Temp7, 2);

  //待加int1和3 然后是处理数据 最好先完善下笔记流程

  //4-配置INT1 推挽 + 高电平有效 映射 DRDY 到 INT1
  uint8_t Temp8[2] = {BMI088_INT1_IO_CTRL & BMI088_WRITE , BMI088_ACC_INT1_IO_ENABLE | BMI088_ACC_INT1_GPIO_PP | BMI088_ACC_INT1_GPIO_HIGH};
  bmi088_acc_write_reg_blocking(handle,Temp8, 2);

  uint8_t Temp9[2] = {BMI088_INT_MAP_DATA & BMI088_WRITE , BMI088_ACC_INT1_DRDY_INTERRUPT};
  bmi088_acc_write_reg_blocking(handle,Temp9, 2);

  //5-配置INT3 使能 gyro 的 DRDY 中断 INT3 推挽 + 高电平有效 映射 DRDY 到 INT3
  uint8_t Temp10[2] = {BMI088_GYRO_CTRL & BMI088_WRITE , BMI088_DRDY_ON};
  bmi088_gyro_write_reg_blocking(handle,Temp10, 2);

  uint8_t Temp11[2] = {BMI088_GYRO_INT3_INT4_IO_CONF & BMI088_WRITE , BMI088_GYRO_INT3_GPIO_PP | BMI088_GYRO_INT3_GPIO_HIGH};
  bmi088_gyro_write_reg_blocking(handle,Temp11, 2);

  uint8_t Temp12[2] = {BMI088_GYRO_INT3_INT4_IO_MAP & BMI088_WRITE , BMI088_GYRO_DRDY_IO_INT3};
  bmi088_gyro_write_reg_blocking(handle,Temp12, 2);
}

/**
 * @brief 读acc寄存器
 * 
 * @param handle BMI088句柄指针
 * @param read_reg_address 读acc寄存器的地址
 * @param rx_data 接收数据地址
 * @param rx_length 接收数据长度
 * @param acc_read_reg_finishedfunction 接收数据回调函数
 */
void bmi088_acc_read_reg(bmi088_handle_t *handle,uint8_t read_reg_address,uint8_t *rx_data,uint16_t rx_length,bmi088_acc_read_reg_finishedfunction acc_read_reg_finishedfunction)
{
  if (handle == NULL) return;
  if (handle->hspi == NULL) return;
  if (rx_data == NULL) return;
  if (rx_length == 0) return;

  if (handle->current_operation != bmi088_no_operation) return;


  if (rx_length + 1 > 260) return;  // 太长就直接拒绝

  // ACC：额外多读 1 个 dummy byte
  uint16_t spi_rx_len = (uint16_t)(rx_length + 1);

  // 保存本次读操作上下文（DMA异步用）
  handle->acc_read_reg_rx_buffer = rx_data;
  handle->acc_read_reg_rx_length = rx_length;// 保存“用户请求的真实长度”（不是 spi_rx_len）
  handle->acc_read_reg_address = read_reg_address;
  handle->acc_read_reg_finishedfunction = acc_read_reg_finishedfunction;

  handle->current_operation = bmi088_reading_acc_reg;

  // 组包：第1字节=寄存器地址|读位(0x80)，后面length个dummy提供时钟
  handle->tx_buffer[0] = (uint8_t)(read_reg_address | BMI088_READ);
  memset(&handle->tx_buffer[1], BMI088_DUMMY, spi_rx_len);

  // （可选）清一下rx，方便调试看波形/数据
  //memset(handle->rx_buffer, 0, (size_t)(1 + length));

  // 发起DMA：Tx_Length=1（只算地址字节），Rx_Length=length（要读的数据字节数）
  uint8_t ret = SPI_Transmit_Receive_Data(handle->hspi,handle->csb1_acc_gpiox,handle->csb1_acc_pin,handle->csb1_acc_gpio_pinstate,handle->tx_buffer,handle->rx_buffer,1,spi_rx_len);

  // 如果DMA没启动成功，要把状态还原（不然会卡在“忙”）
  if (ret != HAL_OK)
  {
      handle->current_operation = bmi088_no_operation;
      handle->acc_read_reg_rx_buffer = NULL;
      handle->acc_read_reg_rx_length = 0;
      handle->acc_read_reg_finishedfunction = NULL;
  }
}

/**
 * @brief 读gyro寄存器
 * 
 * @param handle BMI088句柄指针
 * @param read_reg_address 读gyro寄存器的地址
 * @param rx_data 接收数据地址
 * @param rx_length 接收数据长度
 * @param gyro_read_reg_finishedfunction 接收数据回调函数
 */
void bmi088_gyro_read_reg(bmi088_handle_t *handle,uint8_t read_reg_address,uint8_t *rx_data,uint16_t rx_length,bmi088_gyro_read_reg_finishedfunction gyro_read_reg_finishedfunction)
{
  if (handle == NULL) return;
  if (handle->hspi == NULL) return;
  if (rx_data == NULL) return;
  if (rx_length == 0) return;

  if (handle->current_operation != bmi088_no_operation) return;


  if (rx_length > 260) return;  // 太长就直接拒绝

  // 保存本次读操作上下文（DMA异步用）
  handle->gyro_read_reg_rx_buffer = rx_data;
  handle->gyro_read_reg_rx_length = rx_length;
  handle->gyro_read_reg_address = read_reg_address;
  handle->gyro_read_reg_finishedfunction = gyro_read_reg_finishedfunction;

  handle->current_operation = bmi088_reading_gyro_reg;

  // 组包：第1字节=寄存器地址|读位(0x80)，后面length个dummy提供时钟
  handle->tx_buffer[0] = (uint8_t)(read_reg_address | BMI088_READ);
  memset(&handle->tx_buffer[1], BMI088_DUMMY, rx_length);

  // （可选）清一下rx，方便调试看波形/数据
  //memset(handle->rx_buffer, 0, (size_t)(1 + length));

  // 发起DMA：Tx_Length=1（只算地址字节），Rx_Length=length（要读的数据字节数）
  uint8_t ret = SPI_Transmit_Receive_Data(handle->hspi,handle->csb2_gyro_gpiox,handle->csb2_gyro_pin,handle->csb2_gyro_gpio_pinstate,handle->tx_buffer,handle->rx_buffer,1,rx_length);

  // 如果DMA没启动成功，要把状态还原（不然会卡在“忙”）
  if (ret != HAL_OK)
  {
      handle->current_operation = bmi088_no_operation;
      handle->gyro_read_reg_rx_buffer = NULL;
      handle->gyro_read_reg_rx_length = 0;
      handle->gyro_read_reg_finishedfunction = NULL;
  }
}

/**
 * @brief 写acc寄存器
 *
 * @param handle BMI088句柄指针
 * @param write_reg_address 写acc寄存器的地址
 * @param tx_data 要写入的数据地址
 * @param tx_length 写入数据长度
 * @param acc_write_reg_finishedfunction 写完成回调函数（可为NULL）
 */
void bmi088_acc_write_reg(bmi088_handle_t *handle,uint8_t write_reg_address,uint8_t *tx_data,uint16_t tx_length,bmi088_acc_write_reg_finishedfunction acc_write_reg_finishedfunction)
{
  if (handle == NULL) return;
  if (handle->hspi == NULL) return;
  if (tx_data == NULL) return;
  if (tx_length == 0) return;
  if (handle->current_operation != bmi088_no_operation) return;

  if (tx_length > 260) return;

  // 保存本次写操作上下文（回调用）
  handle->acc_write_reg_address = write_reg_address;
  handle->acc_write_reg_tx_length = tx_length;
  handle->acc_write_reg_finishedfunction = acc_write_reg_finishedfunction;

  handle->current_operation = bmi088_writing_acc_reg;

  // 组包：写操作地址 bit7=0
  handle->tx_buffer[0] = (uint8_t)(write_reg_address & BMI088_WRITE);

  // 拷贝要写的数据
  memcpy(&handle->tx_buffer[1], tx_data, tx_length);

  // 发起DMA：只发不收
  uint8_t ret = SPI_Transmit_Data(handle->hspi,handle->csb1_acc_gpiox,handle->csb1_acc_pin,handle->csb1_acc_gpio_pinstate,handle->tx_buffer,(uint16_t)(1 + tx_length));

  // DMA没启动成功：恢复状态，避免卡“忙”
  if (ret != HAL_OK)
  {
      handle->current_operation = bmi088_no_operation;
      handle->acc_write_reg_tx_length = 0;
      handle->acc_write_reg_finishedfunction = NULL;
  }
}

/**
 * @brief 写gyro寄存器
 *
 * @param handle BMI088句柄指针
 * @param write_reg_address 写gyro寄存器的地址
 * @param tx_data 要写入的数据地址
 * @param tx_length 写入数据长度
 * @param gyro_write_reg_finishedfunction 写完成回调函数（可为NULL）
 */
void bmi088_gyro_write_reg(bmi088_handle_t *handle,uint8_t write_reg_address,uint8_t *tx_data,uint16_t tx_length,bmi088_gyro_write_reg_finishedfunction gyro_write_reg_finishedfunction)
{
  if (handle == NULL) return;
  if (handle->hspi == NULL) return;
  if (tx_data == NULL) return;
  if (tx_length == 0) return;
  if (handle->current_operation != bmi088_no_operation) return;

  if (tx_length > 260) return;

  // 保存本次写操作上下文（回调用）
  handle->gyro_write_reg_address = write_reg_address;
  handle->gyro_write_reg_tx_length = tx_length;
  handle->gyro_write_reg_finishedfunction = gyro_write_reg_finishedfunction;

  handle->current_operation = bmi088_writing_gyro_reg;

  // 组包：写操作地址 bit7=0
  handle->tx_buffer[0] = (uint8_t)(write_reg_address & BMI088_WRITE);

  // 拷贝要写的数据
  memcpy(&handle->tx_buffer[1], tx_data, tx_length);

  // 发起DMA：只发不收
  uint8_t ret = SPI_Transmit_Data(handle->hspi,handle->csb2_gyro_gpiox,handle->csb2_gyro_pin,handle->csb2_gyro_gpio_pinstate,handle->tx_buffer,(uint16_t)(1 + tx_length));

  // DMA没启动成功：恢复状态，避免卡“忙”
  if (ret != HAL_OK)
  {
      handle->current_operation = bmi088_no_operation;
      handle->gyro_write_reg_tx_length = 0;
      handle->gyro_write_reg_finishedfunction = NULL;
  }
}

/**
 * @brief BMI088 TxRx 内部回调函数
 * 
 * @param read_reg_address 读取寄存器地址
 * @param rx_data 接收数据地址
 * @param rx_length 接收数据长度
 */
static void bmi088_spi_txrx_internal_callback(uint8_t read_reg_address, uint8_t *rx_data, uint16_t rx_length)
{
  if (bmi088_handle_global == NULL) return;
  if (rx_data == NULL) return;
  

  uint8_t get_id = rx_data[0];

  if(read_reg_address == BMI088_ACC_CHIP_ID)
  {
    if(rx_length != 1) return;
    if(bmi088_handle_global->readid_acc_finishedfunction != NULL)
    {
      bmi088_handle_global->readid_acc_finishedfunction(get_id);
      bmi088_handle_global->readid_acc_finishedfunction = NULL;
    }
  }
  if(read_reg_address == BMI088_GYRO_CHIP_ID)
  {
    if(rx_length != 1) return;
    if(bmi088_handle_global->readid_gyro_finishedfunction != NULL)
    {
      bmi088_handle_global->readid_gyro_finishedfunction(get_id);
      bmi088_handle_global->readid_gyro_finishedfunction = NULL;
    }
  }
  if(read_reg_address == BMI088_GYRO_X_L)
  {
    if (rx_length != 6) return;
    if(bmi088_handle_global->gyro_get_raw_data_finishedfunction != NULL)
    {
      //拼接 依照时序 发送给回调
      bmi088_handle_global->gyro_get_raw_data_finishedfunction( (int16_t)((((int16_t)rx_data[1]) << 8) | rx_data[0]), 
                                                                (int16_t)((((int16_t)rx_data[3]) << 8) | rx_data[2]), 
                                                                (int16_t)((((int16_t)rx_data[5]) << 8) | rx_data[4]));
    }
  }
  if(read_reg_address == BMI088_ACCEL_XOUT_L)
  {
    if (rx_length != 6) return;
    if(bmi088_handle_global->acc_get_raw_data_finishedfunction != NULL)
    {
       //拼接 依照时序 发送给回调
      bmi088_handle_global->acc_get_raw_data_finishedfunction( (int16_t)((((int16_t)rx_data[1]) << 8) | rx_data[0]),
                                                               (int16_t)((((int16_t)rx_data[3]) << 8) | rx_data[2]),
                                                               (int16_t)((((int16_t)rx_data[5]) << 8) | rx_data[4]));
    }
  }
}

/**
 * @brief bmi088_get_acc_raw_data 获取acc原始加速度（X/Y/Z，共6字节）
 * 
 * @param handle BMI088句柄指针
 * @param acc_get_raw_data_finishedfunction 读取完成回调（三轴int16原始值）
 * @return uint8_t 执行是否成功
 */
uint8_t bmi088_get_acc_raw_data(bmi088_handle_t *handle,bmi088_acc_get_raw_data_finishedfunction acc_get_raw_data_finishedfunction)
{
  if (handle == NULL) return 0;
  if (handle->hspi == NULL) return 0;
  if (handle->current_operation != bmi088_no_operation) return 0;

  handle->acc_get_raw_data_finishedfunction = acc_get_raw_data_finishedfunction;

  /* 从 0x12 连续读 6 字节：0x12~0x17 (X_L X_H Y_L Y_H Z_L Z_H) */
  bmi088_acc_read_reg(handle,BMI088_ACCEL_XOUT_L,handle->acc_raw_data_rx_buffer,6,bmi088_spi_txrx_internal_callback);
  return 1;
}

/**
 * @brief bmi088_get_gyro_raw_data 获取gyro原始角速度（X/Y/Z，共6字节）
 * 
 * @param handle BMI088句柄指针
 * @param gyro_get_raw_data_finishedfunction 读取完成回调（三轴int16原始值）
 * @return uint8_t 执行是否成功
 */
uint8_t bmi088_get_gyro_raw_data(bmi088_handle_t *handle,bmi088_gyro_get_raw_data_finishedfunction gyro_get_raw_data_finishedfunction)
{
  if (handle == NULL) return 0;
  if (handle->hspi == NULL) return 0;
  if (handle->current_operation != bmi088_no_operation) return 0;

  handle->gyro_get_raw_data_finishedfunction = gyro_get_raw_data_finishedfunction;

  /* 从 0x02 连续读 6 字节：0x02~0x07 (X_L X_H Y_L Y_H Z_L Z_H) */
  bmi088_gyro_read_reg(handle,BMI088_GYRO_X_L,handle->gyro_raw_data_rx_buffer,6,bmi088_spi_txrx_internal_callback);
  return 1;
}

/**
 * @brief 读ACC寄存器ID 期望返回值0x1E
 * 
 * @param handle BMI088句柄指针
 * @param readid_acc_finishedfunction 读acc的id的回调函数
 */
void bmi088_readid_acc(bmi088_handle_t *handle,bmi088_readid_acc_finishedfunction readid_acc_finishedfunction)
{
  if (handle == NULL) return;
  if (handle->hspi == NULL) return;

  handle->readid_acc_finishedfunction = readid_acc_finishedfunction;

  bmi088_acc_read_reg(handle,BMI088_ACC_CHIP_ID,&handle->acc_id,1,bmi088_spi_txrx_internal_callback);
}

/**
 * @brief 读取GYRO寄存器ID 期望返回值 0x0F
 * 
 * @param handle BMI088句柄指针
 * @param readid_gyro_finishedfunction 读取gyro的id的回调函数
 */
void bmi088_readid_gyro(bmi088_handle_t *handle,bmi088_readid_gyro_finishedfunction readid_gyro_finishedfunction)
{

  if (handle == NULL) return;
  if (handle->hspi == NULL) return;
 
  handle->readid_gyro_finishedfunction = readid_gyro_finishedfunction;

  bmi088_gyro_read_reg(handle,BMI088_GYRO_CHIP_ID,&handle->gyro_id,1,bmi088_spi_txrx_internal_callback);
}

/**
 * @brief acc软复位
 * 
 * @param handle BMI088句柄指针
 * @param acc_write_reg_finishedfunction 写完成回调函数（可为NULL）
 */
void bmi088_acc_softreset(bmi088_handle_t *handle,bmi088_acc_write_reg_finishedfunction acc_write_reg_finishedfunction)
{
  if (handle == NULL) return;
  if (handle->current_operation != bmi088_no_operation) return;

  uint8_t temp = BMI088_ACC_SOFTRESET_VALUE;
  bmi088_acc_write_reg(handle,BMI088_ACC_SOFTRESET,&temp,1,acc_write_reg_finishedfunction);
}

/**
 * @brief gyro软复位
 * 
 * @param handle BMI088句柄指针
 * @param acc_write_reg_finishedfunction 写完成回调函数（可为NULL）
 */
void bmi088_gyro_softreset(bmi088_handle_t *handle,bmi088_gyro_write_reg_finishedfunction gyro_write_reg_finishedfunction)
{
  if (handle == NULL) return;
  if (handle->current_operation != bmi088_no_operation) return;

  uint8_t temp = BMI088_GYRO_SOFTRESET_VALUE;
  bmi088_gyro_write_reg(handle,BMI088_GYRO_SOFTRESET,&temp,1,gyro_write_reg_finishedfunction);
}

/**
 * @brief acc开始读数据
 * 
 * @param handle BMI088句柄指针
 * @param acc_write_reg_finishedfunction 写完成回调函数（可为NULL）
 */
void bmi088_acc_startread(bmi088_handle_t *handle,bmi088_acc_write_reg_finishedfunction acc_write_reg_finishedfunction)
{
  if (handle == NULL) return;
  if (handle->current_operation != bmi088_no_operation) return;

   //bmi088_acc_read_reg(handle,BMI088_ACC_CHIP_ID,&handle->acc_id,1,bmi088_spi_txrx_readid_callback);
}

/**
 * @brief BMI088 TxRx回调函数
 * 
 * @param Tx_Buffer 发送缓冲区
 * @param Rx_Buffer 接收缓冲区
 * @param Tx_Length 发送长度
 * @param Rx_Length 接受长度
 */
void bmi088_spi_txrxcallback(uint8_t *Tx_Buffer, uint8_t *Rx_Buffer, uint16_t Tx_Length, uint16_t Rx_Length)
{
  if (bmi088_handle_global->current_operation == bmi088_reading_acc_reg)
  {
    if (Rx_Buffer == NULL) return;

    // 读寄存器：规定 Tx_Length=1
    if (Tx_Length != 1) return;

    // Rx_Length必须匹配本次发起时保存的长度
    // 对于ACC：实际 SPI 收到的长度 = 用户长度 + 1(dummy)
    if (Rx_Length !=  (uint16_t)(bmi088_handle_global->acc_read_reg_rx_length + 1)) return;

    // 工程化：先清状态再回调
    bmi088_acc_read_reg_finishedfunction read_reg_finishedfunction = bmi088_handle_global->acc_read_reg_finishedfunction;
    uint8_t *read_reg_rx_buffer = bmi088_handle_global->acc_read_reg_rx_buffer;
    uint16_t read_reg_rx_length = bmi088_handle_global->acc_read_reg_rx_length;
    uint8_t read_reg_address = bmi088_handle_global->acc_read_reg_address;

    // 把有效数据拷贝给用户 但对于acc 要跳过 dummy byte 从 Tx_Length + 1 开始
    if (read_reg_rx_buffer != NULL && read_reg_rx_length > 0)
    {
        memcpy(read_reg_rx_buffer, &Rx_Buffer[Tx_Length + 1], read_reg_rx_length);
    }

    // 先清状态（关键：避免回调里发下一笔被busy挡住）
    bmi088_handle_global->current_operation = bmi088_no_operation;
    bmi088_handle_global->acc_read_reg_rx_buffer = NULL;
    bmi088_handle_global->acc_read_reg_rx_length = 0;
    bmi088_handle_global->acc_read_reg_finishedfunction = NULL;

    // 再回调通知上层（可选）
    if (read_reg_finishedfunction != NULL)
    {
        read_reg_finishedfunction(read_reg_address, read_reg_rx_buffer, read_reg_rx_length);
    }
  }
  else if (bmi088_handle_global->current_operation == bmi088_reading_gyro_reg)
  {
    if (Rx_Buffer == NULL) return;

    // 读寄存器：规定 Tx_Length=1
    if (Tx_Length != 1) return;

    // Rx_Length必须匹配本次发起时保存的长度
    if (Rx_Length != bmi088_handle_global->gyro_read_reg_rx_length) return;

    // 工程化：先清状态再回调
    bmi088_gyro_read_reg_finishedfunction read_reg_finishedfunction = bmi088_handle_global->gyro_read_reg_finishedfunction;
    uint8_t *read_reg_rx_buffer = bmi088_handle_global->gyro_read_reg_rx_buffer;
    uint16_t read_reg_rx_length = bmi088_handle_global->gyro_read_reg_rx_length;
    uint8_t read_reg_address = bmi088_handle_global->gyro_read_reg_address;

    // 把有效数据拷贝给用户：数据从 Rx_Buffer[Tx_Length] 开始
    if (read_reg_rx_buffer != NULL && read_reg_rx_length > 0)
    {
        memcpy(read_reg_rx_buffer, &Rx_Buffer[Tx_Length], read_reg_rx_length);
    }

    // 先清状态（关键：避免回调里发下一笔被busy挡住）
    bmi088_handle_global->current_operation = bmi088_no_operation;
    bmi088_handle_global->gyro_read_reg_rx_buffer = NULL;
    bmi088_handle_global->gyro_read_reg_rx_length = 0;
    bmi088_handle_global->gyro_read_reg_finishedfunction = NULL;

    // 再回调通知上层（可选）
    if (read_reg_finishedfunction != NULL)
    {
        read_reg_finishedfunction(read_reg_address, read_reg_rx_buffer, read_reg_rx_length);
    }
  }
  else if (bmi088_handle_global->current_operation == bmi088_writing_acc_reg)
  {
    // 写寄存器走的是 SPI_Transmit_Data -> HAL_SPI_TxCpltCallback
    // 因此这里 Rx_Buffer 通常为 NULL，Rx_Length 通常为 0
    // 只要能进到这里，说明DMA Tx已完成。

    // 先备份回调参数（工程化：先清状态再回调，避免回调里发下一笔被busy挡住）
    bmi088_acc_write_reg_finishedfunction write_reg_finishedfunction = bmi088_handle_global->acc_write_reg_finishedfunction;
    uint8_t write_reg_address = bmi088_handle_global->acc_write_reg_address;
    uint16_t write_reg_tx_length = bmi088_handle_global->acc_write_reg_tx_length;

    // 写出去的数据在 tx_buffer[1..len]，备份指针（注意：指向handle内存，回调里不要长期保存）
    uint8_t *written = &bmi088_handle_global->tx_buffer[1];

    // 清状态
    bmi088_handle_global->current_operation = bmi088_no_operation;
    bmi088_handle_global->acc_write_reg_tx_length = 0;
    bmi088_handle_global->acc_write_reg_finishedfunction = NULL;

    // 再回调通知上层（可选）
    if (write_reg_finishedfunction != NULL)
    {
        write_reg_finishedfunction(write_reg_address, written, write_reg_tx_length);
    }
  }
  else if (bmi088_handle_global->current_operation == bmi088_writing_gyro_reg)
  {
    // 写寄存器走的是 SPI_Transmit_Data -> HAL_SPI_TxCpltCallback
    // 因此这里 Rx_Buffer 通常为 NULL，Rx_Length 通常为 0
    // 只要能进到这里，说明DMA Tx已完成。

    // 先备份回调参数（工程化：先清状态再回调，避免回调里发下一笔被busy挡住）
    bmi088_gyro_write_reg_finishedfunction write_reg_finishedfunction = bmi088_handle_global->gyro_write_reg_finishedfunction;
    uint8_t write_reg_address = bmi088_handle_global->gyro_write_reg_address;
    uint16_t write_reg_tx_length = bmi088_handle_global->gyro_write_reg_tx_length;

    // 写出去的数据在 tx_buffer[1..len]，备份指针（注意：指向handle内存，回调里不要长期保存）
    uint8_t *written = &bmi088_handle_global->tx_buffer[1];

    // 清状态
    bmi088_handle_global->current_operation = bmi088_no_operation;
    bmi088_handle_global->gyro_write_reg_tx_length = 0;
    bmi088_handle_global->gyro_write_reg_finishedfunction = NULL;

    // 再回调通知上层（可选）
    if (write_reg_finishedfunction != NULL)
    {
        write_reg_finishedfunction(write_reg_address, written, write_reg_tx_length);
    }
  }
}
