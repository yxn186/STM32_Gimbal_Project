/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bmi088reg.h
  * @brief   bmi088寄存器库
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __BMI088REG_H__
#define __BMI088REG_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/*YOUR CODE*/

/*------------------------常用地址------------------------*/


//加速度计软件重置寄存器相关
#define BMI088_ACC_SOFTRESET 0x7E
#define BMI088_ACC_SOFTRESET_VALUE 0xB6

//陀螺仪软件重置寄存器相关
#define BMI088_GYRO_SOFTRESET 0x14
#define BMI088_GYRO_SOFTRESET_VALUE 0xB6

// X/Y/Z轴加速度低/高字节寄存器
#define BMI088_ACCEL_XOUT_L 0x12
#define BMI088_ACCEL_XOUT_M 0x13
#define BMI088_ACCEL_YOUT_L 0x14
#define BMI088_ACCEL_YOUT_M 0x15
#define BMI088_ACCEL_ZOUT_L 0x16
#define BMI088_ACCEL_ZOUT_M 0x17

// X/Y/Z轴陀螺仪低/高字节寄存器
#define BMI088_GYRO_X_L 0x02
#define BMI088_GYRO_X_H 0x03
#define BMI088_GYRO_Y_L 0x04
#define BMI088_GYRO_Y_H 0x05
#define BMI088_GYRO_Z_L 0x06
#define BMI088_GYRO_Z_H 0x07

// 温度传感器高 / 低字节寄存器
#define BMI088_TEMP_M 0x22
#define BMI088_TEMP_L 0x23

// 加速度计ID寄存器地址
#define BMI088_ACC_CHIP_ID 0x00 // the register is  " Who am I "
// 该寄存器的特定ID值
#define BMI088_ACC_CHIP_ID_VALUE 0x1E

// 陀螺仪ID寄存器地址
#define BMI088_GYRO_CHIP_ID 0x00
// 该寄存器的特定ID值
#define BMI088_GYRO_CHIP_ID_VALUE 0x0F	// 陀螺仪芯片 ID 值，用于验证传感器连接

#define BMI088_READ 0x80
#define BMI088_WRITE 0x7F
#define BMI088_DUMMY 0xFF


/*------------------------常用地址------------------------*/

// 错误状态寄存器地址
#define BMI088_ACC_ERR_REG 0x02
#define BMI088_ACCEL_CONGIF_ERROR_SHFITS 0x2
// 配置错误标志（位 2），寄存器位 2=1 时，代表 ACC_CONF 寄存器配置无效
#define BMI088_ACCEL_CONGIF_ERROR (1 << BMI088_ACCEL_CONGIF_ERROR_SHFITS)
#define BMI088_FATAL_ERROR_SHFITS 0x0
// 致命错误标志（位 0），寄存器位 0=1 时，芯片处于非工作状态（需重启）
#define BMI088_FATAL_ERROR (1 << BMI088_FATAL_ERROR)

//==========================数据就绪与数据寄存器宏====================================
#define BMI088_ACC_STATUS 0x03
#define BMI088_ACCEL_DRDY_SHFITS 0x7
// 加速度数据就绪标志，位 7=1 时，X/Y/Z 加速度数据已更新，可读取
#define BMI088_ACCEL_DRDY (1 << BMI088_ACCEL_DRDY_SHFITS)

// 24 位时间戳寄存器
#define BMI088_SENSORTIME_DATA_L 0x18
#define BMI088_SENSORTIME_DATA_M 0x19
#define BMI088_SENSORTIME_DATA_H 0x1A

#define BMI088_ACC_INT_STAT_1 0x1D
#define BMI088_ACCEL_DRDY_INTERRUPT_SHFITS 0x7
#define BMI088_ACCEL_DRDY_INTERRUPT (1 << BMI088_ACCEL_DRDY_INTERRUPT_SHFITS)


#define BMI088_ACC_CONF 0x40			// 配置寄存器地址
#define BMI088_ACC_CONF_MUST_Set 0x80	// 必须置 1 的位（bit7）
#define BMI088_ACC_BWP_SHFITS 0x4

// 滤波模式（4 倍 / 2 倍过采样、普通）
//想低噪：OSR4/OSR2（延迟更大）
//想低延迟：Normal（噪声略高）
#define BMI088_ACC_OSR4 (0x0 << BMI088_ACC_BWP_SHFITS)
#define BMI088_ACC_OSR2 (0x1 << BMI088_ACC_BWP_SHFITS)
#define BMI088_ACC_NORMAL (0x2 << BMI088_ACC_BWP_SHFITS)

// 	输出数据速率（ODR）
// 如 BMI088_ACC_100_HZ = 0x8：对应 ODR=100Hz，每秒输出 100 组数据
//  ODR 建议跟你的控制环/解算频率一致或略高（比如 200Hz / 400Hz）
#define BMI088_ACC_ODR_SHFITS 0x0
#define BMI088_ACC_12_5_HZ (0x5 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_25_HZ (0x6 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_50_HZ (0x7 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_100_HZ (0x8 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_200_HZ (0x9 << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_400_HZ (0xA << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_800_HZ (0xB << BMI088_ACC_ODR_SHFITS)
#define BMI088_ACC_1600_HZ (0xC << BMI088_ACC_ODR_SHFITS)

//---量程---
//机器人底盘/云台一般 ±6g 很够用（分辨率更好）
//震动/冲击大再上 ±12g/±24g
#define BMI088_ACC_RANGE 0x41

#define BMI088_ACC_RANGE_SHFITS 0x0
#define BMI088_ACC_RANGE_3G (0x0 << BMI088_ACC_RANGE_SHFITS)
#define BMI088_ACC_RANGE_6G (0x1 << BMI088_ACC_RANGE_SHFITS)
#define BMI088_ACC_RANGE_12G (0x2 << BMI088_ACC_RANGE_SHFITS)
#define BMI088_ACC_RANGE_24G (0x3 << BMI088_ACC_RANGE_SHFITS)

// 中断配置宏（INT1/INT2 引脚）
#define BMI088_INT1_IO_CTRL 0x53
#define BMI088_ACC_INT1_IO_ENABLE_SHFITS 0x3
// 启用 INT1 为输出引脚（bit3）
#define BMI088_ACC_INT1_IO_ENABLE (0x1 << BMI088_ACC_INT1_IO_ENABLE_SHFITS)
#define BMI088_ACC_INT1_GPIO_MODE_SHFITS 0x2
// 启用推挽模式
#define BMI088_ACC_INT1_GPIO_PP (0x0 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)
// 启用开漏模式
#define BMI088_ACC_INT1_GPIO_OD (0x1 << BMI088_ACC_INT1_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT1_GPIO_LVL_SHFITS 0x1
// 表示中断触发时引脚拉低/拉高
#define BMI088_ACC_INT1_GPIO_LOW (0x0 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)
#define BMI088_ACC_INT1_GPIO_HIGH (0x1 << BMI088_ACC_INT1_GPIO_LVL_SHFITS)

#define BMI088_INT2_IO_CTRL 0x54
#define BMI088_ACC_INT2_IO_ENABLE_SHFITS 0x3
#define BMI088_ACC_INT2_IO_ENABLE (0x1 << BMI088_ACC_INT2_IO_ENABLE_SHFITS)
#define BMI088_ACC_INT2_GPIO_MODE_SHFITS 0x2
#define BMI088_ACC_INT2_GPIO_PP (0x0 << BMI088_ACC_INT2_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT2_GPIO_OD (0x1 << BMI088_ACC_INT2_GPIO_MODE_SHFITS)
#define BMI088_ACC_INT2_GPIO_LVL_SHFITS 0x1
#define BMI088_ACC_INT2_GPIO_LOW (0x0 << BMI088_ACC_INT2_GPIO_LVL_SHFITS)
#define BMI088_ACC_INT2_GPIO_HIGH (0x1 << BMI088_ACC_INT2_GPIO_LVL_SHFITS)

// 中断映射寄存器地址
#define BMI088_INT_MAP_DATA 0x58
#define BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS 0x2
// 数据就绪中断映射到 INT1（bit2），bit2=1 时，数据就绪时触发 INT1 中断
#define BMI088_ACC_INT1_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT1_DRDY_INTERRUPT_SHFITS)
#define BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS 0x6
#define BMI088_ACC_INT2_DRDY_INTERRUPT (0x1 << BMI088_ACC_INT2_DRDY_INTERRUPT_SHFITS)

#define BMI088_ACC_SELF_TEST 0x6D		// 自检寄存器地址
#define BMI088_ACC_SELF_TEST_OFF 0x00	// 关闭自检
// 正 / 负自检配置值
#define BMI088_ACC_SELF_TEST_POSITIVE_SIGNAL 0x0D
#define BMI088_ACC_SELF_TEST_NEGATIVE_SIGNAL 0x09

#define BMI088_ACC_PWR_CONF 0x7C			// 电源配置寄存器地址
#define BMI088_ACC_PWR_SUSPEND_MODE 0x03	// 休眠模式配置值
#define BMI088_ACC_PWR_ACTIVE_MODE 0x00		// 正常模式配置值

#define BMI088_ACC_PWR_CTRL 0x7D		// 电源控制寄存器地址
#define BMI088_ACC_ENABLE_ACC_OFF 0x00	// 软复位寄存器地址
#define BMI088_ACC_ENABLE_ACC_ON 0x04	// 启用加速度计


//=============================================================================================




#define BMI088_GYRO_INT_STAT_1 0x0A
#define BMI088_GYRO_DYDR_SHFITS 0x7
#define BMI088_GYRO_DYDR (0x1 << BMI088_GYRO_DYDR_SHFITS)

//量程
//快速转动（云台/陀螺可能大角速度）：先用 ±2000（最不容易饱和）
//稳定慢速：可以用 ±500/±250 提升分辨率
#define BMI088_GYRO_RANGE 0x0F
#define BMI088_GYRO_RANGE_SHFITS 0x0
#define BMI088_GYRO_2000 (0x0 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_1000 (0x1 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_500 (0x2 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_250 (0x3 << BMI088_GYRO_RANGE_SHFITS)
#define BMI088_GYRO_125 (0x4 << BMI088_GYRO_RANGE_SHFITS)


//控制环 1kHz：常用 0x02 (1000/116)
//控制环 400Hz：常用 0x03 (400/47)
//想更低噪：选更低 BW（但延迟更大）
#define BMI088_GYRO_BANDWIDTH 0x10		// 带宽配置寄存器地址
// the first num means Output data  rate, the second num means bandwidth
#define BMI088_GYRO_BANDWIDTH_MUST_Set 0x80	// 必须置 1 的位（bit7）
#define BMI088_GYRO_2000_532_HZ 0x00
#define BMI088_GYRO_2000_230_HZ 0x01
#define BMI088_GYRO_1000_116_HZ 0x02
#define BMI088_GYRO_400_47_HZ 0x03
#define BMI088_GYRO_200_23_HZ 0x04
#define BMI088_GYRO_100_12_HZ 0x05
#define BMI088_GYRO_200_64_HZ 0x06
#define BMI088_GYRO_100_32_HZ 0x07

// 电源模式寄存器地址
#define BMI088_GYRO_LPM1 0x11
// 三种电源模式，深度休眠时配置寄存器值会丢失，唤醒后需重新配置
#define BMI088_GYRO_NORMAL_MODE 0x00
#define BMI088_GYRO_SUSPEND_MODE 0x80
#define BMI088_GYRO_DEEP_SUSPEND_MODE 0x20



#define BMI088_GYRO_CTRL 0x15	// 中断控制寄存器地址
#define BMI088_DRDY_OFF 0x00
#define BMI088_DRDY_ON 0x80		// 启用陀螺仪数据就绪中断

#define BMI088_GYRO_INT3_INT4_IO_CONF 0x16
#define BMI088_GYRO_INT4_GPIO_MODE_SHFITS 0x3
#define BMI088_GYRO_INT4_GPIO_PP (0x0 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS)
#define BMI088_GYRO_INT4_GPIO_OD (0x1 << BMI088_GYRO_INT4_GPIO_MODE_SHFITS)
#define BMI088_GYRO_INT4_GPIO_LVL_SHFITS 0x2
#define BMI088_GYRO_INT4_GPIO_LOW (0x0 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS)
#define BMI088_GYRO_INT4_GPIO_HIGH (0x1 << BMI088_GYRO_INT4_GPIO_LVL_SHFITS)
#define BMI088_GYRO_INT3_GPIO_MODE_SHFITS 0x1
#define BMI088_GYRO_INT3_GPIO_PP (0x0 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)
#define BMI088_GYRO_INT3_GPIO_OD (0x1 << BMI088_GYRO_INT3_GPIO_MODE_SHFITS)
#define BMI088_GYRO_INT3_GPIO_LVL_SHFITS 0x0
#define BMI088_GYRO_INT3_GPIO_LOW (0x0 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS)
#define BMI088_GYRO_INT3_GPIO_HIGH (0x1 << BMI088_GYRO_INT3_GPIO_LVL_SHFITS)

#define BMI088_GYRO_INT3_INT4_IO_MAP 0x18	// 中断映射寄存器地址

#define BMI088_GYRO_DRDY_IO_OFF 0x00
#define BMI088_GYRO_DRDY_IO_INT3 0x01		// 中断映射到 INT3，写 0x01 时，数据就绪触发 INT3 中断
#define BMI088_GYRO_DRDY_IO_INT4 0x80		// 中断映射到 INT4，写 0x80 时，数据就绪触发 INT4 中断
#define BMI088_GYRO_DRDY_IO_BOTH (BMI088_GYRO_DRDY_IO_INT3 | BMI088_GYRO_DRDY_IO_INT4)

#define BMI088_GYRO_SELF_TEST 0x3C		// 自检寄存器地址
#define BMI088_GYRO_RATE_OK_SHFITS 0x4
#define BMI088_GYRO_RATE_OK (0x1 << BMI088_GYRO_RATE_OK_SHFITS)		// 持续自检正常标志（位 4）
#define BMI088_GYRO_BIST_FAIL_SHFITS 0x2
#define BMI088_GYRO_BIST_FAIL (0x1 << BMI088_GYRO_BIST_FAIL_SHFITS)	// 自检失败标志
#define BMI088_GYRO_BIST_RDY_SHFITS 0x1
#define BMI088_GYRO_BIST_RDY (0x1 << BMI088_GYRO_BIST_RDY_SHFITS)
#define BMI088_GYRO_TRIG_BIST_SHFITS 0x0
#define BMI088_GYRO_TRIG_BIST (0x1 << BMI088_GYRO_TRIG_BIST_SHFITS)	// 触发自检（位 0）






#ifdef __cplusplus
}
#endif

#endif /* __BMI088REG_H__ */
