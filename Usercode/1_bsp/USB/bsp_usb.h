/**
 * @file bsp_usb.cpp
 * @brief 参照yssickjgd
 *
 */

/**
 * 双缓冲接收部分位于usbd_cdc_if.c中, 需要在usbd_cdc_if.c中, 如有需要可在此处修改
 * 具体函数为 static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
 * 此外, 为保证编译通过, user-include部分中, 需要额外声明 typedef int bool
 */

#ifndef DRV_USB_H
#define DRV_USB_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

//#include "1_Middleware/System/Timestamp/sys_timestamp.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include "stdbool.h"
/* Exported macros -----------------------------------------------------------*/

// 缓冲区字节长度
#define USB_BUFFER_SIZE 512

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART通信接收回调函数数据类型
 *
 */
typedef void (*USB_Callback)(uint8_t *Buffer, uint16_t Length);

/**
 * @brief USB通信处理结构体
 */
struct Struct_USB_Manage_Object
{
    UART_HandleTypeDef *UART_Handler;
    USB_Callback Callback_Function;

    // 双缓冲适配的缓冲区以及当前激活的缓冲区
    uint8_t Rx_Buffer_0[USB_BUFFER_SIZE];
    uint8_t Rx_Buffer_1[USB_BUFFER_SIZE];
    uint8_t *Rx_Buffer_Active;
    uint8_t *Rx_Buffer_Ready;

    // 接收时间戳
    uint64_t Rx_Time_Stamp;
};

/* Exported variables --------------------------------------------------------*/

extern bool init_finished;

extern struct Struct_USB_Manage_Object USB0_Manage_Object;

/* Exported function declarations --------------------------------------------*/
/**
 * @brief 初始化USB
 *
 * @param Callback_Function 处理回调函数
 */
void USB_Init(USB_Callback USB_Callback_Funtion);

uint8_t USB_Transmit_Data(uint8_t *Data, uint16_t Length);

void USB_Printf(const char *format, ...);

void USB_ReceiveCallback(uint16_t Size);

#ifdef __cplusplus
}
#endif

#endif

