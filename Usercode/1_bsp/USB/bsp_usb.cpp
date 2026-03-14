/**
 * @file bsp_usb.cpp
 * @brief 参照yssickjgd
 *
 */

/**
 * 由于Cube中 usbd_cdc_if.c 文件为C, 而我们用的是C++, 所以需将该文件扩展名改为cpp
 * static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len) 函数为实际接收回调函数, 在该文件中
 */

/* Includes ------------------------------------------------------------------*/

#include "bsp_usb.h"
#include <stdarg.h>
#include <stdio.h>

/*YOUR CODE*/

Struct_USB_Manage_Object USB0_Manage_Object = {nullptr};

// USB设备句柄, 由 usbd_cdc_if.c 中定义
extern USBD_HandleTypeDef hUsbDeviceFS;
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/**
 * @brief USB虚拟串口回调函数
 *
 * @param Buffer 接收缓冲区
 * @param Length 接收数据长度
 */
void USB_Serial_Callback(uint8_t *Buffer, uint16_t Length)
{
    //回显
    USB_Transmit_Data(Buffer, Length);
}

/**
 * @brief 初始化USB
 *
 * @param Callback_Function 处理回调函数
 */
void USB_Init(USB_Callback USB_Callback_Funtion)
{
    //设置回调函数参数
    //USB0_Manage_Object.Callback_Function = USB_Serial_Callback;

    //自己设置回调
    USB0_Manage_Object.Callback_Function = USB_Callback_Funtion;
    
    USB0_Manage_Object.Rx_Buffer_Active = UserRxBufferFS;
}

/**
 * @brief 发送数据帧
 *
 * @param Data 被发送的数据指针
 * @param Length 长度
 */
uint8_t USB_Transmit_Data(uint8_t *Data, uint16_t Length)
{
    return (CDC_Transmit_FS(Data, Length));
}

/**
 * @brief 自己写的USB通信下一轮接收开启前回调函数, 非HAL库回调函数
 *
 * @param Size 接收数据长度
 */
void USB_ReceiveCallback(uint16_t Size)
{
    //系统还没初始化完成时，不做自己的数据处理逻辑。
    //只把 USB OUT 端点重新挂好（re-arm），保证后续还能继续收包。
    //避免“初始化早期收到数据”导致访问未准备好的模块/回调。
    if (!init_finished)
    {
        USBD_CDC_SetRxBuffer(&hUsbDeviceFS, USB0_Manage_Object.Rx_Buffer_Active);
        USBD_CDC_ReceivePacket(&hUsbDeviceFS);
        return;
    }

    // 自设双缓冲USB
    USB0_Manage_Object.Rx_Buffer_Ready = USB0_Manage_Object.Rx_Buffer_Active;
    if (USB0_Manage_Object.Rx_Buffer_Active == USB0_Manage_Object.Rx_Buffer_0)
    {
        USB0_Manage_Object.Rx_Buffer_Active = USB0_Manage_Object.Rx_Buffer_1;
    }
    else
    {
        USB0_Manage_Object.Rx_Buffer_Active = USB0_Manage_Object.Rx_Buffer_0;
    }

    //USB0_Manage_Object.Rx_Time_Stamp = SYS_Timestamp.Get_Current_Timestamp();

    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, USB0_Manage_Object.Rx_Buffer_Active);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);

    if (USB0_Manage_Object.Callback_Function != nullptr)
    {
        USB0_Manage_Object.Callback_Function(USB0_Manage_Object.Rx_Buffer_Ready, Size);
    }
}

/**
 * @brief USB虚拟串口Printf函数
 * 
 * @param format 
 * @param ... 
 */
void USB_Printf(const char *format, ...)
{
    char printf_temp_buffer[256];

    va_list argument_list;
    va_start(argument_list, format);
    int length = vsnprintf(printf_temp_buffer, sizeof(printf_temp_buffer), format, argument_list);
    va_end(argument_list);

    if (length <= 0)
    {
        return;
    }

    if (length >= (int)sizeof(printf_temp_buffer))
    {
        length = (int)sizeof(printf_temp_buffer) - 1;
    }

    (void)USB_Transmit_Data((uint8_t *)printf_temp_buffer, (uint16_t)length);
}

