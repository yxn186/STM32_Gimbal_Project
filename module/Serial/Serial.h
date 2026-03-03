/**
 * @file    serial.h
 * @brief   STM32 HAL serial module header
 */
#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <stdint.h>
#include <stdarg.h>
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/* ============================= USART/DMA 配置 ============================== */
/* ========================================================================== */
/**
 * @brief Serial UART handle (default: huart1 / USART1).
 * @note  Redefine SERIAL_UART_HANDLE before including Serial.h if needed.
 */
#ifndef SERIAL_UART_HANDLE
#define SERIAL_UART_HANDLE huart1
#endif

/**
 * @brief STM32F405RGT6 + USART1 的推荐 DMA 映射
 *        RX: DMA2_Stream2 Channel4
 *        TX: DMA2_Stream7 Channel4
 */
#ifndef SERIAL_UART_RX_DMA_STREAM
#define SERIAL_UART_RX_DMA_STREAM DMA2_Stream2
#endif

#ifndef SERIAL_UART_TX_DMA_STREAM
#define SERIAL_UART_TX_DMA_STREAM DMA2_Stream7
#endif

#ifndef SERIAL_UART_RX_DMA_CHANNEL
#define SERIAL_UART_RX_DMA_CHANNEL DMA_CHANNEL_4
#endif

#ifndef SERIAL_UART_TX_DMA_CHANNEL
#define SERIAL_UART_TX_DMA_CHANNEL DMA_CHANNEL_4
#endif

/* ========================================================================== */
/* ========================= 用户可配置：包头包尾序列 ========================= */
/* ========================================================================== */
/**
 * @section SerialPacketConfig 包头包尾配置说明（非常重要）
 *
 * 【你提到“不直观”】【所以这里做了升级】：
 * - 你依旧可以像以前那样手动写 SERIAL_PKT_HEAD_LEN / BYTES 等（兼容旧写法）
 * - 但更推荐使用下面新增的“预设切换”方式：只改一行 SERIAL_FRAME_PRESET 即可完成切换
 *
 * 本模块把“包头/包尾”定义为：字节序列（Byte Sequence）
 * - SERIAL_PKT_HEAD_LEN  ：包头长度（字节数）
 * - SERIAL_PKT_HEAD_BYTES：包头内容（字节数组初始化器）
 * - SERIAL_PKT_TAIL_LEN  ：包尾长度（字节数）
 * - SERIAL_PKT_TAIL_BYTES：包尾内容（字节数组初始化器）
 *
 * 你只需要改这一块，不需要改 .c 的解析逻辑。
 *
 * @note
 * 1) 这里用“字节数组初始化器”的形式：{ ... }
 *    例如：{ '[' }、{ 0xAA }、{ '\r', '\n' }、{ 'C','M','D',':' }
 * 2) '\r' 和 '\n' 是单字节字符常量，分别是 0x0D 和 0x0A
 */

/* -------------------------------------------------------------------------- */
/* 【推荐】更直观的切换方式：预设模式（只改 1 行）                               */
/* -------------------------------------------------------------------------- */
/**
 * @brief 包头包尾预设选择开关（只改这一行就能切换）
 *
 * 可选预设：
 * - SERIAL_PRESET_BRACKET_BRACKET : 包头'['  包尾']'        => [payload]
 * - SERIAL_PRESET_BRACKET_CRLF    : 包头'['  包尾"]\r\n"     => [payload]\r\n
 * - SERIAL_PRESET_AA_55           : 包头0xAA 包尾0x55       => AA payload 55
 * - SERIAL_PRESET_CMD_LF          : 包头"CMD:" 包尾'\n'     => CMD:payload\n
 * - SERIAL_PRESET_CUSTOM          : 自定义（只需填 HEAD/Tail 两个初始化器）
 *
 * @note
 * 你如果还想继续用“旧写法”（手动定义 SERIAL_PKT_HEAD_LEN 等），也完全可以：
 * - 只要你在别处先 #define SERIAL_PKT_HEAD_LEN / BYTES ...
 * - 本头文件内部会用 #ifndef 判断，不会覆盖你手动写的配置
 */
#ifndef SERIAL_FRAME_PRESET
#define SERIAL_FRAME_PRESET  SERIAL_PRESET_BRACKET_CRLF
#endif

/* 预设枚举值（一般不用改） */
#define SERIAL_PRESET_BRACKET_BRACKET   1
#define SERIAL_PRESET_BRACKET_CRLF      2
#define SERIAL_PRESET_AA_55             3
#define SERIAL_PRESET_CMD_LF            4
#define SERIAL_PRESET_CUSTOM            99

/* -------------------------------------------------------------------------- */
/* 仅当选择 SERIAL_PRESET_CUSTOM 时，改这里两行即可（仍然很直观）               */
/* -------------------------------------------------------------------------- */
/**
 * @brief 自定义包头包尾（仅当 SERIAL_FRAME_PRESET=SERIAL_PRESET_CUSTOM 生效）
 * @note  必须是“字节数组初始化器”，要带大括号
 *       例：{ (uint8_t)'[' }、{ 0xAA }、{ (uint8_t)'\r',(uint8_t)'\n' }
 */
#ifndef SERIAL_CUSTOM_HEAD_BYTES
#define SERIAL_CUSTOM_HEAD_BYTES   { (uint8_t)'[' }
#endif

#ifndef SERIAL_CUSTOM_TAIL_BYTES
#define SERIAL_CUSTOM_TAIL_BYTES   { (uint8_t)']' }
#endif

/* -------------------------------------------------------------------------- */
/* 下面是“内部映射层”：把预设 -> 转换成 serial.c 需要的四个宏                    */
/* 你一般不需要改动这里                                                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief 计算初始化器元素个数（编译期求值）
 * @details
 * 使用复合字面量技巧：
 * - (uint8_t[]){ ... } 会生成一个临时数组类型
 * - sizeof((uint8_t[]){...}) / sizeof(uint8_t) 就是元素个数
 *
 * @note
 * - 这是“预处理 + 编译期常量”手段，不会带来运行时代码
 * - CubeIDE(GCC) 下稳定可用
 */
#define SERIAL__COUNT_OF_INITLIST(initlist) \
    ((uint16_t)(sizeof((uint8_t[])initlist) / sizeof(uint8_t)))

/* 根据预设选择 HEAD/Tail 的“初始化器” */
#if (SERIAL_FRAME_PRESET == SERIAL_PRESET_BRACKET_BRACKET)
  #define SERIAL__HEAD_INIT   { (uint8_t)'[' }
  #define SERIAL__TAIL_INIT   { (uint8_t)']' }

#elif (SERIAL_FRAME_PRESET == SERIAL_PRESET_BRACKET_CRLF)
  #define SERIAL__HEAD_INIT   { (uint8_t)'[' }
  #define SERIAL__TAIL_INIT   { (uint8_t)']',(uint8_t)'\r', (uint8_t)'\n' }

#elif (SERIAL_FRAME_PRESET == SERIAL_PRESET_AA_55)
  #define SERIAL__HEAD_INIT   { 0xAA }
  #define SERIAL__TAIL_INIT   { 0x55 }

#elif (SERIAL_FRAME_PRESET == SERIAL_PRESET_CMD_LF)
  #define SERIAL__HEAD_INIT   { (uint8_t)'C',(uint8_t)'M',(uint8_t)'D',(uint8_t)':' }
  #define SERIAL__TAIL_INIT   { (uint8_t)'\n' }

#elif (SERIAL_FRAME_PRESET == SERIAL_PRESET_CUSTOM)
  #define SERIAL__HEAD_INIT   SERIAL_CUSTOM_HEAD_BYTES
  #define SERIAL__TAIL_INIT   SERIAL_CUSTOM_TAIL_BYTES

#else
  #error "SERIAL_FRAME_PRESET value is invalid. Please choose a valid preset."
#endif

/* 最终输出给 serial.c 使用的四个宏：如果用户手动定义过，则不覆盖（兼容旧写法） */
#ifndef SERIAL_PKT_HEAD_LEN
#define SERIAL_PKT_HEAD_LEN     SERIAL__COUNT_OF_INITLIST(SERIAL__HEAD_INIT)
#endif
#ifndef SERIAL_PKT_HEAD_BYTES
#define SERIAL_PKT_HEAD_BYTES   SERIAL__HEAD_INIT
#endif

#ifndef SERIAL_PKT_TAIL_LEN
#define SERIAL_PKT_TAIL_LEN     SERIAL__COUNT_OF_INITLIST(SERIAL__TAIL_INIT)
#endif
#ifndef SERIAL_PKT_TAIL_BYTES
#define SERIAL_PKT_TAIL_BYTES   SERIAL__TAIL_INIT
#endif

/**
 * @def SERIAL_PKT_MAX_LEN
 * @brief payload 最大长度（不含包头/包尾）
 * @note  用于保护上层缓冲，避免一次拷贝过多数据
 */
#ifndef SERIAL_PKT_MAX_LEN
#define SERIAL_PKT_MAX_LEN  256
#endif

/* ========================================================================== */
/* ============================= 初始化接口 ================================= */
/* ========================================================================== */

/**
 * @brief  串口模块初始化（启动 USART1 的 ReceiveToIdle DMA）
 * @details
 * 调用逻辑（必须按顺序）：
 * 1) MX_USART1_UART_Init();   // CubeMX生成：初始化串口+DMA句柄
 * 2) Serial_Init();           // 本函数：启动 ReceiveToIdle DMA 接收
 *
 * 本函数做的事情：
 * - 启动 DMA 接收：HAL_UARTEx_ReceiveToIdle_DMA()
 * - 关闭 DMA 半传输中断（HT），减少回调次数，降低复杂度
 */
void Serial_Init(void);

/* ========================================================================== */
/* ======================== 发送接口（STM32->PC） =========================== */
/* ========================================================================== */

/**
 * @brief  DMA 环形队列发送：把数据写入 TX 环形缓冲，由 DMA 后台发送
 * @param  buf  数据指针
 * @param  len  数据长度
 * @return 实际写入环形缓冲的字节数（队列满会少写）
 *
 * @note
 * - 非阻塞：函数不会等待 DMA 发完
 * - 数据会拷贝进 TX 环形缓冲，所以 buf 可以是局部变量
 * - 若返回值 < len，说明 TX 队列满了（你发太快）
 */
uint16_t Serial_SendArray_DMA(const uint8_t *buf, uint16_t len);

/**
 * @brief  发送字符串（不包含 '\0'）
 * @param  str  以 '\0' 结尾的 C 字符串
 * @return 实际写入环形缓冲的字节数
 */
uint16_t Serial_SendString_DMA(const char *str);

/**
 * @brief  printf 同语法输出（推荐用于调试输出到上位机）
 * @param  fmt  格式字符串
 * @param  ...  可变参数
 * @return 实际进入 TX 队列的字节数（队列满会少）
 *
 * @note
 * - 内部使用 vsnprintf 防止溢出
 * - 若你用 "%.2f" 打印 float 失败，需要在工程里开启 printf float 支持
 */
int      Serial_Printf(const char *fmt, ...);

/* ========================================================================== */
/* ======================== 接收接口（PC->STM32） =========================== */
/* ========================================================================== */

/**
 * @brief  RX 环形缓冲中当前可读取的字节数（字节流）
 */
uint16_t Serial_RxAvailable(void);

/**
 * @brief  读取 1 字节（非阻塞）
 * @param  out 输出指针
 * @return 1=读到；0=没数据
 */
uint8_t  Serial_ReadByte(uint8_t *out);

/**
 * @brief  读取多字节（非阻塞）
 * @param  out 输出缓冲
 * @param  len 期望读取长度
 * @return 实际读取长度（可能小于 len）
 */
uint16_t Serial_Read(uint8_t *out, uint16_t len);

/* ========================================================================== */
/* ======================= 简易拆包：包头序列 + payload + 包尾序列 ======================= */
/* ========================================================================== */

/**
 * @brief  非阻塞尝试解析一包： [HEAD_SEQ][payload...][TAIL_SEQ]
 * @param  payload_out  输出payload缓冲（不包含HEAD/TAIL）
 * @param  out_max      payload_out 最大容量
 * @return payload长度；若当前未形成完整包则返回0
 *
 * @details 非阻塞与“扫描一次”原则
 * - 本函数不会 while(1) 清空垃圾直到成功（那会造成不可控的耗时）
 * - 每次调用最多做一次“在当前缓存中扫描 HEAD/Tail 并尝试取一包”
 * - 没有完整包就立刻返回 0，留到下一次调用继续解析
 *
 * @details 解析策略（简单且工程上好用）
 * 1) 在 RX 缓冲中寻找 HEAD 序列
 * 2) 若找不到：丢弃一部分旧数据（避免RX塞满），返回0
 * 3) 若找到 HEAD：丢弃 HEAD 之前的噪声
 * 4) 从 HEAD 之后寻找 TAIL 序列
 *    - 找不到：说明包没收全，保留数据，返回0
 *    - 找到：复制 payload，丢弃整个包，返回 payload_len
 *
 * @note
 * - payload_out 不会自动加 '\0'，如果你把 payload 当字符串用，请自己补 0
 */
uint16_t Serial_TryGetPacket(uint8_t *payload_out, uint16_t out_max);

#ifdef __cplusplus
}
#endif

#endif /* __SERIAL_H__ */
