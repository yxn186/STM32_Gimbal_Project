/**
 * @file    serial.c
 * @brief   STM32 HAL 串口模块（USART1 + DMA + 环形缓冲）实现文件
 * @author  yxn + GPT
 * @version 1.1
 * @date    2026-02-05
 *
 * @details
 * 设计说明（尽量贴近你标准库那套思路）：
 *
 * TX（STM32 -> 电脑）：
 * - 使用 TX 环形队列（s_tx_ring + head/tail）
 * - Serial_SendArray_DMA / Serial_Printf 把数据写入 head
 * - 如果 DMA 空闲：Tx_Kick_Unsafe() 从 tail 开始发送一段“连续内存”
 * - DMA 发送完成回调 HAL_UART_TxCpltCallback() 推进 tail，并再次 kick 发送剩余数据
 *
 * RX（电脑 -> STM32）：
 * - 使用 HAL_UARTEx_ReceiveToIdle_DMA() 接收
 * - 每次回调 HAL_UARTEx_RxEventCallback() 得到 Size
 * - 将 DMA 临时缓冲 s_rx_dma_buf 写入 RX 环形缓冲 s_rx_ring
 * - 主循环通过 Serial_TryGetPacket / Serial_ReadByte / Serial_Read 获取数据
 *
 * 非阻塞保证：
 * - 发送永不等待 DMA 完成
 * - TryGetPacket 每次最多扫描一次当前缓存，不会无限 while 清垃圾
 */

#include "Serial.h"             
#include "usart.h"               /* CubeMX生成：extern UART_HandleTypeDef huart1; */
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>

/* ========================================================================== */
/* =========================== 参数配置区（可改） ============================ */
/* ========================================================================== */

/** RX DMA 临时缓存大小：越大回调频率越低，但延迟略增 */
#define SERIAL_RX_DMA_BUF_SIZE     64

/** RX 环形缓冲大小：上位机发得快就加大 */
#define SERIAL_RX_RING_SIZE        512

/** TX 环形队列大小：printf输出多就加大 */
#define SERIAL_TX_RING_SIZE        512

/** printf 临时格式化缓冲（static，避免占用栈） */
#define SERIAL_PRINTF_BUF_SIZE     256

/* ========================================================================== */
/* ====================== 包头包尾序列（来自 serial.h 宏） ==================== */
/* ========================================================================== */
/**
 * @brief 把 serial.h 里定义的“字节数组初始化器”变成真正的数组
 * @note  这样做的好处：解析时统一用数组+长度，支持任意序列（包括 \r\n）
 */
static const uint8_t s_pkt_head[] = SERIAL_PKT_HEAD_BYTES;
static const uint8_t s_pkt_tail[] = SERIAL_PKT_TAIL_BYTES;

/* ========================================================================== */
/* ============================= UART/DMA 绑定检查 =========================== */
/* ========================================================================== */
static uint8_t Serial_CheckDmaRoute(void)
{
    UART_HandleTypeDef *huart = &SERIAL_UART_HANDLE;

    if ((huart->hdmarx == NULL) || (huart->hdmatx == NULL))
    {
        return 0;
    }

    if (huart->hdmarx->Instance != SERIAL_UART_RX_DMA_STREAM)
    {
        return 0;
    }

    if (huart->hdmatx->Instance != SERIAL_UART_TX_DMA_STREAM)
    {
        return 0;
    }

    if (huart->hdmarx->Init.Channel != SERIAL_UART_RX_DMA_CHANNEL)
    {
        return 0;
    }

    if (huart->hdmatx->Init.Channel != SERIAL_UART_TX_DMA_CHANNEL)
    {
        return 0;
    }

    return 1;
}

/* ========================================================================== */
/* ============================== 内部变量 ================================= */
/* ========================================================================== */

/* ---------------- RX 相关 ---------------- */

/** RX DMA 临时缓存：HAL DMA 先写这里 */
static uint8_t  s_rx_dma_buf[SERIAL_RX_DMA_BUF_SIZE];

/** RX 环形缓冲：保存“字节流” */
static uint8_t  s_rx_ring[SERIAL_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0;   /**< 写指针：DMA回调推进 */
static volatile uint16_t s_rx_tail = 0;   /**< 读指针：主循环推进 */

/* ---------------- TX 相关（贴近你标准库结构） ---------------- */

/** TX 环形队列：DMA 从这里取数据发送 */
static uint8_t  s_tx_ring[SERIAL_TX_RING_SIZE];
static volatile uint16_t s_tx_head = 0;   /**< 写指针：Send/Printf推进 */
static volatile uint16_t s_tx_tail = 0;   /**< 读指针：DMA完成回调推进 */

static volatile uint8_t  s_tx_busy = 0;       /**< DMA是否正在发送 */
static volatile uint16_t s_tx_last_len = 0;   /**< 本次DMA发送长度，用于推进tail */

/* ========================================================================== */
/* ============================== 临界区 =================================== */
/* ========================================================================== */

/**
 * @brief 进入临界区（关全局中断）
 * @details
 * 为什么要关中断：
 * - head/tail 会在 main 和中断回调里同时修改
 * - 不关的话可能读到“半更新”的值，导致环形缓冲计算错误
 *
 * @note 临界区保持尽量短：只包围 head/tail 的读写、kick 的启动判断
 */
static inline void Serial_Lock(void)   { __disable_irq(); }
static inline void Serial_Unlock(void) { __enable_irq();  }

/* ========================================================================== */
/* ============================= RX 工具函数 ================================ */
/* ========================================================================== */

/**
 * @brief 计算 RX 环形缓冲已有数据量（unsafe：不加锁）
 */
static uint16_t Rx_Count_Unsafe(void)
{
    if (s_rx_head >= s_rx_tail) return (uint16_t)(s_rx_head - s_rx_tail);
    return (uint16_t)(SERIAL_RX_RING_SIZE - s_rx_tail + s_rx_head);
}

/**
 * @brief 计算 RX 环形缓冲剩余空间（unsafe：不加锁）
 * @details
 * 采用“空一个字节”方案区分满/空：
 * - 空：head == tail
 * - 满：next(head) == tail
 */
static uint16_t Rx_Free_Unsafe(void)
{
    return (uint16_t)(SERIAL_RX_RING_SIZE - 1 - Rx_Count_Unsafe());
}

/**
 * @brief 把 data 写入 RX 环形缓冲（unsafe：不加锁）
 * @param data 输入数据
 * @param len  写入长度
 * @return 实际写入长度（若空间不足会丢弃部分）
 */
static uint16_t Rx_Push_Unsafe(const uint8_t *data, uint16_t len)
{
    uint16_t free = Rx_Free_Unsafe();
    if (len > free) len = free;   /* 空间不足就丢弃超出部分（简单可靠） */

    for (uint16_t i = 0; i < len; i++)
    {
        s_rx_ring[s_rx_head] = data[i];
        s_rx_head++;
        if (s_rx_head >= SERIAL_RX_RING_SIZE) s_rx_head = 0;
    }
    return len;
}

/**
 * @brief 从 RX 环形缓冲“窥视”offset 位置的字节（unsafe：不加锁）
 * @param offset 相对tail的偏移（0表示tail当前字节）
 * @param out    输出字节
 * @return 1=成功；0=offset超出已有数据量
 */
static uint8_t Rx_PeekByte_Unsafe(uint16_t offset, uint8_t *out)
{
    uint16_t cnt = Rx_Count_Unsafe();
    if (offset >= cnt) return 0;

    uint16_t idx = (uint16_t)(s_rx_tail + offset);
    if (idx >= SERIAL_RX_RING_SIZE) idx = (uint16_t)(idx - SERIAL_RX_RING_SIZE);
    *out = s_rx_ring[idx];
    return 1;
}

/**
 * @brief 丢弃 RX 环形缓冲的 len 个字节（unsafe：不加锁）
 */
static void Rx_Drop_Unsafe(uint16_t len)
{
    uint16_t cnt = Rx_Count_Unsafe();
    if (len > cnt) len = cnt;

    s_rx_tail = (uint16_t)(s_rx_tail + len);
    if (s_rx_tail >= SERIAL_RX_RING_SIZE) s_rx_tail = (uint16_t)(s_rx_tail - SERIAL_RX_RING_SIZE);
}

/**
 * @brief 判断从 offset 开始，是否匹配某个字节序列 seq（unsafe：不加锁）
 * @param offset 相对tail的偏移
 * @param seq    序列指针
 * @param seq_len 序列长度
 * @return 1=匹配；0=不匹配或数据不足
 *
 * @details
 * 这是“支持多字节包头/包尾”的关键函数：
 * - 以前你只能比较单字节：b == HEAD
 * - 现在可以比较多字节：比如 '\r','\n'
 */
static uint8_t Rx_MatchSeq_Unsafe(uint16_t offset, const uint8_t *seq, uint16_t seq_len)
{
    uint16_t cnt = Rx_Count_Unsafe();
    if ((uint32_t)offset + seq_len > cnt) return 0;  /* 数据不够，直接不匹配 */

    for (uint16_t i = 0; i < seq_len; i++)
    {
        uint8_t b;
        Rx_PeekByte_Unsafe((uint16_t)(offset + i), &b);
        if (b != seq[i]) return 0;
    }
    return 1;
}

/* ========================================================================== */
/* ============================= TX 工具函数 ================================ */
/* ========================================================================== */

/**
 * @brief 计算 TX 环形队列已有待发送数据量（unsafe：不加锁）
 */
static uint16_t Tx_Count_Unsafe(void)
{
    if (s_tx_head >= s_tx_tail) return (uint16_t)(s_tx_head - s_tx_tail);
    return (uint16_t)(SERIAL_TX_RING_SIZE - s_tx_tail + s_tx_head);
}

/**
 * @brief 向 TX 环形队列写入数据（unsafe：不加锁）
 * @details
 * 采用你标准库那种“next==tail 判满”的写法：
 * - 好处：逻辑简单，满/空不歧义
 * - 代价：永远空 1 字节（最大可用容量 = size-1）
 * @return 实际写入字节数
 */
static uint16_t Tx_Write_Unsafe(const uint8_t *buf, uint16_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++)
    {
        uint16_t next = (uint16_t)(s_tx_head + 1);
        if (next >= SERIAL_TX_RING_SIZE) next = 0;

        /* 满：next 追上 tail */
        if (next == s_tx_tail) break;

        s_tx_ring[s_tx_head] = buf[i];
        s_tx_head = next;
    }
    return i;
}

/**
 * @brief DMA “空闲就发”启动函数（unsafe：不加锁）
 * @details
 * 若 DMA 空闲 && TX 队列有数据：
 * - 从 tail 开始，发送一段“连续内存”
 * - 如果 head 在 tail 前（环绕情况），则本次只发送到缓冲区末尾
 */
static void Tx_Kick_Unsafe(void)
{
    if (s_tx_busy) return;

    uint16_t cnt = Tx_Count_Unsafe();
    if (cnt == 0) return;

    uint16_t tx_len;
    if (s_tx_head >= s_tx_tail)
    {
        /* tail -> head 是连续内存 */
        tx_len = (uint16_t)(s_tx_head - s_tx_tail);
    }
    else
    {
        /* tail -> buffer末尾 是连续内存（先发这一段） */
        tx_len = (uint16_t)(SERIAL_TX_RING_SIZE - s_tx_tail);
    }

    s_tx_busy = 1;
    s_tx_last_len = tx_len;

    /* DMA 发送的数据来自 s_tx_ring，全局静态，生命周期足够 */
    if (HAL_UART_Transmit_DMA(&SERIAL_UART_HANDLE, (uint8_t*)&s_tx_ring[s_tx_tail], tx_len) != HAL_OK)
    {
        /* 若 HAL_BUSY 等导致启动失败：撤销 busy，等待下次kick */
        s_tx_busy = 0;
        s_tx_last_len = 0;
    }
}

/* ========================================================================== */
/* ============================= 对外接口实现 =============================== */
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
void Serial_Init(void)
{
    UART_HandleTypeDef *huart = &SERIAL_UART_HANDLE;

    if (Serial_CheckDmaRoute() == 0)
    {
        Error_Handler();
        return;
    }
    /**
     * 启动 ReceiveToIdle DMA：
     * - DMA 会把数据写到 s_rx_dma_buf
     * - 一旦出现“串口空闲”，HAL 会回调 HAL_UARTEx_RxEventCallback
     */
    HAL_UARTEx_ReceiveToIdle_DMA(huart, s_rx_dma_buf, SERIAL_RX_DMA_BUF_SIZE);

    /**
     * 关闭半传输中断（HT）：
     * - 否则 DMA 缓冲一半也可能触发回调，回调次数变多
     * - 初学阶段不需要处理 HT，关掉更省心
     */
    if (huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

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
uint16_t Serial_SendArray_DMA(const uint8_t *buf, uint16_t len)
{
    if (!buf || len == 0)
    {
        return 0;
    }

    Serial_Lock();
    uint16_t written = Tx_Write_Unsafe(buf, len); /* 把数据拷贝进TX环形队列 */
    Tx_Kick_Unsafe();                             /* 写完尝试启动DMA（若DMA忙则不影响） */
    Serial_Unlock();

    return written;
}

/**
 * @brief  发送字符串（不包含 '\0'）
 * @param  str  以 '\0' 结尾的 C 字符串
 * @return 实际写入环形缓冲的字节数
 */
uint16_t Serial_SendString_DMA(const char *str)
{
    if (!str) return 0;
    return Serial_SendArray_DMA((const uint8_t*)str, (uint16_t)strlen(str));
}

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
int Serial_Printf(const char *fmt, ...)
{
    if (!fmt) return 0;

    /**
     * 这里使用 static 缓冲：
     * - 避免在栈上开大数组
     * - 注意：static 让函数“不可重入”，但一般不会在中断里同时 printf
     */
    static char s_printf_buf[SERIAL_PRINTF_BUF_SIZE];

    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(s_printf_buf, sizeof(s_printf_buf), fmt, ap);
    va_end(ap);

    if (n <= 0) return n;

    /* vsnprintf 返回“本应写入长度”，若>=bufsize表示被截断 */
    uint16_t send_len = (n >= (int)sizeof(s_printf_buf))
                      ? (uint16_t)(sizeof(s_printf_buf) - 1)
                      : (uint16_t)n;

    return (int)Serial_SendArray_DMA((const uint8_t*)s_printf_buf, send_len);
}

/* ---------------- RX 读取接口 ---------------- */

uint16_t Serial_RxAvailable(void)
{
    Serial_Lock();
    uint16_t n = Rx_Count_Unsafe();
    Serial_Unlock();
    return n;
}

uint8_t Serial_ReadByte(uint8_t *out)
{
    if (!out) return 0;

    Serial_Lock();
    if (Rx_Count_Unsafe() == 0)
    {
        Serial_Unlock();
        return 0;
    }

    *out = s_rx_ring[s_rx_tail];
    s_rx_tail++;
    if (s_rx_tail >= SERIAL_RX_RING_SIZE) s_rx_tail = 0;

    Serial_Unlock();
    return 1;
}

uint16_t Serial_Read(uint8_t *out, uint16_t len)
{
    if (!out || len == 0) return 0;

    Serial_Lock();
    uint16_t cnt = Rx_Count_Unsafe();
    if (len > cnt) len = cnt;

    for (uint16_t i = 0; i < len; i++)
    {
        out[i] = s_rx_ring[s_rx_tail];
        s_rx_tail++;
        if (s_rx_tail >= SERIAL_RX_RING_SIZE) s_rx_tail = 0;
    }
    Serial_Unlock();

    return len;
}

/* ========================================================================== */
/* ========================= 非阻塞拆包：支持多字节HEAD/TAIL ================== */
/* ========================================================================== */

uint16_t Serial_TryGetPacket(uint8_t *payload_out, uint16_t out_max)
{
    if (!payload_out || out_max == 0) return 0;

    Serial_Lock();

    uint16_t cnt = Rx_Count_Unsafe();

    /* 最小要求：至少要能容纳 HEAD + TAIL */
    if (cnt < (uint16_t)(SERIAL_PKT_HEAD_LEN + SERIAL_PKT_TAIL_LEN))
    {
        Serial_Unlock();
        return 0;
    }

    /**
     * 非阻塞原则：
     * - 本函数不使用 while(1) 反复清垃圾
     * - 每次调用只扫描一次当前缓存，最多取出一包
     * - 没找到完整包就立刻返回 0
     */

    /* 1) 扫描 HEAD 序列的位置 head_pos */
    uint16_t head_pos = 0xFFFF;
    for (uint16_t i = 0; i + SERIAL_PKT_HEAD_LEN <= cnt; i++)
    {
        if (Rx_MatchSeq_Unsafe(i, s_pkt_head, SERIAL_PKT_HEAD_LEN))
        {
            head_pos = i;
            break;
        }
    }

    if (head_pos == 0xFFFF)
    {
        /**
         * 没找到 HEAD：当前缓存里可能全是噪声/半截数据
         * 非阻塞策略：丢掉一部分旧数据，给未来留空间
         * - 这里保留 (HEAD_LEN-1) 个字节，避免“HEAD跨边界”被误删
         */
        uint16_t keep = (SERIAL_PKT_HEAD_LEN > 1) ? (uint16_t)(SERIAL_PKT_HEAD_LEN - 1) : 1;
        if (cnt > keep) Rx_Drop_Unsafe((uint16_t)(cnt - keep));
        Serial_Unlock();
        return 0;
    }

    /* 2) 丢掉 HEAD 之前的噪声（一次性丢，不循环） */
    if (head_pos > 0)
    {
        Rx_Drop_Unsafe(head_pos);
        cnt = Rx_Count_Unsafe();

        if (cnt < (uint16_t)(SERIAL_PKT_HEAD_LEN + SERIAL_PKT_TAIL_LEN))
        {
            Serial_Unlock();
            return 0;
        }
    }

    /* 此时 tail 对应位置就是 HEAD 开始 */
    if (!Rx_MatchSeq_Unsafe(0, s_pkt_head, SERIAL_PKT_HEAD_LEN))
    {
        /* 理论不该发生，保护：丢1字节退出 */
        Rx_Drop_Unsafe(1);
        Serial_Unlock();
        return 0;
    }

    /**
     * 3) 从 HEAD 后开始扫描 TAIL 序列
     * tail_pos 表示：TAIL 序列的起始位置（相对当前tail偏移）
     */
    uint16_t tail_pos = 0xFFFF;
    cnt = Rx_Count_Unsafe();

    /* TAIL 必须出现在 HEAD 之后，所以从 head_len 开始找 */
    for (uint16_t i = SERIAL_PKT_HEAD_LEN;
         i + SERIAL_PKT_TAIL_LEN <= cnt;
         i++)
    {
        if (Rx_MatchSeq_Unsafe(i, s_pkt_tail, SERIAL_PKT_TAIL_LEN))
        {
            tail_pos = i;
            break;
        }
    }

    if (tail_pos == 0xFFFF)
    {
        /* 有HEAD但没TAIL：包没收全，保留数据等待下次调用 */
        Serial_Unlock();
        return 0;
    }

    /**
     * 4) 计算 payload 长度：
     * payload = HEAD 与 TAIL 之间的字节
     */
    uint16_t payload_len = (uint16_t)(tail_pos - SERIAL_PKT_HEAD_LEN);

    /* 防止异常超长包导致上层缓冲溢出 */
    if (payload_len > out_max)
    {
        /**
         * 简单策略：丢掉一个字节（通常丢掉HEAD起始），避免卡死在这个包上
         * 你也可以改成：丢整个HEAD序列，这里保持简单。
         */
        Rx_Drop_Unsafe(1);
        Serial_Unlock();
        return 0;
    }

    /* 5) 拷贝 payload */
    for (uint16_t i = 0; i < payload_len; i++)
    {
        uint8_t b;
        Rx_PeekByte_Unsafe((uint16_t)(SERIAL_PKT_HEAD_LEN + i), &b);
        payload_out[i] = b;
    }

    /**
     * 6) 丢弃整包：
     * 丢弃长度 = HEAD_LEN + payload_len + TAIL_LEN
     */
    Rx_Drop_Unsafe((uint16_t)(SERIAL_PKT_HEAD_LEN + payload_len + SERIAL_PKT_TAIL_LEN));

    Serial_Unlock();
    return payload_len;
}

/* ========================================================================== */
/* ============================= HAL 回调函数 =============================== */
/* ========================================================================== */

/**
 * @brief ReceiveToIdle DMA 接收回调
 * @param huart UART句柄
 * @param Size  本次收到的字节数（数据在 s_rx_dma_buf[0..Size-1]）
 *
 * @note
 * 回调里只做两件事（越简单越不容易出 bug）：
 * 1) 把 DMA 临时缓冲写入 RX 环形
 * 2) 重新启动下一轮 ReceiveToIdle DMA
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart != &SERIAL_UART_HANDLE) return;

    Serial_Lock();
    (void)Rx_Push_Unsafe(s_rx_dma_buf, Size);
    Serial_Unlock();

    HAL_UARTEx_ReceiveToIdle_DMA(huart, s_rx_dma_buf, SERIAL_RX_DMA_BUF_SIZE);

    if (huart->hdmarx != NULL)
    {
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

/**
 * @brief DMA 发送完成回调
 * @details
 * 发送完成后：
 * 1) 推进 tail（说明这一段已经发出去了）
 * 2) 清 busy
 * 3) 如果环形队列还有剩余数据，继续 kick 下一段
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &SERIAL_UART_HANDLE) return;

    Serial_Lock();

    /* 推进 tail */
    s_tx_tail = (uint16_t)(s_tx_tail + s_tx_last_len);
    if (s_tx_tail >= SERIAL_TX_RING_SIZE) s_tx_tail = (uint16_t)(s_tx_tail - SERIAL_TX_RING_SIZE);

    s_tx_last_len = 0;
    s_tx_busy = 0;

    /* 如果还有数据，继续发下一段 */
    Tx_Kick_Unsafe();

    Serial_Unlock();
}
