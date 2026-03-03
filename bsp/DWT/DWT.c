/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    DWT.c
  * @brief   DWT
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "DWT.h"
#include "core_cm4.h"   // F1是M3；如果是F4则 core_cm4.h（一般HAL会自动带对）

static uint32_t last_cyccnt = 0;
static uint64_t acc_cycles  = 0;

/**
 * @brief 初始化DWT
 * 
 */
void DWT_Init(void)
{
    // 使能 DWT/ITM 追踪模块
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    // 清零计数器
    DWT->CYCCNT = 0;

    // 使能CYCCNT
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    last_cyccnt = 0;
    acc_cycles  = 0;
}

/**
 * @brief 原封不动把当前 32位 cycle 读出来
 * 
 * @return uint32_t 
 */
uint32_t DWT_GetCYCCNT(void)
{
    return DWT->CYCCNT;//DWT->CYCCNT 是 硬件寄存器，表示“CPU 从启动以来走了多少个时钟周期（cycle）”
}

/**
 * @brief 把 32位的 CYCCNT 变成 64位累计 cycles（acc_cycles）（防止了外部处理回绕）
 * 
 * @return uint64_t 
 */
static uint64_t DWT_GetCycle64(void)
{
    uint32_t now = DWT->CYCCNT;
    uint32_t dt  = now - last_cyccnt;
    last_cyccnt  = now;

    acc_cycles += (uint64_t)dt;//累加
    return acc_cycles;
}

/**
 * @brief 取得当前微妙
 * 
 * @return uint64_t 
 */
uint64_t DWT_GetUs(void)
{
    uint64_t cycles = DWT_GetCycle64();
    uint32_t hz = SystemCoreClock;      // 当前HCLK（一般等于CPU主频）
    return (cycles * 1000000ULL) / hz;
}

/**
 * @brief 取得当前毫秒
 * 
 * @return uint64_t 
 */
uint64_t DWT_GetMs(void)
{
    uint64_t cycles = DWT_GetCycle64();
    uint32_t hz = SystemCoreClock;
    return (cycles * 1000ULL) / hz;
}
