#ifndef __JOLED_H__
#define __JOLED_H__

#include <stdint.h>

/* 1 = use oled_DMA backend, 0 = use blocking oled backend */
#ifndef JOLED_USE_DMA_OLED
#define JOLED_USE_DMA_OLED 1
#endif

#if JOLED_USE_DMA_OLED
#include "oled_DMA.h"
#else
#include "oled.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

void JOLED_Init(void);
void JOLED_Clear(void);
void JOLED_SetAutoRefresh(uint8_t enabled);
void JOLED_Refresh(void);

void JOLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void JOLED_ShowString(uint8_t Line, uint8_t Column, const char *String);

void JOLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void JOLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void JOLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void JOLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);

#ifdef __cplusplus
}
#endif

#endif /* __JOLED_H__ */
