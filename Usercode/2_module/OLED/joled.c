#include "joled.h"
#include <stddef.h>

// -------- 旧库兼容参数（固定 128x64 + 8x16 字符） --------
#define JOLED_SCREEN_W 128u
#define JOLED_SCREEN_H 64u

// 旧库：Line 1~4, Column 1~16, 使用 8x16 字体
#define JOLED_DEFAULT_FONT (&afont16x8)

typedef struct {
  const ASCIIFont *font;
  OLED_ColorMode color;
  uint8_t auto_refresh;
  uint8_t inited;
} JOLED_State;

static JOLED_State g_joled = {
    .font = JOLED_DEFAULT_FONT,
    .color = OLED_COLOR_NORMAL,
    .auto_refresh = 1,
    .inited = 0,
};

static inline uint8_t _joled_max_lines(void)
{
  // 64 / 16 = 4
  return (uint8_t)(JOLED_SCREEN_H / g_joled.font->h);
}

static inline uint8_t _joled_max_cols(void)
{
  // 128 / 8 = 16
  return (uint8_t)(JOLED_SCREEN_W / g_joled.font->w);
}

static inline uint8_t _joled_x_from_col(uint8_t col)
{
  return (uint8_t)((col - 1u) * g_joled.font->w);
}

static inline uint8_t _joled_y_from_line(uint8_t line)
{
  return (uint8_t)((line - 1u) * g_joled.font->h);
}

static inline void _joled_refresh_if_needed(void)
{
  if (g_joled.auto_refresh)
  {
    OLED_ShowFrame();
  }
}

// 仅写入显存，不主动刷新（用于批量显示数字/字符串）
static void _joled_draw_char_to_gram(uint8_t Line, uint8_t Column, char Char)
{
  if (!g_joled.inited)
    return;

  if (Line < 1 || Column < 1)
    return;

  if (Line > _joled_max_lines() || Column > _joled_max_cols())
    return;

  // 旧库只支持可见 ASCII。非法字符用空格代替。
  if (Char < ' ' || Char > '~')
    Char = ' ';

  uint8_t x = _joled_x_from_col(Column);
  uint8_t y = _joled_y_from_line(Line);
  OLED_PrintASCIIChar(x, y, Char, g_joled.font, g_joled.color);
}

static uint32_t _joled_pow_u32(uint32_t x, uint32_t y)
{
  uint32_t r = 1;
  while (y--)
  {
    r *= x;
  }
  return r;
}

void JOLED_Init(void)
{
  OLED_Init();
  g_joled.font = JOLED_DEFAULT_FONT;
  g_joled.color = OLED_COLOR_NORMAL;
  g_joled.auto_refresh = 1;
  g_joled.inited = 1;
}

void JOLED_SetAutoRefresh(uint8_t enabled)
{
  g_joled.auto_refresh = (enabled != 0);
}

void JOLED_Refresh(void)
{
  OLED_ShowFrame();
}

void JOLED_Clear(void)
{
  OLED_NewFrame();
  OLED_ShowFrame();
}

void JOLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{
  _joled_draw_char_to_gram(Line, Column, Char);
  _joled_refresh_if_needed();
}

void JOLED_ShowString(uint8_t Line, uint8_t Column, const char *String)
{
  if (!g_joled.inited)
    return;
  if (String == NULL)
    return;

  if (Line < 1 || Column < 1)
    return;
  if (Line > _joled_max_lines() || Column > _joled_max_cols())
    return;

  // 按旧库的列范围裁剪（1~16）。
  uint8_t col = Column;
  uint8_t idx = 0;
  while (String[idx] != '\0' && col <= _joled_max_cols())
  {
    _joled_draw_char_to_gram(Line, col, String[idx]);
    col++;
    idx++;
  }

  _joled_refresh_if_needed();
}

void JOLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
  if (!g_joled.inited)
    return;

  if (Line < 1 || Column < 1)
    return;
  if (Line > _joled_max_lines() || Column > _joled_max_cols())
    return;

  // 旧库：固定长度，前导 0
  for (uint8_t i = 0; i < Length; i++)
  {
    uint8_t col = (uint8_t)(Column + i);
    if (col > _joled_max_cols())
      break;

    uint32_t div = _joled_pow_u32(10u, (uint32_t)(Length - i - 1u));
    uint8_t digit = (uint8_t)((Number / div) % 10u);
    _joled_draw_char_to_gram(Line, col, (char)('0' + digit));
  }

  _joled_refresh_if_needed();
}

void JOLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
  if (!g_joled.inited)
    return;

  if (Line < 1 || Column < 1)
    return;
  if (Line > _joled_max_lines() || Column > _joled_max_cols())
    return;

  uint32_t absNum;
  if (Number >= 0)
  {
    _joled_draw_char_to_gram(Line, Column, '+');
    absNum = (uint32_t)Number;
  }
  else
  {
    _joled_draw_char_to_gram(Line, Column, '-');
    absNum = (uint32_t)(-Number);
  }

  // 后面 Length 位数字，从 Column+1 开始
  for (uint8_t i = 0; i < Length; i++)
  {
    uint8_t col = (uint8_t)(Column + 1u + i);
    if (col > _joled_max_cols())
      break;

    uint32_t div = _joled_pow_u32(10u, (uint32_t)(Length - i - 1u));
    uint8_t digit = (uint8_t)((absNum / div) % 10u);
    _joled_draw_char_to_gram(Line, col, (char)('0' + digit));
  }

  _joled_refresh_if_needed();
}

void JOLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
  if (!g_joled.inited)
    return;

  if (Line < 1 || Column < 1)
    return;
  if (Line > _joled_max_lines() || Column > _joled_max_cols())
    return;

  for (uint8_t i = 0; i < Length; i++)
  {
    uint8_t col = (uint8_t)(Column + i);
    if (col > _joled_max_cols())
      break;

    uint32_t div = _joled_pow_u32(16u, (uint32_t)(Length - i - 1u));
    uint8_t single = (uint8_t)((Number / div) % 16u);
    char ch = (single < 10u) ? (char)('0' + single) : (char)('A' + (single - 10u));
    _joled_draw_char_to_gram(Line, col, ch);
  }

  _joled_refresh_if_needed();
}

void JOLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
  if (!g_joled.inited)
    return;

  if (Line < 1 || Column < 1)
    return;
  if (Line > _joled_max_lines() || Column > _joled_max_cols())
    return;

  for (uint8_t i = 0; i < Length; i++)
  {
    uint8_t col = (uint8_t)(Column + i);
    if (col > _joled_max_cols())
      break;

    uint32_t div = _joled_pow_u32(2u, (uint32_t)(Length - i - 1u));
    uint8_t bit = (uint8_t)((Number / div) % 2u);
    _joled_draw_char_to_gram(Line, col, (char)('0' + bit));
  }

  _joled_refresh_if_needed();
}
