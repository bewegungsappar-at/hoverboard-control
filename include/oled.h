#pragma once

#include <U8g2lib.h>
#include "config.h"


void setupOLED(void);
void u8g2_prepare(void);

#if defined(TTGO)
  extern U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2;
#else
  extern U8G2_SSD1306_128X32_UNIVISION_1_HW_I2C u8g2;
#endif