#pragma once
#include "Arduino.h"
inline char lcdBuf[2][17] = {"                ","                "};
struct LiquidCrystal_I2C {
  int row = 0, col = 0;
  LiquidCrystal_I2C(int, int, int) {}
  void init() {}
  void backlight() {}
  void clear() {}
  void setCursor(int c, int r) { col = c; row = r; }
  void print(const char *) {}
  void write(uint8_t ch) { if (row >= 0 && row < 2 && col >= 0 && col < 16) lcdBuf[row][col] = (char)ch; col++; }
};
