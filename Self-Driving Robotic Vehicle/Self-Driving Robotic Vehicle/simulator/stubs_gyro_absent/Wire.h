#pragma once
#include "Arduino.h"
#include <cassert>
#define WIRE_HAS_TIMEOUT 1
struct WireMock {
  byte address = 0, reg = 0;
  int writes = 0, index = 0;
  int16_t gyro[3] = {131, -262, 393};
  int16_t accel[3] = {0, 0, 16384};
  bool failRead = false;
  void begin() {}
  void setWireTimeout(unsigned long, bool) {}
  void beginTransmission(byte a) { address = a; writes = 0; }
  void write(byte value) { if (writes++ == 0) reg = value; }
  int endTransmission(bool = true) { return 2; }
  byte requestFrom(byte, byte count) { index = 0; return failRead ? 0 : count; }
  int read() {
    if (reg == 0x75) return 0x68;
    assert(reg == 0x3B);
    int word = index / 2;
    int16_t value = word < 3 ? accel[word] : word == 3 ? 0 : gyro[word - 4];
    bool high = index++ % 2 == 0;
    return high ? (((uint16_t)value >> 8) & 255) : ((uint16_t)value & 255);
  }
};
inline WireMock Wire;
