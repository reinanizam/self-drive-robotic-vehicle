#pragma once
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <string>
#include <sstream>
#include <iomanip>
#include <deque>
using byte = uint8_t;
const int LOW = 0, HIGH = 1, INPUT = 0, OUTPUT = 1, INPUT_PULLUP = 2;
const int A0 = 14, A1 = 15, A2 = 16;
#define F(value) value
typedef char __FlashStringHelper;
inline unsigned long simulatedUs = 0;
inline int inputPins[32] = {};
inline int pwmPins[32] = {};
inline unsigned long echoUs = 0;
inline unsigned long micros() { return simulatedUs; }
inline unsigned long millis() { return simulatedUs / 1000UL; }
inline void delay(unsigned long ms) { simulatedUs += ms * 1000UL; }
inline void delayMicroseconds(unsigned int us) { simulatedUs += us; }
inline void pinMode(int, int) {}
inline int outputPins[32] = {};
inline void digitalWrite(int pin, int value) { outputPins[pin] = value; }
inline int digitalRead(int pin) { return inputPins[pin]; }
inline void analogWrite(int pin, int value) { pwmPins[pin] = value; }
inline unsigned long pulseIn(int, int, unsigned long timeout) {
  simulatedUs += echoUs ? echoUs : timeout;
  return echoUs;
}
inline void noTone(int) {}
inline void tone(int, unsigned int, unsigned long = 0) {}
inline long map(long x, long inLo, long inHi, long outLo, long outHi) {
  return (x - inLo) * (outHi - outLo) / (inHi - inLo) + outLo;
}
template<class A, class B> auto min(A a, B b) -> decltype(a + b) { return a < b ? a : b; }
template<class A, class B> auto max(A a, B b) -> decltype(a + b) { return a > b ? a : b; }
template<class A, class B, class C> A constrain(A a, B lo, C hi) {
  return a < lo ? (A)lo : a > hi ? (A)hi : a;
}
struct SerialMock {
  std::ostringstream output;
  std::deque<char> incoming;
  void begin(long) {}
  int available() { return (int)incoming.size(); }
  int read() { char value = incoming.front(); incoming.pop_front(); return value; }
  template<class T> void print(T value) { output << value; }
  void print(unsigned char value) { output << (int)value; }
  void print(float value, int decimals) { output << std::fixed << std::setprecision(decimals) << value; }
  template<class T> void println(T value) { print(value); output << '\n'; }
  void println() { output << '\n'; }
};
inline SerialMock Serial;
