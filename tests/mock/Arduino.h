// Just enough Arduino to run monitor.ino on a PC against simulated buses (see test_hold.cpp).
#pragma once
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <deque>
#include <vector>
#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT_PULLUP 2
extern uint32_t simUs;
inline uint32_t micros() { return simUs += 2; }
inline uint32_t millis() { return simUs / 1000; }
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int digitalRead(int) { return HIGH; }
struct HardwareSerial {
  std::deque<std::pair<uint32_t, uint8_t>> rx;           // (time it arrives, byte)
  std::vector<std::vector<uint8_t>> sent;
  void (*onSend)(const std::vector<uint8_t> &) = nullptr;
  void setTX(int) {} void setRX(int) {} void setFIFOSize(int) {} void begin(long) {}
  int available() { return !rx.empty() && (int32_t)(simUs - rx.front().first) >= 0; }
  int read() { int b = rx.front().second; rx.pop_front(); return b; }
  void write(const uint8_t *p, int n) {
    std::vector<uint8_t> v(p, p + n);
    for (uint8_t b : v) rx.push_back({simUs += 10, b});  // 1 Mbaud, and we hear our own bytes
    sent.push_back(v);
    if (onSend) onSend(v);
  }
  void flush() {}
};
extern HardwareSerial Serial1, Serial2;
struct Usb { void begin(long) {} int availableForWrite() { return 0; } void printf(const char *, ...) {} };
extern Usb Serial;
