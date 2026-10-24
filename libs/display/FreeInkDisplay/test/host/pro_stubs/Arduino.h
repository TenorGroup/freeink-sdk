#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#define HIGH 1
#define PROGMEM
#define pgm_read_byte(p) (*(p))
inline unsigned long millis() { static unsigned long t; return ++t; }
inline void delay(unsigned long) {}
inline int digitalRead(int) { return 0; }

#include <cassert>
inline bool proSpiTransaction = false;
#ifndef PRO_STUB_NO_SERIAL
struct ProSerial {
  template<class... Args> void printf(const char* format, Args... args) {
    assert(!proSpiTransaction);
    if constexpr (sizeof...(Args) == 0) std::fputs(format, stdout);
    else std::printf(format, args...);
  }
};
inline ProSerial Serial;
#endif
