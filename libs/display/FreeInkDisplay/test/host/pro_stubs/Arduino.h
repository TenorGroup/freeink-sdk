#pragma once
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#define HIGH 1
#define PROGMEM
#define pgm_read_byte(p) (*(p))
// Tests may step the host clock forward to cross time thresholds.
inline unsigned long hostNowMs = 0;
inline unsigned long millis() { return ++hostNowMs; }
inline void delay(unsigned long) {}
inline int digitalRead(int) { return 0; }
