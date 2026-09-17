#pragma once

// Host-test stub for <Arduino.h>.
//
// The BLE HID host public header includes <Arduino.h> (for `byte`) and its
// central-role path prints a few unconditional diagnostics through Serial. Both
// are provided here without any hardware: the clock is the fake, test-controlled
// clock in FakeBle.cpp, so the stale-release timeout in poll() can be aged
// deterministically, and Serial swallows output so a test run stays quiet.

#include <stddef.h>
#include <stdint.h>

// Arduino-ESP32's Arduino.h pulls in FreeRTOS; the library's diagnostics and its
// connection task rely on that.
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

typedef uint8_t byte;

// Monotonic fake clock (FakeBle.cpp); starts at fakeble::nowMs()'s origin.
unsigned long millis();

struct SerialStub {
  void print(char) {}
  void print(const char*) {}
  void print(int) {}
  void println(const char* = "") {}
  void printf(const char*, ...) {}
};

extern SerialStub Serial;
