#pragma once

// Host-test stub for <Preferences.h>: an in-memory NVS namespace, so the bond
// list persistence path (putBytes/getBytes) runs for real without flash.
// fakeble::resetWorld() empties it between tests.

#include <stddef.h>
#include <stdint.h>

#include <string>

class Preferences {
 public:
  bool begin(const char* name, bool readOnly = false);
  void end() {}
  uint8_t getUChar(const char* key, uint8_t defaultValue = 0);
  size_t getBytes(const char* key, void* buf, size_t maxLen);
  size_t putUChar(const char* key, uint8_t value);
  size_t putBytes(const char* key, const void* buf, size_t len);

 private:
  std::string ns_;
};
