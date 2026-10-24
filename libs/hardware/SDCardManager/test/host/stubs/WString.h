#pragma once

#include <string>
#include "Arduino.h"

class String : public std::string {
 public:
  using std::string::string;
  String(const std::string& value) : std::string(value) {}
  // Arduino's String reports whether the allocation held; the host one always does.
  bool reserve(size_t count) {
    std::string::reserve(count);
    return true;
  }
  bool concat(const char* data, unsigned int length) {
    append(data, length);
    return true;
  }
  bool endsWith(const char* suffix) const {
    const size_t count = std::char_traits<char>::length(suffix);
    return size() >= count && compare(size() - count, count, suffix) == 0;
  }
};
