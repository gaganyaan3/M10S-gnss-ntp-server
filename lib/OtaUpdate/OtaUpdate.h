#pragma once

#include <Arduino.h>

class OtaUpdate {
 public:
  void begin(const char* hostname, const char* passwordHash);
  void loop();
  bool started() const;

 private:
  bool started_ = false;
};
