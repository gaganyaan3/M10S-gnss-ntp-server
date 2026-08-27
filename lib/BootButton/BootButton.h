#pragma once

#include <Arduino.h>

enum BootButtonEvent : uint8_t {
  BOOT_BUTTON_NONE = 0,
  BOOT_BUTTON_SHORT_PRESS = 1,
  BOOT_BUTTON_LONG_PRESS = 2,
};

class BootButton {
 public:
  BootButton(uint8_t pin,
             uint8_t activeLevel = LOW,
             uint32_t debounceMs = 40,
             uint32_t shortPressMaxMs = 400,
             uint32_t longPressMs = 3000);

  void begin();
  BootButtonEvent poll(uint32_t nowMs, uint32_t* heldMsOut = nullptr);

 private:
  uint8_t pin_;
  uint8_t activeLevel_;
  uint32_t debounceMs_;
  uint32_t shortPressMaxMs_;
  uint32_t longPressMs_;
  bool stablePressed_ = false;
  bool lastPhysicalPressed_ = false;
  bool longPressConsumed_ = false;
  uint32_t lastEdgeMs_ = 0;
  uint32_t pressedAtMs_ = 0;
};
