#include "BootButton.h"

BootButton::BootButton(uint8_t pin,
                       uint8_t activeLevel,
                       uint32_t debounceMs,
                       uint32_t shortPressMaxMs,
                       uint32_t longPressMs)
    : pin_(pin),
      activeLevel_(activeLevel),
      debounceMs_(debounceMs),
      shortPressMaxMs_(shortPressMaxMs),
      longPressMs_(longPressMs) {}

void BootButton::begin() {
  pinMode(pin_, activeLevel_ == LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
}

BootButtonEvent BootButton::poll(uint32_t nowMs, uint32_t* heldMsOut) {
  if (heldMsOut) {
    *heldMsOut = 0;
  }

  const bool pressedPhysical = digitalRead(pin_) == activeLevel_;
  if (pressedPhysical != lastPhysicalPressed_) {
    lastPhysicalPressed_ = pressedPhysical;
    lastEdgeMs_ = nowMs;
  }

  if (nowMs - lastEdgeMs_ < debounceMs_) {
    return BOOT_BUTTON_NONE;
  }

  if (pressedPhysical == stablePressed_) {
    if (stablePressed_) {
      const uint32_t heldMs = nowMs - pressedAtMs_;
      if (heldMsOut) {
        *heldMsOut = heldMs;
      }
      if (!longPressConsumed_ && heldMs >= longPressMs_) {
        longPressConsumed_ = true;
        return BOOT_BUTTON_LONG_PRESS;
      }
    }
    return BOOT_BUTTON_NONE;
  }

  stablePressed_ = pressedPhysical;
  if (stablePressed_) {
    pressedAtMs_ = nowMs;
    longPressConsumed_ = false;
    return BOOT_BUTTON_NONE;
  }

  const uint32_t heldMs = nowMs - pressedAtMs_;
  if (heldMsOut) {
    *heldMsOut = heldMs;
  }
  if (!longPressConsumed_ && heldMs <= shortPressMaxMs_) {
    return BOOT_BUTTON_SHORT_PRESS;
  }

  return BOOT_BUTTON_NONE;
}
