#pragma once

#include <Arduino.h>

struct CalibratedFqValue {
  int16_t temperatureDeciC;
  uint32_t ticksPerSecond;
};

class CalibratedFqMap {
 public:
  static constexpr uint32_t DEFAULT_TICKS_PER_SECOND = 80000000UL;

  static CalibratedFqValue nearestForTemperatureC(float temperatureC);

 private:
  static int16_t temperatureToDeciC(float temperatureC);
};
