#pragma once

#include <Arduino.h>

struct CalibratedFqValue {
  int16_t temperatureDeciC;
  uint32_t ticksPerSecond;
};

class CalibratedFqMap {
 public:
  static constexpr uint32_t DEFAULT_TICKS_PER_SECOND = 80000000UL;
  static constexpr uint8_t MAX_POINTS = 128;
  static constexpr uint32_t DEFAULT_FREQUENCY_UPDATE_TOLERANCE_HZ = 8;

  CalibratedFqMap();
  CalibratedFqValue nearestForTemperatureC(float temperatureC) const;
  bool updateFromPps(float temperatureC, uint32_t measuredTicksPerSecond,
                     uint32_t frequencyUpdateThresholdHz);
  bool deserialize(const String& serialized);
  String serialize() const;
  uint8_t size() const { return count_; }
  static int16_t temperatureToDeciC(float temperatureC);

 private:
  struct Point {
    int16_t temperatureDeciC;
    uint32_t ticksPerSecond;
  };
  Point points_[MAX_POINTS] = {};
  uint8_t count_ = 0;
};
