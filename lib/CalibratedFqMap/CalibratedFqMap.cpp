#include "CalibratedFqMap.h"

CalibratedFqMap::CalibratedFqMap() = default;

CalibratedFqValue CalibratedFqMap::nearestForTemperatureC(float temperatureC) const {
  const int16_t temperatureDeciC = temperatureToDeciC(temperatureC);
  if (count_ == 0) return {temperatureDeciC, DEFAULT_TICKS_PER_SECOND};
  const Point* selected = &points_[0];
  for (uint8_t i = 1; i < count_; ++i) {
    if (temperatureDeciC < points_[i].temperatureDeciC) break;
    selected = &points_[i];
  }
  return {selected->temperatureDeciC, selected->ticksPerSecond};
}

bool CalibratedFqMap::updateFromPps(float temperatureC, uint32_t measuredTicksPerSecond,
                                    uint32_t frequencyUpdateThresholdHz) {
  if (measuredTicksPerSecond == 0) return false;
  const int16_t temperatureDeciC = temperatureToDeciC(temperatureC);
  for (uint8_t i = 0; i < count_; ++i) {
    if (temperatureDeciC == points_[i].temperatureDeciC) {
      const uint32_t old = points_[i].ticksPerSecond;
      const uint32_t difference = old > measuredTicksPerSecond
                                      ? old - measuredTicksPerSecond
                                      : measuredTicksPerSecond - old;
      if (difference <= frequencyUpdateThresholdHz) return false;
      points_[i].ticksPerSecond = measuredTicksPerSecond;
      return true;
    }
  }
  if (count_ >= MAX_POINTS) return false;
  points_[count_++] = {temperatureDeciC, measuredTicksPerSecond};
  for (uint8_t i = count_ - 1; i > 0 && points_[i].temperatureDeciC < points_[i - 1].temperatureDeciC; --i) {
    const Point temporary = points_[i];
    points_[i] = points_[i - 1];
    points_[i - 1] = temporary;
  }
  return true;
}

String CalibratedFqMap::serialize() const {
  String output;
  output.reserve(count_ * 16);
  for (uint8_t i = 0; i < count_; ++i) {
    if (i != 0) output += '\n';
    output += String(static_cast<float>(points_[i].temperatureDeciC) / 10.0f, 1) + ',' +
              String(points_[i].ticksPerSecond);
  }
  return output;
}

bool CalibratedFqMap::deserialize(const String& serialized) {
  count_ = 0;
  int start = 0;
  while (start < serialized.length() && count_ < MAX_POINTS) {
    int end = serialized.indexOf('\n', start);
    if (end < 0) end = serialized.length();
    String line = serialized.substring(start, end);
    line.trim();
    const int comma = line.indexOf(',');
    if (comma > 0 && line.substring(0, comma).indexOf('.') >= 0) {
      const float temperatureC = line.substring(0, comma).toFloat();
      const long temperature = static_cast<long>(temperatureC * 10.0f +
                                                 (temperatureC >= 0.0f ? 0.5f : -0.5f));
      const uint32_t frequency = static_cast<uint32_t>(line.substring(comma + 1).toInt());
      if (temperature >= INT16_MIN && temperature <= INT16_MAX && frequency != 0) {
        points_[count_++] = {static_cast<int16_t>(temperature), frequency};
      }
    }
    start = end + 1;
  }
  for (uint8_t i = 1; i < count_; ++i) {
    for (uint8_t j = i; j > 0 && points_[j].temperatureDeciC < points_[j - 1].temperatureDeciC; --j) {
      const Point temporary = points_[j];
      points_[j] = points_[j - 1];
      points_[j - 1] = temporary;
    }
  }
  return count_ > 0 || serialized.length() == 0;
}

int16_t CalibratedFqMap::temperatureToDeciC(float temperatureC) {
  return static_cast<int16_t>(temperatureC * 10.0f + (temperatureC >= 0.0f ? 0.5f : -0.5f));
}
