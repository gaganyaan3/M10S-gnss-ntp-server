#pragma once

#include <Arduino.h>

struct SysHealthSnapshot {
  float cpuUtilizationPercent = 0.0f;
  float cpuUtilizationPeakPercent = 0.0f;
  float internalTemperatureC = 0.0f;
  uint32_t freeHeapBytes = 0;
  uint32_t totalHeapBytes = 0;
  uint64_t uptimeSeconds = 0;
};

class SysHealth {
 public:
  void begin();
  void update(uint64_t loopBusyUs);
  SysHealthSnapshot snapshot() const;
  float internalTemperatureC() const;

 private:
  static constexpr uint64_t CPU_SAMPLE_INTERVAL_US = 100000ULL;
  static constexpr uint8_t CPU_REPORT_SAMPLE_COUNT = 10;

  static float readInternalTemperatureC();

  SysHealthSnapshot snapshot_;
  uint64_t cpuSampleWindowStartUs_ = 0;
  uint64_t cpuSampleBusyUs_ = 0;
  float cpuReportSampleSumPercent_ = 0.0f;
  float cpuReportSamplePeakPercent_ = 0.0f;
  uint8_t cpuReportSamples_ = 0;
};
