#include "SysHealth.h"

void SysHealth::begin() {
  snapshot_.internalTemperatureC = readInternalTemperatureC();
  snapshot_.freeHeapBytes = ESP.getFreeHeap();
  snapshot_.uptimeSeconds = millis() / 1000ULL;
}

void SysHealth::update(uint64_t loopBusyUs) {
  const uint64_t nowUs = micros();
  if (cpuSampleWindowStartUs_ == 0) {
    cpuSampleWindowStartUs_ = nowUs;
  }

  cpuSampleBusyUs_ += loopBusyUs;
  const uint64_t sampleElapsedUs = nowUs - cpuSampleWindowStartUs_;
  if (sampleElapsedUs < CPU_SAMPLE_INTERVAL_US) {
    return;
  }

  float samplePercent =
      (static_cast<float>(cpuSampleBusyUs_) / static_cast<float>(sampleElapsedUs)) * 100.0f;
  if (samplePercent > 100.0f) {
    samplePercent = 100.0f;
  }

  cpuReportSampleSumPercent_ += samplePercent;
  if (samplePercent > cpuReportSamplePeakPercent_) {
    cpuReportSamplePeakPercent_ = samplePercent;
  }
  ++cpuReportSamples_;

  cpuSampleBusyUs_ = 0;
  cpuSampleWindowStartUs_ = nowUs;

  if (cpuReportSamples_ >= CPU_REPORT_SAMPLE_COUNT) {
    snapshot_.cpuUtilizationPercent = cpuReportSampleSumPercent_ / cpuReportSamples_;
    snapshot_.cpuUtilizationPeakPercent = cpuReportSamplePeakPercent_;
    snapshot_.internalTemperatureC = readInternalTemperatureC();
    snapshot_.freeHeapBytes = ESP.getFreeHeap();
    snapshot_.uptimeSeconds = millis() / 1000ULL;
    cpuReportSampleSumPercent_ = 0.0f;
    cpuReportSamplePeakPercent_ = 0.0f;
    cpuReportSamples_ = 0;
  }
}

SysHealthSnapshot SysHealth::snapshot() const {
  SysHealthSnapshot current = snapshot_;
  current.freeHeapBytes = ESP.getFreeHeap();
  current.uptimeSeconds = millis() / 1000ULL;
  return current;
}

float SysHealth::internalTemperatureC() const {
  return snapshot_.internalTemperatureC;
}

float SysHealth::readInternalTemperatureC() {
  const float reading = temperatureRead();
  if (reading < -40.0f || reading > 120.0f) {
    return 0.0f;
  }
  return reading;
}
