#pragma once

#include <Arduino.h>
#include "driver/mcpwm.h"

struct PpsPulse {
  uint32_t pulseCount = 0;
  uint32_t elapsedPulses = 0;
  uint32_t edgeTicks = 0;
  uint32_t previousEdgeTicks = 0;
  uint32_t diffTicks = 0;
  uint64_t periodMicros = 0;
  int32_t frequencyDiffTicks = 0;
};

class PpsCapture {
 public:
  static constexpr uint32_t DEFAULT_TICKS_PER_SECOND = 79998000UL;

  explicit PpsCapture(uint32_t ticksPerSecond = DEFAULT_TICKS_PER_SECOND);

  void begin(uint8_t ppsPin);
  uint32_t nowTicks();
  uint64_t nowTicks64();
  bool consumePulse(PpsPulse& pulseOut);
  uint32_t pulseCount() const;
  uint32_t ticksPerSecond() const;
  double lastPeriodSeconds() const;
  bool lastPeriodValid() const;

 private:
  static bool onCapture(mcpwm_unit_t mcpwm,
                        mcpwm_capture_channel_id_t capChannel,
                        const cap_event_data_t* eventData,
                        void* userData);

  static constexpr mcpwm_unit_t kMcpwmUnit = MCPWM_UNIT_0;
  static constexpr mcpwm_io_signals_t kCaptureIo = MCPWM_CAP_0;
  static constexpr mcpwm_capture_channel_id_t kCaptureChannel = MCPWM_SELECT_CAP0;
  static constexpr uint8_t kNowChannel = 1;

  uint32_t ticksPerSecond_;
  volatile uint32_t pulseCount_ = 0;
  volatile uint32_t lastEdgeTicks_ = 0;
  volatile uint32_t previousEdgeTicks_ = 0;
  uint32_t lastConsumedPulseCount_ = 0;
  uint64_t ticksHigh_ = 0;
  uint32_t lastNowTicks_ = 0;
  bool ticks64Initialized_ = false;
  double lastPeriodSeconds_ = 0.0;
  bool lastPeriodValid_ = false;
};
