#include "PpsCapture.h"

#include "soc/mcpwm_struct.h"

PpsCapture::PpsCapture(uint32_t ticksPerSecond) : ticksPerSecond_(ticksPerSecond) {}

void PpsCapture::begin(uint8_t ppsPin) {
  pinMode(ppsPin, INPUT);
  mcpwm_gpio_init(kMcpwmUnit, kCaptureIo, ppsPin);

  mcpwm_capture_config_t captureConfig = {};
  captureConfig.cap_edge = MCPWM_POS_EDGE;
  captureConfig.cap_prescale = 1;
  captureConfig.capture_cb = onCapture;
  captureConfig.user_data = this;
  mcpwm_capture_enable_channel(kMcpwmUnit, kCaptureChannel, &captureConfig);

  // Enable CAP1 purely so it can be sampled on demand via software-forced
  // capture; it is not bound to any GPIO and never triggers on its own.
  MCPWM0.cap_chn_cfg[kNowChannel].capn_en = 1;
  MCPWM0.cap_chn_cfg[kNowChannel].capn_mode = 0;
}

IRAM_ATTR uint32_t PpsCapture::nowTicks() {
  MCPWM0.cap_chn_cfg[kNowChannel].capn_sw = 1;
  __asm__ __volatile__("" ::: "memory");
  (void)MCPWM0.cap_chn_cfg[kNowChannel].capn_sw;
  return MCPWM0.cap_chn[kNowChannel].capn_value;
}

uint64_t PpsCapture::nowTicks64() {
  const uint32_t currentTicks = nowTicks();
  if (!ticks64Initialized_) {
    lastNowTicks_ = currentTicks;
    ticks64Initialized_ = true;
  } else if (currentTicks < lastNowTicks_) {
    ticksHigh_ += (1ULL << 32);
  }
  lastNowTicks_ = currentTicks;
  return ticksHigh_ | static_cast<uint64_t>(currentTicks);
}

bool PpsCapture::consumePulse(PpsPulse& pulseOut) {
  uint32_t pulseCountSnapshot = 0;
  uint32_t edgeTicksSnapshot = 0;
  uint32_t previousEdgeTicksSnapshot = 0;

  noInterrupts();
  pulseCountSnapshot = pulseCount_;
  edgeTicksSnapshot = lastEdgeTicks_;
  previousEdgeTicksSnapshot = previousEdgeTicks_;
  interrupts();

  if (pulseCountSnapshot == lastConsumedPulseCount_) {
    return false;
  }

  pulseOut.pulseCount = pulseCountSnapshot;
  pulseOut.elapsedPulses = pulseCountSnapshot - lastConsumedPulseCount_;
  pulseOut.edgeTicks = edgeTicksSnapshot;
  pulseOut.previousEdgeTicks = previousEdgeTicksSnapshot;
  pulseOut.diffTicks = edgeTicksSnapshot - previousEdgeTicksSnapshot;
  pulseOut.frequencyDiffTicks = static_cast<int32_t>(ticksPerSecond_) -
                                static_cast<int32_t>(pulseOut.diffTicks);
  pulseOut.periodMicros =
      (static_cast<uint64_t>(pulseOut.diffTicks) * 1000000ULL) / ticksPerSecond_;

  lastConsumedPulseCount_ = pulseCountSnapshot;
  if (pulseCountSnapshot > 1) {
    lastPeriodSeconds_ = static_cast<double>(pulseOut.diffTicks) / ticksPerSecond_;
    lastPeriodValid_ = true;
  }

  return true;
}

void PpsCapture::setTicksPerSecond(uint32_t ticksPerSecond) {
  if (ticksPerSecond == 0) {
    return;
  }
  ticksPerSecond_ = ticksPerSecond;
}

uint32_t PpsCapture::pulseCount() const {
  uint32_t pulseCountSnapshot = 0;
  noInterrupts();
  pulseCountSnapshot = pulseCount_;
  interrupts();
  return pulseCountSnapshot;
}

uint32_t PpsCapture::ticksPerSecond() const {
  return ticksPerSecond_;
}

double PpsCapture::lastPeriodSeconds() const {
  return lastPeriodSeconds_;
}

bool PpsCapture::lastPeriodValid() const {
  return lastPeriodValid_;
}

IRAM_ATTR bool PpsCapture::onCapture(mcpwm_unit_t,
                                     mcpwm_capture_channel_id_t,
                                     const cap_event_data_t* eventData,
                                     void* userData) {
  PpsCapture* self = static_cast<PpsCapture*>(userData);
  if (self == nullptr) {
    return false;
  }

  self->previousEdgeTicks_ = self->lastEdgeTicks_;
  self->lastEdgeTicks_ = eventData->cap_value;
  ++self->pulseCount_;
  return false;
}
