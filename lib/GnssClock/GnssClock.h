#pragma once

#include <Arduino.h>
#include "GnssMaxM10S.h"
#include "PpsCapture.h"
#include "WiFiSetupPortal.h"

struct GnssClockDiagnostics {
  uint32_t coarseEpochUseCount = 0;
  int32_t coarseCorrectionLastAppliedUs = 0;
  bool holdoverValid = false;
  bool holdoverTimeoutValid = false;
  uint32_t holdoverRemainingSeconds = 0;
  bool holdoverUtcValid = false;
  uint64_t holdoverEpochUtcMicros = 0;
};

class GnssClock {
 public:
  GnssClock(GnssMaxM10S& gps, PpsCapture& ppsCapture);

  void setHoldoverEnabled(bool enabled);
  bool navPvtCoarseEpochUs(uint64_t& epochUsUtcOut, uint32_t* receivedUsOut = nullptr);
  bool currentNtpEpochUs(uint64_t& epochUsUtcOut);
  bool currentNtpEpochMs(uint64_t& epochMsUtcOut);
  bool currentHoldoverUs(uint64_t& epochUsUtcOut);
  bool holdoverRemainingSeconds(uint32_t& remainingSecondsOut);
  GnssClockDiagnostics diagnostics();
  GpsClockState clockState(const GpsDashboardStatus& status) const;

 private:
  static constexpr uint32_t kHoldoverTimeoutMs = 1440UL * 60UL * 1000UL;
  static constexpr int64_t kMaxPpsCoarseDeltaUs = 5000;
  static constexpr uint32_t kCoarseCorrectionWindowMs = 60000UL;

  uint64_t ppsTicksToUs(uint64_t ticks) const;
  void anchorHoldoverRawUs(uint64_t epochUsUtc);
  void anchorHoldoverUs(uint64_t epochUsUtc);
  void refreshHoldoverTimeout();
  void resetCoarseCorrectionWindow();
  bool updateCoarseCorrection(uint64_t referenceEpochUsUtc,
                              bool applyToHoldover,
                              bool updateServedEpoch,
                              uint64_t& epochUsUtcInOut);

  static bool deltaExceedsThreshold(int64_t deltaUs, int64_t thresholdUs);
  static int32_t microsecondsToMillisecondsPart(int64_t valueUs);
  static int32_t microsecondsToSubMillisecondPart(int64_t valueUs);

  GnssMaxM10S& gps_;
  PpsCapture& ppsCapture_;
  bool holdoverEnabled_ = true;
  uint32_t coarseEpochUseCount_ = 0;
  int64_t coarseCorrectionDeltaSumUs_ = 0;
  uint32_t coarseCorrectionDeltaCount_ = 0;
  uint32_t coarseCorrectionWindowStartMs_ = 0;
  uint32_t coarseCorrectionLastSampleReceivedUs_ = 0;
  bool coarseCorrectionLastSampleValid_ = false;
  int32_t coarseCorrectionLastAppliedUs_ = 0;
  int64_t coarseCorrectionLastDeltaUs_ = 0;
  bool holdoverValid_ = false;
  int64_t holdoverCorrectionOffsetUs_ = 0;
  uint64_t holdoverBaseEpochUsUtc_ = 0;
  uint64_t holdoverBaseTicks_ = 0;
  bool holdoverPpsDisciplined_ = false;
  uint32_t holdoverDeadlineMs_ = 0;
};
