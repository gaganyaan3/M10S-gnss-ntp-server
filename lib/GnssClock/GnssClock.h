#pragma once

#include <Arduino.h>
#include "GnssMaxM10S.h"
#include "PpsCapture.h"
#include "WiFiSetupPortal.h"

struct GnssClockDiagnostics {
  uint32_t correctionUseCount = 0;
  int32_t correctionLastAppliedUs = 0;
  bool holdoverValid = false;
  bool holdoverTimeoutValid = false;
  uint32_t holdoverRemainingSeconds = 0;
  bool holdoverUtcValid = false;
  uint64_t holdoverEpochUtcMicros = 0;
};

class GnssClock {
 public:
  GnssClock(GnssMaxM10S& gps, PpsCapture& ppsCapture);

  void configure(uint32_t holdoverTimeoutMs,
                 uint32_t confirmedTimeHoldoverTimeoutMs,
                 uint32_t maxConfirmedTimeCorrectionDeltaMs,
                 uint32_t maxConfirmedTimeCorrectionIgnoreDeltaMs);

  bool navPvtConfirmedTimeEpochUs(uint64_t& epochUsUtcOut, uint32_t* receivedUsOut = nullptr);
  bool navPvtCoarseEpochUs(uint64_t& epochUsUtcOut, uint32_t* receivedUsOut = nullptr);
  bool currentPpsLockedEpochUs(uint64_t& epochUsUtcOut);
  bool currentNtpEpochUs(uint64_t& epochUsUtcOut);
  bool currentNtpEpochMs(uint64_t& epochMsUtcOut);
  bool currentHoldoverUs(uint64_t& epochUsUtcOut);
  bool holdoverRemainingSeconds(uint32_t& remainingSecondsOut);
  GnssClockDiagnostics diagnostics();
  GpsClockState clockState(const GpsDashboardStatus& status) const;

 private:
  static constexpr uint32_t DEFAULT_HOLDOVER_TIMEOUT_MS = 1440UL * 60UL * 1000UL;
  static constexpr uint32_t DEFAULT_CONFIRMED_TIME_HOLDOVER_TIMEOUT_MS = 60UL * 1000UL;
  static constexpr uint32_t DEFAULT_MAX_CONFIRMED_TIME_CORRECTION_DELTA_MS = 10UL;
  static constexpr uint32_t DEFAULT_MAX_CONFIRMED_TIME_CORRECTION_IGNORE_DELTA_MS = 100UL;

  uint64_t ppsTicksToUs(uint64_t ticks) const;
  void anchorHoldoverRawUs(uint64_t epochUsUtc);
  void anchorHoldoverUs(uint64_t epochUsUtc);
  void refreshHoldoverTimeout(uint32_t timeoutMs);
  void startNavPvtHoldoverTimeout(uint32_t timeoutMs);
  void restartNavPvtHoldoverTimeout(uint32_t timeoutMs);
  bool updateCorrection(uint64_t referenceEpochUsUtc,
                        uint64_t navPvtEpochUsUtc,
                        const char* sourceLabel,
                        uint64_t& epochUsUtcInOut);

  static int64_t abs64(int64_t value);

  GnssMaxM10S& gps_;
  PpsCapture& ppsCapture_;
  uint32_t correctionUseCount_ = 0;
  int32_t correctionLastAppliedUs_ = 0;
  bool holdoverValid_ = false;
  bool holdoverPpsDisciplined_ = false;
  uint32_t holdoverNavPvtReceivedUs_ = 0;
  bool holdoverNavPvtReceivedUsValid_ = false;
  int64_t holdoverCorrectionOffsetUs_ = 0;
  uint64_t holdoverBaseEpochUsUtc_ = 0;
  uint64_t holdoverBaseTicks_ = 0;
  bool holdoverDeadlineValid_ = false;
  bool holdoverNavPvtTimeoutStarted_ = false;
  uint32_t holdoverDeadlineMs_ = 0;
  uint32_t kHoldoverTimeoutMs_ = DEFAULT_HOLDOVER_TIMEOUT_MS;
  uint32_t kConfirmedTimeHoldoverTimeoutMs_ = DEFAULT_CONFIRMED_TIME_HOLDOVER_TIMEOUT_MS;
  uint32_t kMaxConfirmedTimeCorrectionDeltaMs_ = DEFAULT_MAX_CONFIRMED_TIME_CORRECTION_DELTA_MS;
  uint32_t kMaxConfirmedTimeCorrectionIgnoreDeltaMs_ = DEFAULT_MAX_CONFIRMED_TIME_CORRECTION_IGNORE_DELTA_MS;
};
