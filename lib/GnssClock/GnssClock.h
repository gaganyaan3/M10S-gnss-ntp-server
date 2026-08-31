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

  void setHoldoverEnabled(bool enabled);
  bool navPvtConfirmedTimeEpochUs(uint64_t& epochUsUtcOut, uint32_t* receivedUsOut = nullptr);
  bool navPvtCoarseEpochUs(uint64_t& epochUsUtcOut, uint32_t* receivedUsOut = nullptr);
  bool currentNtpEpochUs(uint64_t& epochUsUtcOut);
  bool currentNtpEpochMs(uint64_t& epochMsUtcOut);
  bool currentHoldoverUs(uint64_t& epochUsUtcOut);
  bool holdoverRemainingSeconds(uint32_t& remainingSecondsOut);
  GnssClockDiagnostics diagnostics();
  GpsClockState clockState(const GpsDashboardStatus& status) const;

 private:
  static constexpr uint32_t kHoldoverTimeoutMs = 1440UL * 60UL * 1000UL;
  static constexpr uint32_t kConfirmedTimeHoldoverTimeoutMs = 60UL * 1000UL; // Timeout holdover for ConfirmedTime correction
  static constexpr int64_t kMaxConfirmedTimeCorrectionDeltaMs = 10; // Diff for correction between holdover and NAV-PVT
  static constexpr int64_t kMaxConfirmedTimeCorrectionIgnoreDeltaMs = 100;

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
  bool holdoverEnabled_ = true;
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
};
