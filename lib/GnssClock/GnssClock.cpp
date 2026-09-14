#include "GnssClock.h"

GnssClock::GnssClock(GnssMaxM10S& gps, PpsCapture& ppsCapture)
    : gps_(gps), ppsCapture_(ppsCapture) {}

void GnssClock::configure(uint32_t holdoverTimeoutMs,
                          uint32_t confirmedTimeHoldoverTimeoutMs,
                          uint32_t maxConfirmedTimeCorrectionDeltaMs,
                          uint32_t maxConfirmedTimeCorrectionIgnoreDeltaMs) {
  kHoldoverTimeoutMs_ = holdoverTimeoutMs;
  kConfirmedTimeHoldoverTimeoutMs_ = confirmedTimeHoldoverTimeoutMs;
  kMaxConfirmedTimeCorrectionDeltaMs_ = maxConfirmedTimeCorrectionDeltaMs;
  kMaxConfirmedTimeCorrectionIgnoreDeltaMs_ = maxConfirmedTimeCorrectionIgnoreDeltaMs;
}

bool GnssClock::navPvtCoarseEpochUs(uint64_t& epochUsUtcOut, uint32_t* receivedUsOut) {
  GnssNavPvtData pvt;
  gps_.navPvtSnapshot(pvt);
  if (!pvt.valid || !pvt.fullyResolved || !pvt.gnssFixOK) {
    return false;
  }

  if (!gps_.navPvtUtcEpochUs(epochUsUtcOut)) {
    return false;
  }

  if (receivedUsOut) {
    *receivedUsOut = pvt.receivedUs;
  }
  return true;
}

bool GnssClock::navPvtConfirmedTimeEpochUs(uint64_t& epochUsUtcOut, uint32_t* receivedUsOut) {
  GnssNavPvtData pvt;
  gps_.navPvtSnapshot(pvt);
  if (!pvt.valid ||
      !pvt.validDate ||
      !pvt.validTime ||
      !pvt.fullyResolved ||
      !pvt.confirmedTime) {
    return false;
  }

  if (!gps_.navPvtUtcEpochUs(epochUsUtcOut)) {
    return false;
  }

  if (receivedUsOut) {
    *receivedUsOut = pvt.receivedUs;
  }
  return true;
}

bool GnssClock::currentPpsLockedEpochUs(uint64_t& epochUsUtcOut) {
  uint64_t epochNsUtc = 0;
  if (!currentPpsLockedEpochNs(epochNsUtc)) {
    return false;
  }
  epochUsUtcOut = epochNsUtc / 1000ULL;
  return true;
}

bool GnssClock::currentPpsLockedEpochNs(uint64_t& epochNsUtcOut) {
  if (gps_.currentUtcEpochNsPps(epochNsUtcOut) && gps_.ppsRecentlySeen()) {
    refreshHoldoverTimeout(kHoldoverTimeoutMs_);
    holdoverNavPvtTimeoutStarted_ = false;
    holdoverPpsDisciplined_ = true;
    holdoverNavPvtReceivedUsValid_ = false;
    anchorHoldoverRawNs(epochNsUtcOut);
    return true;
  }

  return false;
}

bool GnssClock::currentNtpEpochUs(uint64_t& epochUsUtcOut) {
  uint64_t epochNsUtc = 0;
  if (!currentNtpEpochNs(epochNsUtc)) {
    return false;
  }
  epochUsUtcOut = epochNsUtc / 1000ULL;
  return true;
}

bool GnssClock::currentNtpEpochNs(uint64_t& epochNsUtcOut) {
  if (currentPpsLockedEpochNs(epochNsUtcOut)) {
    return true;
  }

  uint64_t confirmedTimeEpochUsUtc = 0;
  uint32_t confirmedTimeReceivedUs = 0;
  const bool confirmedTimeEpochValid =
      navPvtConfirmedTimeEpochUs(confirmedTimeEpochUsUtc, &confirmedTimeReceivedUs);
  const char* navPvtFallbackLabel = "ConfirmedTime";

  if (confirmedTimeEpochValid && holdoverValid_) {
    uint64_t holdoverEpochNsUtc = 0;
    if (!currentHoldoverNs(holdoverEpochNsUtc)) {
      return false;
    }

    if (holdoverPpsDisciplined_) {
      uint32_t remainingSeconds = 0;
      if (holdoverRemainingSeconds(remainingSeconds)) {
        epochNsUtcOut = holdoverEpochNsUtc;
        return true;
      }
      holdoverPpsDisciplined_ = false;
    }

    if (!holdoverPpsDisciplined_ && !holdoverNavPvtTimeoutStarted_) {
      epochNsUtcOut = confirmedTimeEpochUsUtc * 1000ULL;
      anchorHoldoverNs(epochNsUtcOut);
      startNavPvtHoldoverTimeout(kConfirmedTimeHoldoverTimeoutMs_);
      holdoverNavPvtReceivedUs_ = confirmedTimeReceivedUs;
      holdoverNavPvtReceivedUsValid_ = true;
      return true;
    }

    if (!holdoverNavPvtTimeoutStarted_) {
      startNavPvtHoldoverTimeout(kConfirmedTimeHoldoverTimeoutMs_);
    }

    uint32_t remainingSeconds = 0;
    if (holdoverRemainingSeconds(remainingSeconds)) {
      epochNsUtcOut = holdoverEpochNsUtc;
      return true;
    }

    uint64_t correctedEpochUsUtc = holdoverEpochNsUtc / 1000ULL;
    const uint32_t correctionCountBefore = correctionUseCount_;
    if (updateCorrection(correctedEpochUsUtc,
                         confirmedTimeEpochUsUtc,
                         navPvtFallbackLabel,
                         correctedEpochUsUtc)) {
      // Preserve the sub-microsecond holdover value when the NAV-PVT sample
      // only causes a skipped/ignored correction.
      epochNsUtcOut = correctionUseCount_ == correctionCountBefore
                          ? holdoverEpochNsUtc
                          : correctedEpochUsUtc * 1000ULL;
      restartNavPvtHoldoverTimeout(kConfirmedTimeHoldoverTimeoutMs_);
      return true;
    }

    epochNsUtcOut = holdoverEpochNsUtc;
    return true;
  }

  if (confirmedTimeEpochValid) {
    epochNsUtcOut = confirmedTimeEpochUsUtc * 1000ULL;
    anchorHoldoverNs(epochNsUtcOut);
    startNavPvtHoldoverTimeout(kConfirmedTimeHoldoverTimeoutMs_);
    holdoverPpsDisciplined_ = false;
    holdoverNavPvtReceivedUs_ = confirmedTimeReceivedUs;
    holdoverNavPvtReceivedUsValid_ = true;
    return true;
  }

  if (holdoverValid_) {
    uint32_t remainingSeconds = 0;
    if (!holdoverRemainingSeconds(remainingSeconds)) {
      holdoverValid_ = false;
      holdoverPpsDisciplined_ = false;
      holdoverNavPvtReceivedUsValid_ = false;
      return false;
    }
    return currentHoldoverNs(epochNsUtcOut);
  }

  return false;
}

bool GnssClock::currentNtpEpochMs(uint64_t& epochMsUtcOut) {
  uint64_t epochUsUtc = 0;
  if (!currentNtpEpochUs(epochUsUtc)) {
    return false;
  }
  epochMsUtcOut = epochUsUtc / 1000ULL;
  return true;
}

bool GnssClock::currentHoldoverUs(uint64_t& epochUsUtcOut) {
  uint64_t epochNsUtc = 0;
  if (!currentHoldoverNs(epochNsUtc)) {
    return false;
  }
  epochUsUtcOut = epochNsUtc / 1000ULL;
  return true;
}

bool GnssClock::currentHoldoverNs(uint64_t& epochNsUtcOut) {
  if (!holdoverValid_) {
    return false;
  }

  const uint64_t elapsedTicks = ppsCapture_.nowTicks64() - holdoverBaseTicks_;
  epochNsUtcOut = holdoverBaseEpochNsUtc_ + ppsTicksToNs(elapsedTicks);
  return true;
}

bool GnssClock::holdoverRemainingSeconds(uint32_t& remainingSecondsOut) {
  if (!holdoverDeadlineValid_) {
    return false;
  }

  const int32_t remainingMs = static_cast<int32_t>(holdoverDeadlineMs_ - millis());
  if (remainingMs <= 0) {
    return false;
  }

  remainingSecondsOut = static_cast<uint32_t>((remainingMs - 1) / 1000L);
  return true;
}

GnssClockDiagnostics GnssClock::diagnostics() {
  GnssClockDiagnostics out;
  out.correctionUseCount = correctionUseCount_;
  out.correctionLastAppliedUs = correctionLastAppliedUs_;
  out.holdoverValid = holdoverValid_;
  if (out.holdoverValid) {
    out.holdoverTimeoutValid = holdoverRemainingSeconds(out.holdoverRemainingSeconds);
    out.holdoverUtcValid = currentHoldoverUs(out.holdoverEpochUtcMicros);
  }
  return out;
}

GpsClockState GnssClock::clockState(const GpsDashboardStatus& status) const {
  if (status.timTpValid &&
      status.timTpUtcEpochValid &&
      !status.timTpTpNotLocked &&
      status.ppsAligned) {
    return GPS_CLOCK_STATE_PPS_ANCHORED;
  }

  if (holdoverPpsDisciplined_ &&
      status.holdoverUtcValid &&
      status.holdoverTimeoutValid &&
      !status.ppsAligned) {
    return GPS_CLOCK_STATE_HOLDOVER;
  }

  if (status.navPvtConfirmedTimeReady) {
    return GPS_CLOCK_STATE_CONFIRMED_TIME;
  }

  if (status.holdoverUtcValid &&
      status.holdoverTimeoutValid &&
      !status.ppsAligned) {
    return GPS_CLOCK_STATE_HOLDOVER;
  }

  return GPS_CLOCK_STATE_UNSYNCED;
}

uint64_t GnssClock::ppsTicksToNs(uint64_t ticks) const {
  const uint32_t ticksPerSecond = ppsCapture_.ticksPerSecond();
  return (ticks / ticksPerSecond) * 1000000000ULL +
         ((ticks % ticksPerSecond) * 1000000000ULL) / ticksPerSecond;
}

void GnssClock::anchorHoldoverRawNs(uint64_t epochNsUtc) {
  holdoverValid_ = true;
  holdoverBaseEpochNsUtc_ = epochNsUtc;
  holdoverBaseTicks_ = ppsCapture_.nowTicks64();
}

void GnssClock::anchorHoldoverNs(uint64_t epochNsUtc) {
  const int64_t correctedEpochNs =
      static_cast<int64_t>(epochNsUtc) + holdoverCorrectionOffsetUs_ * 1000LL;
  if (correctedEpochNs <= 0) {
    return;
  }
  anchorHoldoverRawNs(static_cast<uint64_t>(correctedEpochNs));
}

void GnssClock::anchorHoldoverUs(uint64_t epochUsUtc) {
  const int64_t correctedEpochUs =
      static_cast<int64_t>(epochUsUtc) + holdoverCorrectionOffsetUs_;
  if (correctedEpochUs <= 0) {
    return;
  }
  anchorHoldoverNs(static_cast<uint64_t>(correctedEpochUs) * 1000ULL);
}

void GnssClock::refreshHoldoverTimeout(uint32_t timeoutMs) {
  holdoverDeadlineValid_ = true;
  holdoverDeadlineMs_ = millis() + timeoutMs;
}

void GnssClock::startNavPvtHoldoverTimeout(uint32_t timeoutMs) {
  if (!holdoverNavPvtTimeoutStarted_) {
    refreshHoldoverTimeout(timeoutMs);
    holdoverNavPvtTimeoutStarted_ = true;
  }
}

void GnssClock::restartNavPvtHoldoverTimeout(uint32_t timeoutMs) {
  refreshHoldoverTimeout(timeoutMs);
  holdoverNavPvtTimeoutStarted_ = true;
}

bool GnssClock::updateCorrection(uint64_t referenceEpochUsUtc,
                                 uint64_t navPvtEpochUsUtc,
                                 const char* sourceLabel,
                                 uint64_t& epochUsUtcInOut) {
  const int64_t deltaMs = static_cast<int64_t>(navPvtEpochUsUtc / 1000ULL) -
                          static_cast<int64_t>(referenceEpochUsUtc / 1000ULL);
  const int64_t absDeltaMs = abs64(deltaMs);
  if (absDeltaMs >= kMaxConfirmedTimeCorrectionIgnoreDeltaMs_) {
  Serial.printf("%s correction ignored | deltaMs=%lld thresholdMs=%lld ignoreAtMs=%lld\r\n",
                  sourceLabel,
                  static_cast<long long>(deltaMs),
                  static_cast<unsigned long>(kMaxConfirmedTimeCorrectionDeltaMs_),
                  static_cast<unsigned long>(kMaxConfirmedTimeCorrectionIgnoreDeltaMs_));
    return true;
  }

  if (absDeltaMs <= kMaxConfirmedTimeCorrectionDeltaMs_) {
    Serial.printf("%s correction skipped | deltaMs=%lld thresholdMs=%lld\r\n",
                  sourceLabel,
                  static_cast<long long>(deltaMs),
                  static_cast<unsigned long>(kMaxConfirmedTimeCorrectionDeltaMs_));
    return true;
  }

  const int32_t correctionUs = static_cast<int32_t>(deltaMs * 100LL);
  const int64_t correctionSubMsUs = abs64(correctionUs) % 1000LL;
  correctionLastAppliedUs_ = correctionUs;

  uint64_t currentHoldoverNsValue = 0;
  if (!currentHoldoverNs(currentHoldoverNsValue)) {
    return false;
  }

  const int64_t correctedHoldoverNs =
      static_cast<int64_t>(currentHoldoverNsValue) +
      static_cast<int64_t>(correctionUs) * 1000LL;
  if (correctedHoldoverNs <= 0) {
    return false;
  }

  holdoverCorrectionOffsetUs_ += correctionUs;
  epochUsUtcInOut = static_cast<uint64_t>(correctedHoldoverNs / 1000LL);
  ++correctionUseCount_;
  anchorHoldoverRawNs(static_cast<uint64_t>(correctedHoldoverNs));
  Serial.printf("%s correction applied | deltaMs=%lld correction=%s%ld.%03lldms \r\n",
                sourceLabel,
                static_cast<long long>(deltaMs),
                correctionUs < 0 ? "-" : "",
                static_cast<long>(abs64(correctionUs) / 1000LL),
                static_cast<long long>(correctionSubMsUs));

  return true;
}

int64_t GnssClock::abs64(int64_t value) {
  return value < 0 ? -value : value;
}
