#include "GnssClock.h"

GnssClock::GnssClock(GnssMaxM10S& gps, PpsCapture& ppsCapture)
    : gps_(gps), ppsCapture_(ppsCapture) {}

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
  if (gps_.currentUtcEpochNsPps(epochNsUtc) && gps_.ppsRecentlySeen()) {
    epochUsUtcOut = epochNsUtc / 1000ULL;
    refreshHoldoverTimeout(kHoldoverTimeoutMs);
    holdoverNavPvtTimeoutStarted_ = false;
    holdoverPpsDisciplined_ = true;
    holdoverNavPvtReceivedUsValid_ = false;
    anchorHoldoverUs(epochUsUtcOut);
    return true;
  }

  return false;
}

bool GnssClock::currentNtpEpochUs(uint64_t& epochUsUtcOut) {
  if (currentPpsLockedEpochUs(epochUsUtcOut)) {
    return true;
  }

  uint64_t confirmedTimeEpochUsUtc = 0;
  uint32_t confirmedTimeReceivedUs = 0;
  const bool confirmedTimeEpochValid =
      navPvtConfirmedTimeEpochUs(confirmedTimeEpochUsUtc, &confirmedTimeReceivedUs);
  const char* navPvtFallbackLabel = "ConfirmedTime";

  if (confirmedTimeEpochValid && holdoverValid_) {
    uint64_t holdoverEpochUsUtc = 0;
    if (!currentHoldoverUs(holdoverEpochUsUtc)) {
      return false;
    }

    if (holdoverPpsDisciplined_) {
      uint32_t remainingSeconds = 0;
      if (holdoverRemainingSeconds(remainingSeconds)) {
        epochUsUtcOut = holdoverEpochUsUtc;
        return true;
      }
      holdoverPpsDisciplined_ = false;
    }

    if (!holdoverPpsDisciplined_ && !holdoverNavPvtTimeoutStarted_) {
      epochUsUtcOut = confirmedTimeEpochUsUtc;
      anchorHoldoverUs(epochUsUtcOut);
      startNavPvtHoldoverTimeout(kConfirmedTimeHoldoverTimeoutMs);
      holdoverNavPvtReceivedUs_ = confirmedTimeReceivedUs;
      holdoverNavPvtReceivedUsValid_ = true;
      return true;
    }

    if (!holdoverNavPvtTimeoutStarted_) {
      startNavPvtHoldoverTimeout(kConfirmedTimeHoldoverTimeoutMs);
    }

    uint32_t remainingSeconds = 0;
    if (holdoverRemainingSeconds(remainingSeconds)) {
      epochUsUtcOut = holdoverEpochUsUtc;
      return true;
    }

    epochUsUtcOut = holdoverEpochUsUtc;
    if (updateCorrection(holdoverEpochUsUtc,
                         confirmedTimeEpochUsUtc,
                         navPvtFallbackLabel,
                         epochUsUtcOut)) {
      restartNavPvtHoldoverTimeout(kConfirmedTimeHoldoverTimeoutMs);
      return true;
    }

    return true;
  }

  if (confirmedTimeEpochValid) {
    epochUsUtcOut = confirmedTimeEpochUsUtc;
    anchorHoldoverUs(epochUsUtcOut);
    startNavPvtHoldoverTimeout(kConfirmedTimeHoldoverTimeoutMs);
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
    return currentHoldoverUs(epochUsUtcOut);
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
  if (!holdoverValid_) {
    return false;
  }

  const uint64_t elapsedTicks = ppsCapture_.nowTicks64() - holdoverBaseTicks_;
  epochUsUtcOut = holdoverBaseEpochUsUtc_ + ppsTicksToUs(elapsedTicks);
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

uint64_t GnssClock::ppsTicksToUs(uint64_t ticks) const {
  const uint32_t ticksPerSecond = ppsCapture_.ticksPerSecond();
  return (ticks / ticksPerSecond) * 1000000ULL +
         ((ticks % ticksPerSecond) * 1000000ULL) / ticksPerSecond;
}

void GnssClock::anchorHoldoverRawUs(uint64_t epochUsUtc) {
  holdoverValid_ = true;
  holdoverBaseEpochUsUtc_ = epochUsUtc;
  holdoverBaseTicks_ = ppsCapture_.nowTicks64();
}

void GnssClock::anchorHoldoverUs(uint64_t epochUsUtc) {
  const int64_t correctedEpochUs =
      static_cast<int64_t>(epochUsUtc) + holdoverCorrectionOffsetUs_;
  if (correctedEpochUs <= 0) {
    return;
  }
  anchorHoldoverRawUs(static_cast<uint64_t>(correctedEpochUs));
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
  if (absDeltaMs >= kMaxConfirmedTimeCorrectionIgnoreDeltaMs) {
    Serial.printf("%s correction ignored | deltaMs=%lld thresholdMs=%lld ignoreAtMs=%lld\n",
                  sourceLabel,
                  static_cast<long long>(deltaMs),
                  static_cast<long long>(kMaxConfirmedTimeCorrectionDeltaMs),
                  static_cast<long long>(kMaxConfirmedTimeCorrectionIgnoreDeltaMs));
    return true;
  }

  if (absDeltaMs <= kMaxConfirmedTimeCorrectionDeltaMs) {
    Serial.printf("%s correction skipped | deltaMs=%lld thresholdMs=%lld\n",
                  sourceLabel,
                  static_cast<long long>(deltaMs),
                  static_cast<long long>(kMaxConfirmedTimeCorrectionDeltaMs));
    return true;
  }

  const int32_t correctionUs = static_cast<int32_t>(deltaMs * 100LL);
  const int64_t correctionSubMsUs = abs64(correctionUs) % 1000LL;
  correctionLastAppliedUs_ = correctionUs;

  uint64_t currentHoldoverUsValue = 0;
  if (!currentHoldoverUs(currentHoldoverUsValue)) {
    return false;
  }

  const int64_t correctedHoldoverUs =
      static_cast<int64_t>(currentHoldoverUsValue) + correctionUs;
  if (correctedHoldoverUs <= 0) {
    return false;
  }

  holdoverCorrectionOffsetUs_ += correctionUs;
  epochUsUtcInOut = static_cast<uint64_t>(correctedHoldoverUs);
  ++correctionUseCount_;
  anchorHoldoverRawUs(static_cast<uint64_t>(correctedHoldoverUs));
  Serial.printf("%s correction applied | deltaMs=%lld correction=%s%ld.%03lldms \n",
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
