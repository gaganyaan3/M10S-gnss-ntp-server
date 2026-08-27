#include "GnssClock.h"

GnssClock::GnssClock(GnssMaxM10S& gps, PpsCapture& ppsCapture)
    : gps_(gps), ppsCapture_(ppsCapture) {}

void GnssClock::setHoldoverEnabled(bool enabled) {
  holdoverEnabled_ = enabled;
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

bool GnssClock::currentNtpEpochUs(uint64_t& epochUsUtcOut) {
  uint64_t epochNsUtc = 0;
  if (gps_.currentUtcEpochNsPps(epochNsUtc) && gps_.ppsRecentlySeen()) {
    epochUsUtcOut = epochNsUtc / 1000ULL;
    refreshHoldoverTimeout();
    anchorHoldoverUs(epochUsUtcOut);
    updateCoarseCorrection(epochUsUtcOut, true, false, epochUsUtcOut);
    return true;
  }

  if (holdoverEnabled_ && holdoverValid_) {
    uint32_t remainingSeconds = 0;
    if (!holdoverRemainingSeconds(remainingSeconds)) {
      holdoverValid_ = false;
      return false;
    }

    if (!currentHoldoverUs(epochUsUtcOut)) {
      return false;
    }

    if (coarseCorrectionDeltaCount_ > 0) {
      resetCoarseCorrectionWindow();
    }
    return true;
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
  if (!holdoverPpsDisciplined_) {
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
  out.coarseEpochUseCount = coarseEpochUseCount_;
  out.coarseCorrectionLastAppliedUs = coarseCorrectionLastAppliedUs_;
  out.holdoverValid = holdoverValid_;
  if (out.holdoverValid) {
    out.holdoverTimeoutValid = holdoverRemainingSeconds(out.holdoverRemainingSeconds);
    out.holdoverUtcValid =
        out.holdoverTimeoutValid && currentHoldoverUs(out.holdoverEpochUtcMicros);
  }
  return out;
}

GpsClockState GnssClock::clockState(const GpsDashboardStatus& status) const {
  if (status.timTpValid &&
      status.timTpUtcEpochValid &&
      !status.timTpTpNotLocked &&
      status.ppsAligned) {
    return GPS_CLOCK_STATE_LOCKED;
  }

  if (status.holdoverUtcValid && !status.ppsAligned) {
    return GPS_CLOCK_STATE_HOLDOVER;
  }

  if (status.gpsTimeValid) {
    return GPS_CLOCK_STATE_ACQUIRING;
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

void GnssClock::refreshHoldoverTimeout() {
  holdoverPpsDisciplined_ = true;
  holdoverDeadlineMs_ = millis() + kHoldoverTimeoutMs;
}

void GnssClock::resetCoarseCorrectionWindow() {
  coarseCorrectionDeltaSumUs_ = 0;
  coarseCorrectionDeltaCount_ = 0;
  coarseCorrectionWindowStartMs_ = 0;
}

bool GnssClock::updateCoarseCorrection(uint64_t referenceEpochUsUtc,
                                       bool applyToHoldover,
                                       bool updateServedEpoch,
                                       uint64_t& epochUsUtcInOut) {
  if (gps_.ppsRecentlySeen()) {
    if (coarseCorrectionDeltaCount_ > 0) {
      resetCoarseCorrectionWindow();
    }
    return false;
  }

  uint64_t coarseEpochUsUtc = 0;
  uint32_t coarseReceivedUs = 0;
  if (!navPvtCoarseEpochUs(coarseEpochUsUtc, &coarseReceivedUs) ||
      (coarseCorrectionLastSampleValid_ &&
       coarseReceivedUs == coarseCorrectionLastSampleReceivedUs_)) {
    return false;
  }

  coarseCorrectionLastSampleReceivedUs_ = coarseReceivedUs;
  coarseCorrectionLastSampleValid_ = true;

  const int64_t deltaUs = static_cast<int64_t>(coarseEpochUsUtc) -
                          static_cast<int64_t>(referenceEpochUsUtc);
  coarseCorrectionLastDeltaUs_ = deltaUs;
  const uint32_t nowMs = millis();

  if (coarseCorrectionDeltaCount_ == 0) {
    if (!deltaExceedsThreshold(deltaUs, kMaxPpsCoarseDeltaUs)) {
      return false;
    }
    coarseCorrectionWindowStartMs_ = nowMs;
    Serial.printf("COARSE averaging window start | startDeltaUs=%lld startDeltaMs=%ld startDeltaSubUs=%ld thresholdUs=%lld referenceEpochUs=%llu navPvtEpochUs=%llu receivedUs=%lu\n",
                  static_cast<long long>(coarseCorrectionLastDeltaUs_),
                  static_cast<long>(microsecondsToMillisecondsPart(coarseCorrectionLastDeltaUs_)),
                  static_cast<long>(microsecondsToSubMillisecondPart(coarseCorrectionLastDeltaUs_)),
                  static_cast<long long>(kMaxPpsCoarseDeltaUs),
                  static_cast<unsigned long long>(referenceEpochUsUtc),
                  static_cast<unsigned long long>(coarseEpochUsUtc),
                  static_cast<unsigned long>(coarseReceivedUs));
  }

  coarseCorrectionDeltaSumUs_ += deltaUs;
  ++coarseCorrectionDeltaCount_;
  const uint32_t windowAgeMs = nowMs - coarseCorrectionWindowStartMs_;
  const int64_t runningAvgDeltaUs =
      coarseCorrectionDeltaSumUs_ / static_cast<int64_t>(coarseCorrectionDeltaCount_);
  Serial.printf("COARSE averaging sample | count=%lu avgDeltaUs=%lld avgDeltaMs=%ld avgDeltaSubUs=%ld\n",
                static_cast<unsigned long>(coarseCorrectionDeltaCount_),
                static_cast<long long>(runningAvgDeltaUs),
                static_cast<long>(microsecondsToMillisecondsPart(runningAvgDeltaUs)),
                static_cast<long>(microsecondsToSubMillisecondPart(runningAvgDeltaUs)));

  if (windowAgeMs < kCoarseCorrectionWindowMs) {
    return false;
  }

  const int64_t avgDeltaUs =
      coarseCorrectionDeltaSumUs_ / static_cast<int64_t>(coarseCorrectionDeltaCount_);
  const int32_t correctionUs = static_cast<int32_t>(avgDeltaUs / 10LL);
  coarseCorrectionLastAppliedUs_ = correctionUs;

  bool applied = false;
  uint64_t currentHoldoverUsValue = 0;
  int64_t correctedHoldoverUs = 0;
  if (applyToHoldover && correctionUs != 0) {
    if (currentHoldoverUs(currentHoldoverUsValue)) {
      correctedHoldoverUs = static_cast<int64_t>(currentHoldoverUsValue) + correctionUs;
      if (correctedHoldoverUs > 0) {
        holdoverCorrectionOffsetUs_ += correctionUs;
        if (updateServedEpoch) {
          epochUsUtcInOut = static_cast<uint64_t>(correctedHoldoverUs);
        }
        ++coarseEpochUseCount_;
        anchorHoldoverRawUs(static_cast<uint64_t>(correctedHoldoverUs));
        applied = true;
      }
    }
  }
  Serial.printf("COARSE averaging complete | correctionUs=%ld\n",
                static_cast<long>(correctionUs));

  resetCoarseCorrectionWindow();
  return applied;
}

bool GnssClock::deltaExceedsThreshold(int64_t deltaUs, int64_t thresholdUs) {
  return deltaUs > thresholdUs || deltaUs < -thresholdUs;
}

int32_t GnssClock::microsecondsToMillisecondsPart(int64_t valueUs) {
  return static_cast<int32_t>(valueUs / 1000LL);
}

int32_t GnssClock::microsecondsToSubMillisecondPart(int64_t valueUs) {
  return static_cast<int32_t>(valueUs % 1000LL);
}
