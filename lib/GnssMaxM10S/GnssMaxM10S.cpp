#include "GnssMaxM10S.h"

namespace {

constexpr uint32_t UNIX_GPS_EPOCH_OFFSET_SECONDS = 315964800UL;
constexpr uint32_t SECONDS_PER_WEEK = 7UL * 24UL * 60UL * 60UL;

bool isNmeaTextByte(uint8_t byteValue) {
  return byteValue >= 0x20 && byteValue <= 0x7E;
}

}  // namespace

GnssMaxM10S::GnssMaxM10S(uint8_t rxPin,
                         uint8_t txPin,
                         uint32_t baud,
                         HardwareSerial& serialPort)
    : serial_(serialPort), rxPin_(rxPin), txPin_(txPin), baud_(baud) {}

uint16_t GnssMaxM10S::readU2(const uint8_t* payload, uint16_t offset) {
  return static_cast<uint16_t>(payload[offset]) |
         (static_cast<uint16_t>(payload[offset + 1]) << 8);
}

uint32_t GnssMaxM10S::readU4(const uint8_t* payload, uint16_t offset) {
  return static_cast<uint32_t>(payload[offset]) |
         (static_cast<uint32_t>(payload[offset + 1]) << 8) |
         (static_cast<uint32_t>(payload[offset + 2]) << 16) |
         (static_cast<uint32_t>(payload[offset + 3]) << 24);
}

int32_t GnssMaxM10S::readI4(const uint8_t* payload, uint16_t offset) {
  return static_cast<int32_t>(readU4(payload, offset));
}

bool GnssMaxM10S::navPvtStatusFresh() const {
  return navPvtData_.valid &&
         millis() - navPvtData_.receivedMs <= kMaxNavPvtStatusAgeMs;
}

bool GnssMaxM10S::navDopFresh() const {
  return navDopData_.valid &&
         millis() - navDopData_.receivedMs <= kMaxNavDopAgeMs;
}

void GnssMaxM10S::sendUbxCfgMsg(uint8_t msgClass, uint8_t msgId, uint8_t rate) {
  // UBX-CFG-MSG (0x06 0x01) — 8-byte payload:
  // [0] msgClass  [1] msgId
  // [2..7] rate for each port (I2C, UART1, UART2, USB, SPI, reserved)
  // We only care about UART1 (index 1).
  const uint8_t payload[8] = {msgClass, msgId, 0, rate, 0, 0, 0, 0};
  uint8_t ckA = 0, ckB = 0;
  // Checksum covers class + id + length bytes + payload
  const uint8_t header[4] = {0x06, 0x01, 8, 0};
  for (uint8_t b : header) { ckA += b; ckB += ckA; }
  for (uint8_t b : payload) { ckA += b; ckB += ckA; }

  serial_.write(0xB5);
  serial_.write(0x62);
  serial_.write(header, 4);
  serial_.write(payload, 8);
  serial_.write(ckA);
  serial_.write(ckB);
  serial_.flush();
}

void GnssMaxM10S::sendUbxPoll(uint8_t msgClass, uint8_t msgId) {
  uint8_t ckA = 0, ckB = 0;
  const uint8_t header[4] = {msgClass, msgId, 0, 0};
  for (uint8_t b : header) { ckA += b; ckB += ckA; }

  serial_.write(0xB5);
  serial_.write(0x62);
  serial_.write(header, 4);
  serial_.write(ckA);
  serial_.write(ckB);
  serial_.flush();
}

void GnssMaxM10S::sendUbxCfgValsetBool(const uint32_t* keys, size_t keyCount, bool value) {
  if (keys == nullptr || keyCount == 0) {
    return;
  }

  uint8_t payload[64] = {};
  size_t payloadLength = 4;
  payload[0] = 0x00;  // version
  payload[1] = 0x01;  // RAM layer
  payload[2] = 0x00;  // transaction
  payload[3] = 0x00;  // reserved

  for (size_t i = 0; i < keyCount && payloadLength + 5 <= sizeof(payload); ++i) {
    const uint32_t key = keys[i];
    payload[payloadLength++] = static_cast<uint8_t>(key & 0xFF);
    payload[payloadLength++] = static_cast<uint8_t>((key >> 8) & 0xFF);
    payload[payloadLength++] = static_cast<uint8_t>((key >> 16) & 0xFF);
    payload[payloadLength++] = static_cast<uint8_t>((key >> 24) & 0xFF);
    payload[payloadLength++] = value ? 1 : 0;
  }

  uint8_t ckA = 0, ckB = 0;
  const uint8_t header[4] = {
      0x06,
      0x8A,
      static_cast<uint8_t>(payloadLength & 0xFF),
      static_cast<uint8_t>((payloadLength >> 8) & 0xFF),
  };
  for (uint8_t b : header) { ckA += b; ckB += ckA; }
  for (size_t i = 0; i < payloadLength; ++i) { ckA += payload[i]; ckB += ckA; }

  serial_.write(0xB5);
  serial_.write(0x62);
  serial_.write(header, 4);
  serial_.write(payload, payloadLength);
  serial_.write(ckA);
  serial_.write(ckB);
  serial_.flush();
}

void GnssMaxM10S::sendUbxCfgAntSupervisor() {
  // Legacy UBX-CFG-ANT. This is deprecated on M10 but still accepted by many
  // firmware builds, and explicitly reconfigures the default antenna supervisor
  // pins: ANT_OFF_N=PIO7, ANT_SHORT_N=PIO6, ANT_DETECT=PIO5.
  constexpr uint16_t flags =
      (1U << 0) |  // svcs: supply voltage control
      (1U << 1) |  // scd: short circuit detection
      (1U << 2) |  // ocd: open circuit detection
      (1U << 4);   // recovery: automatic short recovery
  constexpr uint16_t pins =
      7U |          // pinSwitch
      (6U << 5) |   // pinSCD
      (5U << 10) |  // pinOCD
      (1U << 15);   // reconfig
  const uint8_t payload[4] = {
      static_cast<uint8_t>(flags & 0xFF),
      static_cast<uint8_t>((flags >> 8) & 0xFF),
      static_cast<uint8_t>(pins & 0xFF),
      static_cast<uint8_t>((pins >> 8) & 0xFF),
  };

  uint8_t ckA = 0, ckB = 0;
  const uint8_t header[4] = {0x06, 0x13, 4, 0};
  for (uint8_t b : header) { ckA += b; ckB += ckA; }
  for (uint8_t b : payload) { ckA += b; ckB += ckA; }

  serial_.write(0xB5);
  serial_.write(0x62);
  serial_.write(header, 4);
  serial_.write(payload, sizeof(payload));
  serial_.write(ckA);
  serial_.write(ckB);
  serial_.flush();
}

void GnssMaxM10S::sendUbxCfgPrtUart1(uint32_t baudRate) {
  // UBX-CFG-PRT for UART1. Keep both UBX and NMEA enabled; only raise the
  // baud rate so NAV-PVT cannot sit behind seconds of 9600-baud text.
  uint8_t payload[20] = {};
  payload[0] = 1;  // portID: UART1
  payload[4] = 0xD0;
  payload[5] = 0x08;  // 8N1
  payload[8] = static_cast<uint8_t>(baudRate & 0xFF);
  payload[9] = static_cast<uint8_t>((baudRate >> 8) & 0xFF);
  payload[10] = static_cast<uint8_t>((baudRate >> 16) & 0xFF);
  payload[11] = static_cast<uint8_t>((baudRate >> 24) & 0xFF);
  payload[12] = 0x03;  // inProtoMask: UBX + NMEA
  payload[14] = 0x03;  // outProtoMask: UBX + NMEA

  uint8_t ckA = 0, ckB = 0;
  const uint8_t header[4] = {0x06, 0x00, 20, 0};
  for (uint8_t b : header) { ckA += b; ckB += ckA; }
  for (uint8_t b : payload) { ckA += b; ckB += ckA; }

  serial_.write(0xB5);
  serial_.write(0x62);
  serial_.write(header, 4);
  serial_.write(payload, sizeof(payload));
  serial_.write(ckA);
  serial_.write(ckB);
  serial_.flush();
}

void GnssMaxM10S::begin() {
  constexpr uint32_t kDefaultBootBaud = 9600;
  serial_.begin(kDefaultBootBaud, SERIAL_8N1, rxPin_, txPin_);
  // Allow the module a moment to finish its boot NMEA burst before we send
  // configuration commands (the MAX-M10S needs ~100 ms after power-on).
  delay(200);
  if (baud_ != kDefaultBootBaud) {
    sendUbxCfgPrtUart1(baud_);
    delay(100);
    serial_.updateBaudRate(baud_);
    delay(100);
  }
  sendUbxCfgMsg(0x01, 0x07, 1);  // Enable UBX-NAV-PVT  on UART1
  sendUbxCfgMsg(0x01, 0x04, 1);  // Enable UBX-NAV-DOP  on UART1
  sendUbxCfgMsg(0x0D, 0x01, 1);  // Enable UBX-TIM-TP   on UART1
  sendUbxCfgMsg(0x0A, 0x38, 1);  // Enable UBX-MON-RF   on UART1
  const uint32_t antennaSupervisorKeys[] = {
      0x10A3002E,  // CFG-HW-ANT_CFG_VOLTCTRL
      0x10A3002F,  // CFG-HW-ANT_CFG_SHORTDET
      0x10A30031,  // CFG-HW-ANT_CFG_OPENDET
  };
  sendUbxCfgValsetBool(antennaSupervisorKeys,
                       sizeof(antennaSupervisorKeys) / sizeof(antennaSupervisorKeys[0]),
                       true);
  sendUbxCfgAntSupervisor();
  sendUbxPoll(0x0A, 0x38);        // Poll once for receivers that don't stream MON-RF
}

void GnssMaxM10S::poll() {
  const uint32_t nowMs = millis();
  constexpr uint32_t kMonRfPollIntervalMs = 1000UL;
  if (nowMs - lastMonRfPollMs_ >= kMonRfPollIntervalMs) {
    lastMonRfPollMs_ = nowMs;
    sendUbxPoll(0x0A, 0x38);
  }

  while (serial_.available() > 0) {
    const uint8_t byteValue = static_cast<uint8_t>(serial_.read());
    const char c = static_cast<char>(byteValue);
    parseUbxByte(byteValue);

    if (c == '$') {
      rawLineBufferLength_ = 0;
      rawLineBuffer_[rawLineBufferLength_++] = c;
      rawLineBuffer_[rawLineBufferLength_] = '\0';
      gps_.encode(c);
      continue;
    }

    if (rawLineBufferLength_ == 0 || rawLineBuffer_[0] != '$') {
      continue;
    }

    if (c == '\n') {
      if (rawLineBufferLength_ > 0) {
        rawLineBuffer_[rawLineBufferLength_] = '\0';
        // Parse GSA for dilution values used by the stats page.
        parseGsaSentence(rawLineBuffer_);
        if (rawLoggingEnabled_) {
          Serial.print("RAW: ");
          Serial.println(rawLineBuffer_);
        }
        if (rawLineCallback_) {
          rawLineCallback_(String(rawLineBuffer_));
        }
      }
      rawLineBufferLength_ = 0;
      rawLineBuffer_[0] = '\0';
      gps_.encode(c);
    } else if (c != '\r') {
      if (!isNmeaTextByte(byteValue)) {
        rawLineBufferLength_ = 0;
        rawLineBuffer_[0] = '\0';
      } else if (rawLineBufferLength_ < (kMaxRawLineLength - 1)) {
        rawLineBuffer_[rawLineBufferLength_++] = c;
        rawLineBuffer_[rawLineBufferLength_] = '\0';
        gps_.encode(c);
      } else {
        rawLineBufferLength_ = 0;
        rawLineBuffer_[0] = '\0';
      }
    } else {
      gps_.encode(c);
    }
  }
}

void GnssMaxM10S::enableRawLogging(bool enable) {
  rawLoggingEnabled_ = enable;
}

void GnssMaxM10S::onRawLine(RawLineCallback callback) {
  rawLineCallback_ = callback;
}

void GnssMaxM10S::configurePpsHardwareClock(TickClockFn nowTicksFn, uint32_t ticksPerSecond) {
  ppsNowTicksFn_ = nowTicksFn ? nowTicksFn : [](){ return micros(); };
  ppsNominalTicksPerSecond_ = (ticksPerSecond > 0) ? ticksPerSecond : 1000000UL;
}

void GnssMaxM10S::setPpsHardwareTicksPerSecond(uint32_t ticksPerSecond) {
  if (ticksPerSecond == 0) {
    return;
  }
  ppsNominalTicksPerSecond_ = ticksPerSecond;
}

void GnssMaxM10S::onPpsEdge(uint32_t edgeTicks, uint32_t elapsedPulses) {
  if (elapsedPulses == 0) {
    elapsedPulses = 1;
  }

  const uint32_t nowMs = millis();
  const bool previousPpsAnchorFresh =
      ppsAligned_ && (nowMs - ppsEdgeMs_ <= kMaxPpsHoldoverMs);

  uint64_t timTpEpochNsUtc = 0;
  const bool timTpCanAnchor =
      elapsedPulses == 1 && timTpCanAnchorPps(timTpEpochNsUtc);

  if (timTpCanAnchor) {
    // TIM-TP reports the time of the next TIMEPULSE0 edge. Use it as the
    // primary PPS anchor, including the qErr sawtooth correction when valid.
    ppsReferenceEpochSec_ = static_cast<uint32_t>(timTpEpochNsUtc / 1000000000ULL);
    ppsReferenceEpochOffsetNs_ = static_cast<int32_t>(timTpEpochNsUtc % 1000000000ULL);
    timTpPendingForPps_ = false;
  } else if (previousPpsAnchorFresh) {
    // Once anchored, PPS remains the precision source. A late/missing time
    // message should not break lock; each captured PPS advances by pulse count.
    ppsReferenceEpochSec_ += elapsedPulses;
  } else {
    ppsAligned_ = false;
    ppsHasPreviousEdge_ = false;
    return;
  }

  ppsEdgeTicks_ = edgeTicks;
  ppsEdgeMs_ = nowMs;
  ppsAligned_ = true;
  ppsHasPreviousEdge_ = true;
}

ClockStamp GnssMaxM10S::latestUtc() {
  if (navPvtStatusFresh() && navPvtData_.utcEpochValid) {
    return {true, navPvtData_.epochUtc};
  }

  if (!gps_.date.isValid() || !gps_.time.isValid()) {
    return {false, 0};
  }

  const uint32_t epoch = toUnixUtc(
      gps_.date.year(),
      gps_.date.month(),
      gps_.date.day(),
      gps_.time.hour(),
      gps_.time.minute(),
      gps_.time.second());

  return {true, epoch};
}

bool GnssMaxM10S::currentUtcEpoch(uint32_t& epochUtcOut) {
  uint64_t epochMsUtc = 0;
  if (navPvtUtcEpochMs(epochMsUtc)) {
    epochUtcOut = static_cast<uint32_t>(epochMsUtc / 1000ULL);
    return true;
  }

  if (!gps_.date.isValid() || !gps_.time.isValid()) {
    return false;
  }

  // gps_.time.age() is milliseconds elapsed since this time field was last
  // updated, letting us interpolate the current second between fixes.
  const uint32_t ageMs = gps_.time.age();
  constexpr uint32_t MAX_FIX_AGE_MS = 5000UL;
  if (ageMs > MAX_FIX_AGE_MS) {
    return false;
  }

  const uint32_t baseEpoch = toUnixUtc(
      gps_.date.year(),
      gps_.date.month(),
      gps_.date.day(),
      gps_.time.hour(),
      gps_.time.minute(),
      gps_.time.second());

  epochUtcOut = baseEpoch + (ageMs / 1000UL);
  return true;
}

bool GnssMaxM10S::currentUtcEpochMs(uint64_t& epochMsUtcOut) {
  if (navPvtUtcEpochMs(epochMsUtcOut)) {
    return true;
  }

  if (!gps_.date.isValid() || !gps_.time.isValid()) {
    return false;
  }

  // gps_.time.age() is milliseconds elapsed since this time field was last
  // updated, letting us interpolate sub-second position between fixes.
  const uint32_t ageMs = gps_.time.age();
  constexpr uint32_t MAX_FIX_AGE_MS = 5000UL;
  if (ageMs > MAX_FIX_AGE_MS) {
    return false;
  }

  const uint32_t baseEpoch = toUnixUtc(
      gps_.date.year(),
      gps_.date.month(),
      gps_.date.day(),
      gps_.time.hour(),
      gps_.time.minute(),
      gps_.time.second());

  epochMsUtcOut = static_cast<uint64_t>(baseEpoch) * 1000ULL + static_cast<uint64_t>(ageMs);
  return true;
}

bool GnssMaxM10S::currentUtcEpochPps(uint32_t& epochUtcOut) {
  if (!ppsAligned_) {
    return false;
  }
  if (millis() - ppsEdgeMs_ > kMaxPpsHoldoverMs) {
    return false;
  }

  const uint32_t nowTicks = ppsNowTicksFn_();
  const uint32_t elapsedTicks = nowTicks - ppsEdgeTicks_;
  const uint64_t epochNsUtc =
      (static_cast<uint64_t>(ppsReferenceEpochSec_) * 1000000000ULL) +
      static_cast<uint64_t>(ppsReferenceEpochOffsetNs_) +
      ((static_cast<uint64_t>(elapsedTicks) * 1000000000ULL) / ppsNominalTicksPerSecond_);
  epochUtcOut = static_cast<uint32_t>(epochNsUtc / 1000000000ULL);
  return true;
}

bool GnssMaxM10S::currentUtcEpochMsPps(uint64_t& epochMsUtcOut) {
  uint64_t epochNsUtc = 0;
  if (!currentUtcEpochNsPps(epochNsUtc)) {
    return false;
  }
  epochMsUtcOut = epochNsUtc / 1000000ULL;
  return true;
}

bool GnssMaxM10S::currentUtcEpochNsPps(uint64_t& epochNsUtcOut) {
  if (!ppsAligned_) {
    return false;
  }
  if (millis() - ppsEdgeMs_ > kMaxPpsHoldoverMs) {
    return false;
  }

  const uint32_t nowTicks = ppsNowTicksFn_();
  const uint32_t elapsedTicks = nowTicks - ppsEdgeTicks_;
  const uint64_t wholeSeconds = elapsedTicks / ppsNominalTicksPerSecond_;
  const uint64_t remainderTicks = elapsedTicks % ppsNominalTicksPerSecond_;
  const uint64_t subSecondNs =
      (remainderTicks * 1000000000ULL) / ppsNominalTicksPerSecond_;

  epochNsUtcOut =
      (static_cast<uint64_t>(ppsReferenceEpochSec_) + wholeSeconds) * 1000000000ULL +
      static_cast<uint64_t>(ppsReferenceEpochOffsetNs_) +
      subSecondNs;
  return true;
}

bool GnssMaxM10S::navPvtRawEpochUs(uint64_t& epochUsUtcOut) const {
  if (!navPvtData_.valid ||
      !navPvtData_.utcEpochValid ||
      !navPvtData_.validDate ||
      !navPvtData_.validTime ||
      !navPvtData_.fullyResolved) {
    return false;
  }

  int64_t epochSec = static_cast<int64_t>(navPvtData_.epochUtc);
  int64_t nano = static_cast<int64_t>(navPvtData_.nanoUtc);
  while (nano < 0) {
    nano += 1000000000LL;
    --epochSec;
  }
  while (nano >= 1000000000LL) {
    nano -= 1000000000LL;
    ++epochSec;
  }

  if (epochSec < 0) {
    return false;
  }

  epochUsUtcOut = static_cast<uint64_t>(epochSec) * 1000000ULL +
                  static_cast<uint64_t>(nano / 1000LL);
  return true;
}

uint64_t GnssMaxM10S::ppsTicksToUs(uint64_t ticks) const {
  return (ticks / ppsNominalTicksPerSecond_) * 1000000ULL +
         ((ticks % ppsNominalTicksPerSecond_) * 1000000ULL) / ppsNominalTicksPerSecond_;
}

bool GnssMaxM10S::ppsEpochUsAtTicks(uint32_t ticks, uint64_t& epochUsUtcOut) const {
  if (!ppsAligned_) {
    return false;
  }
  if (millis() - ppsEdgeMs_ > kMaxPpsHoldoverMs) {
    return false;
  }

  const uint32_t elapsedTicks = ticks - ppsEdgeTicks_;
  const uint64_t epochNsUtc =
      static_cast<uint64_t>(ppsReferenceEpochSec_) * 1000000000ULL +
      static_cast<uint64_t>(ppsReferenceEpochOffsetNs_) +
      ppsTicksToUs(elapsedTicks) * 1000ULL;
  epochUsUtcOut = epochNsUtc / 1000ULL;
  return true;
}

bool GnssMaxM10S::timTpCanAnchorPps(uint64_t& epochNsUtcOut) const {
  if (!timTpPendingForPps_ ||
      !timTpData_.valid ||
      !timTpData_.utcEpochValid ||
      timTpData_.tpNotLocked) {
    return false;
  }
  if (millis() - timTpData_.receivedMs > kMaxTimTpAgeForPpsMs) {
    return false;
  }

  epochNsUtcOut = timTpPpsEpochUtcNs_;
  return true;
}

void GnssMaxM10S::updateNavPvtLatencyEstimate(uint32_t receivedTicks) {
  if (millis() - ppsEdgeMs_ > kPpsRecentlySeenMs) {
    return;
  }

  uint64_t rawNavPvtEpochUs = 0;
  uint64_t ppsEpochUs = 0;
  if (!navPvtRawEpochUs(rawNavPvtEpochUs) ||
      !ppsEpochUsAtTicks(receivedTicks, ppsEpochUs)) {
    return;
  }

  const int64_t latencyUs = static_cast<int64_t>(ppsEpochUs) -
                            static_cast<int64_t>(rawNavPvtEpochUs);
  constexpr int64_t MIN_NAV_PVT_LATENCY_US = 0;
  constexpr int64_t MAX_NAV_PVT_LATENCY_US = 950000;
  if (latencyUs < MIN_NAV_PVT_LATENCY_US || latencyUs > MAX_NAV_PVT_LATENCY_US) {
    return;
  }

  if (!navPvtLatencyUsValid_) {
    navPvtLatencyUs_ = static_cast<uint32_t>(latencyUs);
    navPvtLatencyUsValid_ = true;
  } else {
    navPvtLatencyUs_ = (navPvtLatencyUs_ * 7UL + static_cast<uint32_t>(latencyUs) + 4UL) / 8UL;
  }

  if (navPvtLatencySamples_ < UINT16_MAX) {
    ++navPvtLatencySamples_;
  }
}

bool GnssMaxM10S::navPvtUtcEpochMs(uint64_t& epochMsUtcOut) {
  if (!navPvtData_.valid ||
      !navPvtData_.utcEpochValid ||
      !navPvtData_.validDate ||
      !navPvtData_.validTime ||
      !navPvtData_.fullyResolved) {
    return false;
  }

  const uint32_t ageMs = millis() - navPvtData_.receivedMs;
  constexpr uint32_t MAX_PVT_AGE_MS = 5000UL;
  if (ageMs > MAX_PVT_AGE_MS) {
    return false;
  }

  uint64_t rawEpochUsUtc = 0;
  if (!navPvtRawEpochUs(rawEpochUsUtc)) {
    return false;
  }

  epochMsUtcOut = (rawEpochUsUtc +
                   static_cast<uint64_t>(navPvtLatencyUsValid_ ? navPvtLatencyUs_ : 0UL) +
                   static_cast<uint64_t>(ageMs) * 1000ULL) / 1000ULL;
  return true;
}

bool GnssMaxM10S::navPvtUtcEpochUs(uint64_t& epochUsUtcOut) {
  const uint32_t ageUs = micros() - navPvtData_.receivedUs;
  constexpr uint32_t MAX_PVT_AGE_MS = 5000UL;
  if (ageUs > MAX_PVT_AGE_MS * 1000UL) {
    return false;
  }

  uint64_t rawEpochUsUtc = 0;
  if (!navPvtRawEpochUs(rawEpochUsUtc)) {
    return false;
  }

  epochUsUtcOut = rawEpochUsUtc +
                  static_cast<uint64_t>(navPvtLatencyUsValid_ ? navPvtLatencyUs_ : 0UL) +
                  static_cast<uint64_t>(ageUs);
  return true;
}

bool GnssMaxM10S::ppsRecentlySeen() const {
  return ppsAligned_ && (millis() - ppsEdgeMs_ <= kPpsRecentlySeenMs);
}

bool GnssMaxM10S::isPpsAligned() const {
  return ppsAligned_;
}

uint32_t GnssMaxM10S::charsProcessed() const {
  return gps_.charsProcessed();
}

bool GnssMaxM10S::hasSatelliteCount() const {
  return navPvtStatusFresh();
}

uint32_t GnssMaxM10S::satelliteCount() {
  if (navPvtStatusFresh()) {
    return navPvtData_.numSatellites;
  }

  return 0;
}

bool GnssMaxM10S::hasHdop() const {
  if (navDopFresh() && navDopData_.horizontalDop > 0) {
    return true;
  }

  return gps_.hdop.isValid();
}

float GnssMaxM10S::hdop() {
  if (navDopFresh() && navDopData_.horizontalDop > 0) {
    return static_cast<float>(navDopData_.horizontalDop) / 100.0f;
  }

  return static_cast<float>(gps_.hdop.hdop());
}

bool GnssMaxM10S::hasPdop() const {
  if (navDopFresh() && navDopData_.positionDop > 0) {
    return true;
  }
  if (navPvtStatusFresh() && navPvtData_.positionDop > 0) {
    return true;
  }

  return pdopValid_;
}

float GnssMaxM10S::pdop() {
  if (navDopFresh() && navDopData_.positionDop > 0) {
    return static_cast<float>(navDopData_.positionDop) / 100.0f;
  }
  if (navPvtStatusFresh() && navPvtData_.positionDop > 0) {
    return static_cast<float>(navPvtData_.positionDop) / 100.0f;
  }

  return pdop_;
}

bool GnssMaxM10S::hasVdop() const {
  if (navDopFresh() && navDopData_.verticalDop > 0) {
    return true;
  }

  return vdopValid_;
}

float GnssMaxM10S::vdop() {
  if (navDopFresh() && navDopData_.verticalDop > 0) {
    return static_cast<float>(navDopData_.verticalDop) / 100.0f;
  }

  return vdop_;
}

bool GnssMaxM10S::hasLocation() const {
  if (navPvtStatusFresh() && navPvtData_.positionValid) {
    return true;
  }

  return gps_.location.isValid();
}

double GnssMaxM10S::latitude() {
  if (navPvtStatusFresh() && navPvtData_.positionValid) {
    return static_cast<double>(navPvtData_.latitudeDeg1e7) / 10000000.0;
  }

  return gps_.location.lat();
}

double GnssMaxM10S::longitude() {
  if (navPvtStatusFresh() && navPvtData_.positionValid) {
    return static_cast<double>(navPvtData_.longitudeDeg1e7) / 10000000.0;
  }

  return gps_.location.lng();
}

bool GnssMaxM10S::hasAltitude() const {
  if (navPvtStatusFresh() && navPvtData_.altitudeValid) {
    return true;
  }

  return gps_.altitude.isValid();
}

double GnssMaxM10S::altitudeMeters() {
  if (navPvtStatusFresh() && navPvtData_.altitudeValid) {
    return static_cast<double>(navPvtData_.heightMslMm) / 1000.0;
  }

  return gps_.altitude.meters();
}

bool GnssMaxM10S::hasSpeed() const {
  if (navPvtStatusFresh() && navPvtData_.speedValid) {
    return true;
  }

  return gps_.speed.isValid();
}

double GnssMaxM10S::speedMps() {
  if (navPvtStatusFresh() && navPvtData_.speedValid) {
    return static_cast<double>(navPvtData_.groundSpeedMmPerSec) / 1000.0;
  }

  return gps_.speed.mps();
}

void GnssMaxM10S::navPvtSnapshot(GnssNavPvtData& out) const {
  out = navPvtData_;
}

void GnssMaxM10S::monRfSnapshot(GnssMonRfData& out) const {
  out = monRfData_;
  if (out.valid && millis() - out.receivedMs > kMonRfStaleMs) {
    out.valid = false;
  }
}

void GnssMaxM10S::timTpSnapshot(GnssTimTpData& out) const {
  out = timTpData_;
}

void GnssMaxM10S::parseUbxByte(uint8_t byteValue) {
  switch (ubxParseState_) {
    case UBX_SYNC_1:
      if (byteValue == 0xB5) {
        ubxParseState_ = UBX_SYNC_2;
      }
      break;

    case UBX_SYNC_2:
      if (byteValue == 0x62) {
        ubxParseState_ = UBX_CLASS;
      } else if (byteValue != 0xB5) {
        ubxParseState_ = UBX_SYNC_1;
      }
      break;

    case UBX_CLASS:
      ubxMessageClass_ = byteValue;
      ubxCkA_ = 0;
      ubxCkB_ = 0;
      ubxCkA_ += byteValue;
      ubxCkB_ += ubxCkA_;
      ubxParseState_ = UBX_ID;
      break;

    case UBX_ID:
      ubxMessageId_ = byteValue;
      ubxCkA_ += byteValue;
      ubxCkB_ += ubxCkA_;
      ubxParseState_ = UBX_LEN_1;
      break;

    case UBX_LEN_1:
      ubxPayloadLength_ = byteValue;
      ubxCkA_ += byteValue;
      ubxCkB_ += ubxCkA_;
      ubxParseState_ = UBX_LEN_2;
      break;

    case UBX_LEN_2:
      ubxPayloadLength_ |= static_cast<uint16_t>(byteValue) << 8;
      ubxCkA_ += byteValue;
      ubxCkB_ += ubxCkA_;
      ubxPayloadPos_ = 0;
      if (ubxPayloadLength_ > kMaxUbxPayloadLength) {
        ubxParseState_ = UBX_SYNC_1;
      } else if (ubxPayloadLength_ == 0) {
        ubxParseState_ = UBX_CK_A;
      } else {
        ubxParseState_ = UBX_PAYLOAD;
      }
      break;

    case UBX_PAYLOAD:
      ubxPayload_[ubxPayloadPos_++] = byteValue;
      ubxCkA_ += byteValue;
      ubxCkB_ += ubxCkA_;
      if (ubxPayloadPos_ >= ubxPayloadLength_) {
        ubxParseState_ = UBX_CK_A;
      }
      break;

    case UBX_CK_A:
      if (byteValue == ubxCkA_) {
        ubxParseState_ = UBX_CK_B;
      } else {
        ubxParseState_ = UBX_SYNC_1;
      }
      break;

    case UBX_CK_B:
      if (byteValue == ubxCkB_) {
        handleUbxFrame(ubxMessageClass_, ubxMessageId_, ubxPayload_, ubxPayloadLength_, millis());
      }
      ubxParseState_ = UBX_SYNC_1;
      break;
  }
}

void GnssMaxM10S::handleUbxFrame(uint8_t messageClass,
                                 uint8_t messageId,
                                 const uint8_t* payload,
                                 uint16_t payloadLength,
                                 uint32_t nowMs) {
  // Dump raw UBX frames when logging is enabled to help diagnose payload issues.
  if (ubxLoggingEnabled_) {
    Serial.print("UBX FRAME: class=0x");
    if (messageClass < 16) Serial.print('0');
    Serial.print(messageClass, HEX);
    Serial.print(" id=0x");
    if (messageId < 16) Serial.print('0');
    Serial.print(messageId, HEX);
    Serial.print(" len=");
    Serial.print(payloadLength);
    Serial.print(" timeMs=");
    Serial.print(nowMs);
    Serial.print(" payload=");
    for (uint16_t i = 0; i < payloadLength; ++i) {
      const uint8_t b = payload[i];
      if (b < 16) Serial.print('0');
      Serial.print(b, HEX);
      if (i + 1 < payloadLength) Serial.print(' ');
    }
    Serial.println();
  }
  constexpr uint8_t UBX_CLASS_NAV = 0x01;
  constexpr uint8_t UBX_CLASS_ACK = 0x05;
  constexpr uint8_t UBX_CLASS_TIM = 0x0D;
  constexpr uint8_t UBX_CLASS_MON = 0x0A;
  constexpr uint8_t UBX_ID_NAV_PVT = 0x07;
  constexpr uint8_t UBX_ID_NAV_DOP = 0x04;
  constexpr uint8_t UBX_ID_TIM_TP = 0x01;
  constexpr uint8_t UBX_ID_MON_RF = 0x38;

  // ---- UBX-ACK-ACK / UBX-ACK-NAK ----------------------------------------
  if (messageClass == UBX_CLASS_ACK && payloadLength >= 2) {
    const bool isAck = messageId == 0x01;
    const bool isNak = messageId == 0x00;
    if (!isAck && !isNak) {
      return;
    }
    const uint8_t ackClass = payload[0];
    const uint8_t ackId = payload[1];
    if (ackClass == 0x06 && ackId == 0x8A) {
      monRfData_.cfgValsetAck = isAck;
      monRfData_.cfgValsetNak = isNak;
    } else if (ackClass == 0x06 && ackId == 0x13) {
      monRfData_.cfgAntAck = isAck;
      monRfData_.cfgAntNak = isNak;
    }
    return;
  }

  // ---- UBX-MON-RF (0x0A / 0x38) -----------------------------------------
  if (messageClass == UBX_CLASS_MON && messageId == UBX_ID_MON_RF && payloadLength >= 4) {
    const uint8_t nBlocks = payload[1];
    if (nBlocks > 0 && payloadLength >= 28) {
      monRfData_.blockId = payload[4];
      monRfData_.noisePerMS = static_cast<uint16_t>(payload[16]) |
                              (static_cast<uint16_t>(payload[17]) << 8);
      monRfData_.agcCnt = static_cast<uint16_t>(payload[18]) |
                          (static_cast<uint16_t>(payload[19]) << 8);
      monRfData_.jamInd = payload[20];
      monRfAgcCntSamples_[monRfAgcCntSamplePos_] = monRfData_.agcCnt;
      monRfAgcCntSamplePos_ =
          (monRfAgcCntSamplePos_ + 1) % (sizeof(monRfAgcCntSamples_) / sizeof(monRfAgcCntSamples_[0]));
      if (monRfAgcCntSampleCount_ < sizeof(monRfAgcCntSamples_) / sizeof(monRfAgcCntSamples_[0])) {
        ++monRfAgcCntSampleCount_;
      }
      uint16_t agcCntSum = 0;
      for (uint8_t i = 0; i < monRfAgcCntSampleCount_; ++i) {
        agcCntSum += monRfAgcCntSamples_[i];
      }
      monRfData_.agcCntAvg =
          (agcCntSum + (monRfAgcCntSampleCount_ / 2U)) / monRfAgcCntSampleCount_;
      monRfData_.receivedMs = nowMs;
      monRfData_.valid = true;
    }
    return;
  }

  // ---- UBX-TIM-TP (0x0D / 0x01) -----------------------------------------
  if (messageClass == UBX_CLASS_TIM && messageId == UBX_ID_TIM_TP && payloadLength >= 16) {
    timTpData_.towMs =
        static_cast<uint32_t>(payload[0]) |
        (static_cast<uint32_t>(payload[1]) << 8) |
        (static_cast<uint32_t>(payload[2]) << 16) |
        (static_cast<uint32_t>(payload[3]) << 24);
    const uint32_t towSubMs =
        static_cast<uint32_t>(payload[4]) |
        (static_cast<uint32_t>(payload[5]) << 8) |
        (static_cast<uint32_t>(payload[6]) << 16) |
        (static_cast<uint32_t>(payload[7]) << 24);
    timTpData_.qErrPs = static_cast<int32_t>(
        static_cast<uint32_t>(payload[8]) |
        (static_cast<uint32_t>(payload[9]) << 8) |
        (static_cast<uint32_t>(payload[10]) << 16) |
        (static_cast<uint32_t>(payload[11]) << 24));
    const uint16_t week =
        static_cast<uint16_t>(payload[12]) |
        (static_cast<uint16_t>(payload[13]) << 8);
    const uint8_t flags = payload[14];
    const bool utcTimeBase = (flags & 0x01) != 0;
    const bool utcAvailable = (flags & 0x02) != 0;
    timTpData_.qErrInvalid = (flags & 0x10) != 0;
    timTpData_.tpNotLocked = (flags & 0x20) != 0;
    timTpData_.utcEpochValid = false;
    timTpPpsEpochUtcNs_ = 0;
    if (utcTimeBase && utcAvailable && !timTpData_.tpNotLocked) {
      const uint64_t towNs =
          static_cast<uint64_t>(timTpData_.towMs) * 1000000ULL +
          ((static_cast<uint64_t>(towSubMs) * 1000000ULL + 0x80000000ULL) >> 32);
      int64_t epochNs =
          static_cast<int64_t>(UNIX_GPS_EPOCH_OFFSET_SECONDS) * 1000000000LL +
          static_cast<int64_t>(week) * static_cast<int64_t>(SECONDS_PER_WEEK) * 1000000000LL +
          static_cast<int64_t>(towNs);
      if (!timTpData_.qErrInvalid) {
        const int32_t qErrNs =
            (timTpData_.qErrPs >= 0)
                ? static_cast<int32_t>((timTpData_.qErrPs + 500) / 1000)
                : static_cast<int32_t>((timTpData_.qErrPs - 500) / 1000);
        epochNs += qErrNs;
      }
      if (epochNs > 0) {
        timTpPpsEpochUtcNs_ = static_cast<uint64_t>(epochNs);
        timTpData_.utcEpochValid = true;
      }
    }
    timTpData_.receivedMs = nowMs;
    timTpData_.receivedUs = micros();
    timTpData_.valid = true;
    timTpPendingForPps_ = timTpData_.utcEpochValid;
    return;
  }

  if (messageClass != UBX_CLASS_NAV) {
    return;
  }
  // if (messageId == UBX_ID_NAV_PVT) {
  //     Serial.printf("PVT length=%d\r\n", payloadLength);

  //     for (int i = 0; i < payloadLength; i++) {
  //         Serial.printf("%02X ", payload[i]);
  //     }
  //     Serial.println();
  // }
  // ---- UBX-NAV-PVT (0x01 / 0x07) ----------------------------------------
  // Payload must be at least 78 bytes to reach pDOP (bytes 76..77).
  if (messageId == UBX_ID_NAV_PVT && payloadLength >= 78) {
    const uint16_t year = static_cast<uint16_t>(payload[4]) |
                          (static_cast<uint16_t>(payload[5]) << 8);
    const uint8_t month = payload[6];
    const uint8_t day = payload[7];
    const uint8_t hour = payload[8];
    const uint8_t minute = payload[9];
    const uint8_t second = payload[10];
    const uint8_t validByte  = payload[11];   // validity flags
    const uint32_t timeAccuracyNs =
        static_cast<uint32_t>(payload[12]) |
        (static_cast<uint32_t>(payload[13]) << 8) |
        (static_cast<uint32_t>(payload[14]) << 16) |
        (static_cast<uint32_t>(payload[15]) << 24);
    const int32_t nanoUtc = static_cast<int32_t>(
        static_cast<uint32_t>(payload[16]) |
        (static_cast<uint32_t>(payload[17]) << 8) |
        (static_cast<uint32_t>(payload[18]) << 16) |
        (static_cast<uint32_t>(payload[19]) << 24));
    const uint8_t fixTypeByte = payload[20];  // fix type
    const uint8_t flagsByte  = payload[21];   // gnssFixOK etc.
    const uint8_t flags2Byte = payload[22];   // confirmedDate/Time
    const bool fixOk = (flagsByte & 0x01) != 0;
    const bool hasPositionFix =
        fixOk && (fixTypeByte == 2 || fixTypeByte == 3 || fixTypeByte == 4);

    navPvtData_.fixType       = fixTypeByte;
    navPvtData_.numSatellites = payload[23];
    navPvtData_.gnssFixOK     = fixOk;
    navPvtData_.validDate     = (validByte  & 0x01) != 0;
    navPvtData_.validTime     = (validByte  & 0x02) != 0;
    navPvtData_.fullyResolved = (validByte  & 0x04) != 0;
    navPvtData_.confirmedDate = (flags2Byte & 0x40) != 0;
    navPvtData_.confirmedTime = (flags2Byte & 0x80) != 0;
    navPvtData_.positionValid = hasPositionFix;
    navPvtData_.altitudeValid = hasPositionFix;
    navPvtData_.speedValid = hasPositionFix;
    navPvtData_.timeAccuracyNs = timeAccuracyNs;
    navPvtData_.nanoUtc        = nanoUtc;
    navPvtData_.longitudeDeg1e7 = readI4(payload, 24);
    navPvtData_.latitudeDeg1e7 = readI4(payload, 28);
    navPvtData_.heightMm = readI4(payload, 32);
    navPvtData_.heightMslMm = readI4(payload, 36);
    navPvtData_.horizontalAccuracyMm = readU4(payload, 40);
    navPvtData_.verticalAccuracyMm = readU4(payload, 44);
    navPvtData_.groundSpeedMmPerSec = readI4(payload, 60);
    navPvtData_.speedAccuracyMmPerSec = readU4(payload, 68);
    navPvtData_.positionDop = readU2(payload, 76);
    navPvtData_.utcEpochValid =
        navPvtData_.validDate &&
        navPvtData_.validTime &&
        year >= 2020 &&
        month >= 1 && month <= 12 &&
        day >= 1 && day <= 31 &&
        hour <= 23 &&
        minute <= 59 &&
        second <= 60;
    if (navPvtData_.utcEpochValid) {
      navPvtData_.epochUtc = toUnixUtc(
          static_cast<int>(year),
          static_cast<int>(month),
          static_cast<int>(day),
          static_cast<int>(hour),
          static_cast<int>(minute),
          static_cast<int>(second > 59 ? 59 : second));
    }
    navPvtData_.receivedMs = nowMs;
    navPvtData_.receivedUs = micros();
    navPvtData_.valid         = true;
    updateNavPvtLatencyEstimate(ppsNowTicksFn_());
    return;
  }

  // ---- UBX-NAV-DOP (0x01 / 0x04) ----------------------------------------
  if (messageId == UBX_ID_NAV_DOP && payloadLength >= 18) {
    navDopData_.geometricDop = readU2(payload, 4);
    navDopData_.positionDop = readU2(payload, 6);
    navDopData_.timeDop = readU2(payload, 8);
    navDopData_.verticalDop = readU2(payload, 10);
    navDopData_.horizontalDop = readU2(payload, 12);
    navDopData_.northingDop = readU2(payload, 14);
    navDopData_.eastingDop = readU2(payload, 16);
    navDopData_.receivedMs = nowMs;
    navDopData_.valid = true;
    return;
  }

}

void GnssMaxM10S::enableUbxLogging(bool enable) {
  ubxLoggingEnabled_ = enable;
}

void GnssMaxM10S::parseGsaSentence(const char* line) {
  String sentence(line);
  if (!sentence.startsWith("$") || sentence.indexOf("GSA") < 0) {
    return;
  }

  const int checksumAt = sentence.indexOf('*');
  if (checksumAt > 0) {
    sentence = sentence.substring(0, checksumAt);
  }
  if (sentence.length() < 6) {
    return;
  }

  const String messageType = sentence.substring(3, 6);
  if (messageType != "GSA") {
    return;
  }

  String fields[18];
  size_t fieldCount = 0;
  int start = 0;
  while (start >= 0 && fieldCount < 18) {
    const int comma = sentence.indexOf(',', start);
    if (comma < 0) {
      fields[fieldCount++] = sentence.substring(start);
      break;
    }
    fields[fieldCount++] = sentence.substring(start, comma);
    start = comma + 1;
  }

  if (fieldCount < 17) {
    return;
  }

  // GSA fields: [0] = "$GxGSA", [1]=mode, [2]=fixType, [3..14]=PRNs (12),
  // [15]=PDOP, [16]=HDOP, [17]=VDOP
  if (fieldCount < 18) {
    return;
  }

  const float parsedPdop = fields[15].toFloat();
  const float parsedHdop = fields[16].toFloat();
  const float parsedVdop = fields[17].toFloat();

  if (parsedPdop > 0.0f) {
    pdop_ = parsedPdop;
    pdopValid_ = true;
  }
  if (parsedHdop > 0.0f) {
    // TinyGPSPlus handles HDOP; but if it's not populated, we can keep a
    // local override by writing into gps_.hdop if available. For now, mark
    // hdop as valid by ensuring TinyGPSPlus will report it when present.
    (void)parsedHdop; // parsedHdop retained for future use if needed
  }
  if (parsedVdop > 0.0f) {
    vdop_ = parsedVdop;
    vdopValid_ = true;
  }

}

uint32_t GnssMaxM10S::toUnixUtc(
    int year,
    int month,
    int day,
    int hour,
    int minute,
    int second) {
  year -= month <= 2;
  const int era = (year >= 0 ? year : year - 399) / 400;
  const unsigned yoe = static_cast<unsigned>(year - era * 400);
  const unsigned doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + static_cast<unsigned>(day) - 1;
  const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  const int days = era * 146097 + static_cast<int>(doe) - 719468;

  const uint32_t secondsPerDay = 86400UL;
  return static_cast<uint32_t>(days) * secondsPerDay +
         static_cast<uint32_t>(hour) * 3600UL +
         static_cast<uint32_t>(minute) * 60UL +
         static_cast<uint32_t>(second);
}
