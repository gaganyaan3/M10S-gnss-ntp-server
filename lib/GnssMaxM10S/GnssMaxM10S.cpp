#include "GnssMaxM10S.h"

GnssMaxM10S::GnssMaxM10S(uint8_t rxPin,
                         uint8_t txPin,
                         uint32_t baud,
                         HardwareSerial& serialPort)
    : serial_(serialPort), rxPin_(rxPin), txPin_(txPin), baud_(baud) {}

void GnssMaxM10S::begin() {
  serial_.begin(baud_, SERIAL_8N1, rxPin_, txPin_);
}

void GnssMaxM10S::poll() {
  while (serial_.available() > 0) {
    const char c = static_cast<char>(serial_.read());
    if (c == '\n') {
      if (rawLineBufferLength_ > 0) {
        rawLineBuffer_[rawLineBufferLength_] = '\0';
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
    } else if (c != '\r') {
      if (rawLineBufferLength_ < (kMaxRawLineLength - 1)) {
        rawLineBuffer_[rawLineBufferLength_++] = c;
        rawLineBuffer_[rawLineBufferLength_] = '\0';
      }
    }
    gps_.encode(c);
  }
}

void GnssMaxM10S::enableRawLogging(bool enable) {
  rawLoggingEnabled_ = enable;
}

void GnssMaxM10S::onRawLine(RawLineCallback callback) {
  rawLineCallback_ = callback;
}

void GnssMaxM10S::onPpsEdge(uint32_t edgeMicros) {
  if (!gps_.date.isValid() || !gps_.time.isValid()) {
    ppsAligned_ = false;
    return;
  }

  const uint32_t baseEpoch = toUnixUtc(
      gps_.date.year(),
      gps_.date.month(),
      gps_.date.day(),
      gps_.time.hour(),
      gps_.time.minute(),
      gps_.time.second());

  ppsReferenceEpochSec_ = baseEpoch + 1UL;
  ppsEdgeMicros_ = edgeMicros;
  ppsAligned_ = true;
}

ClockStamp GnssMaxM10S::latestUtc() {
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

  const uint32_t nowMicros = micros();
  const uint32_t elapsedMicros = nowMicros >= ppsEdgeMicros_
                                     ? nowMicros - ppsEdgeMicros_
                                     : (0xFFFFFFFFUL - ppsEdgeMicros_) + nowMicros + 1UL;
  epochUtcOut = ppsReferenceEpochSec_ + (elapsedMicros / 1000000UL);
  return true;
}

bool GnssMaxM10S::currentUtcEpochMsPps(uint64_t& epochMsUtcOut) {
  if (!ppsAligned_) {
    return false;
  }

  const uint32_t nowMicros = micros();
  const uint32_t elapsedMicros = nowMicros >= ppsEdgeMicros_
                                     ? nowMicros - ppsEdgeMicros_
                                     : (0xFFFFFFFFUL - ppsEdgeMicros_) + nowMicros + 1UL;
  epochMsUtcOut = static_cast<uint64_t>(ppsReferenceEpochSec_) * 1000ULL +
                  static_cast<uint64_t>(elapsedMicros / 1000UL);
  return true;
}

uint32_t GnssMaxM10S::charsProcessed() const {
  return gps_.charsProcessed();
}

bool GnssMaxM10S::hasSatelliteCount() const {
  return gps_.satellites.isValid();
}

uint32_t GnssMaxM10S::satelliteCount() {
  return gps_.satellites.value();
}

bool GnssMaxM10S::hasHdop() const {
  return gps_.hdop.isValid();
}

float GnssMaxM10S::hdop() {
  return static_cast<float>(gps_.hdop.hdop());
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
