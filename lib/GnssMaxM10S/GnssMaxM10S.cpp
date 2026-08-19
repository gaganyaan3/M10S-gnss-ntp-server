#include "GnssMaxM10S.h"

namespace {

int parseIntToken(const String& token, int fallback) {
  if (token.length() == 0) {
    return fallback;
  }
  return token.toInt();
}

}  // namespace

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
    const uint8_t byteValue = static_cast<uint8_t>(serial_.read());
    const char c = static_cast<char>(byteValue);
    parseUbxByte(byteValue);

    if (c == '\n') {
      if (rawLineBufferLength_ > 0) {
        rawLineBuffer_[rawLineBufferLength_] = '\0';
        if (rawLineBuffer_[0] == '$') {
          // Parse GSA for dilution values and GSV for per-satellite records
          // only when stats collection is enabled to avoid unnecessary CPU work.
          if (statsEnabled_) {
            parseGsaSentence(rawLineBuffer_);
            parseGsvSentence(rawLineBuffer_);
          }
          if (rawLoggingEnabled_) {
            Serial.print("RAW: ");
            Serial.println(rawLineBuffer_);
          }
          if (rawLineCallback_) {
            rawLineCallback_(String(rawLineBuffer_));
          }
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

void GnssMaxM10S::enableStats(bool enable) {
  statsEnabled_ = enable;
  if (!statsEnabled_) {
    resetSatelliteRecords();
    lastNavSatUpdateMs_ = 0;
  }
}

void GnssMaxM10S::onRawLine(RawLineCallback callback) {
  rawLineCallback_ = callback;
}

void GnssMaxM10S::configurePpsHardwareClock(TickClockFn nowTicksFn, uint32_t ticksPerSecond) {
  ppsNowTicksFn_ = nowTicksFn ? nowTicksFn : [](){ return micros(); };
  ppsNominalTicksPerSecond_ = (ticksPerSecond > 0) ? ticksPerSecond : 1000000UL;
  ppsCalibratedTicksPerSecond_ = static_cast<double>(ppsNominalTicksPerSecond_);
}

double GnssMaxM10S::ppsCalibratedTicksPerSecond() const {
  return ppsCalibratedTicksPerSecond_;
}

void GnssMaxM10S::onPpsEdge(uint32_t edgeTicks) {
  if (!gps_.date.isValid() || !gps_.time.isValid()) {
    ppsAligned_ = false;
    ppsHasPreviousEdge_ = false;
    return;
  }

  if (ppsHasPreviousEdge_) {
    // Unsigned subtraction naturally unwraps the free-running tick counter.
    const uint32_t measuredTicks = edgeTicks - ppsEdgeTicks_;
    const double nominal = static_cast<double>(ppsNominalTicksPerSecond_);
    // Reject missed edges / startup glitches instead of corrupting the estimate.
    if (measuredTicks >= nominal * 0.99 && measuredTicks <= nominal * 1.01) {
      constexpr double kSmoothing = 0.1;  // EMA weight per new PPS interval
      ppsCalibratedTicksPerSecond_ = ppsCalibratedTicksPerSecond_ * (1.0 - kSmoothing) +
                                      static_cast<double>(measuredTicks) * kSmoothing;
    }
  }

  const uint32_t baseEpoch = toUnixUtc(
      gps_.date.year(),
      gps_.date.month(),
      gps_.date.day(),
      gps_.time.hour(),
      gps_.time.minute(),
      gps_.time.second());

  ppsReferenceEpochSec_ = baseEpoch + 1UL;
  ppsEdgeTicks_ = edgeTicks;
  ppsAligned_ = true;
  ppsHasPreviousEdge_ = true;
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

  const uint32_t nowTicks = ppsNowTicksFn_();
  const uint32_t elapsedTicks = nowTicks - ppsEdgeTicks_;
  const double elapsedSeconds = static_cast<double>(elapsedTicks) / ppsCalibratedTicksPerSecond_;
  epochUtcOut = ppsReferenceEpochSec_ + static_cast<uint32_t>(elapsedSeconds);
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

  const uint32_t nowTicks = ppsNowTicksFn_();
  const uint32_t elapsedTicks = nowTicks - ppsEdgeTicks_;
  const double elapsedSeconds = static_cast<double>(elapsedTicks) / ppsCalibratedTicksPerSecond_;
  const uint64_t wholeSeconds = static_cast<uint64_t>(elapsedSeconds);
  const double subSecondSeconds = elapsedSeconds - static_cast<double>(wholeSeconds);
  const uint64_t subSecondNs = static_cast<uint64_t>(subSecondSeconds * 1000000000.0 + 0.5);

  epochNsUtcOut = (static_cast<uint64_t>(ppsReferenceEpochSec_) + wholeSeconds) * 1000000000ULL + subSecondNs;
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

bool GnssMaxM10S::hasPdop() const {
  return pdopValid_;
}

float GnssMaxM10S::pdop() {
  return pdop_;
}

bool GnssMaxM10S::hasVdop() const {
  return vdopValid_;
}

float GnssMaxM10S::vdop() {
  return vdop_;
}

bool GnssMaxM10S::hasLocation() const {
  return gps_.location.isValid();
}

double GnssMaxM10S::latitude() {
  return gps_.location.lat();
}

double GnssMaxM10S::longitude() {
  return gps_.location.lng();
}

bool GnssMaxM10S::hasAltitude() const {
  return gps_.altitude.isValid();
}

double GnssMaxM10S::altitudeMeters() {
  return gps_.altitude.meters();
}

bool GnssMaxM10S::hasSpeed() const {
  return gps_.speed.isValid();
}

double GnssMaxM10S::speedMps() {
  return gps_.speed.mps();
}

void GnssMaxM10S::navSatSnapshot(GnssNavSatSnapshot& out) const {
  out = GnssNavSatSnapshot{};

  const uint32_t nowMs = millis();
  for (size_t i = 0; i < kMaxSatelliteRecords; ++i) {
    const SatelliteRecord& record = satelliteRecords_[i];
    if (!record.inUse) {
      continue;
    }
    if (nowMs - record.lastSeenMs > kNavSatStaleMs) {
      continue;
    }

    if (out.satelliteCount < GnssNavSatSnapshot::kMaxSatellites) {
      out.satellites[out.satelliteCount++] = record.info;
    }

    switch (record.info.constellation) {
      case GNSS_CONSTELLATION_GPS:
        ++out.gpsCount;
        break;
      case GNSS_CONSTELLATION_GALILEO:
        ++out.galileoCount;
        break;
      case GNSS_CONSTELLATION_BEIDOU:
        ++out.beidouCount;
        break;
      case GNSS_CONSTELLATION_GLONASS:
        ++out.glonassCount;
        break;
      case GNSS_CONSTELLATION_QZSS:
        ++out.qzssCount;
        break;
      case GNSS_CONSTELLATION_SBAS:
        ++out.sbasCount;
        break;
      case GNSS_CONSTELLATION_OTHER:
        ++out.otherCount;
        break;
      default:
        break;
    }
  }

  out.valid = (nowMs - lastNavSatUpdateMs_ <= kNavSatStaleMs) && (out.satelliteCount > 0);
}

GnssConstellation GnssMaxM10S::constellationFromUbxGnssId(uint8_t gnssId) {
  if (gnssId == 0) {
    return GNSS_CONSTELLATION_GPS;
  }
  if (gnssId == 1) {
    return GNSS_CONSTELLATION_SBAS;
  }
  if (gnssId == 2) {
    return GNSS_CONSTELLATION_GALILEO;
  }
  if (gnssId == 3) {
    return GNSS_CONSTELLATION_BEIDOU;
  }
  if (gnssId == 5) {
    return GNSS_CONSTELLATION_QZSS;
  }
  if (gnssId == 6) {
    return GNSS_CONSTELLATION_GLONASS;
  }
  return GNSS_CONSTELLATION_OTHER;
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
  constexpr uint8_t UBX_ID_NAV_SAT = 0x35;

  if (messageClass != UBX_CLASS_NAV || messageId != UBX_ID_NAV_SAT || payloadLength < 8) {
    return;
  }

  // Honor stats collection flag: if disabled, don't process NAV-SAT payloads.
  if (!statsEnabled_) {
    return;
  }

  const uint8_t numSvs = payload[5];
  const uint16_t availableRecords = static_cast<uint16_t>((payloadLength - 8) / 12);
  // Some firmware/variants may set numSvs to 0 or an incorrect value; prefer
  // the number of available records when numSvs looks invalid.
  const uint16_t recordsToRead = (numSvs > 0) ? (numSvs < availableRecords ? numSvs : availableRecords)
                                                    : availableRecords;

  resetSatelliteRecords();

  for (uint16_t i = 0; i < recordsToRead; ++i) {
    const uint16_t offset = static_cast<uint16_t>(8 + i * 12);
    const uint8_t gnssId = payload[offset + 0];
    const uint8_t svId = payload[offset + 1];
    const uint8_t cno = payload[offset + 2];
    const int8_t elevation = static_cast<int8_t>(payload[offset + 3]);
    const int16_t azimuth = static_cast<int16_t>(
        static_cast<uint16_t>(payload[offset + 4]) |
        (static_cast<uint16_t>(payload[offset + 5]) << 8));

    // Prefer the GNSS ID when it yields a known constellation, but fall back
    // to PRN-based detection when GNSS ID is missing or unrecognized.
    GnssConstellation constellation = constellationFromUbxGnssId(gnssId);
    if (constellation == GNSS_CONSTELLATION_OTHER || constellation == GNSS_CONSTELLATION_UNKNOWN) {
      const int prnCandidate = static_cast<int>(svId);
      const GnssConstellation byPrn = constellationFromPrn(prnCandidate);
      if (byPrn != GNSS_CONSTELLATION_UNKNOWN) {
        constellation = byPrn;
      } else if (constellation == GNSS_CONSTELLATION_UNKNOWN) {
        constellation = GNSS_CONSTELLATION_OTHER;
      }
    }
    const int cn0DbHz = cno > 0 ? static_cast<int>(cno) : -1;

    upsertSatellite(constellation,
                   static_cast<int>(svId),
                   static_cast<int>(elevation),
                   static_cast<int>(azimuth),
                   cn0DbHz,
                   nowMs);
  }

  lastNavSatUpdateMs_ = nowMs;
}

void GnssMaxM10S::resetSatelliteRecords() {
  for (size_t i = 0; i < kMaxSatelliteRecords; ++i) {
    satelliteRecords_[i].inUse = false;
    satelliteRecords_[i].lastSeenMs = 0;
    satelliteRecords_[i].info = GnssSatelliteInfo{};
  }
}

void GnssMaxM10S::enableUbxLogging(bool enable) {
  ubxLoggingEnabled_ = enable;
}

GnssConstellation GnssMaxM10S::constellationFromPrn(int prn) {
  if (prn >= 1 && prn <= 32) {
    return GNSS_CONSTELLATION_GPS;
  }
  if (prn >= 33 && prn <= 64) {
    return GNSS_CONSTELLATION_SBAS;
  }
  if (prn >= 65 && prn <= 96) {
    return GNSS_CONSTELLATION_GLONASS;
  }
  if (prn >= 120 && prn <= 158) {
    return GNSS_CONSTELLATION_SBAS;
  }
  if (prn >= 193 && prn <= 199) {
    return GNSS_CONSTELLATION_QZSS;
  }
  if (prn >= 201 && prn <= 237) {
    return GNSS_CONSTELLATION_BEIDOU;
  }
  if (prn >= 301 && prn <= 336) {
    return GNSS_CONSTELLATION_GALILEO;
  }

  return GNSS_CONSTELLATION_UNKNOWN;
}

GnssConstellation constellationFromTalker(const String& talker) {
  if (talker == "GP") {
    return GNSS_CONSTELLATION_GPS;
  }
  if (talker == "GA") {
    return GNSS_CONSTELLATION_GALILEO;
  }
  if (talker == "GB" || talker == "BD") {
    return GNSS_CONSTELLATION_BEIDOU;
  }
  if (talker == "GL") {
    return GNSS_CONSTELLATION_GLONASS;
  }
  if (talker == "GQ") {
    return GNSS_CONSTELLATION_QZSS;
  }
  if (talker == "GS" || talker == "SB") {
    return GNSS_CONSTELLATION_SBAS;
  }
  return GNSS_CONSTELLATION_UNKNOWN;
}

void GnssMaxM10S::parseGsvSentence(const char* line) {
  String sentence(line);
  if (!sentence.startsWith("$") || sentence.indexOf("GSV") < 0) {
    return;
  }

  const int checksumAt = sentence.indexOf('*');
  if (checksumAt > 0) {
    sentence = sentence.substring(0, checksumAt);
  }
  if (sentence.length() < 6) {
    return;
  }

  const String talker = sentence.substring(1, 3);
  const String messageType = sentence.substring(3, 6);
  if (messageType != "GSV") {
    return;
  }

  String fields[24];
  size_t fieldCount = 0;
  int start = 0;
  while (start >= 0 && fieldCount < 24) {
    const int comma = sentence.indexOf(',', start);
    if (comma < 0) {
      fields[fieldCount++] = sentence.substring(start);
      break;
    }
    fields[fieldCount++] = sentence.substring(start, comma);
    start = comma + 1;
  }

  if (fieldCount < 8) {
    return;
  }

  const GnssConstellation talkerConstellation = constellationFromTalker(talker);
  const uint32_t nowMs = millis();
  for (size_t i = 4; i + 3 < fieldCount; i += 4) {
    const int prn = parseIntToken(fields[i], -1);
    if (prn <= 0) {
      continue;
    }

    const int elev = parseIntToken(fields[i + 1], -1);
    const int az = parseIntToken(fields[i + 2], -1);
    const int cn0 = parseIntToken(fields[i + 3], -1);

    GnssConstellation constellation = talkerConstellation;
    if (constellation == GNSS_CONSTELLATION_UNKNOWN || talker == "GN") {
      constellation = constellationFromPrn(prn);
    }
    if (constellation == GNSS_CONSTELLATION_UNKNOWN) {
      constellation = GNSS_CONSTELLATION_OTHER;
    }

    upsertSatellite(constellation, prn, elev, az, cn0, nowMs);
    lastNavSatUpdateMs_ = nowMs;
  }

  pruneStaleSatellites(nowMs);
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

void GnssMaxM10S::upsertSatellite(GnssConstellation constellation,
                                  int prn,
                                  int elevationDeg,
                                  int azimuthDeg,
                                  int cn0DbHz,
                                  uint32_t nowMs) {
  size_t freeSlot = kMaxSatelliteRecords;
  for (size_t i = 0; i < kMaxSatelliteRecords; ++i) {
    if (!satelliteRecords_[i].inUse) {
      if (freeSlot == kMaxSatelliteRecords) {
        freeSlot = i;
      }
      continue;
    }
    if (satelliteRecords_[i].info.constellation == constellation &&
        satelliteRecords_[i].info.prn == static_cast<uint16_t>(prn)) {
      satelliteRecords_[i].info.elevationDeg = static_cast<int16_t>(elevationDeg);
      satelliteRecords_[i].info.azimuthDeg = static_cast<int16_t>(azimuthDeg);
      satelliteRecords_[i].info.cn0DbHz = static_cast<int16_t>(cn0DbHz);
      satelliteRecords_[i].lastSeenMs = nowMs;
      return;
    }
  }

  if (freeSlot == kMaxSatelliteRecords) {
    return;
  }

  satelliteRecords_[freeSlot].inUse = true;
  satelliteRecords_[freeSlot].info.constellation = constellation;
  satelliteRecords_[freeSlot].info.prn = static_cast<uint16_t>(prn);
  satelliteRecords_[freeSlot].info.elevationDeg = static_cast<int16_t>(elevationDeg);
  satelliteRecords_[freeSlot].info.azimuthDeg = static_cast<int16_t>(azimuthDeg);
  satelliteRecords_[freeSlot].info.cn0DbHz = static_cast<int16_t>(cn0DbHz);
  satelliteRecords_[freeSlot].lastSeenMs = nowMs;
}

void GnssMaxM10S::pruneStaleSatellites(uint32_t nowMs) {
  for (size_t i = 0; i < kMaxSatelliteRecords; ++i) {
    if (!satelliteRecords_[i].inUse) {
      continue;
    }
    if (nowMs - satelliteRecords_[i].lastSeenMs > kNavSatStaleMs) {
      satelliteRecords_[i].inUse = false;
    }
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
