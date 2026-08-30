#pragma once

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <functional>

enum GnssConstellation : uint8_t {
  GNSS_CONSTELLATION_UNKNOWN = 0,
  GNSS_CONSTELLATION_GPS = 1,
  GNSS_CONSTELLATION_GALILEO = 2,
  GNSS_CONSTELLATION_BEIDOU = 3,
  GNSS_CONSTELLATION_GLONASS = 4,
  GNSS_CONSTELLATION_QZSS = 5,
  GNSS_CONSTELLATION_SBAS = 6,
  GNSS_CONSTELLATION_OTHER = 7,
};

struct GnssSatelliteInfo {
  uint8_t constellation = GNSS_CONSTELLATION_UNKNOWN;
  uint16_t prn = 0;
  int16_t elevationDeg = 0;
  int16_t azimuthDeg = 0;
  int16_t cn0DbHz = -1;
  bool navSatFlagsValid = false;
  uint8_t qualityInd = 0;
  bool svUsed = false;
};

// Subset of UBX-NAV-PVT fields relevant to fix quality and time validity.
// fixType values: 0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS+DR, 5=time only
struct GnssNavPvtData {
  bool valid = false;           // true once a NAV-PVT frame has been received
  uint8_t fixType = 0;          // payload byte 20
  bool gnssFixOK = false;       // flags byte 21, bit 0
  bool validDate = false;       // valid byte 11, bit 0
  bool validTime = false;       // valid byte 11, bit 1
  bool fullyResolved = false;   // valid byte 11, bit 2
  bool confirmedDate = false;   // flags2 byte 22, bit 6
  bool confirmedTime = false;   // flags2 byte 22, bit 7
  bool utcEpochValid = false;
  uint32_t epochUtc = 0;
  int32_t nanoUtc = 0;           // payload bytes 16..19, signed nanosecond fraction
  uint32_t timeAccuracyNs = 0;   // payload bytes 12..15
  uint32_t receivedMs = 0;
  uint32_t receivedUs = 0;
};

struct GnssNavSatSnapshot {
  static constexpr size_t kMaxSatellites = 64;

  bool valid = false;
  uint16_t gpsCount = 0;
  uint16_t galileoCount = 0;
  uint16_t beidouCount = 0;
  uint16_t glonassCount = 0;
  uint16_t qzssCount = 0;
  uint16_t sbasCount = 0;
  uint16_t otherCount = 0;
  size_t satelliteCount = 0;
  GnssSatelliteInfo satellites[kMaxSatellites];
};

struct GnssMonRfData {
  bool valid = false;
  uint8_t jamInd = 0;
  uint16_t noisePerMS = 0;
  uint16_t agcCnt = 0;
  uint16_t agcCntAvg = 0;
  uint8_t blockId = 0;
  bool cfgValsetAck = false;
  bool cfgValsetNak = false;
  bool cfgAntAck = false;
  bool cfgAntNak = false;
  uint32_t receivedMs = 0;
};

struct GnssTimTpData {
  bool valid = false;
  uint32_t towMs = 0;
  int32_t qErrPs = 0;
  uint8_t flags = 0;
  uint8_t refInfo = 0;
  bool qErrInvalid = false;
  bool tpNotLocked = false;
  bool utcEpochValid = false;
  uint32_t receivedMs = 0;
  uint32_t receivedUs = 0;
};

struct ClockStamp {
  bool valid;
  uint32_t epochUtc;
};

class GnssMaxM10S {
 public:
  using RawLineCallback = std::function<void(const String&)>;

  GnssMaxM10S(uint8_t rxPin,
              uint8_t txPin,
              uint32_t baud = 9600,
              HardwareSerial& serialPort = Serial2);

  void begin();
  void poll();
  void enableRawLogging(bool enable);
  void enableStats(bool enable);
  void onRawLine(RawLineCallback callback);

  // Provides the PPS edge/clock code with a hardware tick source instead of
  // the default micros() fallback. `nowTicksFn` must return the live value of
  // a free-running counter clocked at `ticksPerSecond` (e.g. an MCPWM capture
  // timer sampled via a software-forced capture), in the SAME domain as the
  // ticks passed to onPpsEdge(), so elapsed-time math is a pure tick subtraction
  // instead of relying on a separately-clocked software timer.
  using TickClockFn = std::function<uint32_t()>;
  void configurePpsHardwareClock(TickClockFn nowTicksFn, uint32_t ticksPerSecond);
  void setPpsHardwareTicksPerSecond(uint32_t ticksPerSecond);
  void onPpsEdge(uint32_t edgeTicks, uint32_t elapsedPulses = 1);
  ClockStamp latestUtc();
  bool currentUtcEpoch(uint32_t& epochUtcOut);
  bool currentUtcEpochMs(uint64_t& epochMsUtcOut);
  bool currentUtcEpochPps(uint32_t& epochUtcOut);
  bool currentUtcEpochMsPps(uint64_t& epochMsUtcOut);
  bool currentUtcEpochNsPps(uint64_t& epochNsUtcOut);
  bool navPvtUtcEpochMs(uint64_t& epochMsUtcOut);
  bool navPvtUtcEpochUs(uint64_t& epochUsUtcOut);
  bool ppsRecentlySeen() const;
  bool isPpsAligned() const;
  uint32_t charsProcessed() const;
  bool hasSatelliteCount() const;
  uint32_t satelliteCount();
  bool hasHdop() const;
  float hdop();
  bool hasPdop() const;
  float pdop();
  bool hasVdop() const;
  float vdop();
  bool hasLocation() const;
  double latitude();
  double longitude();
  bool hasAltitude() const;
  double altitudeMeters();
  bool hasSpeed() const;
  double speedMps();
  void navSatSnapshot(GnssNavSatSnapshot& out) const;
  void navPvtSnapshot(GnssNavPvtData& out) const;
  void monRfSnapshot(GnssMonRfData& out) const;
  void timTpSnapshot(GnssTimTpData& out) const;
  void enableUbxLogging(bool enable);

 private:
  static constexpr size_t kMaxRawLineLength = 128;
  static constexpr size_t kMaxSatelliteRecords = 64;
  static constexpr size_t kMaxUbxPayloadLength = 1024;
  static constexpr uint32_t kNavSatStaleMs = 6000UL;
  static constexpr uint32_t kSatelliteCountStaleMs = 6000UL;
  static constexpr uint32_t kMaxPvtAgeForPpsMs = 1500UL;
  static constexpr uint32_t kMaxTimTpAgeForPpsMs = 1500UL;
  static constexpr uint32_t kMaxPpsHoldoverMs = 10000UL;
  static constexpr uint32_t kPpsRecentlySeenMs = 2500UL;
  static constexpr uint32_t kMonRfStaleMs = 3000UL;
  static uint32_t toUnixUtc(int year, int month, int day, int hour, int minute, int second);
  static GnssConstellation constellationFromPrn(int prn);
  static GnssConstellation constellationFromUbxGnssId(uint8_t gnssId);
  void parseGsvSentence(const char* line);
  void parseGsaSentence(const char* line);
  void parseUbxByte(uint8_t byteValue);
  void sendUbxCfgPrtUart1(uint32_t baudRate);
  void sendUbxCfgMsg(uint8_t msgClass, uint8_t msgId, uint8_t rate);
  void sendUbxCfgAntSupervisor();
  void sendUbxCfgValsetBool(const uint32_t* keys, size_t keyCount, bool value);
  void sendUbxPoll(uint8_t msgClass, uint8_t msgId);
  void handleUbxFrame(uint8_t messageClass,
                      uint8_t messageId,
                      const uint8_t* payload,
                      uint16_t payloadLength,
                      uint32_t nowMs);
  bool navPvtRawEpochUs(uint64_t& epochUsUtcOut) const;
  uint64_t ppsTicksToUs(uint64_t ticks) const;
  bool ppsEpochUsAtTicks(uint32_t ticks, uint64_t& epochUsUtcOut) const;
  void updateNavPvtLatencyEstimate(uint32_t receivedTicks);
  void resetSatelliteRecords();
  void upsertSatellite(GnssConstellation constellation,
                       int prn,
                       int elevationDeg,
                       int azimuthDeg,
                       int cn0DbHz,
                       bool navSatFlagsValid,
                       uint8_t qualityInd,
                       bool svUsed,
                       uint32_t nowMs);
  void pruneStaleSatellites(uint32_t nowMs);
  bool timTpCanAnchorPps(uint64_t& epochNsUtcOut) const;

  struct SatelliteRecord {
    bool inUse = false;
    GnssSatelliteInfo info;
    uint32_t lastSeenMs = 0;
  };

  HardwareSerial& serial_;
  uint8_t rxPin_;
  uint8_t txPin_;
  TinyGPSPlus gps_;
  uint32_t baud_;
  bool rawLoggingEnabled_ = false;
  char rawLineBuffer_[kMaxRawLineLength] = {};
  size_t rawLineBufferLength_ = 0;
  RawLineCallback rawLineCallback_;
  bool ppsAligned_ = false;
  bool ppsHasPreviousEdge_ = false;
  uint32_t ppsReferenceEpochSec_ = 0;
  int32_t ppsReferenceEpochOffsetNs_ = 0;
  uint32_t ppsEdgeTicks_ = 0;
  uint32_t ppsEdgeMs_ = 0;
  TickClockFn ppsNowTicksFn_ = [](){ return micros(); };
  uint32_t ppsNominalTicksPerSecond_ = 1000000UL;
  float pdop_ = 0.0f;
  float vdop_ = 0.0f;
  bool pdopValid_ = false;
  bool vdopValid_ = false;

  enum UbxParseState : uint8_t {
    UBX_SYNC_1 = 0,
    UBX_SYNC_2,
    UBX_CLASS,
    UBX_ID,
    UBX_LEN_1,
    UBX_LEN_2,
    UBX_PAYLOAD,
    UBX_CK_A,
    UBX_CK_B,
  };

  UbxParseState ubxParseState_ = UBX_SYNC_1;
  uint8_t ubxMessageClass_ = 0;
  uint8_t ubxMessageId_ = 0;
  uint16_t ubxPayloadLength_ = 0;
  uint16_t ubxPayloadPos_ = 0;
  uint8_t ubxCkA_ = 0;
  uint8_t ubxCkB_ = 0;
  uint8_t ubxPayload_[kMaxUbxPayloadLength] = {};

  bool ubxLoggingEnabled_ = false;
  bool statsEnabled_ = true;
  bool navPvtLatencyUsValid_ = false;
  uint32_t navPvtLatencyUs_ = 0;
  uint16_t navPvtLatencySamples_ = 0;

  GnssNavPvtData navPvtData_ = {};
  GnssMonRfData monRfData_ = {};
  GnssTimTpData timTpData_ = {};
  bool timTpPendingForPps_ = false;
  uint64_t timTpPpsEpochUtcNs_ = 0;
  uint16_t monRfAgcCntSamples_[5] = {};
  uint8_t monRfAgcCntSamplePos_ = 0;
  uint8_t monRfAgcCntSampleCount_ = 0;
  SatelliteRecord satelliteRecords_[kMaxSatelliteRecords] = {};
  uint32_t lastNavSatUpdateMs_ = 0;
  uint32_t lastMonRfPollMs_ = 0;
};
