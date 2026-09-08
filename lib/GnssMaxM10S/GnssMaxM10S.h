#pragma once

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <functional>

// Subset of UBX-NAV-PVT fields relevant to fix, time, and dashboard status.
// fixType values: 0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS+DR, 5=time only
struct GnssNavPvtData {
  bool valid = false;           // true once a NAV-PVT frame has been received
  uint8_t fixType = 0;          // payload byte 20
  uint8_t numSatellites = 0;    // payload byte 23
  bool gnssFixOK = false;       // flags byte 21, bit 0
  bool validDate = false;       // valid byte 11, bit 0
  bool validTime = false;       // valid byte 11, bit 1
  bool fullyResolved = false;   // valid byte 11, bit 2
  bool confirmedDate = false;   // flags2 byte 22, bit 6
  bool confirmedTime = false;   // flags2 byte 22, bit 7
  bool positionValid = false;
  bool altitudeValid = false;
  bool speedValid = false;
  bool utcEpochValid = false;
  uint32_t epochUtc = 0;
  int32_t nanoUtc = 0;           // payload bytes 16..19, signed nanosecond fraction
  uint32_t timeAccuracyNs = 0;   // payload bytes 12..15
  int32_t longitudeDeg1e7 = 0;   // payload bytes 24..27
  int32_t latitudeDeg1e7 = 0;    // payload bytes 28..31
  int32_t heightMm = 0;          // payload bytes 32..35, ellipsoid height
  int32_t heightMslMm = 0;       // payload bytes 36..39, mean sea level height
  uint32_t horizontalAccuracyMm = 0;
  uint32_t verticalAccuracyMm = 0;
  int32_t groundSpeedMmPerSec = 0;
  uint32_t speedAccuracyMmPerSec = 0;
  uint16_t positionDop = 0;      // payload bytes 76..77, scale 0.01
  uint32_t receivedMs = 0;
  uint32_t receivedUs = 0;
};

struct GnssNavDopData {
  bool valid = false;
  uint16_t geometricDop = 0;
  uint16_t positionDop = 0;
  uint16_t timeDop = 0;
  uint16_t verticalDop = 0;
  uint16_t horizontalDop = 0;
  uint16_t northingDop = 0;
  uint16_t eastingDop = 0;
  uint32_t receivedMs = 0;
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
  void navPvtSnapshot(GnssNavPvtData& out) const;
  void monRfSnapshot(GnssMonRfData& out) const;
  void timTpSnapshot(GnssTimTpData& out) const;
  void enableUbxLogging(bool enable);

 private:
  static constexpr size_t kMaxRawLineLength = 128;
  static constexpr size_t kMaxUbxPayloadLength = 128;
  static constexpr uint32_t kMaxPvtAgeForPpsMs = 1500UL;
  static constexpr uint32_t kMaxNavPvtStatusAgeMs = 5000UL;
  static constexpr uint32_t kMaxNavDopAgeMs = 5000UL;
  static constexpr uint32_t kMaxTimTpAgeForPpsMs = 1500UL;
  static constexpr uint32_t kMaxPpsHoldoverMs = 10000UL;
  static constexpr uint32_t kPpsRecentlySeenMs = 2500UL;
  static constexpr uint32_t kMonRfStaleMs = 3000UL;
  static uint32_t toUnixUtc(int year, int month, int day, int hour, int minute, int second);
  static int32_t readI4(const uint8_t* payload, uint16_t offset);
  static uint32_t readU4(const uint8_t* payload, uint16_t offset);
  static uint16_t readU2(const uint8_t* payload, uint16_t offset);
  bool navPvtStatusFresh() const;
  bool navDopFresh() const;
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
  bool timTpCanAnchorPps(uint64_t& epochNsUtcOut) const;

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
  bool navPvtLatencyUsValid_ = false;
  uint32_t navPvtLatencyUs_ = 0;
  uint16_t navPvtLatencySamples_ = 0;

  GnssNavPvtData navPvtData_ = {};
  GnssNavDopData navDopData_ = {};
  GnssMonRfData monRfData_ = {};
  GnssTimTpData timTpData_ = {};
  bool timTpPendingForPps_ = false;
  uint64_t timTpPpsEpochUtcNs_ = 0;
  uint16_t monRfAgcCntSamples_[5] = {};
  uint8_t monRfAgcCntSamplePos_ = 0;
  uint8_t monRfAgcCntSampleCount_ = 0;
  uint32_t lastMonRfPollMs_ = 0;
};
