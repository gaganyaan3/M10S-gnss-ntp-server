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
  void onPpsEdge(uint32_t edgeTicks);
  ClockStamp latestUtc();
  bool currentUtcEpoch(uint32_t& epochUtcOut);
  bool currentUtcEpochMs(uint64_t& epochMsUtcOut);
  bool currentUtcEpochPps(uint32_t& epochUtcOut);
  bool currentUtcEpochMsPps(uint64_t& epochMsUtcOut);
  bool currentUtcEpochNsPps(uint64_t& epochNsUtcOut);
  // Measured PPS tick rate (Hz), continuously disciplined against the GPS's
  // own 1 Hz edge since the ESP32 crystal's absolute frequency is only
  // accurate to tens of ppm -- far coarser than the module's PPS spec.
  double ppsCalibratedTicksPerSecond() const;
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
  void enableUbxLogging(bool enable);

 private:
  static constexpr size_t kMaxRawLineLength = 128;
  static constexpr size_t kMaxSatelliteRecords = 64;
  static constexpr size_t kMaxUbxPayloadLength = 1024;
  static constexpr uint32_t kNavSatStaleMs = 6000UL;
  static uint32_t toUnixUtc(int year, int month, int day, int hour, int minute, int second);
  static GnssConstellation constellationFromPrn(int prn);
  static GnssConstellation constellationFromUbxGnssId(uint8_t gnssId);
  void parseGsvSentence(const char* line);
  void parseGsaSentence(const char* line);
  void parseUbxByte(uint8_t byteValue);
  void handleUbxFrame(uint8_t messageClass,
                      uint8_t messageId,
                      const uint8_t* payload,
                      uint16_t payloadLength,
                      uint32_t nowMs);
  void resetSatelliteRecords();
  void upsertSatellite(GnssConstellation constellation,
                       int prn,
                       int elevationDeg,
                       int azimuthDeg,
                       int cn0DbHz,
                       uint32_t nowMs);
  void pruneStaleSatellites(uint32_t nowMs);

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
  uint32_t ppsEdgeTicks_ = 0;
  TickClockFn ppsNowTicksFn_ = [](){ return micros(); };
  uint32_t ppsNominalTicksPerSecond_ = 1000000UL;
  double ppsCalibratedTicksPerSecond_ = 1000000.0;
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

  SatelliteRecord satelliteRecords_[kMaxSatelliteRecords] = {};
  uint32_t lastNavSatUpdateMs_ = 0;
};
