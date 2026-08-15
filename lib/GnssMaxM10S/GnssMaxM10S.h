#pragma once

#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <functional>

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
  void onPpsEdge(uint32_t edgeMicros);
  ClockStamp latestUtc();
  bool currentUtcEpoch(uint32_t& epochUtcOut);
  bool currentUtcEpochMs(uint64_t& epochMsUtcOut);
  bool currentUtcEpochPps(uint32_t& epochUtcOut);
  bool currentUtcEpochMsPps(uint64_t& epochMsUtcOut);
  uint32_t charsProcessed() const;
  bool hasSatelliteCount() const;
  uint32_t satelliteCount();
  bool hasHdop() const;
  float hdop();

 private:
  static constexpr size_t kMaxRawLineLength = 128;
  static uint32_t toUnixUtc(int year, int month, int day, int hour, int minute, int second);

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
  uint32_t ppsReferenceEpochSec_ = 0;
  uint32_t ppsEdgeMicros_ = 0;
};
