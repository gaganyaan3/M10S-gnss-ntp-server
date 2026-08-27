#pragma once

#include <Arduino.h>
#include <U8g2lib.h>

struct OledClockPage {
  const char* gpsTimeLine = "No GNSS";
  const char* gpsDateLine = "fix";
  const char* ntpTimeLine = "Syncing";
  const char* statusLine = "";
  uint32_t satelliteCount = 0;
};

class OledClockDisplay {
 public:
  explicit OledClockDisplay(U8G2& display);

  void begin();
  void setEnabled(bool enabled);
  bool enabled() const;
  void showBoot();
  void showResetHold(uint32_t heldMs, uint32_t holdSeconds);
  void drawClockPage(const OledClockPage& page);

  static bool formatIstTimeOnlyFromEpoch(uint32_t epochUtc, char* out, size_t outLen);
  static bool formatIstDateOnlyFromEpoch(uint32_t epochUtc, char* out, size_t outLen);
  static bool formatIstTimeFromEpoch(uint32_t epochUtc, char* out, size_t outLen);
  static bool isPrintableText(const char* text);

 private:
  static constexpr uint32_t kIstOffsetSeconds = 19800UL;

  U8G2& display_;
  bool enabled_ = true;
};
