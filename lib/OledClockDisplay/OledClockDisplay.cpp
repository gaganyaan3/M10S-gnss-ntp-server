#include "OledClockDisplay.h"

#include <time.h>

OledClockDisplay::OledClockDisplay(U8G2& display) : display_(display) {}

void OledClockDisplay::begin() {
  display_.begin();
}

void OledClockDisplay::setEnabled(bool enabled) {
  enabled_ = enabled;
  display_.setPowerSave(enabled ? 0 : 1);
}

bool OledClockDisplay::enabled() const {
  return enabled_;
}

void OledClockDisplay::showBoot() {
  if (!enabled_) {
    return;
  }
  display_.clearBuffer();
  display_.setFont(u8g2_font_5x7_tf);
  display_.drawStr(0, 16, "Booting...");
  display_.sendBuffer();
}

void OledClockDisplay::showResetHold(uint32_t heldMs, uint32_t holdSeconds) {
  if (!enabled_) {
    return;
  }

  char holdLine[32];
  snprintf(holdLine,
           sizeof(holdLine),
           "Hold: %lus / %lus",
           static_cast<unsigned long>(heldMs / 1000UL),
           static_cast<unsigned long>(holdSeconds));

  display_.clearBuffer();
  display_.setFont(u8g2_font_6x13_tf);
  display_.drawStr(0, 18, "WiFiSetup...");
  display_.setFont(u8g2_font_6x10_tf);
  display_.drawStr(0, 36, "Release to cancel");
  display_.drawStr(0, 56, holdLine);
  display_.sendBuffer();
}

void OledClockDisplay::drawClockPage(const OledClockPage& page) {
  if (!enabled_) {
    return;
  }

  display_.clearBuffer();
  display_.setFont(u8g2_font_6x10_tf);
  display_.drawStr(0, 14, page.statusLine);

  display_.setFont(u8g2_font_6x13_tf);
  display_.drawStr(0, 56, "GNSS:");
  display_.setFont(u8g2_font_7x13_tf);
  display_.drawStr(28, 56, page.gpsTimeLine);
  display_.setFont(u8g2_font_5x7_tf);
  display_.drawStr(86, 57, page.gpsDateLine);

  display_.sendBuffer();
}

bool OledClockDisplay::formatIstTimeOnlyFromEpoch(uint32_t epochUtc, char* out, size_t outLen) {
  if (outLen == 0) {
    return false;
  }
  out[0] = '\0';
  time_t istEpoch = static_cast<time_t>(epochUtc + kIstOffsetSeconds);
  struct tm tmValue;
  if (gmtime_r(&istEpoch, &tmValue) == nullptr) {
    return false;
  }
  return strftime(out, outLen, "%H:%M:%S", &tmValue) > 0;
}

bool OledClockDisplay::formatIstDateOnlyFromEpoch(uint32_t epochUtc, char* out, size_t outLen) {
  if (outLen == 0) {
    return false;
  }
  out[0] = '\0';
  time_t istEpoch = static_cast<time_t>(epochUtc + kIstOffsetSeconds);
  struct tm tmValue;
  if (gmtime_r(&istEpoch, &tmValue) == nullptr) {
    return false;
  }
  return strftime(out, outLen, "%d/%m/%y", &tmValue) > 0;
}

bool OledClockDisplay::formatIstTimeFromEpoch(uint32_t epochUtc, char* out, size_t outLen) {
  if (outLen == 0) {
    return false;
  }
  out[0] = '\0';
  char timeText[16] = {};
  char dateText[16] = {};
  if (!formatIstTimeOnlyFromEpoch(epochUtc, timeText, sizeof(timeText)) ||
      !formatIstDateOnlyFromEpoch(epochUtc, dateText, sizeof(dateText))) {
    return false;
  }
  return snprintf(out, outLen, "%s %s", timeText, dateText) > 0;
}

bool OledClockDisplay::isPrintableText(const char* text) {
  if (text == nullptr || text[0] == '\0') {
    return false;
  }
  for (size_t i = 0; text[i] != '\0'; ++i) {
    const uint8_t byteValue = static_cast<uint8_t>(text[i]);
    if (byteValue < 0x20 || byteValue > 0x7E) {
      return false;
    }
  }
  return true;
}
