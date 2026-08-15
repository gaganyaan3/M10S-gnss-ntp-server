#include <Arduino.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <esp_system.h>
#include <time.h>
#include <sys/time.h>

#include "GnssMaxM10S.h"
#include "NmeaServer.h"
#include "NtpServer.h"
#include "WiFiSetupPortal.h"

namespace {
constexpr uint32_t IST_OFFSET_SECONDS = 19800UL;
constexpr uint32_t DISPLAY_INTERVAL_MS = 1000UL;
constexpr uint32_t DISPLAY_TOGGLE_DEBOUNCE_MS = 40UL;
constexpr uint32_t DISPLAY_TOGGLE_MAX_HOLD_MS = 400UL;
constexpr uint32_t WIFI_RESET_HOLD_MS = 3000UL;
constexpr uint32_t RESET_OVERLAY_DELAY_MS = 500UL;
constexpr uint8_t DISPLAY_PREF_MAGIC = 0xA7;
constexpr uint32_t GNSS_DASHBOARD_CONFIG_MAGIC = 0x47505331UL;

// ESP32 board pin map:
// OLED SH1106 I2C: SDA=GPIO32, SCL=GPIO33
// GNSS UART: MAX-M10S TX -> ESP32 RX (GPIO22), MAX-M10S RX -> ESP32 TX (GPIO21)
// GNSS 1PPS: MAX-M10S PPS -> ESP32 GPIO27
// BOOT button (GPIO0, active LOW) is used for short/long press actions.
constexpr uint8_t I2C_SDA_PIN = 32;
constexpr uint8_t I2C_SCL_PIN = 33;
constexpr uint8_t GNSS_RX_PIN = 22;
constexpr uint8_t GNSS_TX_PIN = 21;
constexpr uint8_t PPS_PIN = 27;
constexpr uint8_t BOOT_BUTTON_PIN = 0;

const char* NTP_SERVER_1 = "time.cloudflare.com";
const char* NTP_SERVER_2 = "time.google.com";
const char* NTP_SERVER_3 = "time.aws.com";

WiFiSetupPortal wifiPortal;
GnssMaxM10S gps(GNSS_RX_PIN, GNSS_TX_PIN);
NmeaServer nmeaServer;
NtpServer ntpServer;
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

struct StoredGpsDashboardConfig {
  uint32_t magic;
  uint8_t rawLoggingEnabled;
  uint8_t ntpHoldoverEnabled;
  uint8_t reserved[10];
};

unsigned long lastDisplayUpdateMs = 0;
unsigned long lastGpsStatusLogMs = 0;
bool wasGpsValid = false;
bool ntpSynced = false;
char apSsid[24] = "ESP-Setup";
volatile uint32_t ppsPulseCount = 0;
volatile uint32_t ppsLastEdgeMicros = 0;
volatile uint32_t ppsPreviousEdgeMicros = 0;
uint32_t lastLoggedPpsPulseCount = 0;
bool displayEnabled = true;
bool flashButtonStablePressed = false;
bool flashButtonLastPhysicalPressed = false;
bool flashButtonLongPressConsumed = false;
unsigned long flashButtonLastEdgeMs = 0;
unsigned long flashButtonPressedAtMs = 0;
GpsDashboardSettings gpsDashboardSettings;
bool gpsSeenSatelliteSinceBoot = false;
bool ntpHoldoverValid = false;
uint64_t ntpHoldoverBaseEpochMsUtc = 0;
unsigned long ntpHoldoverSetAtMs = 0;
bool holdoverClockValid = false;
uint32_t holdoverBaseEpochUtc = 0;
unsigned long holdoverSetAtMs = 0;

IRAM_ATTR void onPpsInterrupt() {
  const uint32_t nowMicros = micros();
  ++ppsPulseCount;
  ppsPreviousEdgeMicros = ppsLastEdgeMicros;
  ppsLastEdgeMicros = nowMicros;
  gps.onPpsEdge(nowMicros);
}

void setDisplayEnabled(bool enable) {
  displayEnabled = enable;
  gpsDashboardSettings.displayEnabled = enable;
  display.setPowerSave(enable ? 0 : 1);
}

void saveDisplayPreference() {
  EEPROM.begin(WiFiSetupPortal::EEPROM_BYTES);
  EEPROM.write(WiFiSetupPortal::DISPLAY_PREF_ADDR, DISPLAY_PREF_MAGIC);
  EEPROM.write(WiFiSetupPortal::DISPLAY_PREF_ADDR + 1, displayEnabled ? 1 : 0);
  EEPROM.commit();
}

void loadDisplayPreference() {
  EEPROM.begin(WiFiSetupPortal::EEPROM_BYTES);
  const uint8_t magic = EEPROM.read(WiFiSetupPortal::DISPLAY_PREF_ADDR);
  if (magic == DISPLAY_PREF_MAGIC) {
    const bool savedEnabled = EEPROM.read(WiFiSetupPortal::DISPLAY_PREF_ADDR + 1) != 0;
    setDisplayEnabled(savedEnabled);
  } else {
    setDisplayEnabled(true);
  }
}

void loadGpsDashboardSettings() {
  EEPROM.begin(WiFiSetupPortal::EEPROM_BYTES);

  StoredGpsDashboardConfig stored = {};
  EEPROM.get(WiFiSetupPortal::GNSS_CONFIG_ADDR, stored);

  if (stored.magic != GNSS_DASHBOARD_CONFIG_MAGIC) {
    gpsDashboardSettings.rawLoggingEnabled = false;
    gpsDashboardSettings.ntpHoldoverEnabled = true;
    return;
  }

  gpsDashboardSettings.rawLoggingEnabled = stored.rawLoggingEnabled != 0;
  gpsDashboardSettings.ntpHoldoverEnabled = stored.ntpHoldoverEnabled != 0;
}

void applyGpsDashboardSettings() {
  gps.enableRawLogging(gpsDashboardSettings.rawLoggingEnabled);
}

bool saveGpsDashboardSettings(const GpsDashboardSettings& settings) {
  StoredGpsDashboardConfig stored = {};
  stored.magic = GNSS_DASHBOARD_CONFIG_MAGIC;
  stored.rawLoggingEnabled = settings.rawLoggingEnabled ? 1 : 0;
  stored.ntpHoldoverEnabled = settings.ntpHoldoverEnabled ? 1 : 0;

  EEPROM.begin(WiFiSetupPortal::EEPROM_BYTES);
  EEPROM.put(WiFiSetupPortal::GNSS_CONFIG_ADDR, stored);
  const bool committed = EEPROM.commit();
  if (committed) {
    gpsDashboardSettings = settings;
    setDisplayEnabled(gpsDashboardSettings.displayEnabled);
    saveDisplayPreference();
    applyGpsDashboardSettings();
  }
  return committed;
}

bool updateGpsHoldoverFromLiveFix(unsigned long nowMs, uint64_t& epochMsUtcOut) {
  if (!gps.hasSatelliteCount() || gps.satelliteCount() == 0) {
    return false;
  }

  bool valid = gps.currentUtcEpochMsPps(epochMsUtcOut);
  if (!valid) {
    valid = gps.currentUtcEpochMs(epochMsUtcOut);
  }
  if (!valid) {
    return false;
  }

  gpsSeenSatelliteSinceBoot = true;
  ntpHoldoverValid = true;
  ntpHoldoverBaseEpochMsUtc = epochMsUtcOut;
  ntpHoldoverSetAtMs = nowMs;
  return true;
}

bool currentGpsNtpEpochMs(uint64_t& epochMsUtcOut) {
  const unsigned long nowMs = millis();
  if (updateGpsHoldoverFromLiveFix(nowMs, epochMsUtcOut)) {
    return true;
  }

  if (!gpsDashboardSettings.ntpHoldoverEnabled || !gpsSeenSatelliteSinceBoot || !ntpHoldoverValid) {
    return false;
  }

  epochMsUtcOut = ntpHoldoverBaseEpochMsUtc + static_cast<uint64_t>(nowMs - ntpHoldoverSetAtMs);
  return true;
}

void fillGpsDashboardData(GpsDashboardSettings& settingsOut, GpsDashboardStatus& statusOut) {
  settingsOut = gpsDashboardSettings;
  statusOut.satelliteCountValid = gps.hasSatelliteCount();
  if (statusOut.satelliteCountValid) {
    statusOut.satelliteCount = gps.satelliteCount();
  }
  statusOut.seenSatelliteSinceBoot = gpsSeenSatelliteSinceBoot;
  uint64_t epochMsUtc = 0;
  statusOut.gpsTimeValid = currentGpsNtpEpochMs(epochMsUtc);
  if (statusOut.gpsTimeValid) {
    statusOut.gpsEpochUtc = static_cast<uint32_t>(epochMsUtc / 1000ULL);
  }
  statusOut.gpsUartLabel = F("GPIO22 <- GNSS TX, GPIO21 -> GNSS RX");
  statusOut.ppsPinLabel = F("GPIO27");
  statusOut.gpsBaudRate = 9600;
  statusOut.nmeaPort = NmeaServer::DEFAULT_PORT;
  statusOut.ntpPort = NtpServer::DEFAULT_PORT;
}

void handleDisplayToggleButton(unsigned long nowMs) {
  const bool pressedPhysical = (digitalRead(BOOT_BUTTON_PIN) == LOW);
  if (pressedPhysical != flashButtonLastPhysicalPressed) {
    flashButtonLastPhysicalPressed = pressedPhysical;
    flashButtonLastEdgeMs = nowMs;
  }

  if (nowMs - flashButtonLastEdgeMs < DISPLAY_TOGGLE_DEBOUNCE_MS) {
    return;
  }

  if (pressedPhysical == flashButtonStablePressed) {
    if (flashButtonStablePressed && !flashButtonLongPressConsumed &&
        (nowMs - flashButtonPressedAtMs) >= WIFI_RESET_HOLD_MS) {
      flashButtonLongPressConsumed = true;
    }
    return;
  }

  flashButtonStablePressed = pressedPhysical;
  if (flashButtonStablePressed) {
    flashButtonPressedAtMs = nowMs;
    flashButtonLongPressConsumed = false;
    return;
  }

  const unsigned long heldMs = nowMs - flashButtonPressedAtMs;
  if (!flashButtonLongPressConsumed && heldMs <= DISPLAY_TOGGLE_MAX_HOLD_MS) {
    setDisplayEnabled(!displayEnabled);
    saveDisplayPreference();
    Serial.printf("Display %s (BOOT short press: %lums)\n",
                  displayEnabled ? "ON" : "OFF",
                  static_cast<unsigned long>(heldMs));
  }
}

void formatIstTimeOnlyFromEpoch(uint32_t epochUtc, char* out, size_t outLen) {
  time_t istEpoch = static_cast<time_t>(epochUtc + IST_OFFSET_SECONDS);
  struct tm tmValue;
  gmtime_r(&istEpoch, &tmValue);
  strftime(out, outLen, "%H:%M:%S", &tmValue);
}

void formatIstDateOnlyFromEpoch(uint32_t epochUtc, char* out, size_t outLen) {
  time_t istEpoch = static_cast<time_t>(epochUtc + IST_OFFSET_SECONDS);
  struct tm tmValue;
  gmtime_r(&istEpoch, &tmValue);
  strftime(out, outLen, "%d/%m/%y", &tmValue);
}

void formatIstTimeFromEpoch(uint32_t epochUtc, char* out, size_t outLen) {
  char timeText[16];
  char dateText[16];
  formatIstTimeOnlyFromEpoch(epochUtc, timeText, sizeof(timeText));
  formatIstDateOnlyFromEpoch(epochUtc, dateText, sizeof(dateText));
  snprintf(out, outLen, "%s %s", timeText, dateText);
}

void formatHdopValue(float hdopValue, bool hdopValid, char* out, size_t outLen) {
  if (hdopValid) {
    snprintf(out, outLen, "%.1f", hdopValue);
  } else {
    snprintf(out, outLen, "--");
  }
}

const char* resetReasonToString(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_UNKNOWN: return "UNKNOWN";
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXTERNAL";
    case ESP_RST_SW: return "SOFTWARE";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INTERRUPT_WATCHDOG";
    case ESP_RST_TASK_WDT: return "TASK_WATCHDOG";
    case ESP_RST_WDT: return "WATCHDOG";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    default: return "OTHER";
  }
}

bool getNtpUtcEpoch(uint32_t& epochUtcOut) {
  time_t nowUtc = time(nullptr);
  if (nowUtc < 1700000000) {
    return false;
  }
  epochUtcOut = static_cast<uint32_t>(nowUtc);
  return true;
}

bool getNtpUtcEpochUs(uint64_t& epochUsOut) {
  timeval tv;
  gettimeofday(&tv, nullptr);
  if (tv.tv_sec < 1700000000) {
    return false;
  }
  epochUsOut = static_cast<uint64_t>(tv.tv_sec) * 1000000ULL + static_cast<uint64_t>(tv.tv_usec);
  return true;
}

// Renders the diff at the smallest unit (ms/us/ns) needed to show a non-zero
// magnitude, since GNSS/NTP can agree down to the millisecond level.
void formatDiffLabel(char* out, size_t outLen, bool diffValid, int64_t diffUs) {
  if (!diffValid) {
    snprintf(out, outLen, "df:--");
    return;
  }
  const int64_t absUs = diffUs < 0 ? -diffUs : diffUs;
  if (absUs >= 1000) {
    snprintf(out, outLen, "df:%ldms", static_cast<long>(diffUs / 1000));
  } else if (absUs > 0) {
    snprintf(out, outLen, "df:%ldus", static_cast<long>(diffUs));
  } else {
    snprintf(out, outLen, "df:%ldns", static_cast<long>(diffUs * 1000));
  }
}

void drawClockPage(const char* gpsTimeLine, const char* gpsDateLine, const char* ntpTimeLine, const char* ntpDateLine, const char* statusLine) {
  display.clearBuffer();

  display.setFont(u8g2_font_6x13_tf);
  display.drawStr(0, 14, "GNSS:");
  display.setFont(u8g2_font_6x13_tf);
  display.drawStr(28, 14, gpsTimeLine);
  display.setFont(u8g2_font_5x7_tf);
  display.drawStr(80, 15, gpsDateLine);

  display.setFont(u8g2_font_6x13_tf);
  display.drawStr(0, 33, "NTP:");
  display.setFont(u8g2_font_6x13_tf);
  display.drawStr(28, 33, ntpTimeLine);
  display.setFont(u8g2_font_5x7_tf);
  display.drawStr(80, 34, ntpDateLine);

  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 56, statusLine);

  display.sendBuffer();
}

void syncNtp() {
  configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2, NTP_SERVER_3);
}

String formatUtcTimeForNmea(uint32_t epochUtc) {
  time_t utcEpoch = static_cast<time_t>(epochUtc);
  struct tm tmValue;
  gmtime_r(&utcEpoch, &tmValue);
  char timeBuffer[16];
  snprintf(timeBuffer, sizeof(timeBuffer), "%02d%02d%02d", tmValue.tm_hour, tmValue.tm_min, tmValue.tm_sec);
  return String(timeBuffer);
}

String applyPpsTimeToNmeaLine(const String& line) {
  uint32_t correctedEpochUtc = 0;
  if (!gps.currentUtcEpochPps(correctedEpochUtc)) {
    return line;
  }

  const String correctedTime = formatUtcTimeForNmea(correctedEpochUtc);
  const String prefix = String(line.substring(0, 6));
  if (prefix == "$GPRMC" || prefix == "$GNRMC" || prefix == "$GPGGA" || prefix == "$GNGGA") {
    const size_t fieldStart = 6 + 1;
    const size_t fieldEnd = line.indexOf(',', fieldStart);
    if (fieldEnd != static_cast<size_t>(-1)) {
      return line.substring(0, fieldStart) + correctedTime + line.substring(fieldEnd);
    }
  }
  return line;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(100);

  const esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.printf("Reset reason: %s\n", resetReasonToString(resetReason));

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  display.begin();
  display.clearBuffer();
  display.setFont(u8g2_font_5x7_tf);
  display.drawStr(0, 16, "Booting...");
  display.sendBuffer();
  loadDisplayPreference();
  loadGpsDashboardSettings();

  pinMode(PPS_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(PPS_PIN), onPpsInterrupt, RISING);
  Serial.printf("PPS input enabled on GPIO%d\n", PPS_PIN);

  // Build AP hotspot name from the last 3 MAC bytes: ESP-AABBCC
  {
    uint8_t mac[6] = {0};
    WiFi.macAddress(mac);
    snprintf(apSsid, sizeof(apSsid), "ESP-%02X%02X%02X", mac[3], mac[4], mac[5]);
  }
  wifiPortal.setApCredentials(apSsid);
  wifiPortal.setGpsDashboardCallbacks(fillGpsDashboardData, saveGpsDashboardSettings);
  // BOOT button = GPIO0 on most ESP32 DevKit boards (active LOW, INPUT_PULLUP).
  // Hold for 3 s to wipe stored credentials and re-enter setup mode.
  wifiPortal.setResetPin(BOOT_BUTTON_PIN, 3000, LOW);
  wifiPortal.begin();
  if (wifiPortal.isConnected()) {
    syncNtp();
    ntpSynced = true;
  }

  gps.begin();
  applyGpsDashboardSettings();
  nmeaServer.setLineTransformer([](const String& line) { return applyPpsTimeToNmeaLine(line); });
  gps.onRawLine([](const String& line) { nmeaServer.broadcastLine(line); });
  Serial.println("GNSS serial started at 9600 baud on GPIO22/GPIO21");

  nmeaServer.begin();
  Serial.printf("NMEA server listening on port %u\n", NmeaServer::DEFAULT_PORT);
  Serial.printf("Setup hotspot SSID (if needed): %s\n", apSsid);

  ntpServer.setTimeProvider([](uint32_t& epochUtcOut, uint32_t& fractionOut) {
    uint64_t epochMsUtcOut = 0;
    if (!currentGpsNtpEpochMs(epochMsUtcOut)) {
      return false;
    }

    epochUtcOut = static_cast<uint32_t>(epochMsUtcOut / 1000ULL);
    fractionOut = static_cast<uint32_t>((epochMsUtcOut % 1000ULL) * 4294967296ULL / 1000ULL);
    return true;
  });
  ntpServer.begin();
  Serial.printf("GNSS-backed NTP server listening on port %u\n", NtpServer::DEFAULT_PORT);

}

void loop() {
  const unsigned long nowMs = millis();
  handleDisplayToggleButton(nowMs);

  wifiPortal.loop();
  bool showingResetHoldOverlay = false;

  // Reset pin feedback: update display immediately on every loop() tick while
  // the BOOT button is held so the user sees a live countdown.
  if (wifiPortal.isResetPending()) {
    const uint32_t heldMs = wifiPortal.resetHeldMs();
    if (heldMs >= RESET_OVERLAY_DELAY_MS) {
      showingResetHoldOverlay = true;
      const uint32_t heldSec = heldMs / 1000;
      char holdLine[32];
      snprintf(holdLine, sizeof(holdLine), "Hold: %lus / 3s", static_cast<unsigned long>(heldSec));
      if (displayEnabled) {
        display.clearBuffer();
        display.setFont(u8g2_font_6x13_tf);
        display.drawStr(0, 18, "WiFiSetup...");
        display.setFont(u8g2_font_6x10_tf);
        display.drawStr(0, 36, "Release to cancel");
        display.drawStr(0, 56, holdLine);
        display.sendBuffer();
      }
      lastDisplayUpdateMs = nowMs;  // suppress normal 1s update this tick
    }
  }

  // Sync NTP the first time (or after reconfiguration) we get a connection.
  if (wifiPortal.consumeCredentialsChanged() || (!ntpSynced && wifiPortal.isConnected())) {
    syncNtp();
    ntpSynced = true;
    Serial.printf("NTP synced. IP: %s\n", wifiPortal.localIP().toString().c_str());
  }
  if (!wifiPortal.isConnected()) {
    ntpSynced = false;
  }

  gps.poll();
  uint64_t gpsHoldoverEpochMs = 0;
  updateGpsHoldoverFromLiveFix(nowMs, gpsHoldoverEpochMs);
  nmeaServer.poll();
  ntpServer.poll();

  uint32_t pulseCountSnapshot = 0;
  uint32_t edgeMicrosSnapshot = 0;
  uint32_t previousEdgeMicrosSnapshot = 0;
  noInterrupts();
  pulseCountSnapshot = ppsPulseCount;
  edgeMicrosSnapshot = ppsLastEdgeMicros;
  previousEdgeMicrosSnapshot = ppsPreviousEdgeMicros;
  interrupts();
  if (pulseCountSnapshot != lastLoggedPpsPulseCount) {
    lastLoggedPpsPulseCount = pulseCountSnapshot;
    const uint32_t diffMicros = (edgeMicrosSnapshot > previousEdgeMicrosSnapshot)
                                    ? (edgeMicrosSnapshot - previousEdgeMicrosSnapshot)
                                    : 0;
    Serial.printf("PPS pulse #%lu at %lu us | diff: %lu\n",
                  static_cast<unsigned long>(pulseCountSnapshot),
                  static_cast<unsigned long>(edgeMicrosSnapshot),
                  static_cast<unsigned long>(diffMicros));
  }

  if (nowMs - lastDisplayUpdateMs < DISPLAY_INTERVAL_MS) {
    return;
  }
  lastDisplayUpdateMs = nowMs;

  char gpsLine[32];
  char ntpLine[32];
  char gpsTimeLine[16];
  char gpsDateLine[16];
  char ntpTimeLine[16];
  char ntpDateLine[16];
  char statusLine[64];
  char hdopText[8];
  char diffLabel[24];
  bool diffValid = false;
  int64_t diffUs = 0;

  uint32_t satCount = 0;
  float hdopValue = 0.0f;
  const bool satValid = gps.hasSatelliteCount();
  const bool hdopValid = gps.hasHdop();
  if (satValid) {
    satCount = gps.satelliteCount();
  }
  if (hdopValid) {
    hdopValue = gps.hdop();
  }
  formatHdopValue(hdopValue, hdopValid, hdopText, sizeof(hdopText));

  const bool gpsHasFix = satValid && (satCount > 0);

  ClockStamp gpsStamp = {false, 0};
  uint32_t correctedEpochUtc = 0;
  bool gpsTimeFresh = false;
  if (gpsHasFix) {
    if (gps.currentUtcEpochPps(correctedEpochUtc)) {
      gpsStamp = {true, correctedEpochUtc};
      gpsTimeFresh = true;
    } else if (gps.currentUtcEpoch(correctedEpochUtc)) {
      gpsStamp = {true, correctedEpochUtc};
      gpsTimeFresh = true;
    }
  } else {
    const ClockStamp storedStamp = gps.latestUtc();
    if (storedStamp.valid) {
      if (!holdoverClockValid || holdoverBaseEpochUtc != storedStamp.epochUtc) {
        holdoverClockValid = true;
        holdoverBaseEpochUtc = storedStamp.epochUtc;
        holdoverSetAtMs = nowMs;
      }
      const uint32_t elapsedSec = static_cast<uint32_t>((nowMs - holdoverSetAtMs) / 1000UL);
      gpsStamp = {true, holdoverBaseEpochUtc + elapsedSec};
    } else {
      holdoverClockValid = false;
    }
  }

  uint64_t gpsEpochMs = 0;
  bool gpsMsValid = false;
  if (gpsHasFix) {
    gpsMsValid = gps.currentUtcEpochMsPps(gpsEpochMs);
    if (!gpsMsValid) {
      gpsMsValid = gps.currentUtcEpochMs(gpsEpochMs);
    }
  }
  uint64_t ntpEpochUs = 0;
  const bool ntpUsValid = getNtpUtcEpochUs(ntpEpochUs);
  diffValid = gpsMsValid && ntpUsValid;
  diffUs = diffValid ? static_cast<int64_t>(ntpEpochUs) - static_cast<int64_t>(gpsEpochMs) * 1000 : 0;
  formatDiffLabel(diffLabel, sizeof(diffLabel), diffValid, diffUs);

  if (gpsStamp.valid) {
    formatIstTimeFromEpoch(gpsStamp.epochUtc, gpsLine, sizeof(gpsLine));
    formatIstTimeOnlyFromEpoch(gpsStamp.epochUtc, gpsTimeLine, sizeof(gpsTimeLine));
    formatIstDateOnlyFromEpoch(gpsStamp.epochUtc, gpsDateLine, sizeof(gpsDateLine));
    if (!gpsTimeFresh) {
      strncat(gpsTimeLine, "*", sizeof(gpsTimeLine) - strlen(gpsTimeLine) - 1);
      if (!gpsHasFix) {
        snprintf(gpsDateLine, sizeof(gpsDateLine), "NoFix");
      }
    }
    if (!wasGpsValid) {
      Serial.println("GNSS fix acquired");
    }
    Serial.printf(
        "GNSS time IST: %s | sats: %lu | hdop: %s | %s\n",
        gpsLine,
        static_cast<unsigned long>(satCount),
        hdopText,
        diffLabel);
    wasGpsValid = true;
  } else {
    snprintf(gpsLine, sizeof(gpsLine), "No GNSS fix");
    snprintf(gpsTimeLine, sizeof(gpsTimeLine), "No GNSS");
    snprintf(gpsDateLine, sizeof(gpsDateLine), "fix");
    const unsigned long nowLogMs = millis();
    if (nowLogMs - lastGpsStatusLogMs >= 1000UL) {
      Serial.printf(
          "GNSS waiting... chars: %lu | sats: %lu | hdop: %s | %s\n",
          static_cast<unsigned long>(gps.charsProcessed()),
          static_cast<unsigned long>(satCount),
          hdopText,
          diffLabel);
      lastGpsStatusLogMs = nowLogMs;
    }
    if (wasGpsValid) {
      Serial.println("GNSS fix lost");
    }
    wasGpsValid = false;
  }

  uint32_t ntpEpochUtc = 0;
  const bool ntpValid = getNtpUtcEpoch(ntpEpochUtc);
  if (ntpValid) {
    formatIstTimeFromEpoch(ntpEpochUtc, ntpLine, sizeof(ntpLine));
    formatIstTimeOnlyFromEpoch(ntpEpochUtc, ntpTimeLine, sizeof(ntpTimeLine));
    formatIstDateOnlyFromEpoch(ntpEpochUtc, ntpDateLine, sizeof(ntpDateLine));
  } else {
    snprintf(ntpLine, sizeof(ntpLine), "Syncing...");
    snprintf(ntpTimeLine, sizeof(ntpTimeLine), "Syncing");
    ntpDateLine[0] = '\0';
  }

  if (wifiPortal.isActive()) {
    // Portal hotspot running — prompt user on the status line.
    snprintf(statusLine, sizeof(statusLine), "WiFiSetup: %s", apSsid);
  } else if (wifiPortal.isConnected()) {
    if (satValid) {
      snprintf(statusLine, sizeof(statusLine), "Sat:%lu %s wf:%ld", static_cast<unsigned long>(satCount), diffLabel, static_cast<long>(WiFi.RSSI()));
    } else {
      snprintf(statusLine, sizeof(statusLine), "Sat:-- %s wf:%ld", diffLabel, static_cast<long>(WiFi.RSSI()));
    }
  } else {
    if (satValid) {
      snprintf(statusLine, sizeof(statusLine), "Sat:%lu %s wf:--", static_cast<unsigned long>(satCount), diffLabel);
    } else {
      snprintf(statusLine, sizeof(statusLine), "Sat:-- %s wf:--", diffLabel);
    }
  }

  if (displayEnabled && !showingResetHoldOverlay) {
    drawClockPage(gpsTimeLine, gpsDateLine, ntpTimeLine, ntpDateLine, statusLine);
  }
}
