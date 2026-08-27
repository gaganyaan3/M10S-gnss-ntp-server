#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <U8g2lib.h>
#include <esp_system.h>
#include <time.h>

#include "BootButton.h"
#include "DashboardStore.h"
#include "GnssClock.h"
#include "GnssMaxM10S.h"
#include "NmeaServer.h"
#include "NtpServer.h"
#include "OledClockDisplay.h"
#include "PpsCapture.h"
#include "WiFiSetupPortal.h"

namespace {
constexpr uint32_t DISPLAY_INTERVAL_MS = 1000UL;
constexpr uint32_t RESET_OVERLAY_DELAY_MS = 500UL;
constexpr uint32_t WIFI_RESET_HOLD_MS = 3000UL;
constexpr uint32_t MAIN_LOOP_IDLE_DELAY_MS = 1UL;
constexpr uint64_t CPU_SAMPLE_INTERVAL_US = 100000ULL;
constexpr uint8_t CPU_REPORT_SAMPLE_COUNT = 10;

constexpr uint8_t I2C_SDA_PIN = 32;
constexpr uint8_t I2C_SCL_PIN = 33;
constexpr uint8_t GNSS_RX_PIN = 22;
constexpr uint8_t GNSS_TX_PIN = 21;
constexpr uint8_t PPS_PIN = 27;
constexpr uint8_t BOOT_BUTTON_PIN = 0;

// constexpr uint32_t PPS_MCPWM_TICKS_PER_SECOND = 80000000UL;  // don't delete this line
constexpr uint32_t PPS_MCPWM_TICKS_PER_SECOND = 79998000UL;  // used calibrated MCPWM - don't delete this line
constexpr uint32_t GNSS_BAUD_RATE = 115200UL;

WiFiSetupPortal wifiPortal;
GnssMaxM10S gps(GNSS_RX_PIN, GNSS_TX_PIN, GNSS_BAUD_RATE);
NmeaServer nmeaServer;
NtpServer ntpServer;
PpsCapture ppsCapture(PPS_MCPWM_TICKS_PER_SECOND);
GnssClock gnssClock(gps, ppsCapture);
DashboardStore dashboardStore;
BootButton bootButton(BOOT_BUTTON_PIN, LOW, 40, 400, WIFI_RESET_HOLD_MS);
U8G2_SH1106_128X64_NONAME_F_HW_I2C displayDriver(U8G2_R0, U8X8_PIN_NONE);
OledClockDisplay oledDisplay(displayDriver);

unsigned long lastDisplayUpdateMs = 0;
unsigned long lastGpsStatusLogMs = 0;
bool wasGpsValid = false;
char apSsid[24] = "ESP-Setup";
GpsDashboardSettings gpsDashboardSettings;
bool holdoverClockValid = false;
uint32_t holdoverBaseEpochUtc = 0;
unsigned long holdoverSetAtMs = 0;
float currentCpuUtilizationPercent = 0.0f;
float currentCpuUtilizationPeakPercent = 0.0f;
float currentInternalTemperatureC = 0.0f;
uint64_t cpuSampleWindowStartUs = 0;
uint64_t cpuSampleBusyUs = 0;
float cpuReportSampleSumPercent = 0.0f;
float cpuReportSamplePeakPercent = 0.0f;
uint8_t cpuReportSamples = 0;

void applyGpsDashboardSettings() {
  gps.enableRawLogging(gpsDashboardSettings.rawLoggingEnabled);
  gps.enableStats(gpsDashboardSettings.enableStats);
  gnssClock.setHoldoverEnabled(gpsDashboardSettings.ntpHoldoverEnabled);
}

bool saveGpsDashboardSettings(const GpsDashboardSettings& settings) {
  if (!dashboardStore.saveSettings(settings)) {
    return false;
  }

  gpsDashboardSettings = settings;
  oledDisplay.setEnabled(gpsDashboardSettings.displayEnabled);
  dashboardStore.saveDisplayEnabled(gpsDashboardSettings.displayEnabled);
  applyGpsDashboardSettings();
  return true;
}

void fillGpsDashboardData(GpsDashboardSettings& settingsOut, GpsDashboardStatus& statusOut) {
  settingsOut = gpsDashboardSettings;
  statusOut.locationEnabled = settingsOut.enableLocation;
  statusOut.satelliteCountValid = gps.hasSatelliteCount();
  if (statusOut.satelliteCountValid) {
    statusOut.satelliteCount = gps.satelliteCount();
  }

  statusOut.ppsSeenSinceBoot = ppsCapture.pulseCount() > 0;
  statusOut.ppsAligned = gps.ppsRecentlySeen();
  statusOut.ppsPeriodValid = statusOut.ppsAligned && ppsCapture.lastPeriodValid();
  statusOut.ppsPeriodSeconds = ppsCapture.lastPeriodSeconds();

  uint64_t epochUsUtc = 0;
  statusOut.gpsTimeValid = gnssClock.navPvtCoarseEpochUs(epochUsUtc);
  if (statusOut.gpsTimeValid) {
    statusOut.gpsEpochUtcMicros = epochUsUtc;
  }

  uint64_t ntpEpochUsUtc = 0;
  statusOut.ntpReady = gnssClock.currentNtpEpochUs(ntpEpochUsUtc);
  const GnssClockDiagnostics clockDiagnostics = gnssClock.diagnostics();
  statusOut.coarseEpochUseCount = clockDiagnostics.coarseEpochUseCount;
  statusOut.correctionUs = clockDiagnostics.coarseCorrectionLastAppliedUs;
  statusOut.holdoverUtcValid = clockDiagnostics.holdoverValid;
  if (statusOut.holdoverUtcValid) {
    statusOut.holdoverTimeoutValid = clockDiagnostics.holdoverTimeoutValid;
    statusOut.holdoverRemainingSeconds = clockDiagnostics.holdoverRemainingSeconds;
    statusOut.holdoverUtcValid = clockDiagnostics.holdoverUtcValid;
    statusOut.holdoverEpochUtcMicros = clockDiagnostics.holdoverEpochUtcMicros;
  }

  {
    GnssMonRfData monRf;
    gps.monRfSnapshot(monRf);
    statusOut.monRfValid = monRf.valid;
    statusOut.monRfAgeMs = monRf.receivedMs == 0 ? 0 : millis() - monRf.receivedMs;
    statusOut.jammingState = monRf.jammingState;
    statusOut.jamInd = monRf.jamInd;
    statusOut.noisePerMS = monRf.noisePerMS;
    statusOut.agcCnt = monRf.agcCnt;
    statusOut.antStatus = monRf.antStatus;
    statusOut.antPower = monRf.antPower;
    statusOut.ofsI = monRf.ofsI;
    statusOut.ofsQ = monRf.ofsQ;
    statusOut.magI = monRf.magI;
    statusOut.magQ = monRf.magQ;
    statusOut.agcCntAvg = monRf.agcCntAvg;
    statusOut.monRfCfgValsetAck = monRf.cfgValsetAck;
    statusOut.monRfCfgValsetNak = monRf.cfgValsetNak;
    statusOut.monRfCfgAntAck = monRf.cfgAntAck;
    statusOut.monRfCfgAntNak = monRf.cfgAntNak;
  }

  {
    GnssTimTpData timTp;
    gps.timTpSnapshot(timTp);
    statusOut.timTpValid = timTp.valid;
    statusOut.timTpTowMs = timTp.towMs;
    statusOut.timTpTowSubMs = timTp.towSubMs;
    statusOut.timTpQErrPs = timTp.qErrPs;
    statusOut.timTpWeek = timTp.week;
    statusOut.timTpFlags = timTp.flags;
    statusOut.timTpRefInfo = timTp.refInfo;
    statusOut.timTpUtcTimeBase = timTp.utcTimeBase;
    statusOut.timTpUtcAvailable = timTp.utcAvailable;
    statusOut.timTpQErrInvalid = timTp.qErrInvalid;
    statusOut.timTpTpNotLocked = timTp.tpNotLocked;
    statusOut.timTpUtcEpochValid = timTp.utcEpochValid;
    statusOut.timTpEpochUtcNs = timTp.epochUtcNs;
    statusOut.timTpAppliedQErrNs = timTp.appliedQErrNs;
  }

  statusOut.clockState = gnssClock.clockState(statusOut);
  statusOut.gpsUartLabel = F("GPIO22 <- GNSS TX, GPIO21 -> GNSS RX");
  statusOut.ppsPinLabel = F("GPIO27");
  statusOut.gpsBaudRate = GNSS_BAUD_RATE;
  statusOut.cpuUtilizationPercent = currentCpuUtilizationPercent;
  statusOut.cpuUtilizationPeakPercent = currentCpuUtilizationPeakPercent;
  statusOut.internalTemperatureC = currentInternalTemperatureC;

  {
    GnssNavPvtData pvt;
    gps.navPvtSnapshot(pvt);
    statusOut.navPvtFixType = pvt.fixType;
    statusOut.navPvtGnssFixOK = pvt.gnssFixOK;
    statusOut.navPvtValidDate = pvt.validDate;
    statusOut.navPvtValidTime = pvt.validTime;
    statusOut.navPvtFullyResolved = pvt.fullyResolved;
    statusOut.navPvtConfirmedDate = pvt.confirmedDate;
    statusOut.navPvtConfirmedTime = pvt.confirmedTime;
  }

  if (!settingsOut.enableStats) {
    return;
  }

  statusOut.latitudeValid = gps.hasLocation();
  if (statusOut.latitudeValid) {
    statusOut.latitude = gps.latitude();
  }

  statusOut.longitudeValid = gps.hasLocation();
  if (statusOut.longitudeValid) {
    statusOut.longitude = gps.longitude();
  }

  statusOut.altitudeValid = gps.hasAltitude();
  if (statusOut.altitudeValid) {
    statusOut.altitudeMeters = gps.altitudeMeters();
  }

  statusOut.speedValid = gps.hasSpeed();
  if (statusOut.speedValid) {
    statusOut.speedMps = gps.speedMps();
  }

  statusOut.hdopValid = gps.hasHdop();
  if (statusOut.hdopValid) {
    statusOut.hdop = gps.hdop();
  }

  statusOut.pdopValid = gps.hasPdop();
  if (statusOut.pdopValid) {
    statusOut.pdop = gps.pdop();
  }

  statusOut.vdopValid = gps.hasVdop();
  if (statusOut.vdopValid) {
    statusOut.vdop = gps.vdop();
  }

  GnssNavSatSnapshot navSat;
  gps.navSatSnapshot(navSat);
  statusOut.navSatValid = navSat.valid;
  statusOut.gpsSatCount = navSat.gpsCount;
  statusOut.galileoSatCount = navSat.galileoCount;
  statusOut.beidouSatCount = navSat.beidouCount;
  statusOut.glonassSatCount = navSat.glonassCount;
  statusOut.qzssSatCount = navSat.qzssCount;
  statusOut.sbasSatCount = navSat.sbasCount;
  statusOut.otherSatCount = navSat.otherCount;
  statusOut.navSatDetailCount =
      navSat.satelliteCount < GpsDashboardStatus::MAX_NAV_SAT_DETAILS
          ? navSat.satelliteCount
          : GpsDashboardStatus::MAX_NAV_SAT_DETAILS;
  for (size_t i = 0; i < statusOut.navSatDetailCount; ++i) {
    statusOut.navSatDetails[i].constellation = navSat.satellites[i].constellation;
    statusOut.navSatDetails[i].prn = navSat.satellites[i].prn;
    statusOut.navSatDetails[i].elevationDeg = navSat.satellites[i].elevationDeg;
    statusOut.navSatDetails[i].azimuthDeg = navSat.satellites[i].azimuthDeg;
    statusOut.navSatDetails[i].cn0DbHz = navSat.satellites[i].cn0DbHz;
    statusOut.navSatDetails[i].navSatFlagsValid = navSat.satellites[i].navSatFlagsValid;
    statusOut.navSatDetails[i].qualityInd = navSat.satellites[i].qualityInd;
    statusOut.navSatDetails[i].svUsed = navSat.satellites[i].svUsed;
  }

  if (navSat.valid) {
    statusOut.satelliteCountValid = true;
    statusOut.satelliteCount = static_cast<uint32_t>(navSat.satelliteCount);
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

float readEspTemperatureC() {
  const float reading = temperatureRead();
  if (reading < -40.0f || reading > 120.0f) {
    return 0.0f;
  }
  return reading;
}

void updateCpuUtilization(uint64_t loopBusyUs) {
  const uint64_t nowUs = micros();
  if (cpuSampleWindowStartUs == 0) {
    cpuSampleWindowStartUs = nowUs;
  }

  cpuSampleBusyUs += loopBusyUs;
  const uint64_t sampleElapsedUs = nowUs - cpuSampleWindowStartUs;
  if (sampleElapsedUs < CPU_SAMPLE_INTERVAL_US) {
    return;
  }

  float samplePercent =
      (static_cast<float>(cpuSampleBusyUs) / static_cast<float>(sampleElapsedUs)) * 100.0f;
  if (samplePercent > 100.0f) {
    samplePercent = 100.0f;
  }

  cpuReportSampleSumPercent += samplePercent;
  if (samplePercent > cpuReportSamplePeakPercent) {
    cpuReportSamplePeakPercent = samplePercent;
  }
  ++cpuReportSamples;

  cpuSampleBusyUs = 0;
  cpuSampleWindowStartUs = nowUs;

  if (cpuReportSamples >= CPU_REPORT_SAMPLE_COUNT) {
    currentCpuUtilizationPercent = cpuReportSampleSumPercent / cpuReportSamples;
    currentCpuUtilizationPeakPercent = cpuReportSamplePeakPercent;
    currentInternalTemperatureC = readEspTemperatureC();
    cpuReportSampleSumPercent = 0.0f;
    cpuReportSamplePeakPercent = 0.0f;
    cpuReportSamples = 0;
  }
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

void processBootButton(uint32_t nowMs) {
  uint32_t heldMs = 0;
  const BootButtonEvent event = bootButton.poll(nowMs, &heldMs);
  if (event != BOOT_BUTTON_SHORT_PRESS) {
    return;
  }

  gpsDashboardSettings.displayEnabled = !gpsDashboardSettings.displayEnabled;
  oledDisplay.setEnabled(gpsDashboardSettings.displayEnabled);
  dashboardStore.saveDisplayEnabled(gpsDashboardSettings.displayEnabled);
  Serial.printf("Display %s (BOOT short press: %lums)\n",
                oledDisplay.enabled() ? "ON" : "OFF",
                static_cast<unsigned long>(heldMs));
}

void processPpsCapture() {
  PpsPulse pulse;
  if (!ppsCapture.consumePulse(pulse)) {
    return;
  }

  gps.onPpsEdge(pulse.edgeTicks, pulse.elapsedPulses);
  Serial.printf("PPS pulse #%lu %lu ticks | PPS: %llu us",
                static_cast<unsigned long>(pulse.pulseCount),
                static_cast<unsigned long>(pulse.edgeTicks),
                static_cast<unsigned long long>(pulse.periodMicros));
  if (pulse.elapsedPulses > 1) {
    Serial.printf(" | skipped main-loop pulses: %lu",
                  static_cast<unsigned long>(pulse.elapsedPulses - 1));
  }
  Serial.printf(" | fq=%lu | fqdiff=%ld | CPU: %.1f C",
                static_cast<unsigned long>(pulse.diffTicks),
                static_cast<long>(pulse.frequencyDiffTicks),
                currentInternalTemperatureC);
  Serial.println();
}

void updateDisplay(uint32_t nowMs) {
  char gpsTimeLine[16] = "No GNSS";
  char gpsDateLine[16] = "fix";
  char statusLine[64] = "";

  uint32_t satCount = 0;
  const bool satValid = gps.hasSatelliteCount();
  if (satValid) {
    GnssNavSatSnapshot navSat;
    gps.navSatSnapshot(navSat);
    satCount = navSat.valid ? static_cast<uint32_t>(navSat.satelliteCount) : gps.satelliteCount();
  }

  const bool gpsHasFix = satValid && satCount > 0;
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

  if (gpsStamp.valid) {
    const bool gpsTimeValid =
        OledClockDisplay::formatIstTimeOnlyFromEpoch(gpsStamp.epochUtc, gpsTimeLine, sizeof(gpsTimeLine)) &&
        OledClockDisplay::isPrintableText(gpsTimeLine);
    const bool gpsDateValid =
        OledClockDisplay::formatIstDateOnlyFromEpoch(gpsStamp.epochUtc, gpsDateLine, sizeof(gpsDateLine)) &&
        OledClockDisplay::isPrintableText(gpsDateLine);
    if (!gpsTimeValid) {
      snprintf(gpsTimeLine, sizeof(gpsTimeLine), "No GNSS");
    }
    if (!gpsDateValid) {
      snprintf(gpsDateLine, sizeof(gpsDateLine), "fix");
    }
    if (!gpsTimeFresh) {
      strncat(gpsTimeLine, "*", sizeof(gpsTimeLine) - strlen(gpsTimeLine) - 1);
      if (!gpsHasFix) {
        snprintf(gpsDateLine, sizeof(gpsDateLine), "NoFix");
      }
    }
    if (!wasGpsValid) {
      Serial.println("GNSS fix acquired");
    }
    wasGpsValid = true;
  } else {
    const unsigned long nowLogMs = millis();
    if (nowLogMs - lastGpsStatusLogMs >= 1000UL) {
      Serial.printf("GNSS waiting... chars: %lu | sats: %lu\n",
                    static_cast<unsigned long>(gps.charsProcessed()),
                    static_cast<unsigned long>(satCount));
      lastGpsStatusLogMs = nowLogMs;
    }
    if (wasGpsValid) {
      Serial.println("GNSS fix lost");
    }
    wasGpsValid = false;
  }

  if (wifiPortal.isActive()) {
    snprintf(statusLine, sizeof(statusLine), "WiFiSetup: %s", apSsid);
  } else if (wifiPortal.isConnected()) {
    if (satValid) {
      snprintf(statusLine, sizeof(statusLine), "Sat:%lu wf:%ld", static_cast<unsigned long>(satCount), static_cast<long>(WiFi.RSSI()));
    } else {
      snprintf(statusLine, sizeof(statusLine), "Sat:-- wf:%ld", static_cast<long>(WiFi.RSSI()));
    }
  } else if (satValid) {
    snprintf(statusLine, sizeof(statusLine), "Sat:%lu wf:--", static_cast<unsigned long>(satCount));
  } else {
    snprintf(statusLine, sizeof(statusLine), "Sat:-- wf:--");
  }

  OledClockPage page;
  page.gpsTimeLine = gpsTimeLine;
  page.gpsDateLine = gpsDateLine;
  page.statusLine = statusLine;
  oledDisplay.drawClockPage(page);
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(100);

  const esp_reset_reason_t resetReason = esp_reset_reason();
  Serial.printf("Reset reason: %s\n", resetReasonToString(resetReason));

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  oledDisplay.begin();
  oledDisplay.showBoot();

  dashboardStore.begin();
  bool displayEnabled = true;
  dashboardStore.loadDisplayEnabled(displayEnabled);
  oledDisplay.setEnabled(displayEnabled);
  gpsDashboardSettings.displayEnabled = displayEnabled;
  dashboardStore.loadSettings(gpsDashboardSettings);
  gpsDashboardSettings.displayEnabled = displayEnabled;
  applyGpsDashboardSettings();

  bootButton.begin();
  ppsCapture.begin(PPS_PIN);
  gps.configurePpsHardwareClock([]() { return ppsCapture.nowTicks(); }, ppsCapture.ticksPerSecond());
  Serial.printf("PPS hardware capture enabled on GPIO%d (MCPWM CAP0, %lu Hz)\n",
                PPS_PIN, static_cast<unsigned long>(ppsCapture.ticksPerSecond()));

  uint8_t mac[6] = {0};
  WiFi.macAddress(mac);
  snprintf(apSsid, sizeof(apSsid), "ESP-%02X%02X%02X", mac[3], mac[4], mac[5]);

  wifiPortal.setApCredentials(apSsid);
  wifiPortal.setGpsDashboardCallbacks(fillGpsDashboardData, saveGpsDashboardSettings);
  wifiPortal.setResetPin(BOOT_BUTTON_PIN, WIFI_RESET_HOLD_MS, LOW);
  wifiPortal.begin();

  gps.begin();
  applyGpsDashboardSettings();
  nmeaServer.setLineTransformer([](const String& line) { return applyPpsTimeToNmeaLine(line); });
  gps.onRawLine([](const String& line) { nmeaServer.broadcastLine(line); });
  Serial.printf("GNSS serial started at %lu baud on GPIO22/GPIO21\n",
                static_cast<unsigned long>(GNSS_BAUD_RATE));

  nmeaServer.begin();
  Serial.printf("NMEA server listening on port %u\n", NmeaServer::DEFAULT_PORT);
  Serial.printf("Setup hotspot SSID (if needed): %s\n", apSsid);

  ntpServer.setTimeProvider([](uint32_t& epochUtcOut, uint32_t& fractionOut) {
    uint64_t epochUsUtcOut = 0;
    if (!gnssClock.currentNtpEpochUs(epochUsUtcOut)) {
      return false;
    }

    epochUtcOut = static_cast<uint32_t>(epochUsUtcOut / 1000000ULL);
    fractionOut = static_cast<uint32_t>((epochUsUtcOut % 1000000ULL) * 4294967296ULL / 1000000ULL);
    return true;
  });
  ntpServer.begin();
  Serial.printf("GNSS-backed NTP server listening on port %u\n", NtpServer::DEFAULT_PORT);
}

void loop() {
  const uint64_t loopStartUs = micros();
  const uint32_t nowMs = millis();
  processBootButton(nowMs);

  wifiPortal.loop();
  bool showingResetHoldOverlay = false;
  if (wifiPortal.isResetPending()) {
    const uint32_t heldMs = wifiPortal.resetHeldMs();
    if (heldMs >= RESET_OVERLAY_DELAY_MS) {
      showingResetHoldOverlay = true;
      oledDisplay.showResetHold(heldMs, WIFI_RESET_HOLD_MS / 1000UL);
      lastDisplayUpdateMs = nowMs;
    }
  }

  if (wifiPortal.consumeCredentialsChanged()) {
    Serial.printf("WiFi connected. IP: %s\n", wifiPortal.localIP().toString().c_str());
  }

  processPpsCapture();
  gps.poll();
  ppsCapture.nowTicks64();
  nmeaServer.poll();
  ntpServer.poll();

  if (nowMs - lastDisplayUpdateMs >= DISPLAY_INTERVAL_MS) {
    lastDisplayUpdateMs = nowMs;
    if (!showingResetHoldOverlay) {
      updateDisplay(nowMs);
    }
  }

  updateCpuUtilization(micros() - loopStartUs);
  delay(MAIN_LOOP_IDLE_DELAY_MS);
}
