#include <Arduino.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <esp_system.h>
#include <time.h>
#include <sys/time.h>
#include <algorithm>
#include "driver/mcpwm.h"
#include "soc/mcpwm_struct.h"

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
constexpr uint32_t NTP_HOLDOVER_TIMEOUT_MS = 1440UL * 60UL * 1000UL; //10UL=10min just for comment
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

// PPS is timestamped by the MCPWM hardware capture unit instead of a plain
// GPIO interrupt: the edge instant is latched by dedicated hardware into a
// free-running 80 MHz (APB_CLK) counter, independent of interrupt-service
// jitter. CAP1 is left unbound to any pin and is only ever read via a
// "software forced capture", which lets us sample the very same free-running
// counter on demand -- giving a "now" value in the identical tick domain as
// the PPS edge, so elapsed-time math is exact integer tick subtraction
// (12.5 ns resolution) rather than a cross-domain micros() comparison.
constexpr mcpwm_unit_t PPS_MCPWM_UNIT = MCPWM_UNIT_0;
constexpr mcpwm_io_signals_t PPS_MCPWM_CAP_IO = MCPWM_CAP_0;
constexpr mcpwm_capture_channel_id_t PPS_MCPWM_CAP_CHANNEL = MCPWM_SELECT_CAP0;
constexpr uint8_t PPS_MCPWM_NOW_CHANNEL = 1;
//constexpr uint32_t PPS_MCPWM_TICKS_PER_SECOND = 80000000UL;  // don't delete this line
constexpr uint32_t PPS_MCPWM_TICKS_PER_SECOND = 79998000UL; // used calibrated MCPWM - don't delete this line

const char* NTP_SERVER_1 = "time.cloudflare.com";
const char* NTP_SERVER_2 = "time.google.com";
const char* NTP_SERVER_3 = "time.aws.com";

WiFiSetupPortal wifiPortal;
constexpr uint32_t GNSS_BAUD_RATE = 115200UL;
GnssMaxM10S gps(GNSS_RX_PIN, GNSS_TX_PIN, GNSS_BAUD_RATE);
NmeaServer nmeaServer;
NtpServer ntpServer;
U8G2_SH1106_128X64_NONAME_F_HW_I2C display(U8G2_R0, U8X8_PIN_NONE);

struct StoredGpsDashboardConfig {
  uint32_t magic;
  uint8_t rawLoggingEnabled;
  uint8_t ntpHoldoverEnabled;
  uint8_t enableStatsEnabled;
  uint8_t locationEnabled;
  uint8_t reserved[9];
};

unsigned long lastDisplayUpdateMs = 0;
unsigned long lastGpsStatusLogMs = 0;
bool wasGpsValid = false;
bool ntpSynced = false;
char apSsid[24] = "ESP-Setup";
uint32_t coarseEpochUseCount = 0;
int64_t coarseCorrectionDeltaSumUs = 0;
uint32_t coarseCorrectionDeltaCount = 0;
unsigned long coarseCorrectionWindowStartMs = 0;
uint32_t coarseCorrectionLastSampleReceivedUs = 0;
bool coarseCorrectionLastSampleValid = false;
int32_t coarseCorrectionLastAppliedUs = 0;
int64_t coarseCorrectionLastDeltaUs = 0;
volatile uint32_t ppsPulseCount = 0;
volatile uint32_t ppsLastEdgeTicks = 0;
volatile uint32_t ppsPreviousEdgeTicks = 0;
uint32_t lastLoggedPpsPulseCount = 0;
double lastPpsPeriodSeconds = 0.0;
bool lastPpsPeriodValid = false;
bool displayEnabled = true;
bool flashButtonStablePressed = false;
bool flashButtonLastPhysicalPressed = false;
bool flashButtonLongPressConsumed = false;
unsigned long flashButtonLastEdgeMs = 0;
unsigned long flashButtonPressedAtMs = 0;
GpsDashboardSettings gpsDashboardSettings;
bool ntpHoldoverValid = false;
int64_t ntpHoldoverCorrectionOffsetUs = 0;
uint64_t ntpHoldoverBaseEpochUsUtc = 0;
uint64_t ntpHoldoverBaseTicks = 0;
bool ntpHoldoverPpsDisciplined = false;
uint32_t ntpHoldoverDeadlineMs = 0;
uint64_t ppsCaptureTicksHigh = 0;
uint32_t ppsCaptureLastNowTicks = 0;
bool ppsCaptureTicks64Initialized = false;
bool holdoverClockValid = false;
uint32_t holdoverBaseEpochUtc = 0;
unsigned long holdoverSetAtMs = 0;
float currentCpuUtilizationPercent = 0.0f;
float currentInternalTemperatureC = 0.0f;
uint64_t cpuUtilWindowStartUs = 0;
uint64_t cpuUtilBusyUs = 0;

IRAM_ATTR bool onPpsCaptureEvent(mcpwm_unit_t mcpwm,
                                 mcpwm_capture_channel_id_t cap_channel,
                                 const cap_event_data_t* edata,
                                 void* user_data) {
  ppsPreviousEdgeTicks = ppsLastEdgeTicks;
  ppsLastEdgeTicks = edata->cap_value;
  ++ppsPulseCount;
  return false;
}

// Forces the MCPWM capture timer (shared by all its capture channels) to
// latch its current value into the unused CAP1 channel, giving a live "now"
// read in the exact same 80 MHz tick domain as the PPS edge capture.
//
// Writing capn_sw=1 triggers the latch in hardware, but the peripheral needs
// 1–2 APB clock cycles before capn_value is updated. A compiler memory barrier
// plus one dummy register read ensures the write has propagated before we
// sample the result, preventing a stale latch from being returned.
IRAM_ATTR uint32_t ppsCaptureNowTicks() {
  MCPWM0.cap_chn_cfg[PPS_MCPWM_NOW_CHANNEL].capn_sw = 1;
  // Compiler barrier: prevent reordering of the write above with the read below.
  __asm__ __volatile__("" ::: "memory");
  // Dummy read of the config register to let the latch settle (1–2 APB cycles).
  (void)MCPWM0.cap_chn_cfg[PPS_MCPWM_NOW_CHANNEL].capn_sw;
  return MCPWM0.cap_chn[PPS_MCPWM_NOW_CHANNEL].capn_value;
}

uint64_t ppsCaptureNowTicks64() {
  const uint32_t nowTicks = ppsCaptureNowTicks();
  if (!ppsCaptureTicks64Initialized) {
    ppsCaptureLastNowTicks = nowTicks;
    ppsCaptureTicks64Initialized = true;
  } else if (nowTicks < ppsCaptureLastNowTicks) {
    ppsCaptureTicksHigh += (1ULL << 32);
  }
  ppsCaptureLastNowTicks = nowTicks;
  return ppsCaptureTicksHigh | static_cast<uint64_t>(nowTicks);
}

uint64_t ppsTicksToUs(uint64_t ticks) {
  return (ticks / PPS_MCPWM_TICKS_PER_SECOND) * 1000000ULL +
         ((ticks % PPS_MCPWM_TICKS_PER_SECOND) * 1000000ULL) / PPS_MCPWM_TICKS_PER_SECOND;
}

void anchorNtpHoldoverRawUs(uint64_t epochUsUtc) {
  ntpHoldoverValid = true;
  ntpHoldoverBaseEpochUsUtc = epochUsUtc;
  ntpHoldoverBaseTicks = ppsCaptureNowTicks64();
}

void refreshNtpHoldoverTimeout() {
  ntpHoldoverPpsDisciplined = true;
  ntpHoldoverDeadlineMs = millis() + NTP_HOLDOVER_TIMEOUT_MS;
}

bool ntpHoldoverRemainingSeconds(uint32_t& remainingSecondsOut) {
  if (!ntpHoldoverPpsDisciplined) {
    return false;
  }

  const int32_t remainingMs = static_cast<int32_t>(ntpHoldoverDeadlineMs - millis());
  if (remainingMs <= 0) {
    return false;
  }

  remainingSecondsOut = static_cast<uint32_t>((remainingMs - 1) / 1000L);
  return true;
}

void anchorNtpHoldoverUs(uint64_t epochUsUtc) {
  const int64_t correctedEpochUs =
      static_cast<int64_t>(epochUsUtc) + ntpHoldoverCorrectionOffsetUs;
  if (correctedEpochUs <= 0) {
    return;
  }
  anchorNtpHoldoverRawUs(static_cast<uint64_t>(correctedEpochUs));
}

bool currentNtpHoldoverUs(uint64_t& epochUsUtcOut) {
  if (!ntpHoldoverValid) {
    return false;
  }

  const uint64_t elapsedTicks = ppsCaptureNowTicks64() - ntpHoldoverBaseTicks;
  epochUsUtcOut = ntpHoldoverBaseEpochUsUtc + ppsTicksToUs(elapsedTicks);
  return true;
}

void resetCoarseCorrectionWindow() {
  coarseCorrectionDeltaSumUs = 0;
  coarseCorrectionDeltaCount = 0;
  coarseCorrectionWindowStartMs = 0;
}

bool deltaExceedsThreshold(int64_t deltaUs, int64_t thresholdUs) {
  return deltaUs > thresholdUs || deltaUs < -thresholdUs;
}

int32_t microsecondsToMillisecondsPart(int64_t valueUs) {
  return static_cast<int32_t>(valueUs / 1000LL);
}

int32_t microsecondsToSubMillisecondPart(int64_t valueUs) {
  return static_cast<int32_t>(valueUs % 1000LL);
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
    gpsDashboardSettings.enableStats = true;
    gpsDashboardSettings.enableLocation = false;
    return;
  }

  gpsDashboardSettings.rawLoggingEnabled = stored.rawLoggingEnabled != 0;
  gpsDashboardSettings.ntpHoldoverEnabled = stored.ntpHoldoverEnabled != 0;
  gpsDashboardSettings.enableStats = stored.enableStatsEnabled != 0;
  gpsDashboardSettings.enableLocation = stored.locationEnabled != 0;
}

void applyGpsDashboardSettings() {
  gps.enableRawLogging(gpsDashboardSettings.rawLoggingEnabled);
  gps.enableStats(gpsDashboardSettings.enableStats);
}

bool saveGpsDashboardSettings(const GpsDashboardSettings& settings) {
  StoredGpsDashboardConfig stored = {};
  stored.magic = GNSS_DASHBOARD_CONFIG_MAGIC;
  stored.rawLoggingEnabled = settings.rawLoggingEnabled ? 1 : 0;
  stored.ntpHoldoverEnabled = settings.ntpHoldoverEnabled ? 1 : 0;
  stored.enableStatsEnabled = settings.enableStats ? 1 : 0;
  stored.locationEnabled = settings.enableLocation ? 1 : 0;

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

bool gpsNavPvtCoarseEpochUs(uint64_t& epochUsUtcOut, uint32_t* receivedUsOut = nullptr) {
  // Only accept the timestamp for NTP / holdover purposes when the GNSS
  // module reports fullyResolved and gnssFixOK, meaning UTC is unambiguous
  // and backed by a valid GNSS fix.
  GnssNavPvtData pvt;
  gps.navPvtSnapshot(pvt);
  if (!pvt.valid || !pvt.fullyResolved || !pvt.gnssFixOK) {
    return false;
  }

  if (!gps.navPvtUtcEpochUs(epochUsUtcOut)) {
    return false;
  }

  if (receivedUsOut) {
    *receivedUsOut = pvt.receivedUs;
  }

  return true;
}

bool updateCoarseCorrection(uint64_t referenceEpochUsUtc,
                            bool applyToHoldover,
                            bool updateServedEpoch,
                            uint64_t& epochUsUtcInOut) {
  constexpr int64_t MAX_PPS_COARSE_DELTA_US = 5000; //50ms, calles updateCoarseCorrection()
  constexpr unsigned long COARSE_CORRECTION_WINDOW_MS = 60000UL;

  if (gps.ppsRecentlySeen()) {
    if (coarseCorrectionDeltaCount > 0) {
      resetCoarseCorrectionWindow();
    }
    return false;
  }

  uint64_t coarseEpochUsUtc = 0;
  uint32_t coarseReceivedUs = 0;
  if (!gpsNavPvtCoarseEpochUs(coarseEpochUsUtc, &coarseReceivedUs) ||
      (coarseCorrectionLastSampleValid &&
       coarseReceivedUs == coarseCorrectionLastSampleReceivedUs)) {
    return false;
  }

  coarseCorrectionLastSampleReceivedUs = coarseReceivedUs;
  coarseCorrectionLastSampleValid = true;

  const int64_t deltaUs = static_cast<int64_t>(coarseEpochUsUtc) -
                          static_cast<int64_t>(referenceEpochUsUtc);
  coarseCorrectionLastDeltaUs = deltaUs;
  const unsigned long nowMs = millis();

  if (coarseCorrectionDeltaCount == 0) {
    if (!deltaExceedsThreshold(deltaUs, MAX_PPS_COARSE_DELTA_US)) {
      return false;
    }
    coarseCorrectionWindowStartMs = nowMs;
    Serial.printf("COARSE averaging window start | startDeltaUs=%lld startDeltaMs=%ld startDeltaSubUs=%ld thresholdUs=%lld referenceEpochUs=%llu navPvtEpochUs=%llu receivedUs=%lu\n",
                  static_cast<long long>(coarseCorrectionLastDeltaUs),
                  static_cast<long>(microsecondsToMillisecondsPart(coarseCorrectionLastDeltaUs)),
                  static_cast<long>(microsecondsToSubMillisecondPart(coarseCorrectionLastDeltaUs)),
                  static_cast<long long>(MAX_PPS_COARSE_DELTA_US),
                  static_cast<unsigned long long>(referenceEpochUsUtc),
                  static_cast<unsigned long long>(coarseEpochUsUtc),
                  static_cast<unsigned long>(coarseReceivedUs));
  }

  coarseCorrectionDeltaSumUs += deltaUs;
  ++coarseCorrectionDeltaCount;
  const unsigned long windowAgeMs = nowMs - coarseCorrectionWindowStartMs;
  const int64_t runningAvgDeltaUs =
      coarseCorrectionDeltaSumUs / static_cast<int64_t>(coarseCorrectionDeltaCount);
  Serial.printf("COARSE averaging sample | count=%lu avgDeltaUs=%lld avgDeltaMs=%ld avgDeltaSubUs=%ld\n",
                static_cast<unsigned long>(coarseCorrectionDeltaCount),
                static_cast<long long>(runningAvgDeltaUs),
                static_cast<long>(microsecondsToMillisecondsPart(runningAvgDeltaUs)),
                static_cast<long>(microsecondsToSubMillisecondPart(runningAvgDeltaUs)));

  if (windowAgeMs < COARSE_CORRECTION_WINDOW_MS) {
    return false;
  }

  const int64_t avgDeltaUs =
      coarseCorrectionDeltaSumUs / static_cast<int64_t>(coarseCorrectionDeltaCount);
  const int32_t correctionUs = static_cast<int32_t>(avgDeltaUs / 10LL);
  coarseCorrectionLastAppliedUs = correctionUs;

  bool applied = false;
  uint64_t currentHoldoverUs = 0;
  int64_t correctedHoldoverUs = 0;
  if (applyToHoldover && correctionUs != 0) {
    if (currentNtpHoldoverUs(currentHoldoverUs)) {
      correctedHoldoverUs = static_cast<int64_t>(currentHoldoverUs) + correctionUs;
      if (correctedHoldoverUs > 0) {
        ntpHoldoverCorrectionOffsetUs += correctionUs;
        if (updateServedEpoch) {
          epochUsUtcInOut = static_cast<uint64_t>(correctedHoldoverUs);
        }
        ++coarseEpochUseCount;
        anchorNtpHoldoverRawUs(static_cast<uint64_t>(correctedHoldoverUs));
        applied = true;
      }
    }
  }
  Serial.printf("COARSE averaging complete | correctionUs=%ld\n",
                static_cast<long>(correctionUs));

  resetCoarseCorrectionWindow();
  return applied;
}

bool currentGpsNtpEpochUs(uint64_t& epochUsUtcOut) {
  // Normal operation: UBX-TIM-TP establishes the UTC nanosecond timestamp for
  // the next PPS edge, then PPS/hardware ticks provide the precise NTP
  // timestamp. Short PPS outages continue from the last TIM-TP/PPS anchor with
  // the local oscillator for a bounded interval.
  uint64_t epochNsUtc = 0;
  if (gps.currentUtcEpochNsPps(epochNsUtc) && gps.ppsRecentlySeen()) {
    epochUsUtcOut = epochNsUtc / 1000ULL;
    refreshNtpHoldoverTimeout();
    anchorNtpHoldoverUs(epochUsUtcOut);
    // Only let NAV-PVT coarse correction act during a PPS outage. While PPS is
    // fresh, PPS/hardware ticks remain authoritative.
    updateCoarseCorrection(epochUsUtcOut, true, false, epochUsUtcOut);
    return true;
  }

  if (gpsDashboardSettings.ntpHoldoverEnabled && ntpHoldoverValid) {
    uint32_t remainingSeconds = 0;
    if (!ntpHoldoverRemainingSeconds(remainingSeconds)) {
      ntpHoldoverValid = false;
      return false;
    }

    if (!currentNtpHoldoverUs(epochUsUtcOut)) {
      return false;
    }

    // During HOLDOVER, run only from the last PPS-disciplined timer frequency.
    // Do not let NAV-PVT coarse correction pull the served clock until this
    // timeout has expired and the state machine falls back to ACQUIRING/UNSYNCED.
    if (coarseCorrectionDeltaCount > 0) {
      resetCoarseCorrectionWindow();
    }
    return true;
  }

  return false;
}

bool currentGpsNtpEpochMs(uint64_t& epochMsUtcOut) {
  uint64_t epochUsUtc = 0;
  if (!currentGpsNtpEpochUs(epochUsUtc)) {
    return false;
  }

  epochMsUtcOut = epochUsUtc / 1000ULL;
  return true;
}

GpsClockState currentGpsClockState(const GpsDashboardStatus& status) {
  if (status.timTpValid &&
      status.timTpUtcEpochValid &&
      !status.timTpTpNotLocked &&
      status.ppsAligned) {
    return GPS_CLOCK_STATE_LOCKED;
  }

  if (status.holdoverUtcValid && !status.ppsAligned) {
    return GPS_CLOCK_STATE_HOLDOVER;
  }

  if (status.gpsTimeValid) {
    return GPS_CLOCK_STATE_ACQUIRING;
  }

  return GPS_CLOCK_STATE_UNSYNCED;
}

void processPpsCapture() {
  uint32_t pulseCountSnapshot = 0;
  uint32_t edgeTicksSnapshot = 0;
  uint32_t previousEdgeTicksSnapshot = 0;
  noInterrupts();
  pulseCountSnapshot = ppsPulseCount;
  edgeTicksSnapshot = ppsLastEdgeTicks;
  previousEdgeTicksSnapshot = ppsPreviousEdgeTicks;
  interrupts();

  if (pulseCountSnapshot == lastLoggedPpsPulseCount) {
    return;
  }

  const uint32_t elapsedPulses = pulseCountSnapshot - lastLoggedPpsPulseCount;
  lastLoggedPpsPulseCount = pulseCountSnapshot;
  gps.onPpsEdge(edgeTicksSnapshot, elapsedPulses);

  const uint32_t diffTicks = edgeTicksSnapshot - previousEdgeTicksSnapshot;
  const int32_t fqDiffTicks = static_cast<int32_t>(PPS_MCPWM_TICKS_PER_SECOND) -
                              static_cast<int32_t>(diffTicks);
  const double periodSeconds = static_cast<double>(diffTicks) / PPS_MCPWM_TICKS_PER_SECOND;
  const uint64_t periodMicros =
      (static_cast<uint64_t>(diffTicks) * 1000000ULL) / PPS_MCPWM_TICKS_PER_SECOND;
  if (pulseCountSnapshot > 1) {
    lastPpsPeriodSeconds = periodSeconds;
    lastPpsPeriodValid = true;
  }
  Serial.printf("PPS pulse #%lu %lu ticks | PPS: %llu us",
                static_cast<unsigned long>(pulseCountSnapshot),
                static_cast<unsigned long>(edgeTicksSnapshot),
                static_cast<unsigned long long>(periodMicros));
  if (elapsedPulses > 1) {
    Serial.printf(" | skipped main-loop pulses: %lu",
                  static_cast<unsigned long>(elapsedPulses - 1));
  }
  Serial.printf(" | fq=%lu | fqdiff=%ld | CPU: %.1f C",
                static_cast<unsigned long>(diffTicks),
                static_cast<long>(fqDiffTicks),
                currentInternalTemperatureC);
  Serial.println();
}

void fillGpsDashboardData(GpsDashboardSettings& settingsOut, GpsDashboardStatus& statusOut) {
  settingsOut = gpsDashboardSettings;
  statusOut.locationEnabled = settingsOut.enableLocation;
  statusOut.satelliteCountValid = gps.hasSatelliteCount();
  if (statusOut.satelliteCountValid) {
    statusOut.satelliteCount = gps.satelliteCount();
  }
  uint32_t ppsPulseCountSnapshot = 0;
  noInterrupts();
  ppsPulseCountSnapshot = ppsPulseCount;
  interrupts();
  statusOut.ppsSeenSinceBoot = ppsPulseCountSnapshot > 0;
  statusOut.ppsAligned = gps.ppsRecentlySeen();
  statusOut.ppsPeriodValid = statusOut.ppsAligned && lastPpsPeriodValid;
  statusOut.ppsPeriodSeconds = lastPpsPeriodSeconds;
  uint64_t epochUsUtc = 0;
  statusOut.gpsTimeValid = gpsNavPvtCoarseEpochUs(epochUsUtc);
  if (statusOut.gpsTimeValid) {
    statusOut.gpsEpochUtcMicros = epochUsUtc;
  }
  statusOut.coarseEpochUseCount = coarseEpochUseCount;
  uint64_t ntpEpochUsUtc = 0;
  statusOut.ntpReady = currentGpsNtpEpochUs(ntpEpochUsUtc);
  statusOut.correctionUs = coarseCorrectionLastAppliedUs;
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
  statusOut.holdoverUtcValid = ntpHoldoverValid;
  if (statusOut.holdoverUtcValid) {
    statusOut.holdoverTimeoutValid =
        ntpHoldoverRemainingSeconds(statusOut.holdoverRemainingSeconds);
    statusOut.holdoverUtcValid =
        statusOut.holdoverTimeoutValid &&
        currentNtpHoldoverUs(statusOut.holdoverEpochUtcMicros);
  }
  statusOut.clockState = currentGpsClockState(statusOut);
  statusOut.gpsUartLabel = F("GPIO22 <- GNSS TX, GPIO21 -> GNSS RX");
  statusOut.ppsPinLabel = F("GPIO27");
  statusOut.gpsBaudRate = GNSS_BAUD_RATE;
  statusOut.nmeaPort = NmeaServer::DEFAULT_PORT;
  statusOut.ntpPort = NtpServer::DEFAULT_PORT;
  statusOut.cpuUtilizationPercent = currentCpuUtilizationPercent;
  statusOut.internalTemperatureC = currentInternalTemperatureC;

  // NAV-PVT detail fields — parsed directly from UBX binary, always available.
  {
    GnssNavPvtData pvt;
    gps.navPvtSnapshot(pvt);
    statusOut.navPvtFixType        = pvt.fixType;
    statusOut.navPvtGnssFixOK      = pvt.gnssFixOK;
    statusOut.navPvtValidDate      = pvt.validDate;
    statusOut.navPvtValidTime      = pvt.validTime;
    statusOut.navPvtFullyResolved  = pvt.fullyResolved;
    statusOut.navPvtConfirmedDate  = pvt.confirmedDate;
    statusOut.navPvtConfirmedTime  = pvt.confirmedTime;
  }

  if (!settingsOut.enableStats) {
    // Skip stats-only fields so /stats-related work stays off when disabled.
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
  // Prefer the detailed navSat snapshot count (can exceed 12) when valid.
  if (navSat.valid) {
    statusOut.satelliteCountValid = true;
    statusOut.satelliteCount = static_cast<uint32_t>(navSat.satelliteCount);
  }
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

bool formatIstTimeOnlyFromEpoch(uint32_t epochUtc, char* out, size_t outLen) {
  if (outLen == 0) {
    return false;
  }
  out[0] = '\0';
  time_t istEpoch = static_cast<time_t>(epochUtc + IST_OFFSET_SECONDS);
  struct tm tmValue;
  if (gmtime_r(&istEpoch, &tmValue) == nullptr) {
    return false;
  }
  return strftime(out, outLen, "%H:%M:%S", &tmValue) > 0;
}

bool formatIstDateOnlyFromEpoch(uint32_t epochUtc, char* out, size_t outLen) {
  if (outLen == 0) {
    return false;
  }
  out[0] = '\0';
  time_t istEpoch = static_cast<time_t>(epochUtc + IST_OFFSET_SECONDS);
  struct tm tmValue;
  if (gmtime_r(&istEpoch, &tmValue) == nullptr) {
    return false;
  }
  return strftime(out, outLen, "%d/%m/%y", &tmValue) > 0;
}

bool formatIstTimeFromEpoch(uint32_t epochUtc, char* out, size_t outLen) {
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

bool isPrintableText(const char* text) {
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

float readEspTemperatureC() {
  const float reading = temperatureRead();
  if (reading < -40.0f || reading > 120.0f) {
    return 0.0f;
  }
  return reading;
}

void updateCpuUtilization(uint64_t loopBusyUs) {
  if (cpuUtilWindowStartUs == 0) {
    cpuUtilWindowStartUs = micros();
  }

  cpuUtilBusyUs += loopBusyUs;
  const uint64_t elapsedUs = micros() - cpuUtilWindowStartUs;
  if (elapsedUs >= 1000000ULL) {
    const float busyRatio = static_cast<float>(cpuUtilBusyUs) / static_cast<float>(elapsedUs);
    currentCpuUtilizationPercent = busyRatio * 100.0f;
    if (currentCpuUtilizationPercent > 100.0f) {
      currentCpuUtilizationPercent = 100.0f;
    }
    currentInternalTemperatureC = readEspTemperatureC();
    cpuUtilBusyUs = 0;
    cpuUtilWindowStartUs = micros();
  }
}

void drawClockPage(const char* gpsTimeLine, const char* gpsDateLine, const char* ntpTimeLine, const char* ntpDateLine, const char* statusLine, unsigned long satCount) {
  display.clearBuffer();
  // Status line on top
  display.setFont(u8g2_font_6x10_tf);
  display.drawStr(0, 14, statusLine);

  // NTP in the middle
  display.setFont(u8g2_font_6x13_tf);
  display.drawStr(0, 33, "NTP:");
  display.setFont(u8g2_font_6x13_tf);
  display.drawStr(28, 33, ntpTimeLine);
  // Display satellite count after NTP time on the same line
  display.setFont(u8g2_font_6x13_tf);
  char satText[16];
  snprintf(satText, sizeof(satText), "sats: %lu", satCount);
  int ntpWidth = display.getStrWidth(ntpTimeLine);
  int satWidth = display.getStrWidth(satText);
  int satX = 28 + ntpWidth + 4;
  if (satX + satWidth > 128) {
    satX = 128 - satWidth;
    if (satX < 0) satX = 0;
  }
  display.drawStr(satX, 33, satText);

  // GNSS at the bottom
  display.setFont(u8g2_font_6x13_tf);
  display.drawStr(0, 56, "GNSS:");
  display.setFont(u8g2_font_7x13_tf);
  display.drawStr(28, 56, gpsTimeLine);
  display.setFont(u8g2_font_5x7_tf);
  display.drawStr(86, 57, gpsDateLine);

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
  mcpwm_gpio_init(PPS_MCPWM_UNIT, PPS_MCPWM_CAP_IO, PPS_PIN);
  mcpwm_capture_config_t ppsCaptureConfig = {};
  ppsCaptureConfig.cap_edge = MCPWM_POS_EDGE;
  ppsCaptureConfig.cap_prescale = 1;
  ppsCaptureConfig.capture_cb = onPpsCaptureEvent;
  ppsCaptureConfig.user_data = nullptr;
  mcpwm_capture_enable_channel(PPS_MCPWM_UNIT, PPS_MCPWM_CAP_CHANNEL, &ppsCaptureConfig);
  // Enable CAP1 purely so it can be sampled on demand via software-forced
  // capture; it is not bound to any GPIO and never triggers on its own.
  MCPWM0.cap_chn_cfg[PPS_MCPWM_NOW_CHANNEL].capn_en = 1;
  MCPWM0.cap_chn_cfg[PPS_MCPWM_NOW_CHANNEL].capn_mode = 0;
  gps.configurePpsHardwareClock(ppsCaptureNowTicks, PPS_MCPWM_TICKS_PER_SECOND);
  Serial.printf("PPS hardware capture enabled on GPIO%d (MCPWM CAP0, %lu Hz)\n",
                PPS_PIN, static_cast<unsigned long>(PPS_MCPWM_TICKS_PER_SECOND));

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
  Serial.printf("GNSS serial started at %lu baud on GPIO22/GPIO21\n",
                static_cast<unsigned long>(GNSS_BAUD_RATE));

  nmeaServer.begin();
  Serial.printf("NMEA server listening on port %u\n", NmeaServer::DEFAULT_PORT);
  Serial.printf("Setup hotspot SSID (if needed): %s\n", apSsid);

  ntpServer.setTimeProvider([](uint32_t& epochUtcOut, uint32_t& fractionOut) {
    uint64_t epochUsUtcOut = 0;
    if (!currentGpsNtpEpochUs(epochUsUtcOut)) {
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

  // Consume PPS before serial polling. u-blox NAV-PVT commonly arrives after
  // the PPS edge for that UTC second; processing PPS first avoids anchoring an
  // already-fired edge to a just-parsed PVT timestamp for the same second.
  processPpsCapture();

  gps.poll();
  ppsCaptureNowTicks64();
  nmeaServer.poll();
  ntpServer.poll();

  if (nowMs - lastDisplayUpdateMs < DISPLAY_INTERVAL_MS) {
    return;
  }
  lastDisplayUpdateMs = nowMs;

  char gpsLine[32] = "Time unavailable";
  char ntpLine[32] = "Syncing...";
  char gpsTimeLine[16] = "No GNSS";
  char gpsDateLine[16] = "fix";
  char ntpTimeLine[16] = "Syncing";
  char ntpDateLine[16] = "";
  char statusLine[64] = "";
  char hdopText[8] = "--";

  uint32_t satCount = 0;
  float hdopValue = 0.0f;
  const bool satValid = gps.hasSatelliteCount();
  const bool hdopValid = gps.hasHdop();
  if (satValid) {
    // Prefer the detailed NAV-SAT/GSV snapshot (which can report >12 sats)
    // when available; fall back to TinyGPSPlus's satellite count otherwise.
    GnssNavSatSnapshot navSat;
    gps.navSatSnapshot(navSat);
    if (navSat.valid) {
      satCount = static_cast<uint32_t>(navSat.satelliteCount);
    } else {
      satCount = gps.satelliteCount();
    }
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

  if (gpsStamp.valid) {
    const bool gpsLineValid = formatIstTimeFromEpoch(gpsStamp.epochUtc, gpsLine, sizeof(gpsLine)) &&
                              isPrintableText(gpsLine);
    const bool gpsTimeValid = formatIstTimeOnlyFromEpoch(gpsStamp.epochUtc, gpsTimeLine, sizeof(gpsTimeLine)) &&
                              isPrintableText(gpsTimeLine);
    const bool gpsDateValid = formatIstDateOnlyFromEpoch(gpsStamp.epochUtc, gpsDateLine, sizeof(gpsDateLine)) &&
                              isPrintableText(gpsDateLine);
    if (!gpsLineValid) {
      snprintf(gpsLine, sizeof(gpsLine), "Time unavailable");
    }
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
    // Serial.printf(
    //     "GNSS time IST: %s | sats: %lu\n",
    //     gpsLine,
    //     static_cast<unsigned long>(satCount));
    wasGpsValid = true;
  } else {
    snprintf(gpsLine, sizeof(gpsLine), "No GNSS fix");
    snprintf(gpsTimeLine, sizeof(gpsTimeLine), "No GNSS");
    snprintf(gpsDateLine, sizeof(gpsDateLine), "fix");
    const unsigned long nowLogMs = millis();
    if (nowLogMs - lastGpsStatusLogMs >= 1000UL) {
      Serial.printf(
          "GNSS waiting... chars: %lu | sats: %lu\n",
          static_cast<unsigned long>(gps.charsProcessed()),
          static_cast<unsigned long>(satCount));
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
    if (!formatIstTimeFromEpoch(ntpEpochUtc, ntpLine, sizeof(ntpLine)) ||
        !isPrintableText(ntpLine)) {
      snprintf(ntpLine, sizeof(ntpLine), "Time unavailable");
    }
    if (!formatIstTimeOnlyFromEpoch(ntpEpochUtc, ntpTimeLine, sizeof(ntpTimeLine)) ||
        !isPrintableText(ntpTimeLine)) {
      snprintf(ntpTimeLine, sizeof(ntpTimeLine), "Syncing");
    }
    if (!formatIstDateOnlyFromEpoch(ntpEpochUtc, ntpDateLine, sizeof(ntpDateLine)) ||
        !isPrintableText(ntpDateLine)) {
      ntpDateLine[0] = '\0';
    }
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
      snprintf(statusLine, sizeof(statusLine), "Sat:%lu wf:%ld", static_cast<unsigned long>(satCount), static_cast<long>(WiFi.RSSI()));
    } else {
      snprintf(statusLine, sizeof(statusLine), "Sat:-- wf:%ld", static_cast<long>(WiFi.RSSI()));
    }
  } else {
    if (satValid) {
      snprintf(statusLine, sizeof(statusLine), "Sat:%lu wf:--", static_cast<unsigned long>(satCount));
    } else {
      snprintf(statusLine, sizeof(statusLine), "Sat:-- wf:--");
    }
  }

  if (displayEnabled && !showingResetHoldOverlay) {
    drawClockPage(gpsTimeLine, gpsDateLine, ntpTimeLine, ntpDateLine, statusLine, satCount);
  }

  updateCpuUtilization(micros() - loopStartUs);
}
