#pragma once

#include <Arduino.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFi.h>
#include <functional>

struct GpsDashboardSettings {
  bool rawLoggingEnabled = false;
  bool displayEnabled = true;
  uint32_t kHoldoverTimeoutMs = 1440UL * 60UL * 1000UL;
  uint32_t kConfirmedTimeHoldoverTimeoutMs = 60UL * 1000UL;
  uint32_t kMaxConfirmedTimeCorrectionDeltaMs = 5UL;
  uint32_t kMaxConfirmedTimeCorrectionIgnoreDeltaMs = 100UL;
};

enum GpsClockState : uint8_t {
  GPS_CLOCK_STATE_UNSYNCED = 0,
  GPS_CLOCK_STATE_CONFIRMED_TIME = 1,
  GPS_CLOCK_STATE_PPS_ANCHORED = 2,
  GPS_CLOCK_STATE_HOLDOVER = 3,
};

struct GpsDashboardStatus {
  bool satelliteCountValid = false;
  uint32_t satelliteCount = 0;
  bool ppsLastSeenAgeValid = false;
  uint32_t ppsLastSeenAgeSeconds = 0;
  bool ppsAligned = false;
  bool ppsPeriodValid = false;
  double ppsPeriodSeconds = 0.0;
  GpsClockState clockState = GPS_CLOCK_STATE_UNSYNCED;
  bool holdoverTimeoutValid = false;
  uint32_t holdoverRemainingSeconds = 0;
  bool navPvtConfirmedTimeReady = false;
  uint64_t navPvtConfirmedTimeEpochUtcMicros = 0;
  uint64_t gpsEpochUtcMicros = 0;
  bool stillUtcEpochValid = false;
  uint64_t stillUtcEpochMicros = 0;
  bool holdoverStillDriftValid = false;
  int64_t holdoverStillDriftMicros = 0;
  bool holdoverStillPpmValid = false;
  double holdoverStillPpm = 0.0;
  bool ppsCalculatedPpmValid = false;
  double ppsCalculatedPpm = 0.0;
  uint32_t correctionUseCount = 0;
  int32_t correctionUs = 0;
  bool monRfValid = false;
  uint8_t jamInd = 0;
  uint16_t noisePerMS = 0;
  uint16_t agcCnt = 0;
  uint16_t agcCntAvg = 0;
  uint32_t monRfAgeMs = 0;
  bool monRfCfgValsetAck = false;
  bool monRfCfgValsetNak = false;
  bool monRfCfgAntAck = false;
  bool monRfCfgAntNak = false;
  bool timTpValid = false;
  uint32_t timTpTowMs = 0;
  int32_t timTpQErrPs = 0;
  bool timTpQErrInvalid = false;
  bool timTpTpNotLocked = false;
  bool timTpUtcEpochValid = false;
  bool ntpReady = false;
  bool holdoverUtcValid = false;
  uint64_t holdoverEpochUtcMicros = 0;
  bool latitudeValid = false;
  double latitude = 0.0;
  bool longitudeValid = false;
  double longitude = 0.0;
  bool altitudeValid = false;
  double altitudeMeters = 0.0;
  bool speedValid = false;
  double speedMps = 0.0;
  bool hdopValid = false;
  double hdop = 0.0;
  bool pdopValid = false;
  double pdop = 0.0;
  bool vdopValid = false;
  double vdop = 0.0;
  uint8_t navPvtFixType = 0;
  bool navPvtGnssFixOK = false;
  bool navPvtValidDate = false;
  bool navPvtValidTime = false;
  bool navPvtFullyResolved = false;
  bool navPvtConfirmedDate = false;
  bool navPvtConfirmedTime = false;
  String gpsUartLabel;
  String ppsPinLabel;
  uint32_t gpsBaudRate = 9600;
  float cpuUtilizationPercent = 0.0f;
  float cpuUtilizationPeakPercent = 0.0f;
  float internalTemperatureC = 0.0f;
  uint32_t freeHeapBytes = 0;
  uint32_t totalHeapBytes = 0;
  uint64_t uptimeSeconds = 0;
};

// Captive-portal Wi-Fi provisioning for ESP32.
// On first boot (no saved credentials) the device starts an open hotspot named
// via setApCredentials().  The user connects, opens any page, and is redirected
// to a simple form to enter their Wi-Fi SSID / password.  On success the
// credentials are persisted to NVS Preferences and the hotspot shuts down.
//
// If saved credentials exist the device tries to connect on begin().  Hold the
// optional reset pin (setResetPin) to wipe stored credentials and re-enter
// setup mode.
class WiFiSetupPortal {
 public:
  using GpsConfigProvider = std::function<void(GpsDashboardSettings&, GpsDashboardStatus&)>;
  using GpsConfigSaver = std::function<bool(const GpsDashboardSettings&)>;
  using CMapProvider = std::function<String()>;
  using CMapSaver = std::function<bool(const String&)>;
  using CMapEnabledProvider = std::function<bool()>;
  using CMapThresholdProvider = std::function<uint32_t()>;
  using CMapSettingsSaver = std::function<bool(bool, uint32_t)>;

  WiFiSetupPortal();

  // Set the AP SSID (and optional password) shown during setup.
  // Call before begin().  Default: "ESP-Setup".
  void setApCredentials(const char* ssid, const char* password = nullptr);

  // Maximum time (ms) to wait when attempting a Wi-Fi connection.
  void setConnectTimeout(uint32_t timeoutMs);

  // Optional GPIO button: hold for holdMs to wipe credentials and restart AP.
  void setResetPin(uint8_t pin, uint32_t holdMs = 3000, uint8_t activeLevel = HIGH);

  // Call once in setup().
  void begin();

  // Call every iteration of loop().
  void loop();

  // Returns true (once) after credentials are saved and device connects.
  bool consumeCredentialsChanged();

  // True while the setup hotspot is running.
  bool isActive() const;

  // True when connected to the target Wi-Fi network.
  bool isConnected() const;

  // True while the reset pin is held but before the hold threshold fires.
  bool isResetPending() const;

  // Milliseconds the reset pin has been held so far (0 if not held).
  uint32_t resetHeldMs() const;

  // SSID of the network the device is connected to.
  const String& connectedSsid() const;

  // Local IP in station mode.
  IPAddress localIP() const;

  // Provide current GNSS settings/status and handle GNSS settings saves.
  void setGpsDashboardCallbacks(GpsConfigProvider provider, GpsConfigSaver saver);
  void setCalibratedMapCallbacks(CMapProvider provider, CMapProvider savedProvider,
                                 CMapProvider statusProvider, CMapSaver saver);
  void setDynamicCalibrationCallbacks(CMapEnabledProvider provider,
                                      CMapThresholdProvider thresholdProvider,
                                      CMapSettingsSaver saver);

 private:
  void startHttpServer();
  void startAccessPoint();
  void startDnsServer(const IPAddress& apIp);
  void stopDnsServer();
  bool loadSavedCredentials(String& ssid, String& password);
  void saveCredentials(const String& ssid, const String& password);
  void clearSavedCredentials();
  void checkResetPin();
  void resetToSetupMode();
  void handleRoot();
  void handleConnect();
  void handleConnectNow();
  void handleConnectSuccess();
  void handleConnectFailed();
  void handleGpsDashboard();
  void handleStats();
  void handleMap();
  void handleMapSave();
  void handleMapSettingsSave();
  void handleGpsSave();
  void handleNotFound();
  bool connectToWiFi(const String& ssid, const String& password, const char* source = nullptr);
  void stopAccessPoint();
  String htmlPage() const;
  String wifiNetworkOptionsHtml() const;
  String gpsDashboardPage(const GpsDashboardSettings& settings, const GpsDashboardStatus& status) const;
  String statsPage(const GpsDashboardStatus& status) const;
  String mapPage() const;
  String calibratedMapPage() const;

  WebServer server_;
  DNSServer dnsServer_;
  const char* apSsid_ = "ESP-Setup";
  const char* apPassword_ = nullptr;
  uint32_t connectTimeoutMs_ = 15000;
  String connectedSsid_;
  String connectedPassword_;
  uint8_t resetPin_ = 255;
  uint8_t resetPinActiveLevel_ = HIGH;
  uint32_t resetHoldMs_ = 5000;
  uint32_t resetStartedAt_ = 0;
  uint32_t lastResetLogMs_ = 0;
  uint32_t pendingShutdownAt_ = 0;
  uint32_t nextReconnectAtMs_ = 0;
  bool resetHandled_ = false;
  bool credentialsChanged_ = false;
  bool active_ = false;
  bool httpServerStarted_ = false;
  GpsConfigProvider gpsConfigProvider_;
  GpsConfigSaver gpsConfigSaver_;
  CMapProvider cMapProvider_;
  CMapProvider savedCMapProvider_;
  CMapProvider cMapStatusProvider_;
  CMapSaver cMapSaver_;
  CMapEnabledProvider cMapEnabledProvider_;
  CMapThresholdProvider cMapThresholdProvider_;
  CMapSettingsSaver cMapSettingsSaver_;
};
