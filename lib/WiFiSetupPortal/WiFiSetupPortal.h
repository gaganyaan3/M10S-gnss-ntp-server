#pragma once

#include <Arduino.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFi.h>
#include <functional>

struct GpsDashboardSettings {
  bool rawLoggingEnabled = false;
  bool ntpHoldoverEnabled = true;
  bool enableStats = true;
  bool enableLocation = false;
  bool displayEnabled = true;
};

enum GpsClockState : uint8_t {
  GPS_CLOCK_STATE_UNSYNCED = 0,
  GPS_CLOCK_STATE_ACQUIRING = 1,
  GPS_CLOCK_STATE_LOCKED = 2,
  GPS_CLOCK_STATE_HOLDOVER = 3,
};

struct GpsDashboardStatus {
  static constexpr size_t MAX_NAV_SAT_DETAILS = 48;

  struct NavSatDetail {
    uint8_t constellation = 0;
    uint16_t prn = 0;
    int16_t elevationDeg = 0;
    int16_t azimuthDeg = 0;
    int16_t cn0DbHz = -1;
    bool navSatFlagsValid = false;
    uint8_t qualityInd = 0;
    bool svUsed = false;
  };

  bool satelliteCountValid = false;
  uint32_t satelliteCount = 0;
  bool ppsSeenSinceBoot = false;
  bool ppsAligned = false;
  bool ppsPeriodValid = false;
  double ppsPeriodSeconds = 0.0;
  GpsClockState clockState = GPS_CLOCK_STATE_UNSYNCED;
  bool holdoverTimeoutValid = false;
  uint32_t holdoverRemainingSeconds = 0;
  bool gpsTimeValid = false;
  uint64_t gpsEpochUtcMicros = 0;
  uint32_t coarseEpochUseCount = 0;
  int32_t correctionUs = 0;
  bool monRfValid = false;
  uint8_t jammingState = 0;
  uint8_t jamInd = 0;
  uint16_t noisePerMS = 0;
  uint16_t agcCnt = 0;
  uint8_t antStatus = 0;
  uint8_t antPower = 0;
  int8_t ofsI = 0;
  int8_t ofsQ = 0;
  uint8_t magI = 0;
  uint8_t magQ = 0;
  uint16_t agcCntAvg = 0;
  uint32_t monRfAgeMs = 0;
  bool monRfCfgValsetAck = false;
  bool monRfCfgValsetNak = false;
  bool monRfCfgAntAck = false;
  bool monRfCfgAntNak = false;
  bool timTpValid = false;
  uint32_t timTpTowMs = 0;
  uint32_t timTpTowSubMs = 0;
  int32_t timTpQErrPs = 0;
  uint16_t timTpWeek = 0;
  uint8_t timTpFlags = 0;
  uint8_t timTpRefInfo = 0;
  bool timTpUtcTimeBase = false;
  bool timTpUtcAvailable = false;
  bool timTpQErrInvalid = false;
  bool timTpTpNotLocked = false;
  bool timTpUtcEpochValid = false;
  uint64_t timTpEpochUtcNs = 0;
  int32_t timTpAppliedQErrNs = 0;
  bool ntpReady = false;
  bool holdoverUtcValid = false;
  uint64_t holdoverEpochUtcMicros = 0;
  bool latitudeValid = false;
  double latitude = 0.0;
  bool longitudeValid = false;
  double longitude = 0.0;
  bool altitudeValid = false;
  double altitudeMeters = 0.0;
  bool locationEnabled = false;
  bool speedValid = false;
  double speedMps = 0.0;
  bool hdopValid = false;
  double hdop = 0.0;
  bool pdopValid = false;
  double pdop = 0.0;
  bool vdopValid = false;
  double vdop = 0.0;
  bool navSatValid = false;
  uint8_t navPvtFixType = 0;
  bool navPvtGnssFixOK = false;
  bool navPvtValidDate = false;
  bool navPvtValidTime = false;
  bool navPvtFullyResolved = false;
  bool navPvtConfirmedDate = false;
  bool navPvtConfirmedTime = false;
  uint16_t gpsSatCount = 0;
  uint16_t galileoSatCount = 0;
  uint16_t beidouSatCount = 0;
  uint16_t glonassSatCount = 0;
  uint16_t qzssSatCount = 0;
  uint16_t sbasSatCount = 0;
  uint16_t otherSatCount = 0;
  size_t navSatDetailCount = 0;
  NavSatDetail navSatDetails[MAX_NAV_SAT_DETAILS];
  String gpsUartLabel;
  String ppsPinLabel;
  uint32_t gpsBaudRate = 9600;
  float cpuUtilizationPercent = 0.0f;
  float cpuUtilizationPeakPercent = 0.0f;
  float internalTemperatureC = 0.0f;
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
  void handleGpsSave();
  void handleNotFound();
  bool connectToWiFi(const String& ssid, const String& password, const char* source = nullptr);
  void stopAccessPoint();
  String htmlPage() const;
  String gpsDashboardPage(const GpsDashboardSettings& settings, const GpsDashboardStatus& status) const;
  String statsPage(const GpsDashboardStatus& status) const;

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
};
