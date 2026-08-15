#pragma once

#include <Arduino.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFi.h>
#include <functional>

struct GpsDashboardSettings {
  bool rawLoggingEnabled = false;
  bool ntpHoldoverEnabled = true;
  bool displayEnabled = true;
};

struct GpsDashboardStatus {
  bool satelliteCountValid = false;
  uint32_t satelliteCount = 0;
  bool seenSatelliteSinceBoot = false;
  bool gpsTimeValid = false;
  uint32_t gpsEpochUtc = 0;
  String gpsUartLabel;
  String ppsPinLabel;
  uint32_t gpsBaudRate = 9600;
  uint16_t nmeaPort = 0;
  uint16_t ntpPort = 0;
};

// Captive-portal Wi-Fi provisioning for ESP32.
// On first boot (no saved credentials) the device starts an open hotspot named
// via setApCredentials().  The user connects, opens any page, and is redirected
// to a simple form to enter their Wi-Fi SSID / password.  On success the
// credentials are persisted to EEPROM and the hotspot shuts down.
//
// If saved credentials exist the device tries to connect on begin().  Hold the
// optional reset pin (setResetPin) to wipe stored credentials and re-enter
// setup mode.
class WiFiSetupPortal {
 public:
  using GpsConfigProvider = std::function<void(GpsDashboardSettings&, GpsDashboardStatus&)>;
  using GpsConfigSaver = std::function<bool(const GpsDashboardSettings&)>;

  static constexpr size_t EEPROM_BYTES = 256;
  static constexpr size_t DISPLAY_PREF_ADDR = 126;
  static constexpr size_t GNSS_CONFIG_ADDR = 128;

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
  void handleGpsSave();
  void handleNotFound();
  bool connectToWiFi(const String& ssid, const String& password, const char* source = nullptr);
  void stopAccessPoint();
  String htmlPage() const;
  String gpsDashboardPage(const GpsDashboardSettings& settings, const GpsDashboardStatus& status) const;

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
