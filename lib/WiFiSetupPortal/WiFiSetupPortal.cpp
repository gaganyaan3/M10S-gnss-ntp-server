#include "WiFiSetupPortal.h"

#include <Preferences.h>
#include <time.h>

namespace {

constexpr const char* WIFI_PREFS_NAMESPACE = "wifi_setup";
constexpr const char* WIFI_SSID_KEY = "ssid";
constexpr const char* WIFI_PASSWORD_KEY = "pass";
constexpr uint32_t WIFI_RETRY_INTERVAL_MS = 15000UL;

String htmlEscape(const String& input) {
  String out;
  out.reserve(input.length() + 16);
  for (size_t i = 0; i < input.length(); ++i) {
    char c = input[i];
    if (c == '&') out += "&amp;";
    else if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else if (c == '"') out += "&quot;";
    else if (c == '\'') out += "&#39;";
    else out += c;
  }
  return out;
}

String yesNoLabel(bool value) {
  return value ? F("Yes") : F("No");
}

String clockStateLabel(GpsClockState state) {
  switch (state) {
    case GPS_CLOCK_STATE_CONFIRMED_TIME:
      return F("ConfirmedTime");
    case GPS_CLOCK_STATE_PPS_LOCKED:
      return F("PPSLocked");
    case GPS_CLOCK_STATE_HOLDOVER:
      return F("HOLDOVER");
    case GPS_CLOCK_STATE_UNSYNCED:
    default:
      return F("UNSYNCED");
  }
}

String formatBytes(uint32_t bytes) {
  if (bytes >= 1048576UL) {
    return String(static_cast<float>(bytes) / 1048576.0f, 1) + F(" MB");
  }
  if (bytes >= 1024UL) {
    return String(static_cast<float>(bytes) / 1024.0f, 1) + F(" KB");
  }
  return String(bytes) + F(" B");
}

String formatUptime(uint64_t seconds) {
  const uint64_t days = seconds / 86400ULL;
  seconds %= 86400ULL;
  const uint8_t hours = seconds / 3600ULL;
  seconds %= 3600ULL;
  const uint8_t minutes = seconds / 60ULL;
  const uint8_t secs = seconds % 60ULL;

  char buffer[40];
  if (days > 0) {
    snprintf(buffer,
             sizeof(buffer),
             "%llud %02uh %02um %02us",
             static_cast<unsigned long long>(days),
             static_cast<unsigned>(hours),
             static_cast<unsigned>(minutes),
             static_cast<unsigned>(secs));
  } else {
    snprintf(buffer,
             sizeof(buffer),
             "%02uh %02um %02us",
             static_cast<unsigned>(hours),
             static_cast<unsigned>(minutes),
             static_cast<unsigned>(secs));
  }
  return String(buffer);
}

String formatElapsedCompact(uint32_t seconds) {
  const uint32_t days = seconds / 86400UL;
  seconds %= 86400UL;
  const uint8_t hours = seconds / 3600UL;
  seconds %= 3600UL;
  const uint8_t minutes = seconds / 60UL;
  const uint8_t secs = seconds % 60UL;

  char buffer[40];
  if (days > 0) {
    snprintf(buffer,
             sizeof(buffer),
             "%lud %02uh %02um %02us",
             static_cast<unsigned long>(days),
             static_cast<unsigned>(hours),
             static_cast<unsigned>(minutes),
             static_cast<unsigned>(secs));
  } else if (hours > 0) {
    snprintf(buffer,
             sizeof(buffer),
             "%uh %02um %02us",
             static_cast<unsigned>(hours),
             static_cast<unsigned>(minutes),
             static_cast<unsigned>(secs));
  } else if (minutes > 0) {
    snprintf(buffer,
             sizeof(buffer),
             "%um %02us",
             static_cast<unsigned>(minutes),
             static_cast<unsigned>(secs));
  } else {
    snprintf(buffer, sizeof(buffer), "%us", static_cast<unsigned>(secs));
  }
  return String(buffer);
}

String clockStateDetail(GpsClockState state) {
  switch (state) {
    case GPS_CLOCK_STATE_CONFIRMED_TIME:
      return F("NAV-PVT date/time is fully resolved and confirmed; serving quick NTP.");
    case GPS_CLOCK_STATE_PPS_LOCKED:
      return F("TIM-TP valid, PPS valid, timer disciplined.");
    case GPS_CLOCK_STATE_HOLDOVER:
      return F("PPS disappeared; running from last disciplined timer.");
    case GPS_CLOCK_STATE_UNSYNCED:
    default:
      return F("No trustworthy UTC.");
  }
}

String holdoverCountdownLabel(uint32_t remainingSeconds) {
  char buffer[8];
  snprintf(buffer,
           sizeof(buffer),
           "%lu:%02lu",
           static_cast<unsigned long>(remainingSeconds / 60UL),
           static_cast<unsigned long>(remainingSeconds % 60UL));
  return String(buffer);
}

String hexByteLabel(uint8_t value) {
  char buffer[6];
  snprintf(buffer, sizeof(buffer), "0x%02X", value);
  return String(buffer);
}

String monRfAgcStatusLabel(const GpsDashboardStatus& status) {
  if (!status.monRfValid) {
    return F("Unavailable");
  }
  const uint16_t agcCnt = status.agcCntAvg;
  if (agcCnt > 5000) {
    return String(F("Disconnected-")) + String(agcCnt);
  }
  if (agcCnt > 3000) {
    return String(F("VLow-")) + String(agcCnt);
  }
  if (agcCnt > 2000) {
    return String(F("Low-")) + String(agcCnt);
  }
  if (agcCnt > 1000) {
    return String(F("OK-")) + String(agcCnt);
  }
  return String(F("Good-")) + String(agcCnt);
}

const char* constellationLabel(uint8_t constellation) {
  switch (constellation) {
    case 1: return "GPS";
    case 2: return "Galileo";
    case 3: return "BeiDou";
    case 4: return "GLONASS";
    case 5: return "QZSS";
    case 6: return "SBAS";
    case 7: return "eGPS / Other";
    default: return "Unknown";
  }
}

const char* constellationDomId(uint8_t constellation) {
  switch (constellation) {
    case 1: return "gps";
    case 2: return "galileo";
    case 3: return "beidou";
    case 4: return "glonass";
    case 5: return "qzss";
    case 6: return "sbas";
    case 7: return "other";
    default: return "unknown";
  }
}

String formatIstFromEpoch(uint32_t epochUtc) {
  constexpr uint32_t IST_OFFSET_SECONDS = 19800UL;
  time_t istEpoch = static_cast<time_t>(epochUtc + IST_OFFSET_SECONDS);
  struct tm tmValue;
  gmtime_r(&istEpoch, &tmValue);

  char buffer[24];
  strftime(buffer, sizeof(buffer), "%H:%M:%S %d/%m/%y IST", &tmValue);
  return String(buffer);
}

String formatEpochMicros(uint64_t epochUtcMicros) {
  char buffer[32];
  snprintf(buffer,
           sizeof(buffer),
           "%llu,%06llu",
           static_cast<unsigned long long>(epochUtcMicros / 1000000ULL),
           static_cast<unsigned long long>(epochUtcMicros % 1000000ULL));
  return String(buffer);
}

}  // namespace

WiFiSetupPortal::WiFiSetupPortal() : server_(80) {}

void WiFiSetupPortal::setApCredentials(const char* ssid, const char* password) {
  apSsid_ = ssid;
  apPassword_ = password;
}

void WiFiSetupPortal::setConnectTimeout(uint32_t timeoutMs) {
  connectTimeoutMs_ = timeoutMs;
}

void WiFiSetupPortal::setResetPin(uint8_t pin, uint32_t holdMs, uint8_t activeLevel) {
  resetPin_ = pin;
  resetHoldMs_ = holdMs;
  resetPinActiveLevel_ = activeLevel;
  pinMode(resetPin_, activeLevel == HIGH ? INPUT : INPUT_PULLUP);
}

namespace {
void applyMaxWiFiTxPower() {
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
}

void configureWiFiStationMode() {
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
}
}

void WiFiSetupPortal::setGpsDashboardCallbacks(GpsConfigProvider provider, GpsConfigSaver saver) {
  gpsConfigProvider_ = provider;
  gpsConfigSaver_ = saver;
}

void WiFiSetupPortal::startHttpServer() {
  if (httpServerStarted_) {
    return;
  }

  server_.on("/", HTTP_GET, [this]() { handleRoot(); });
  server_.on("/connect", HTTP_POST, [this]() { handleConnect(); });
  server_.on("/connect-now", HTTP_POST, [this]() { handleConnectNow(); });
  server_.on("/connect-success", HTTP_GET, [this]() { handleConnectSuccess(); });
  server_.on("/connect-failed", HTTP_GET, [this]() { handleConnectFailed(); });
  server_.on("/gnss", HTTP_GET, [this]() { handleGpsDashboard(); });
  server_.on("/stats", HTTP_GET, [this]() { handleStats(); });
  server_.on("/gnss-save", HTTP_POST, [this]() { handleGpsSave(); });
  server_.onNotFound([this]() { handleNotFound(); });
  server_.begin();
  httpServerStarted_ = true;
}

void WiFiSetupPortal::begin() {
  active_ = false;
  WiFi.softAPdisconnect(true);
  configureWiFiStationMode();
  applyMaxWiFiTxPower();

  String savedSsid;
  String savedPassword;
  if (!loadSavedCredentials(savedSsid, savedPassword)) {
    Serial.println("No saved Wi-Fi credentials found; starting setup hotspot");
    startAccessPoint();
    startHttpServer();
    return;
  }

  connectedSsid_ = savedSsid;
  connectedPassword_ = savedPassword;

  if (!connectToWiFi(savedSsid, savedPassword, "saved configuration")) {
    nextReconnectAtMs_ = millis() + WIFI_RETRY_INTERVAL_MS;
    Serial.printf("Saved Wi-Fi connect failed. Retrying in %lu seconds.\n",
                  static_cast<unsigned long>(WIFI_RETRY_INTERVAL_MS / 1000));
  }

  startHttpServer();
}

void WiFiSetupPortal::startAccessPoint() {
  IPAddress apIp(192, 168, 11, 1);
  IPAddress gateway(192, 168, 11, 1);
  IPAddress subnet(255, 255, 255, 0);

  WiFi.mode(WIFI_AP_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(false);
  applyMaxWiFiTxPower();
  WiFi.softAPConfig(apIp, gateway, subnet);

  if (apPassword_ != nullptr && strlen(apPassword_) >= 8) {
    WiFi.softAP(apSsid_, apPassword_, 1, false, 1);
  } else {
    WiFi.softAP(apSsid_, nullptr, 1, false, 1);
  }

  startDnsServer(apIp);
  active_ = true;

  Serial.println("========================================");
  Serial.println("SETUP MODE: Wi-Fi Hotspot Active");
  Serial.println("========================================");
  Serial.printf("Hotspot SSID : %s\n", apSsid_);
  Serial.printf("Setup URL    : http://%s/\n", WiFi.softAPIP().toString().c_str());
  Serial.println("Connect to the hotspot and visit the URL to enter Wi-Fi credentials.");
  Serial.println("========================================");
}

void WiFiSetupPortal::startDnsServer(const IPAddress& apIp) {
  dnsServer_.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer_.start(53, "*", apIp);
  Serial.println("Captive portal DNS started (all domains -> setup page)");
}

void WiFiSetupPortal::stopDnsServer() {
  dnsServer_.stop();
}

bool WiFiSetupPortal::loadSavedCredentials(String& ssid, String& password) {
  Preferences prefs;
  if (prefs.begin(WIFI_PREFS_NAMESPACE, true)) {
    if (prefs.isKey(WIFI_SSID_KEY)) {
      ssid = prefs.getString(WIFI_SSID_KEY, "");
      password = prefs.getString(WIFI_PASSWORD_KEY, "");
      prefs.end();
      if (!ssid.isEmpty()) {
        return true;
      }
    } else {
      prefs.end();
    }
  }
  return false;
}

void WiFiSetupPortal::saveCredentials(const String& ssid, const String& password) {
  Preferences prefs;
  if (prefs.begin(WIFI_PREFS_NAMESPACE, false)) {
    prefs.putString(WIFI_SSID_KEY, ssid);
    prefs.putString(WIFI_PASSWORD_KEY, password);
    prefs.end();
  }
  credentialsChanged_ = true;
}

void WiFiSetupPortal::clearSavedCredentials() {
  Preferences prefs;
  if (prefs.begin(WIFI_PREFS_NAMESPACE, false)) {
    prefs.remove(WIFI_SSID_KEY);
    prefs.remove(WIFI_PASSWORD_KEY);
    prefs.end();
  }

  Serial.println("Saved Wi-Fi credentials cleared from NVS Preferences");
}

void WiFiSetupPortal::checkResetPin() {
  if (resetPin_ == 255) {
    return;
  }

  const bool pressed = (digitalRead(resetPin_) == resetPinActiveLevel_);
  if (!pressed) {
    if (resetStartedAt_ != 0 && !resetHandled_) {
      Serial.println("Reset pin released before timeout; Wi-Fi credentials kept.");
    }
    resetStartedAt_ = 0;
    lastResetLogMs_ = 0;
    resetHandled_ = false;
    return;
  }

  if (resetStartedAt_ == 0) {
    resetStartedAt_ = millis();
    lastResetLogMs_ = resetStartedAt_;
    Serial.printf("[Reset] Pin pressed — hold for %lus to clear Wi-Fi credentials.\n",
                  static_cast<unsigned long>(resetHoldMs_ / 1000));
    return;
  }

  const uint32_t heldMs = millis() - resetStartedAt_;

  // Log progress every second while held.
  if (millis() - lastResetLogMs_ >= 1000) {
    lastResetLogMs_ = millis();
    Serial.printf("[Reset] Holding... %lus / %lus\n",
                  static_cast<unsigned long>(heldMs / 1000),
                  static_cast<unsigned long>(resetHoldMs_ / 1000));
  }

  if (!resetHandled_ && heldMs >= resetHoldMs_) {
    resetHandled_ = true;
    resetToSetupMode();
  }
}

void WiFiSetupPortal::resetToSetupMode() {
  Serial.println("Reset pin held — clearing saved credentials and restarting setup hotspot");
  clearSavedCredentials();
  connectedSsid_ = "";
  connectedPassword_ = "";
  credentialsChanged_ = true;
  WiFi.disconnect(true);
  delay(100);
  startAccessPoint();
}

void WiFiSetupPortal::loop() {
  checkResetPin();
  if (active_) {
    dnsServer_.processNextRequest();
  }
  if (httpServerStarted_) {
    server_.handleClient();
  }

  if (active_) {
    // Deferred AP shutdown: keep hotspot alive so the success page remains
    // accessible on mobile for 30 s before we tear it down.
    if (pendingShutdownAt_ != 0 && millis() >= pendingShutdownAt_) {
      pendingShutdownAt_ = 0;
      stopAccessPoint();
    }
  }

  if (!active_) {
    if (isConnected()) {
      nextReconnectAtMs_ = 0;
      return;
    }

    if (connectedSsid_.isEmpty()) {
      return;
    }

    const uint32_t nowMs = millis();
    if (nextReconnectAtMs_ != 0 && nowMs < nextReconnectAtMs_) {
      return;
    }

    if (!connectToWiFi(connectedSsid_, connectedPassword_, "retry")) {
      nextReconnectAtMs_ = nowMs + WIFI_RETRY_INTERVAL_MS;
      Serial.printf("Wi-Fi still disconnected. Next retry in %lu seconds.\n",
                    static_cast<unsigned long>(WIFI_RETRY_INTERVAL_MS / 1000));
    } else {
      nextReconnectAtMs_ = 0;
      credentialsChanged_ = true;
    }
  }
}

bool WiFiSetupPortal::consumeCredentialsChanged() {
  const bool changed = credentialsChanged_;
  credentialsChanged_ = false;
  return changed;
}

bool WiFiSetupPortal::isActive() const {
  return active_;
}

bool WiFiSetupPortal::isConnected() const {
  return WiFi.status() == WL_CONNECTED;
}

bool WiFiSetupPortal::isResetPending() const {
  return (resetStartedAt_ != 0) && !resetHandled_;
}

uint32_t WiFiSetupPortal::resetHeldMs() const {
  if (resetStartedAt_ == 0) {
    return 0;
  }
  return millis() - resetStartedAt_;
}

const String& WiFiSetupPortal::connectedSsid() const {
  return connectedSsid_;
}

IPAddress WiFiSetupPortal::localIP() const {
  return WiFi.localIP();
}

bool WiFiSetupPortal::connectToWiFi(const String& ssid, const String& password, const char* source) {
  if (source != nullptr) {
    Serial.printf("Connecting to Wi-Fi (%s): %s\n", source, ssid.c_str());
  } else {
    Serial.printf("Connecting to Wi-Fi: %s\n", ssid.c_str());
  }

  WiFi.begin(ssid.c_str(), password.c_str());

  const unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED) {
    if (millis() - start > connectTimeoutMs_) {
      Serial.println("Wi-Fi connection timed out");
      return false;
    }
    delay(500);
  }

  Serial.printf("Wi-Fi connected. IP: %s\n", WiFi.localIP().toString().c_str());
  return true;
}

void WiFiSetupPortal::stopAccessPoint() {
  stopDnsServer();
  WiFi.softAPdisconnect(true);
  configureWiFiStationMode();
  applyMaxWiFiTxPower();
  active_ = false;
  Serial.println("Setup hotspot stopped");
}

// ---- HTTP handlers --------------------------------------------------------

void WiFiSetupPortal::handleRoot() {
  if (isConnected() && !active_) {
    handleGpsDashboard();
    return;
  }
  server_.send(200, "text/html", htmlPage());
}

void WiFiSetupPortal::handleConnect() {
  if (!server_.hasArg("ssid")) {
    server_.send(400, "text/plain", "Missing ssid");
    return;
  }

  String ssid = server_.arg("ssid");
  String password = server_.hasArg("pass") ? server_.arg("pass") : "";
  ssid.trim();

  if (ssid.length() == 0) {
    server_.send(400, "text/plain", "SSID cannot be empty");
    return;
  }

  const String safeSsid = htmlEscape(ssid);
  const String safePassword = htmlEscape(password);

  String page = F("<!DOCTYPE html><html><head>"
                  "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
                  "<meta charset=\"UTF-8\"><title>Connecting...</title></head><body>"
                  "<h2>Connecting to Wi-Fi&hellip;</h2>"
                  "<p>Please wait while the device tries to connect.</p>"
                  "<p><strong>SSID:</strong> ");
  page += safeSsid;
  page += F("</p><form id=\"cf\" action=\"/connect-now\" method=\"POST\">"
            "<input type=\"hidden\" name=\"ssid\" value=\"");
  page += safeSsid;
  page += F("\"><input type=\"hidden\" name=\"pass\" value=\"");
  page += safePassword;
  page += F("\"></form>"
            "<script>setTimeout(function(){document.getElementById('cf').submit();},150);</script>"
            "</body></html>");
  server_.send(200, "text/html", page);
}

void WiFiSetupPortal::handleConnectNow() {
  if (!server_.hasArg("ssid")) {
    server_.send(400, "text/plain", "Missing ssid");
    return;
  }

  String ssid = server_.arg("ssid");
  String password = server_.hasArg("pass") ? server_.arg("pass") : "";
  ssid.trim();

  if (ssid.length() == 0) {
    server_.send(400, "text/plain", "SSID cannot be empty");
    return;
  }

  connectToWiFi(ssid, password, "setup page");
  const wl_status_t status = WiFi.status();

  if (status == WL_CONNECTED) {
    connectedSsid_ = ssid;
    connectedPassword_ = password;
    saveCredentials(ssid, password);
    server_.sendHeader("Location", "/connect-success", true);
    server_.send(302, "text/plain", "");
    return;
  }

  // Encode failure reason at connection time so the redirect target doesn't
  // re-read live Wi-Fi status (which may have changed by the time it's served).
  const char* reason = "unknown";
  if (status == WL_CONNECT_FAILED) {
    reason = "auth";
  } else if (status == WL_NO_SSID_AVAIL) {
    reason = "no_ssid";
  } else if (status == WL_CONNECTION_LOST) {
    reason = "lost";
  }
  server_.sendHeader("Location", String("/connect-failed?r=") + reason, true);
  server_.send(302, "text/plain", "");
}

void WiFiSetupPortal::handleConnectSuccess() {
  const String ip        = WiFi.localIP().toString();
  const String safeSsid  = htmlEscape(connectedSsid_.length() ? connectedSsid_ : String("(unknown)"));
  const String safeIp    = htmlEscape(ip);
  const String ntpCmd    = "ntpdate -q " + ip;
  const String nmeaUrl   = "tcp://" + ip + ":10110";
  const String ntpAddr   = "udp " + ip + ":123";
  const String gpsUrl    = "http://" + ip + "/gnss";

  Serial.println("========================================");
  Serial.println("Wi-Fi connected — device ready");
  Serial.printf("Network    : %s\n", connectedSsid_.c_str());
  Serial.printf("IP Address : %s\n", ip.c_str());
  Serial.printf("GNSS time  : ntpdate -q %s\n", ip.c_str());
  Serial.printf("NMEA stream: tcp://%s:10110\n", ip.c_str());
  Serial.printf("NTP server : udp %s:123\n", ip.c_str());
  Serial.println("========================================");

  String page = F("<!DOCTYPE html><html><head>"
                  "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
                  "<meta charset=\"UTF-8\"><title>Connected</title>"
                  "<style>"
                  "body{font-family:sans-serif;max-width:500px;margin:32px auto;padding:0 16px}"
                  "h2{color:#198754}h3{margin:20px 0 4px}"
                  "code{display:block;background:#f4f4f4;border:1px solid #ddd;"
                  "padding:8px 10px;border-radius:4px;font-size:14px;"
                  "word-break:break-all;user-select:all}"
                  "#bar{height:6px;background:#198754;border-radius:3px;"
                  "transition:width 1s linear;width:100%}"
                  "#msg{font-size:13px;color:#666;margin-top:6px}"
                  "</style></head><body>"
                  "<h2>&#10003; Connected to Wi-Fi!</h2>"
                  "<p><strong>Network:</strong> ");
  page += safeSsid;
  page += F("</p><p><strong>IP Address:</strong> ");
  page += safeIp;
  page += F("</p>"
            "<h3>GNSS Time (quick check)</h3>"
            "<code>");
  page += htmlEscape(ntpCmd);
  page += F("</code>"
            "<h3>NMEA Stream &mdash; OpenCPN / GNSSd</h3>"
            "<code>");
  page += htmlEscape(nmeaUrl);
  page += F("</code>"
            "<h3>NTP Server</h3>"
            "<code>");
  page += htmlEscape(ntpAddr);
  page += F("</code>"
            "<h3>GNSS Config Dashboard</h3>"
            "<code>");
  page += htmlEscape(gpsUrl);
  page += F("</code>"
            "<p style=\"margin-top:24px\">Credentials saved. "
            "Hotspot shuts down in <strong><span id='cd'>30</span>s</strong>.</p>"
            "<div id='bar'></div>"
            "<p id='msg'>Keep this page open &mdash; tap any code block to copy.</p>"
            "<script>"
            "var s=30;"
            "document.getElementById('bar').style.width='100%';"
            "var t=setInterval(function(){"
            "s--;document.getElementById('cd').textContent=s;"
            "document.getElementById('bar').style.width=(s/30*100)+'%';"
            "if(s<=0){clearInterval(t);"
            "document.getElementById('msg').textContent='Hotspot closed. You may close this page.';"
            "document.getElementById('bar').style.width='0';}"
            "},1000);"
            "</script>"
            "</body></html>");
  server_.send(200, "text/html", page);

  // Keep the hotspot running for 30 s so the page stays visible on mobile.
  pendingShutdownAt_ = millis() + 30000;
}

void WiFiSetupPortal::handleConnectFailed() {
  String page = F("<!DOCTYPE html><html><head>"
                  "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
                  "<meta charset=\"UTF-8\"><title>Connection Failed</title></head><body>"
                  "<h2>Wi-Fi connection failed</h2>");

  const String reason = server_.arg("r");
  if (reason == "auth") {
    page += F("<p>Reason: authentication failed (wrong password).</p>");
  } else if (reason == "no_ssid") {
    page += F("<p>Reason: SSID not found.</p>");
  } else if (reason == "lost") {
    page += F("<p>Reason: connection lost.</p>");
  } else {
    page += F("<p>Reason: timeout or unknown error.</p>");
  }
  page += F("<p>Please check your credentials and try again.</p>"
            "<p><a href=\"/\">Back</a></p></body></html>");
  server_.send(200, "text/html", page);
}

void WiFiSetupPortal::handleGpsDashboard() {
  if (!gpsConfigProvider_) {
    server_.send(503, "text/plain", "GPS dashboard is not configured");
    return;
  }

  GpsDashboardSettings settings;
  GpsDashboardStatus status;
  gpsConfigProvider_(settings, status);
  server_.send(200, "text/html", gpsDashboardPage(settings, status));
}

void WiFiSetupPortal::handleStats() {
  if (!gpsConfigProvider_) {
    server_.send(503, "text/plain", "GPS dashboard is not configured");
    return;
  }

  GpsDashboardSettings settings;
  GpsDashboardStatus status;
  gpsConfigProvider_(settings, status);
  if (!settings.enableStats) {
    // Keep the endpoint effectively disabled when stats collection is off.
    server_.send(404, "text/plain", "Stats page disabled");
    return;
  }

  server_.send(200, "text/html", statsPage(status));
}

void WiFiSetupPortal::handleGpsSave() {
  if (!gpsConfigProvider_ || !gpsConfigSaver_) {
    server_.send(503, "text/plain", "GPS dashboard is not configured");
    return;
  }

  GpsDashboardSettings settings;
  GpsDashboardStatus status;
  gpsConfigProvider_(settings, status);

  settings.rawLoggingEnabled = server_.hasArg("raw_logging");
  settings.ntpHoldoverEnabled = server_.hasArg("ntp_holdover");
  settings.enableStats = server_.hasArg("enable_stats");
  settings.enableLocation = server_.hasArg("enable_location");
  settings.displayEnabled = server_.hasArg("display_enabled");

  if (!gpsConfigSaver_(settings)) {
    server_.send(500, "text/plain", "Failed to save GNSS settings");
    return;
  }

  server_.sendHeader("Location", "/gnss#saved", true);
  server_.send(302, "text/plain", "");
}

void WiFiSetupPortal::handleNotFound() {
  // Captive portal: redirect everything to the setup page so mobile devices
  // detect the portal and open it automatically.
  if (active_) {
    server_.sendHeader("Location", "http://192.168.11.1/", true);
    server_.send(302, "text/plain", "");
    return;
  }

  if (isConnected()) {
    server_.sendHeader("Location", "/gnss", true);
    server_.send(302, "text/plain", "");
    return;
  }

  server_.send(404, "text/plain", "Not found");
}

// ---- HTML -----------------------------------------------------------------

String WiFiSetupPortal::htmlPage() const {
  String page = F("<!DOCTYPE html><html><head>"
                  "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
                  "<meta charset=\"UTF-8\"><title>Wi-Fi Setup</title>"
                  "<style>"
                  "*{box-sizing:border-box;margin:0;padding:0}"
                  "body{"
                    "font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;"
                    "min-height:100vh;"
                    "background:linear-gradient(135deg,#1a1a2e 0%,#16213e 50%,#0f3460 100%);"
                    "display:flex;align-items:center;justify-content:center;padding:16px}"
                  ".card{"
                    "background:rgba(255,255,255,0.07);"
                    "backdrop-filter:blur(12px);"
                    "-webkit-backdrop-filter:blur(12px);"
                    "border:1px solid rgba(255,255,255,0.18);"
                    "border-radius:16px;"
                    "padding:32px 28px;"
                    "width:100%;max-width:380px;"
                    "box-shadow:0 8px 32px rgba(0,0,0,0.4)"
                  "}"
                  ".icon{font-size:40px;text-align:center;margin-bottom:8px}"
                  "h2{color:#fff;font-size:22px;font-weight:700;text-align:center;margin-bottom:6px}"
                  "p{color:rgba(255,255,255,0.6);font-size:14px;text-align:center;margin-bottom:24px}"
                  "label{"
                    "display:block;color:rgba(255,255,255,0.8);"
                    "font-size:13px;font-weight:600;letter-spacing:.5px;"
                    "text-transform:uppercase;margin-bottom:16px"
                  "}"
                  "input{"
                    "display:block;width:100%;margin-top:6px;"
                    "padding:11px 14px;font-size:15px;"
                    "background:rgba(255,255,255,0.1);"
                    "border:1px solid rgba(255,255,255,0.25);"
                    "border-radius:8px;color:#fff;outline:none;"
                    "transition:border-color .2s,background .2s"
                  "}"
                  "input:focus{"
                    "border-color:#4fc3f7;"
                    "background:rgba(255,255,255,0.15)"
                  "}"
                  "input::placeholder{color:rgba(255,255,255,0.35)}"
                  "button{"
                    "width:100%;margin-top:8px;padding:13px;"
                    "font-size:16px;font-weight:700;letter-spacing:.3px;"
                    "color:#fff;border:none;border-radius:8px;cursor:pointer;"
                    "background:linear-gradient(90deg,#0d6efd,#6610f2);"
                    "box-shadow:0 4px 14px rgba(13,110,253,0.45);"
                    "transition:opacity .2s,transform .1s"
                  "}"
                  "button:active{opacity:.85;transform:scale(.98)}"
                  ".ssid-name{"
                    "color:#fff;font-size:11px;font-weight:400;"
                    "text-align:right;margin-top:4px;opacity:.5"
                  "}"
                  "</style></head><body>"
                  "<div class=\"card\">"
                  "<div class=\"icon\">&#x1F4F6;</div>"
                  "<h2>Wi-Fi Setup</h2>"
                  "<p>Connect this device to your home network.</p>"
                  "<form action=\"/connect\" method=\"POST\">"
                  "<label>Network (SSID)"
                    "<input type=\"text\" name=\"ssid\" placeholder=\"e.g. MyHomeWiFi\""
                    " autocomplete=\"off\" required>"
                  "</label>"
                  "<label>Password"
                    "<input type=\"password\" name=\"pass\" placeholder=\"Wi-Fi password\""
                    " autocomplete=\"off\">"
                  "</label>"
                  "<button type=\"submit\">Connect &#x2192;</button>"
                  "</form>"
                  "</div>"
                  "</body></html>");
  return page;
}

String WiFiSetupPortal::gpsDashboardPage(const GpsDashboardSettings& settings,
                                         const GpsDashboardStatus& status) const {
  String gpsTime = F("Unavailable");
  String gpsUtcEpoch = F("Unavailable");
  String holdoverUtcEpoch = F("Unavailable");
  const bool gpsEpochAvailable =
      status.gpsEpochUtcMicros > 0 &&
      (status.clockState == GPS_CLOCK_STATE_PPS_LOCKED ||
       status.clockState == GPS_CLOCK_STATE_CONFIRMED_TIME);
  if (gpsEpochAvailable) {
    gpsTime = formatIstFromEpoch(static_cast<uint32_t>(status.gpsEpochUtcMicros / 1000000ULL));
    gpsUtcEpoch = formatEpochMicros(status.gpsEpochUtcMicros);
  }
  if (status.holdoverUtcValid) {
    holdoverUtcEpoch = formatEpochMicros(status.holdoverEpochUtcMicros);
  }

  String cpuText = String(status.cpuUtilizationPercent, 1) + F("% avg / ") +
                   String(status.cpuUtilizationPeakPercent, 1) + F("% peak");
  String temperatureText = F("--");
  if (status.internalTemperatureC > -40.0f && status.internalTemperatureC < 120.0f) {
    temperatureText = String(status.internalTemperatureC, 1) + F("°C");
  }
  const String freeHeapText = formatBytes(status.freeHeapBytes);
  const String uptimeText = formatUptime(status.uptimeSeconds);

  const String satelliteCount = status.satelliteCountValid ? String(status.satelliteCount) : String(F("--"));
  String page = F("<!DOCTYPE html><html><head>"
                  "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
                  "<meta charset=\"UTF-8\"><title>GNSS Config</title>"
                  "<style>"
                  "*{box-sizing:border-box;margin:0;padding:0}"
                  "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;min-height:100vh;"
                    "background:linear-gradient(135deg,#1a1a2e 0%,#16213e 50%,#0f3460 100%);color:#fff;padding:20px}"
                  ".wrap{max-width:760px;margin:0 auto;display:grid;gap:18px}"
                  ".card{background:rgba(255,255,255,0.08);backdrop-filter:blur(12px);-webkit-backdrop-filter:blur(12px);"
                    "border:1px solid rgba(255,255,255,0.18);border-radius:18px;padding:24px;box-shadow:0 10px 30px rgba(0,0,0,0.28)}"
                  ".header-row{display:flex;justify-content:space-between;align-items:flex-start;gap:12px}"
                  "h1{font-size:28px;margin-bottom:8px}"
                  "h2{font-size:18px;margin-bottom:14px}"
                  "p.lead{color:rgba(255,255,255,0.72)}"
                  ".metrics{font-size:11px;line-height:1.5;text-align:right;color:rgba(255,255,255,0.82);letter-spacing:.06em;"
                    "text-transform:uppercase;padding-top:4px}"
                  ".banner{margin-top:14px;background:rgba(25,135,84,0.18);border:1px solid rgba(25,135,84,0.55);"
                    "color:#bbf7d0;border-radius:12px;padding:12px 14px}"
                  ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}"
                  ".item{background:rgba(255,255,255,0.05);border-radius:12px;padding:14px}"
                  ".label{display:block;font-size:12px;letter-spacing:.08em;text-transform:uppercase;color:rgba(255,255,255,0.56);margin-bottom:6px}"
                  ".label .unit{text-transform:none}"
                  ".value{font-size:18px;font-weight:700;word-break:break-word}"
                  "form{display:grid;gap:14px}"
                  ".toggle{display:flex;gap:12px;align-items:flex-start;background:rgba(255,255,255,0.05);border-radius:12px;padding:14px}"
                  ".toggle input{margin-top:4px;transform:scale(1.2)}"
                  ".toggle strong{display:block;margin-bottom:4px}"
                  ".toggle span{display:block;color:rgba(255,255,255,0.68);font-size:14px;line-height:1.4}"
                  "button{width:100%;padding:13px 16px;font-size:16px;font-weight:700;border:none;border-radius:10px;color:#fff;cursor:pointer;"
                    "background:linear-gradient(90deg,#0d6efd,#20c997);box-shadow:0 4px 14px rgba(13,110,253,0.35)}"
                  "a.action{display:inline-block;margin-top:14px;padding:10px 14px;font-size:14px;font-weight:700;text-decoration:none;color:#fff;"
                    "border-radius:10px;background:linear-gradient(90deg,#0d6efd,#20c997);box-shadow:0 4px 14px rgba(13,110,253,0.35)}"
                  "</style></head><body><div class=\"wrap\">"
                  "<div class=\"card\"><div class=\"header-row\"><div><h1>GNSS Config Dashboard</h1><p class=\"lead\">Runtime GNSS status and saved service settings.</p></div>"
                  "<div class=\"metrics\"><div>CPU: ");
  page += htmlEscape(cpuText);
  page += F("</div><div>Temp: ");
  page += htmlEscape(temperatureText);
  page += F("</div><div>Free heap: ");
  page += htmlEscape(freeHeapText);
  page += F("</div><div>Uptime: ");
  page += htmlEscape(uptimeText);
  page += F("</div></div></div>");

  if (settings.enableStats) {
    page += F("<a class=\"action\" href=\"/stats\">Open Stats</a>");
  }

  page += F("</div><div class=\"card\"><h2>Runtime Status</h2><div class=\"grid\">"
            "<div class=\"item\"><span class=\"label\">PPS</span><span class=\"value\">");
  if (status.ppsAligned && status.ppsPeriodValid) {
    char ppsBuf[20];
    snprintf(ppsBuf, sizeof(ppsBuf), "%.9f", status.ppsPeriodSeconds);
    page += htmlEscape(String(ppsBuf));
  } else {
    page += F("No");
    if (status.ppsLastSeenAgeValid) {
      page += F(", LastSeen: ");
      page += htmlEscape(formatElapsedCompact(status.ppsLastSeenAgeSeconds));
    }
  }
  page += F("</span></div><div class=\"item\"><span class=\"label\">Satellites</span><span class=\"value\">");
  page += htmlEscape(satelliteCount);
  page += F("</span></div><div class=\"item\"><span class=\"label\">GNSS Time (IST)</span><span class=\"value\">");
  page += htmlEscape(gpsTime);
  page += F("</span></div><div class=\"item\"><span class=\"label\">GNSS UTC Epoch <span class=\"unit\">(µs)</span></span><span class=\"value\">");
  page += htmlEscape(gpsUtcEpoch);
  page += F("</span></div><div class=\"item\"><span class=\"label\">Holdover UTC <span class=\"unit\">(µs)</span></span><span class=\"value\">");
  page += htmlEscape(holdoverUtcEpoch);
  page += F("</span></div><div class=\"item\"><span class=\"label\">NTP Ready</span><span class=\"value\">");
  page += status.ntpReady ? F("Yes") : F("No");
  page += F("</span></div>"
            "<div class=\"item\"><span class=\"label\">COARSE Since Boot</span><span class=\"value\">");
  page += htmlEscape(String(status.coarseEpochUseCount));
  page += F("</span></div><div class=\"item\"><span class=\"label\">correction <span class=\"unit\">(µs)</span></span><span class=\"value\">");
  page += htmlEscape(String(status.correctionUs));
  page += F("</span></div><div class=\"item\"><span class=\"label\">ANTStatus-agcCnt</span><span class=\"value\">");
  page += htmlEscape(monRfAgcStatusLabel(status));
  page += F("</span></div>");
  page += F("</div></div>");

  page += F("<div class=\"card\"><h2>Saved Settings</h2><form action=\"/gnss-save\" method=\"POST\">"
            "<label class=\"toggle\"><input type=\"checkbox\" name=\"raw_logging\"");
  if (settings.rawLoggingEnabled) {
    page += F(" checked");
  }
  page += F("><span><strong>Raw NMEA logging</strong><span>Print every received GNSS sentence to Serial.</span></span></label>"
            "<label class=\"toggle\"><input type=\"checkbox\" name=\"ntp_holdover\"");
  if (settings.ntpHoldoverEnabled) {
    page += F(" checked");
  }
  page += F(">"
            "<span><strong>NTP holdover after PPSLocked or ConfirmedTime</strong><span>Keep answering from the last PPS-locked or NAV-PVT confirmed-time UTC during short outages.</span></span></label>"
            "<label class=\"toggle\"><input type=\"checkbox\" name=\"enable_stats\""
  );
  if (settings.enableStats) {
    page += F(" checked");
  }
  page += F("><span><strong>Stats enabled</strong><span>Collect GNSS statistics for the stats page.</span></span></label>"
            "<label class=\"toggle\"><input type=\"checkbox\" name=\"enable_location\""
  );
  if (settings.enableLocation) {
    page += F(" checked");
  }
  page += F("><span><strong>Location enabled</strong><span>Show latitude, longitude, and altitude on the stats page.</span></span></label>"
            "<label class=\"toggle\"><input type=\"checkbox\" name=\"display_enabled\""
  );
  if (settings.displayEnabled) {
    page += F(" checked");
  }
  page += F("><span><strong>Display enabled</strong><span>Turn the OLED display on or off.</span></span></label>"
            "<button type=\"submit\">Save</button>"
            "<div class=\"banner\" id=\"save-banner\" style=\"display:none\">GPS settings saved to NVS.</div>"
            "<script>(function(){"
            "var b=document.getElementById('save-banner');"
            "if(window.location.hash==='\\#saved'&&b){"
            "b.style.display='block';"
            "setTimeout(function(){b.remove();},3000);"
            "if(window.history&&window.history.replaceState){window.history.replaceState(null,'','/gnss');}"
            "else{window.location.hash='';}"
            "}"
            "})();</script>");
  page += F("</form></div></div></body></html>");
  return page;
}

String WiFiSetupPortal::statsPage(const GpsDashboardStatus& status) const {
  String latitudeText = F("--");
  if (!status.locationEnabled) {
    latitudeText = F("***");
  } else if (status.latitudeValid) {
    latitudeText = String(status.latitude, 6);
  }

  String longitudeText = F("--");
  if (!status.locationEnabled) {
    longitudeText = F("***");
  } else if (status.longitudeValid) {
    longitudeText = String(status.longitude, 6);
  }

  String altitudeText = F("--");
  if (!status.locationEnabled) {
    altitudeText = F("***");
  } else if (status.altitudeValid) {
    altitudeText = String(status.altitudeMeters, 1) + F(" m");
  }

  String speedText = F("--");
  if (status.speedValid) {
    speedText = String(status.speedMps, 2) + F(" m/s");
  }

  String hdopText = F("--");
  if (status.hdopValid) {
    hdopText = String(status.hdop, 1);
  }

  // fixType human-readable label (u-blox interface spec, table UBX-NAV-PVT)
  const char* fixTypeLabels[] = {
    "0 - No fix",
    "1 - Dead reckoning",
    "2 - 2D fix",
    "3 - 3D fix",
    "4 - GNSS + DR",
    "5 - Time only",
  };
  const char* fixTypeLabel = (status.navPvtFixType < 6)
      ? fixTypeLabels[status.navPvtFixType]
      : "Unknown";

  struct SatCountItem {
    uint8_t constellation;
    uint16_t count;
  };

  const SatCountItem counts[] = {
      {1, status.gpsSatCount},
      {2, status.galileoSatCount},
      {3, status.beidouSatCount},
      {4, status.glonassSatCount},
      {5, status.qzssSatCount},
      {6, status.sbasSatCount},
      {7, status.otherSatCount},
  };

  String page = F("<!DOCTYPE html><html><head>"
                  "<meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
                  "<meta charset=\"UTF-8\"><title>GNSS Stats</title>"
                  "<style>"
                  "*{box-sizing:border-box;margin:0;padding:0}"
                  "body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;min-height:100vh;"
                    "background:linear-gradient(135deg,#1a1a2e 0%,#16213e 50%,#0f3460 100%);color:#fff;padding:20px}"
                  ".wrap{max-width:760px;margin:0 auto;display:grid;gap:18px}"
                  ".card{background:rgba(255,255,255,0.08);backdrop-filter:blur(12px);-webkit-backdrop-filter:blur(12px);"
                    "border:1px solid rgba(255,255,255,0.18);border-radius:18px;padding:24px;box-shadow:0 10px 30px rgba(0,0,0,0.28)}"
                  "h1{font-size:28px;margin-bottom:8px}"
                  "h2{font-size:18px;margin-bottom:14px}"
                  "p.lead{color:rgba(255,255,255,0.72)}"
                  ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}"
                  ".item{background:rgba(255,255,255,0.05);border-radius:12px;padding:14px}"
                  ".label{display:block;font-size:12px;letter-spacing:.08em;text-transform:uppercase;color:rgba(255,255,255,0.56);margin-bottom:6px}"
                  ".value{font-size:18px;font-weight:700;word-break:break-word}"
                  ".state-value{font-size:28px;font-weight:800;letter-spacing:0}"
                  ".state-detail{margin-top:6px;color:rgba(255,255,255,0.72);font-size:14px;line-height:1.4}"
                  ".count-btn{width:100%;text-align:left;background:rgba(255,255,255,0.05);border:1px solid rgba(255,255,255,0.2);"
                    "border-radius:12px;padding:14px;color:#fff;font-size:16px;font-weight:700;cursor:pointer}"
                  ".count-btn:hover{background:rgba(255,255,255,0.1)}"
                  ".details-help{margin-top:10px;font-size:13px;color:rgba(255,255,255,0.68)}"
                  ".details-group{display:none;gap:10px;grid-template-columns:1fr;margin-top:12px}"
                  ".details-group.active{display:grid}"
                  ".sat-item{background:rgba(255,255,255,0.05);border-radius:12px;padding:14px;line-height:1.5}"
                  ".sat-title{font-size:17px;font-weight:700;margin-bottom:4px}"
                  "a.action{display:inline-block;margin-top:14px;padding:10px 14px;font-size:14px;font-weight:700;text-decoration:none;color:#fff;"
                    "border-radius:10px;background:linear-gradient(90deg,#0d6efd,#20c997);box-shadow:0 4px 14px rgba(13,110,253,0.35)}"
                  "</style></head><body><div class=\"wrap\">"
                  "<div class=\"card\"><h1>Stats</h1>"
                  "<p class=\"lead\">Live navigation snapshot rendered with the GNSS Config Dashboard theme.</p>"
                  "<a class=\"action\" href=\"/gnss\">Back to GNSS Config Dashboard</a>"
                  "</div><div class=\"card\"><h2>Clock state</h2>"
                  "<div class=\"grid\"><div class=\"item\"><span class=\"label\">Clock state</span><span class=\"state-value\">");
  page += htmlEscape(clockStateLabel(status.clockState));
  page += F("</span><div class=\"state-detail\">");
  page += htmlEscape(clockStateDetail(status.clockState));
  page += F("</div></div><div class=\"item\"><span class=\"label\">Holdover timeout</span><span class=\"state-value\" id=\"holdover-countdown\"");
  const bool showHoldoverTimeout =
      (status.clockState == GPS_CLOCK_STATE_HOLDOVER ||
       status.clockState == GPS_CLOCK_STATE_CONFIRMED_TIME) &&
      status.holdoverTimeoutValid;
  if (showHoldoverTimeout) {
    page += F(" data-remaining=\"");
    page += htmlEscape(String(status.holdoverRemainingSeconds));
    page += F("\"");
  }
  page += F(">");
  if (showHoldoverTimeout) {
    page += htmlEscape(holdoverCountdownLabel(status.holdoverRemainingSeconds));
  } else {
    page += F("--");
  }
  page += F("</span></div></div></div>"
            "<div class=\"card\"><h2>UBX-NAV-PVT</h2><div class=\"grid\">"
                  "<div class=\"item\"><span class=\"label\">Latitude</span><span class=\"value\">");
  page += htmlEscape(latitudeText);
  page += F("</span></div><div class=\"item\"><span class=\"label\">Longitude</span><span class=\"value\">");
  page += htmlEscape(longitudeText);
  page += F("</span></div><div class=\"item\"><span class=\"label\">Altitude</span><span class=\"value\">");
  page += htmlEscape(altitudeText);
  page += F("</span></div><div class=\"item\"><span class=\"label\">Speed</span><span class=\"value\">");
  page += htmlEscape(speedText);
  page += F("</span></div><div class=\"item\"><span class=\"label\">HDOP</span><span class=\"value\">");
  page += htmlEscape(hdopText);
  page += F("</span></div><div class=\"item\"><span class=\"label\">PDOP</span><span class=\"value\">");
  page += htmlEscape(status.pdopValid ? String(status.pdop, 1) : String("--"));
  page += F("</span></div><div class=\"item\"><span class=\"label\">VDOP</span><span class=\"value\">");
  page += htmlEscape(status.vdopValid ? String(status.vdop, 1) : String("--"));
  page += F("</span></div>"
            "<div class=\"item\"><span class=\"label\">fixType</span><span class=\"value\">");
  page += htmlEscape(String(fixTypeLabel));
  page += F("</span></div>"
            "<div class=\"item\"><span class=\"label\">gnssFixOK</span><span class=\"value\">");
  page += status.navPvtGnssFixOK ? F("Yes") : F("No");
  page += F("</span></div>"
            "<div class=\"item\"><span class=\"label\">validDate</span><span class=\"value\">");
  page += status.navPvtValidDate ? F("Yes") : F("No");
  page += F("</span></div>"
            "<div class=\"item\"><span class=\"label\">validTime</span><span class=\"value\">");
  page += status.navPvtValidTime ? F("Yes") : F("No");
  page += F("</span></div>"
            "<div class=\"item\"><span class=\"label\">fullyResolved</span><span class=\"value\">");
  page += status.navPvtFullyResolved ? F("Yes") : F("No");
  page += F("</span></div>"
            "<div class=\"item\"><span class=\"label\">confirmedDate</span><span class=\"value\">");
  page += status.navPvtConfirmedDate ? F("Yes") : F("No");
  page += F("</span></div>"
            "<div class=\"item\"><span class=\"label\">confirmedTime</span><span class=\"value\">");
  page += status.navPvtConfirmedTime ? F("Yes") : F("No");
  page += F("</span></div></div></div>");

  page += F("<div class=\"card\"><h2>UBX-MON-RF</h2><div class=\"grid\">"
            "<div class=\"item\"><span class=\"label\">ANTStatus-agcCnt</span><span class=\"value\">");
  page += htmlEscape(status.monRfValid ? monRfAgcStatusLabel(status) : String(F("--")));
  page += F("</span></div><div class=\"item\"><span class=\"label\">jamInd</span><span class=\"value\">");
  page += htmlEscape(status.monRfValid ? String(status.jamInd) : String(F("--")));
  page += F("</span></div><div class=\"item\"><span class=\"label\">noisePerMS</span><span class=\"value\">");
  page += htmlEscape(status.monRfValid ? String(status.noisePerMS) : String(F("--")));
  page += F("</span></div><div class=\"item\"><span class=\"label\">agcCnt</span><span class=\"value\">");
  page += htmlEscape(status.monRfValid ? String(status.agcCnt) : String(F("--")));
  page += F("</span></div></div></div>");

  page += F("<div class=\"card\"><h2>UBX-TIM-TP</h2><div class=\"grid\">"
            "<div class=\"item\"><span class=\"label\">towMS</span><span class=\"value\">");
  page += htmlEscape(status.timTpValid ? String(status.timTpTowMs) : String(F("--")));
  page += F(" ms</span></div><div class=\"item\"><span class=\"label\">qErr</span><span class=\"value\">");
  if (status.timTpValid && !status.timTpQErrInvalid) {
    page += htmlEscape(String(status.timTpQErrPs));
    page += F(" ps");
  } else {
    page += F("--");
  }
  page += F("</span></div><div class=\"item\"><span class=\"label\">flags</span><span class=\"value\">");
  page += htmlEscape(status.timTpValid ? hexByteLabel(status.timTpFlags) : String(F("--")));
  page += F("</span></div><div class=\"item\"><span class=\"label\">refInfo</span><span class=\"value\">");
  page += htmlEscape(status.timTpValid ? hexByteLabel(status.timTpRefInfo) : String(F("--")));
  page += F("</span></div><div class=\"item\"><span class=\"label\">qErr invalid</span><span class=\"value\">");
  page += htmlEscape(status.timTpValid ? yesNoLabel(status.timTpQErrInvalid) : String(F("--")));
  page += F("</span></div><div class=\"item\"><span class=\"label\">TP not locked</span><span class=\"value\">");
  page += htmlEscape(status.timTpValid ? yesNoLabel(status.timTpTpNotLocked) : String(F("--")));
  page += F("</span></div></div></div>");

  page += F("<div class=\"card\"><h2>UBX-NAV-SAT</h2><div class=\"grid\">");
  for (size_t i = 0; i < sizeof(counts) / sizeof(counts[0]); ++i) {
    const String label = String(constellationLabel(counts[i].constellation));
    const String domId = String(constellationDomId(counts[i].constellation));
    page += F("<button type=\"button\" class=\"count-btn\" data-target=\"");
    page += htmlEscape(domId);
    page += F("\">");
    page += htmlEscape(label);
    page += F(": ");
    page += htmlEscape(String(counts[i].count));
    page += F("</button>");
  }
  page += F("</div><div class=\"details-help\">Tap a constellation count to view all satellite details.</div>");

  const uint8_t constellations[] = {1, 2, 3, 4, 5, 6, 7};
  for (size_t c = 0; c < sizeof(constellations) / sizeof(constellations[0]); ++c) {
    const String domId = String(constellationDomId(constellations[c]));
    page += F("<div class=\"details-group\" id=\"sat-");
    page += htmlEscape(domId);
    page += F("\">");

    bool hasAny = false;
    for (size_t i = 0; i < status.navSatDetailCount; ++i) {
      const auto& detail = status.navSatDetails[i];
      if (detail.constellation != constellations[c]) {
        continue;
      }
      hasAny = true;
      page += F("<div class=\"sat-item\"><div class=\"sat-title\">");
      page += htmlEscape(String(constellationLabel(detail.constellation)));
      page += F(" PRN ");
      page += htmlEscape(String(detail.prn));
      page += F("</div><div>Elevation: ");
      page += htmlEscape(String(detail.elevationDeg));
      page += F("&deg;</div><div>Azimuth: ");
      page += htmlEscape(String(detail.azimuthDeg));
      page += F("&deg;</div><div>C/N0: ");
      if (detail.cn0DbHz >= 0) {
        page += htmlEscape(String(detail.cn0DbHz));
        page += F(" dB-Hz");
      } else {
        page += F("--");
      }
      page += F("</div><div>qualityInd: ");
      if (detail.navSatFlagsValid) {
        page += htmlEscape(String(detail.qualityInd));
      } else {
        page += F("--");
      }
      page += F("</div><div>svUsed: ");
      if (detail.navSatFlagsValid) {
        page += F(detail.svUsed ? "Yes" : "No");
      } else {
        page += F("--");
      }
      page += F("</div></div>");
    }

    if (!hasAny) {
      page += F("<div class=\"sat-item\">No satellites reported for this constellation.</div>");
    }
    page += F("</div>");
  }

  page += F("<script>(function(){"
            "var groups=document.querySelectorAll('.details-group');"
            "var buttons=document.querySelectorAll('.count-btn');"
            "function openGroup(id){"
            "groups.forEach(function(g){g.classList.remove('active');});"
            "var target=document.getElementById('sat-'+id);"
            "if(target){target.classList.add('active');}"
            "}"
            "buttons.forEach(function(btn){"
            "btn.addEventListener('click',function(){openGroup(btn.dataset.target);});"
            "});"
            "var countdown=document.getElementById('holdover-countdown');"
            "if(countdown&&countdown.dataset.remaining){"
            "var remaining=parseInt(countdown.dataset.remaining,10);"
            "function renderCountdown(){"
            "var minutes=Math.floor(remaining/60);"
            "var seconds=remaining%60;"
            "countdown.textContent=minutes+':'+String(seconds).padStart(2,'0');"
            "}"
            "renderCountdown();"
            "setInterval(function(){"
            "remaining-=1;"
            "renderCountdown();"
            "if(remaining<=0){window.location.reload();return;}"
            "},1000);"
            "}"
            "})();</script>");

  page += F("</div></div></body></html>");

  return page;
}
