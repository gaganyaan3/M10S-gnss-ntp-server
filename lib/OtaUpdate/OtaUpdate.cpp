#include "OtaUpdate.h"

#include <ArduinoOTA.h>
#include <WiFi.h>

void OtaUpdate::begin(const char* hostname, const char* passwordHash) {
  if (started_ || WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (hostname == nullptr || hostname[0] == '\0') {
    Serial.println("OTA not started: hostname is empty");
    return;
  }

  if (passwordHash == nullptr || passwordHash[0] == '\0') {
    Serial.println("OTA not started: password hash is empty");
    return;
  }

  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPasswordHash(passwordHash);

  ArduinoOTA.onStart([]() {
    Serial.println("OTA update started");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("OTA update finished; rebooting");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    if (total == 0) {
      return;
    }
  Serial.printf("OTA progress: %u%%\r\n", progress * 100U / total);
  });

  ArduinoOTA.onError([](ota_error_t error) {
  Serial.printf("OTA error: %u\r\n", static_cast<unsigned int>(error));
  });

  ArduinoOTA.begin();
  started_ = true;
  Serial.printf("OTA ready: %s.local\r\n", hostname);
}

void OtaUpdate::loop() {
  if (started_) {
    ArduinoOTA.handle();
  }
}

bool OtaUpdate::started() const {
  return started_;
}
