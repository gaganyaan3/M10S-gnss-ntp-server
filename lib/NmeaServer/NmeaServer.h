#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <functional>

// Broadcasts raw NMEA-0183 sentences to any connected TCP client(s) on the
// standard NMEA network port (10110), so tools like OpenCPN can consume the
// GNSS feed over Wi-Fi.
class NmeaServer {
 public:
  static constexpr uint16_t DEFAULT_PORT = 10110;
  static constexpr uint8_t MAX_CLIENTS = 4;

  using LineTransformer = std::function<String(const String&)>;

  explicit NmeaServer(uint16_t port = DEFAULT_PORT);

  void begin();
  void poll();
  void setLineTransformer(LineTransformer transformer);
  void broadcastLine(const String& line);
  uint8_t clientCount();

 private:
  WiFiServer server_;
  WiFiClient clients_[MAX_CLIENTS];
  LineTransformer lineTransformer_;
};
