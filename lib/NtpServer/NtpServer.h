#pragma once

#include <Arduino.h>
#include <WiFiUdp.h>
#include <functional>

// A minimal SNTP/NTP (RFC 5905) server that answers client time requests
// using an external time source (e.g. GNSS) as a stratum-1 reference clock.
class NtpServer {
 public:
  static constexpr uint16_t DEFAULT_PORT = 123;

  // Returns true and sets epochUtcOut/fractionOut when a current UTC time is available.
  using TimeProvider = std::function<bool(uint32_t& epochUtcOut, uint32_t& fractionOut)>;

  explicit NtpServer(uint16_t port = DEFAULT_PORT);

  void begin();
  void poll();
  void setTimeProvider(TimeProvider provider);

 private:
  static constexpr size_t NTP_PACKET_SIZE = 48;
  static constexpr uint32_t NTP_UNIX_EPOCH_DELTA = 2208988800UL;

  static void writeTimestamp(uint8_t* buffer, size_t offset, uint32_t seconds, uint32_t fraction);

  WiFiUDP udp_;
  uint16_t port_;
  TimeProvider timeProvider_;
};
