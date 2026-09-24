#include "NtpServer.h"

#include <WiFi.h>
#include <string.h>

NtpServer::NtpServer(uint16_t port) : port_(port) {}

void NtpServer::begin() {
  hasSocketCheck_ = false;
  maintainSocket(millis());
}

void NtpServer::setTimeProvider(TimeProvider provider) {
  timeProvider_ = provider;
}

void NtpServer::poll() {
  maintainSocket(millis());
  if (!socketReady_) {
    return;
  }

  const int packetSize = udp_.parsePacket();
  if (packetSize <= 0) {
    return;
  }
  ++status_.packetsReceived;
  if (packetSize < static_cast<int>(NTP_PACKET_SIZE)) {
    udp_.flush();
    return;
  }

  uint8_t requestBuffer[NTP_PACKET_SIZE];
  const int bytesRead = udp_.read(requestBuffer, NTP_PACKET_SIZE);
  // ESP32 WiFiUDP keeps the current datagram buffered until all bytes are
  // consumed. Discard any extension/authentication bytes so a >48-byte packet
  // cannot block parsePacket() from delivering the next request.
  udp_.flush();
  if (bytesRead != static_cast<int>(NTP_PACKET_SIZE)) {
    return;
  }

  const uint8_t requestVersion = (requestBuffer[0] >> 3) & 0x07;
  const uint8_t requestMode = requestBuffer[0] & 0x07;
  if (requestMode != 3 || requestVersion > 4) {
    return;
  }
  ++status_.validRequests;

  const IPAddress remoteIp = udp_.remoteIP();
  const uint16_t remotePort = udp_.remotePort();

  uint32_t epochUtc = 0;
  uint32_t fraction = 0;
  if (!timeProvider_ || !timeProvider_(epochUtc, fraction)) {
    // No trustworthy time source yet (e.g. GNSS has no fix) -- stay silent
    // rather than answer with a bogus timestamp.
    return;
  }

  const uint8_t effectiveVersion = requestVersion == 0 ? 3 : requestVersion;

  uint8_t response[NTP_PACKET_SIZE];
  memset(response, 0, sizeof(response));

  response[0] = static_cast<uint8_t>((effectiveVersion << 3) | 0x04);  // LI=0, VN=client's, Mode=4 (server)
  response[1] = 1;     // Stratum 1: primary reference clock (GNSS)
  response[2] = 6;     // Poll interval: ~2^6 = 64s
  response[3] = 0xE2;  // Precision: 2^-30s (~1ns) timestamp quantization

  // Root Delay and Root Dispersion left at 0.

  // Reference Identifier: "GNSS"
  response[12] = 'G';
  response[13] = 'P';
  response[14] = 'S';
  response[15] = 0;

  const uint32_t ntpSeconds = epochUtc + NTP_UNIX_EPOCH_DELTA;

  writeTimestamp(response, 16, ntpSeconds, fraction);  // Reference Timestamp

  // Originate Timestamp: echo back the client's own Transmit Timestamp.
  memcpy(response + 24, requestBuffer + 40, 8);

  writeTimestamp(response, 32, ntpSeconds, fraction);  // Receive Timestamp
  writeTimestamp(response, 40, ntpSeconds, fraction);  // Transmit Timestamp

  ++status_.repliesAttempted;
  if (!udp_.beginPacket(remoteIp, remotePort)) {
    invalidateSocket();
    return;
  }
  if (udp_.write(response, NTP_PACKET_SIZE) != NTP_PACKET_SIZE || !udp_.endPacket()) {
    invalidateSocket();
  }
}

void NtpServer::maintainSocket(uint32_t nowMs) {
  const bool wifiConnected = WiFi.status() == WL_CONNECTED;
  if (!wifiConnected) {
    if (socketReady_) {
      udp_.stop();
      socketReady_ = false;
    }
    hasSocketCheck_ = false;
    return;
  }

  if (socketReady_) {
    return;
  }
  if (hasSocketCheck_ && static_cast<uint32_t>(nowMs - lastSocketCheckMs_) < 2000UL) {
    return;
  }

  hasSocketCheck_ = true;
  lastSocketCheckMs_ = nowMs;
  socketReady_ = udp_.begin(port_) != 0;
}

void NtpServer::invalidateSocket() {
  udp_.stop();
  socketReady_ = false;
  hasSocketCheck_ = false;
}

void NtpServer::writeTimestamp(uint8_t* buffer, size_t offset, uint32_t seconds, uint32_t fraction) {
  buffer[offset + 0] = static_cast<uint8_t>((seconds >> 24) & 0xFF);
  buffer[offset + 1] = static_cast<uint8_t>((seconds >> 16) & 0xFF);
  buffer[offset + 2] = static_cast<uint8_t>((seconds >> 8) & 0xFF);
  buffer[offset + 3] = static_cast<uint8_t>(seconds & 0xFF);
  buffer[offset + 4] = static_cast<uint8_t>((fraction >> 24) & 0xFF);
  buffer[offset + 5] = static_cast<uint8_t>((fraction >> 16) & 0xFF);
  buffer[offset + 6] = static_cast<uint8_t>((fraction >> 8) & 0xFF);
  buffer[offset + 7] = static_cast<uint8_t>(fraction & 0xFF);
}
