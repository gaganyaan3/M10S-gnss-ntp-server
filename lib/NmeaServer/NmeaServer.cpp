#include "NmeaServer.h"

NmeaServer::NmeaServer(uint16_t port) : server_(port) {}

void NmeaServer::begin() {
  server_.begin();
  server_.setNoDelay(true);
}

void NmeaServer::setLineTransformer(LineTransformer transformer) {
  lineTransformer_ = std::move(transformer);
}

void NmeaServer::poll() {
  WiFiClient newClient = server_.available();
  if (newClient) {
    bool placed = false;
    for (uint8_t i = 0; i < MAX_CLIENTS; ++i) {
      if (!clients_[i] || !clients_[i].connected()) {
        clients_[i] = newClient;
        placed = true;
        Serial.printf("NMEA client connected (slot %u)\r\n", i);
        break;
      }
    }
    if (!placed) {
      Serial.println("NMEA server full, rejecting client");
      newClient.stop();
    }
  }

  for (uint8_t i = 0; i < MAX_CLIENTS; ++i) {
    if (clients_[i] && !clients_[i].connected()) {
      clients_[i].stop();
    }
  }
}

void NmeaServer::broadcastLine(const String& line) {
  bool hasClient = false;
  for (uint8_t i = 0; i < MAX_CLIENTS; ++i) {
    if (clients_[i] && clients_[i].connected()) {
      hasClient = true;
      break;
    }
  }
  if (!hasClient) {
    return;
  }

  const String outputLine = lineTransformer_ ? lineTransformer_(line) : line;
  for (uint8_t i = 0; i < MAX_CLIENTS; ++i) {
    if (clients_[i] && clients_[i].connected()) {
      clients_[i].print(outputLine);
      clients_[i].print("\r\n");
    }
  }
}

uint8_t NmeaServer::clientCount() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < MAX_CLIENTS; ++i) {
    if (clients_[i] && clients_[i].connected()) {
      ++count;
    }
  }
  return count;
}
