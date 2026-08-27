#pragma once

#include <Arduino.h>
#include "WiFiSetupPortal.h"

class DashboardStore {
 public:
  void begin();
  bool loadDisplayEnabled(bool& enabledOut);
  bool saveDisplayEnabled(bool enabled);
  void loadSettings(GpsDashboardSettings& settingsOut);
  bool saveSettings(const GpsDashboardSettings& settings);

 private:
  static constexpr uint8_t kDisplayPrefMagic = 0xA7;
  static constexpr uint32_t kGnssDashboardConfigMagic = 0x47505331UL;

  struct StoredGpsDashboardConfig {
    uint32_t magic;
    uint8_t rawLoggingEnabled;
    uint8_t ntpHoldoverEnabled;
    uint8_t enableStatsEnabled;
    uint8_t locationEnabled;
    uint8_t reserved[9];
  };
};
