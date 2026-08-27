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
  void applyDefaultSettings(GpsDashboardSettings& settingsOut);
};
