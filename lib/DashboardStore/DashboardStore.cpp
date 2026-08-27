#include "DashboardStore.h"

#include <EEPROM.h>

void DashboardStore::begin() {
  EEPROM.begin(WiFiSetupPortal::EEPROM_BYTES);
}

bool DashboardStore::loadDisplayEnabled(bool& enabledOut) {
  begin();
  const uint8_t magic = EEPROM.read(WiFiSetupPortal::DISPLAY_PREF_ADDR);
  if (magic != kDisplayPrefMagic) {
    enabledOut = true;
    return false;
  }

  enabledOut = EEPROM.read(WiFiSetupPortal::DISPLAY_PREF_ADDR + 1) != 0;
  return true;
}

bool DashboardStore::saveDisplayEnabled(bool enabled) {
  begin();
  EEPROM.write(WiFiSetupPortal::DISPLAY_PREF_ADDR, kDisplayPrefMagic);
  EEPROM.write(WiFiSetupPortal::DISPLAY_PREF_ADDR + 1, enabled ? 1 : 0);
  return EEPROM.commit();
}

void DashboardStore::loadSettings(GpsDashboardSettings& settingsOut) {
  begin();

  StoredGpsDashboardConfig stored = {};
  EEPROM.get(WiFiSetupPortal::GNSS_CONFIG_ADDR, stored);

  if (stored.magic != kGnssDashboardConfigMagic) {
    settingsOut.rawLoggingEnabled = false;
    settingsOut.ntpHoldoverEnabled = true;
    settingsOut.enableStats = true;
    settingsOut.enableLocation = false;
    return;
  }

  settingsOut.rawLoggingEnabled = stored.rawLoggingEnabled != 0;
  settingsOut.ntpHoldoverEnabled = stored.ntpHoldoverEnabled != 0;
  settingsOut.enableStats = stored.enableStatsEnabled != 0;
  settingsOut.enableLocation = stored.locationEnabled != 0;
}

bool DashboardStore::saveSettings(const GpsDashboardSettings& settings) {
  StoredGpsDashboardConfig stored = {};
  stored.magic = kGnssDashboardConfigMagic;
  stored.rawLoggingEnabled = settings.rawLoggingEnabled ? 1 : 0;
  stored.ntpHoldoverEnabled = settings.ntpHoldoverEnabled ? 1 : 0;
  stored.enableStatsEnabled = settings.enableStats ? 1 : 0;
  stored.locationEnabled = settings.enableLocation ? 1 : 0;

  begin();
  EEPROM.put(WiFiSetupPortal::GNSS_CONFIG_ADDR, stored);
  return EEPROM.commit();
}
