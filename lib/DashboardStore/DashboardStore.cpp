#include "DashboardStore.h"

#include <Preferences.h>

namespace {

constexpr const char* DASHBOARD_PREFS_NAMESPACE = "gnss_dash";
constexpr const char* DISPLAY_KEY = "display";
constexpr const char* RAW_LOGGING_KEY = "raw";
constexpr const char* NTP_HOLDOVER_KEY = "holdover";
constexpr const char* STATS_KEY = "stats";
constexpr const char* LOCATION_KEY = "location";

}  // namespace

void DashboardStore::begin() {
}

bool DashboardStore::loadDisplayEnabled(bool& enabledOut) {
  begin();

  Preferences prefs;
  if (!prefs.begin(DASHBOARD_PREFS_NAMESPACE, true)) {
    enabledOut = true;
    return false;
  }

  if (!prefs.isKey(DISPLAY_KEY)) {
    prefs.end();
    enabledOut = true;
    return false;
  }

  enabledOut = prefs.getBool(DISPLAY_KEY, true);
  prefs.end();
  return true;
}

bool DashboardStore::saveDisplayEnabled(bool enabled) {
  begin();

  Preferences prefs;
  if (!prefs.begin(DASHBOARD_PREFS_NAMESPACE, false)) {
    return false;
  }
  const bool ok = prefs.putBool(DISPLAY_KEY, enabled) > 0;
  prefs.end();
  return ok;
}

void DashboardStore::loadSettings(GpsDashboardSettings& settingsOut) {
  begin();

  Preferences prefs;
  if (!prefs.begin(DASHBOARD_PREFS_NAMESPACE, true)) {
    applyDefaultSettings(settingsOut);
    return;
  }

  if (!prefs.isKey(RAW_LOGGING_KEY) ||
      !prefs.isKey(NTP_HOLDOVER_KEY) ||
      !prefs.isKey(STATS_KEY) ||
      !prefs.isKey(LOCATION_KEY)) {
    prefs.end();
    applyDefaultSettings(settingsOut);
    return;
  }

  settingsOut.rawLoggingEnabled = prefs.getBool(RAW_LOGGING_KEY, false);
  settingsOut.ntpHoldoverEnabled = prefs.getBool(NTP_HOLDOVER_KEY, true);
  settingsOut.enableStats = prefs.getBool(STATS_KEY, true);
  settingsOut.enableLocation = prefs.getBool(LOCATION_KEY, false);
  prefs.end();
}

bool DashboardStore::saveSettings(const GpsDashboardSettings& settings) {
  begin();

  Preferences prefs;
  if (!prefs.begin(DASHBOARD_PREFS_NAMESPACE, false)) {
    return false;
  }

  bool ok = true;
  ok = prefs.putBool(RAW_LOGGING_KEY, settings.rawLoggingEnabled) > 0 && ok;
  ok = prefs.putBool(NTP_HOLDOVER_KEY, settings.ntpHoldoverEnabled) > 0 && ok;
  ok = prefs.putBool(STATS_KEY, settings.enableStats) > 0 && ok;
  ok = prefs.putBool(LOCATION_KEY, settings.enableLocation) > 0 && ok;
  prefs.end();
  return ok;
}

void DashboardStore::applyDefaultSettings(GpsDashboardSettings& settingsOut) {
  settingsOut.rawLoggingEnabled = false;
  settingsOut.ntpHoldoverEnabled = true;
  settingsOut.enableStats = true;
  settingsOut.enableLocation = false;
}
