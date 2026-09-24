#include "DashboardStore.h"

#include <Preferences.h>

namespace {

constexpr const char* DASHBOARD_PREFS_NAMESPACE = "gnss_dash";
constexpr const char* DISPLAY_KEY = "display";
constexpr const char* METRICS_KEY = "metrics";
constexpr const char* RAW_LOGGING_KEY = "raw";
constexpr const char* HOLDOVER_TIMEOUT_KEY = "hold_timeout";
constexpr const char* CONFIRMED_TIMEOUT_KEY = "conf_timeout";
constexpr const char* CORRECTION_DELTA_KEY = "corr_delta";
constexpr const char* CORRECTION_IGNORE_KEY = "corr_ignore";

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

  if (!prefs.isKey(RAW_LOGGING_KEY)) {
    prefs.end();
    applyDefaultSettings(settingsOut);
    return;
  }

  settingsOut.rawLoggingEnabled = prefs.getBool(RAW_LOGGING_KEY, false);
  settingsOut.metricsEnabled = prefs.getBool(METRICS_KEY, true);
  settingsOut.kHoldoverTimeoutMs = prefs.getUInt(HOLDOVER_TIMEOUT_KEY, settingsOut.kHoldoverTimeoutMs);
  settingsOut.kConfirmedTimeHoldoverTimeoutMs = prefs.getUInt(
      CONFIRMED_TIMEOUT_KEY, settingsOut.kConfirmedTimeHoldoverTimeoutMs);
  settingsOut.kMaxConfirmedTimeCorrectionDeltaMs = prefs.getUInt(
      CORRECTION_DELTA_KEY, settingsOut.kMaxConfirmedTimeCorrectionDeltaMs);
  settingsOut.kMaxConfirmedTimeCorrectionIgnoreDeltaMs = prefs.getUInt(
      CORRECTION_IGNORE_KEY, settingsOut.kMaxConfirmedTimeCorrectionIgnoreDeltaMs);
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
  ok = prefs.putBool(METRICS_KEY, settings.metricsEnabled) > 0 && ok;
  ok = prefs.putUInt(HOLDOVER_TIMEOUT_KEY, settings.kHoldoverTimeoutMs) > 0 && ok;
  ok = prefs.putUInt(CONFIRMED_TIMEOUT_KEY, settings.kConfirmedTimeHoldoverTimeoutMs) > 0 && ok;
  ok = prefs.putUInt(CORRECTION_DELTA_KEY, settings.kMaxConfirmedTimeCorrectionDeltaMs) > 0 && ok;
  ok = prefs.putUInt(CORRECTION_IGNORE_KEY, settings.kMaxConfirmedTimeCorrectionIgnoreDeltaMs) > 0 && ok;
  prefs.end();
  return ok;
}

void DashboardStore::applyDefaultSettings(GpsDashboardSettings& settingsOut) {
  settingsOut.rawLoggingEnabled = false;
  settingsOut.displayEnabled = true;
  settingsOut.metricsEnabled = true;
  settingsOut.kHoldoverTimeoutMs = 1440UL * 60UL * 1000UL;
  settingsOut.kConfirmedTimeHoldoverTimeoutMs = 60UL * 1000UL;
  settingsOut.kMaxConfirmedTimeCorrectionDeltaMs = 5UL;
  settingsOut.kMaxConfirmedTimeCorrectionIgnoreDeltaMs = 100UL;
}
