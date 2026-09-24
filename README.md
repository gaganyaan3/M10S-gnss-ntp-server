# ESP32 + SmartElex u-blox MAX-M10S GNSS + NTP Clock (SH1106 OLED)

This project runs on an ESP32 and shows GNSS time from a SmartElex
u-blox MAX-M10S module on a SH1106 128x64 OLED and serve as NTP server.

## ESP32 Wiring

### OLED SH1106 (I2C)
- VCC -> 3V3
- GND -> GND
- SDA -> GPIO32
- SCL -> GPIO33

### SmartElex u-blox MAX-M10S GNSS (UART)
- VCC -> 3V3
- GND -> GND
- TX -> GPIO22 (ESP32 RX)
- RX -> GPIO21 (ESP32 TX)

### MAX-M10S PPS (1PPS)
- PPS -> GPIO27
- GND (module) must be common with ESP32 GND

> **PPS is captured by the ESP32's MCPWM hardware capture unit (CAP0) clocked
> at 80 MHz (APB_CLK), not a software GPIO interrupt.** The rising edge instant
> is latched into a free-running 80 MHz counter by dedicated hardware,
> eliminating interrupt-service jitter. A second capture channel (CAP1) is
> left unbound to any pin and sampled via a software-forced capture to read the
> same counter on demand — giving a "now" tick in the exact same domain as the
> PPS edge, so elapsed-time math is pure integer tick subtraction at 12.5 ns
> resolution.

### BOOT Button (already on most ESP32 DevKit boards)
- BOOT -> GPIO0 (active LOW)
- Short press: toggle OLED display ON/OFF
- Hold ~3 seconds: clear saved Wi-Fi credentials and re-enter setup portal

## Network Services

### NMEA TCP Server
The device publishes raw NMEA sentences on TCP port **10110**.

Examples:

```bash
nc <device-ip> 10110
```

or use OpenCPN with `tcp://<device-ip>:10110`.

### GNSS-Backed NTP Server
The device serves NTP on UDP port **123** using GNSS time as a stratum-1 source.
It reads the standard 48-byte NTP header and discards any remaining datagram
bytes. This is necessary with ESP32 `WiFiUDP`: unread bytes can keep the current
datagram buffered and prevent subsequent requests from being parsed. Extension
or authentication data beyond the standard header is ignored.

Quick check:

```bash
ntpdate -q <device-ip>
```

If no valid PPS, NAV-PVT confirmed-time, or holdover time is
available, the NTP server stays silent rather than replying with invalid
timestamps.

After the first valid PPS lock or NAV-PVT confirmed-time,
optional holdover keeps NTP responses available if live PPS or satellite fix is
temporarily lost.

#### PPS-Disciplined Clock

The NTP server does not set or re-base its served clock directly from every
serial NAV-PVT timestamp. UBX-TIM-TP is preferred to establish the UTC time of
the next PPS edge:

```text
The next PPS edge corresponds to UTC time N, plus valid qErr correction.
```

Once that relationship is known, the PPS edge captured by MCPWM becomes the
precision timing source. Between PPS edges, the server interpolates from the
captured PPS tick using the same 80 MHz hardware counter. The interpolation is
kept at nanosecond resolution through the holdover and NTP paths. ConfirmedTime
is the NAV-PVT fallback when NAV-PVT has validDate, validTime, fullyResolved,
and confirmedTime.

Each PPS interval must contain exactly one pulse and measure between 990 ms and
1010 ms (1 second +/- 1%). Intervals outside this range are rejected: they do
not re-anchor the GNSS clock, update PPM, update the temperature calibration,
or appear as a valid MCPWM frequency metric. The first PPS edge is accepted to
establish the initial UTC anchor; subsequent edges must pass this validation.

The MCPWM counter provides 12.5 ns tick resolution at 80 MHz. NTP timestamps
preserve the resulting nanosecond value in the NTP fractional-seconds field.
This describes timestamp resolution, not guaranteed UTC accuracy: holdover
accuracy is limited by MCPWM frequency/temperature drift, PPS jitter, GNSS
timing, and network delay and asymmetry. The dashboard's legacy epoch and
diagnostic fields remain displayed in microseconds.

If PPS disappears, the server does not immediately switch to setting time from
each NAV-PVT message. Holdover continues from the PPS-disciplined timestamp. In
ConfirmedTime mode, the first valid NAV-PVT UTC time anchors holdover and starts
the shorter holdover timeout (`kConfirmedTimeHoldoverTimeoutMs`). After that
timeout expires, the millisecond delta between NAV-PVT and holdover is checked.
If the delta is greater than `kMaxConfirmedTimeCoarseDeltaMs` and below
`kMaxConfirmedTimeCoarseIgnoreDeltaMs`, holdover is nudged by 10% of that
millisecond delta. Larger deltas are ignored as unstable samples. The `/gnss`
dashboard shows the last applied correction in microseconds, and serial logs
each skipped, ignored, or applied correction checkpoint.

New time-query API added to `GnssMaxM10S`:

| Method | Returns |
|---|---|
| `currentUtcEpochPps(uint32_t&)` | Whole-second UTC epoch via PPS |
| `currentUtcEpochMsPps(uint64_t&)` | Millisecond UTC epoch via PPS |
| `currentUtcEpochNsPps(uint64_t&)` | Nanosecond UTC epoch via PPS |
| `navPvtUtcEpochMs(uint64_t&)` | Coarse millisecond UTC epoch from fresh, fully resolved NAV-PVT |
| `ppsRecentlySeen()` | Whether PPS has been seen recently enough for precise lock status |
| `configurePpsHardwareClock(fn, hz)` | Inject custom tick source + nominal rate |

`GnssClock::navPvtConfirmedTimeEpochUs(uint64_t&)` exposes the quick
ConfirmedTime source used for NTP when NAV-PVT has validDate, validTime,
fullyResolved, and confirmedTime.

`GnssClock::currentHoldoverNs(uint64_t&)` and
`GnssClock::currentNtpEpochNs(uint64_t&)` expose nanosecond-resolution
holdover and NTP epochs. The microsecond APIs remain available as compatibility
wrappers.

NTP source priority is PPSAnchored with PPS first, then ConfirmedTime, then valid
holdover.

After a PPSAnchored state has anchored holdover, losing PPS or receiving an
invalid PPS interval enters HOLDOVER after PPS has not been seen for 1.5 seconds
(`kPpsRecentlySeenMs` and `kMaxPpsHoldoverMs`). It keeps serving from that
PPS-disciplined timestamp for `kHoldoverTimeoutMs`, using the MCPWM counter for
interpolation. Live
ConfirmedTime NAV-PVT data does not shorten that PPS holdover deadline; it
is only used after PPS holdover expires or when no PPS-disciplined holdover
exists.

When ConfirmedTime is the active source without prior PPS lock, each fresh
NAV-PVT sample re-anchors fallback holdover so the `/gnss` Holdover UTC value
continues to move from the latest confirmed GNSS UTC.

## PPS pulse:

- Once PPS is locked, MCPWM provides 12.5 ns tick resolution at 80 MHz for
  interpolation between PPS edges; this is resolution, not guaranteed 12.5 ns
  UTC accuracy.

```
NAV-PVT: "The time is around 12:00:00.555..."
                         ↓
PPS:     "RIGHT HERE is exactly 12:00:01.000000000"
                         ↓
ESP timer counts from PPS
                         ↓
+100 ms = 12:00:01.100000000
+500 ms = 12:00:01.500000000
+999 ms = 12:00:01.999000000
                         ↓
next PPS
                         ↓
          12:00:02.000000000
```
- UBX-TIM-TP

```
towMS    = 345601000
towSubMS = 0
qErr     = -32000 ps
week     = 2433

              UBX-TIM-TP
                   │
                   │ tells you which time
                   │ pulse represents;
                   │ valid qErr is applied
                   ↓
PPS edge ──────────●───────────
                   ↑
             hardware capture
             ESP32 timer
```

## GNSS Dashboard

When connected to Wi-Fi, open:

```text
http://<device-ip>/gnss
```

Dashboard features:
- Runtime GNSS status (satellites, GNSS epoch, PPS, NTP readiness)
- System health from `SysHealth`: CPU avg/peak, ESP32 temperature, free heap RAM, uptime
- Raw NMEA logging toggle (saved in NVS Preferences)
- Display enable toggle (saved in NVS Preferences)
- Editable GNSS clock settings (holdover timeouts and ConfirmedTime correction
  thresholds, saved in NVS Preferences)

Live NAV-PVT-style stats page:

```text
http://<device-ip>/stats
```

Shows:
- Latitude
- Longitude
- Altitude (m)
- Speed (m/s)

## Build and Upload (PlatformIO)

From this project folder:

```bash
pio run
pio run -t upload
pio device monitor -b 115200
```

If upload port is not auto-detected:

```bash
pio run -t upload --upload-port /dev/ttyUSB0
```

### OTA Upload

OTA is implemented as a separate `lib/OtaUpdate` wrapper around `ArduinoOTA`.
It starts only after the ESP32 is connected to station Wi-Fi and the setup
hotspot is inactive.

Create a local secrets file from the example:

```bash
cp include/OtaSecrets.example.h include/OtaSecrets.h
```

Put the MD5 hash of your OTA password in `include/OtaSecrets.h`. This file is
ignored by git.

Generate the hash:

```bash
printf '%s' 'your-real-ota-password' | md5sum
```

Upload over OTA:

```bash
OTA_PASSWORD='your-real-ota-password' pio run -e esp32dev_ota -t upload
```

The OTA target IP is configured in `platformio.ini` as `192.168.10.96`.
Prefer a router DHCP reservation so this IP stays stable.

## future plan: can be added 
```
UBX-NAV-PVT
UBX-NAV-SIG
UBX-TIM-TP

UBX-NAV-CLOCK
UBX-NAV-TIMEUTC
UBX-NAV-TIMEGPS

UBX-TIM-TP
    TIM-TP = tells you WHICH second
    PPS    = tells you EXACTLY WHEN that second occurs
    MCPWM  = measures time BETWEEN PPS pulses

The internal Digitally Controlled Oscillator (NCO) clock inside the M10 generation runs at a fast 64 MHz (an upgrade from the older M8 generation's 48 MHz). Because the clock steps are so small, the raw physical hardware 1PPS pulse generated by the M10S has an inherently low hardware jitter—typically around ±15.6 nanoseconds (which corresponds exactly to one 1 / 64 MHz clock cycle)

```
- option to force update GNSS NAV-PVT time from dashboard
- ESP32 MCPWM tick-rate compensation is applied from `CalibratedFqMap` based on the sampled CPU temperature. Set `PPS_MCPWM_TEMPERATURE_CALIBRATION_ENABLED` false in `src/main.cpp` to use `CalibratedFqMap::DEFAULT_TICKS_PER_SECOND`.
- Typical coax with velocity factor 0.66–0.85: about **4–5 ns per meter*
- chec if MAIN_LOOP_IDLE_DELAY_MS=1ms is needed? it came as flase cpu utilization
```
PPS_MCPWM_TICKS_PER_SECOND = 80000000UL;
ppsMcpwmCalibratedTicksPerSecond = 79998000UL; // calibrated MCPWM
If you assume exactly 80 MHz, but your measured PPS interval is 79,998,037 ticks, your clock is slow by:
80,000,000 - 79,998,037 = 1,963 ticks/sec , thats 1963×12.5=24,537.5 ns = 24.5 microsec slow every sec
PPS disappears (holdover): it does accumulate if you assume exactly 80 MHz. 1 hour  = ~88 ms
calibrated 79,998,038 - 79998000  = 38 = 475ns slow/sec = 1.71ms per hours
```
- Track clock state explicitly

```
UNSYNCED
   ↓
ConfirmedTime
   ↓
PPSAnchored
   ↓
HOLDOVER
----------------------
UNSYNCED
No trustworthy UTC.

ConfirmedTime
NAV-PVT validDate, validTime, fullyResolved, and confirmedTime are set.
NTP serves quick NAV-PVT time while waiting for a GNSS fix and PPS lock.

PPSAnchored
TIM-TP valid.
PPS valid.
Timer being disciplined.

HOLDOVER
PPS disappeared.
Running from previously measured timer frequency.
```

## Example PlatformIO Environment

```ini
[env:esp32dev_ota]
extends = env:esp32dev
upload_protocol = espota
upload_port = 192.168.10.90
;upload_flags =
;  --auth=your-real-ota-password
upload_flags =
  --auth=${sysenv.OTA_PASSWORD}
```

Upload over OTA with:

```bash
pio run -e esp32dev_ota -t upload

#OR
export OTA_PASSWORD='your-real-ota-password'
pio run -e esp32dev_ota -t upload --upload-port 192.168.10.90
```

## Project Structure

- [src/main.cpp](src/main.cpp): firmware orchestration — setup wiring, loop polling order, Wi-Fi/NMEA/NTP callbacks, dashboard data assembly
- [lib/CalibratedFqMap/CalibratedFqMap.h](lib/CalibratedFqMap/CalibratedFqMap.h): CPU-temperature-to-MCPWM-frequency calibration API
- [lib/CalibratedFqMap/CalibratedFqMap.cpp](lib/CalibratedFqMap/CalibratedFqMap.cpp): calibrated `PPS_MCPWM_TICKS_PER_SECOND_MAP` and nearest-temperature lookup
- [lib/PpsCapture/PpsCapture.h](lib/PpsCapture/PpsCapture.h): MCPWM PPS capture interface, pulse snapshots, tick source API
- [lib/PpsCapture/PpsCapture.cpp](lib/PpsCapture/PpsCapture.cpp): PPS capture ISR, software-forced "now" capture, 64-bit tick extension, period stats
- [lib/GnssClock/GnssClock.h](lib/GnssClock/GnssClock.h): GNSS-backed NTP clock interface, holdover diagnostics, clock-state helper
- [lib/GnssClock/GnssClock.cpp](lib/GnssClock/GnssClock.cpp): PPS-disciplined NTP time selection, holdover, coarse correction
- [lib/DashboardStore/DashboardStore.h](lib/DashboardStore/DashboardStore.h): NVS-backed dashboard/display settings API
- [lib/DashboardStore/DashboardStore.cpp](lib/DashboardStore/DashboardStore.cpp): display preference and GNSS dashboard config persistence
- [lib/SysHealth/SysHealth.h](lib/SysHealth/SysHealth.h): ESP32 system-health sampler for CPU, temperature, free heap, and uptime
- [lib/SysHealth/SysHealth.cpp](lib/SysHealth/SysHealth.cpp): system-health sampling and dashboard snapshot implementation
- [lib/OledClockDisplay/OledClockDisplay.h](lib/OledClockDisplay/OledClockDisplay.h): OLED display model and IST formatting helpers
- [lib/OledClockDisplay/OledClockDisplay.cpp](lib/OledClockDisplay/OledClockDisplay.cpp): SH1106 boot screen, reset overlay, clock page rendering
- [lib/BootButton/BootButton.h](lib/BootButton/BootButton.h): BOOT button debounce and press-event interface
- [lib/BootButton/BootButton.cpp](lib/BootButton/BootButton.cpp): short/long press detection implementation
- [lib/GnssMaxM10S/GnssMaxM10S.h](lib/GnssMaxM10S/GnssMaxM10S.h): GNSS parser/clock interface — hardware tick source config, ns-resolution PPS time query
- [lib/GnssMaxM10S/GnssMaxM10S.cpp](lib/GnssMaxM10S/GnssMaxM10S.cpp): GNSS UART polling, NAV-PVT UTC-second anchoring, PPS-assisted time interpolation
- [lib/NmeaServer/NmeaServer.h](lib/NmeaServer/NmeaServer.h): TCP NMEA broadcast server (port 10110)
- [lib/NmeaServer/NmeaServer.cpp](lib/NmeaServer/NmeaServer.cpp): NMEA server implementation
- [lib/NtpServer/NtpServer.h](lib/NtpServer/NtpServer.h): GNSS-backed NTP server (UDP port 123)
- [lib/NtpServer/NtpServer.cpp](lib/NtpServer/NtpServer.cpp): NTP server implementation
- [lib/WiFiSetupPortal/WiFiSetupPortal.h](lib/WiFiSetupPortal/WiFiSetupPortal.h): captive portal + GNSS dashboard interface
- [lib/WiFiSetupPortal/WiFiSetupPortal.cpp](lib/WiFiSetupPortal/WiFiSetupPortal.cpp): portal, credential storage, dashboard pages

Prometheus/VictoriaMetrics can scrape `GET /metrics`. It exposes `cpu_temp`,
`ntp_packets_received` (the NTP Status card's NTP Packets Received value),
the latest valid measured `mcpwm_frequncy`, and `clock_state` (PPS=1, holdover=2, confirmed=3,
untrusted=4). `PPM` is emitted only after the STILL reference has been
running for at least one hour and is the average `STILL − HOLDOVER` PPM error.
Metrics can be enabled or disabled from the `Enable metrics` checkbox in
Saved Settings on `/gnss`; disabled metrics return 404 from `/metrics`.
The asterisk shown beside the dashboard PPM Error is removed after one hour
of STILL reference data.
- [platformio.ini](platformio.ini): ESP32 PlatformIO environment and dependencies
