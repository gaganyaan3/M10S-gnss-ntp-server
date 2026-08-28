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

Quick check:

```bash
ntpdate -q <device-ip>
```

If no valid PPS or GNSS time is available, the NTP server stays silent rather than
replying with invalid timestamps.

After the first valid TIM-TP/PPS lock, optional holdover keeps NTP responses
available if live PPS or satellite fix is temporarily lost.

#### PPS-Disciplined Clock

The NTP server does not set or re-base its served clock directly from every
serial NAV-PVT timestamp. UBX-TIM-TP is preferred to establish the UTC time of
the next PPS edge:

```text
The next PPS edge corresponds to UTC time N, plus valid qErr correction.
```

Once that relationship is known, the PPS edge captured by MCPWM becomes the
precision timing source. Between PPS edges, the server interpolates from the
captured PPS tick using the same 80 MHz hardware counter. NAV-PVT remains a
coarse fallback only when it is fullyResolved and gnssFixOK.

If PPS disappears, the server does not immediately switch to setting time from
each NAV-PVT message. Holdover continues from the PPS-disciplined timestamp.
When corrected NAV-PVT differs from holdover/PPS by more than the coarse
threshold MAX_PPS_COARSE_DELTA_US,  a 60-second averaging window starts. After the window completes,
holdover is adjusted by `avgDeltaUs / 10`, preserving microsecond correction
instead of jumping directly to NAV-PVT. The `/gnss` dashboard shows the last
applied correction in microseconds, and serial logs when averaging starts.

New time-query API added to `GnssMaxM10S`:

| Method | Returns |
|---|---|
| `currentUtcEpochPps(uint32_t&)` | Whole-second UTC epoch via PPS |
| `currentUtcEpochMsPps(uint64_t&)` | Millisecond UTC epoch via PPS |
| `currentUtcEpochNsPps(uint64_t&)` | Nanosecond UTC epoch via PPS |
| `navPvtUtcEpochMs(uint64_t&)` | Coarse millisecond UTC epoch from fresh, fully resolved NAV-PVT |
| `ppsRecentlySeen()` | Whether PPS has been seen recently enough for precise lock status |
| `configurePpsHardwareClock(fn, hz)` | Inject custom tick source + nominal rate |

## PPS pulse:

- So i don't have to rely on NAV-PVT once i start getting PPS signal and MCPWM gives 12.5ns accurach at 80mhz

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
flags    = ...
refInfo  = ...

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
- Raw NMEA logging toggle (saved in NVS Preferences)
- NTP holdover toggle (saved in NVS Preferences)
- Display enable toggle (saved in NVS Preferences)

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

## future plan: can be added 
```
UBX-NAV-PVT
UBX-NAV-SAT
UBX-NAV-SIG
UBX-TIM-TP

UBX-NAV-CLOCK
UBX-NAV-TIMEUTC
UBX-NAV-TIMEGPS

UBX-MON-RF
    antenna status
    jamming/interference information

Current GNSS solution valid?
    gnssFixOK
    fixType >= 2/3 as appropriate

Current timing source healthy?
    PPS arriving
    PPS interval reasonable
    recent valid GNSS information

UBX-TIM-TP
    TIM-TP = tells you WHICH second
    PPS    = tells you EXACTLY WHEN that second occurs
    MCPWM  = measures time BETWEEN PPS pulses

The internal Digitally Controlled Oscillator (NCO) clock inside the M10 generation runs at a fast 64 MHz (an upgrade from the older M8 generation's 48 MHz). Because the clock steps are so small, the raw physical hardware 1PPS pulse generated by the M10S has an inherently low hardware jitter—typically around ±15.6 nanoseconds (which corresponds exactly to one 1 / 64 MHz clock cycle)

```
- should i keep the COARSE correction (µs) even if PPS is present? NO
- option to force update GNSS NAV-PVT time from dashboard
- start serve ntp query fast with validDate, validTime, fullyResolved ?
- Discipline the ESP32 oscillator MCPWM clock diff between PPS tick 1 and 2 should be ~80Mhz and compasate with cpu temp
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
ACQUIRING
   ↓
LOCKED
   ↓
HOLDOVER
----------------------
UNSYNCED
No trustworthy UTC.

ACQUIRING
NAV-PVT UTC is valid.
Waiting for valid TIM-TP + PPS.

LOCKED
TIM-TP valid.
PPS valid.
Timer being disciplined.

HOLDOVER
PPS disappeared.
Running from previously measured timer frequency.

Rename clockstate GPS_CLOCK_STATE_ACQUIRING to GPS_CLOCK_STATE_Acquired ?
```

## Project Structure

- [src/main.cpp](src/main.cpp): firmware orchestration — setup wiring, loop polling order, Wi-Fi/NMEA/NTP callbacks, dashboard data assembly
- [lib/PpsCapture/PpsCapture.h](lib/PpsCapture/PpsCapture.h): MCPWM PPS capture interface, pulse snapshots, tick source API
- [lib/PpsCapture/PpsCapture.cpp](lib/PpsCapture/PpsCapture.cpp): PPS capture ISR, software-forced "now" capture, 64-bit tick extension, period stats
- [lib/GnssClock/GnssClock.h](lib/GnssClock/GnssClock.h): GNSS-backed NTP clock interface, holdover diagnostics, clock-state helper
- [lib/GnssClock/GnssClock.cpp](lib/GnssClock/GnssClock.cpp): PPS-disciplined NTP time selection, holdover, coarse correction averaging
- [lib/DashboardStore/DashboardStore.h](lib/DashboardStore/DashboardStore.h): NVS-backed dashboard/display settings API
- [lib/DashboardStore/DashboardStore.cpp](lib/DashboardStore/DashboardStore.cpp): display preference and GNSS dashboard config persistence
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
- [platformio.ini](platformio.ini): ESP32 PlatformIO environment and dependencies
