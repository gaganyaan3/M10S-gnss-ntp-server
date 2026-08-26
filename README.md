# ESP32 + SmartElex u-blox MAX-M10S GNSS + NTP Clock (SH1106 OLED)

This project runs on an ESP32 and shows both GNSS time (from a SmartElex
u-blox MAX-M10S module) and internet NTP time on a SH1106 128x64 OLED.

Displayed time is IST and refreshes every second in this format:

`dd/mm/yy hh:mm:ss IST`

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

If no valid GNSS time is available, the NTP server stays silent rather than
replying with invalid timestamps.

After the first fullyResolved GNSS time with gnssFixOK is seen since boot,
optional holdover keeps NTP responses available if live PPS or satellite fix is
temporarily lost.

#### PPS-Disciplined Clock

The NTP server does not set or re-base its served clock directly from every
serial NAV-PVT timestamp. NAV-PVT is used to establish and verify the UTC
second:

```text
The next PPS edge corresponds to UTC second N.
```

Once that relationship is known, the PPS edge captured by MCPWM becomes the
precision timing source. Between PPS edges, the server interpolates from the
captured PPS tick using the same 80 MHz hardware counter.

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
                   │ pulse represents
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
- Runtime GNSS status (satellites, GNSS epoch, service ports)
- Raw NMEA logging toggle (saved in EEPROM)
- NTP holdover toggle (saved in EEPROM)
- Display enable toggle (saved in EEPROM)

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

## future plan: can be added into 
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

UTC identity valid?
    validDate
    validTime
    fullyResolved

Current GNSS solution valid?
    gnssFixOK
    fixType >= 2/3 as appropriate

Current timing source healthy?
    PPS arriving
    PPS interval reasonable
    recent valid GNSS information
```
- should i keep the COARSE correction (µs) even if PPS is present?
- Have 3 GNSS NAV-PVT time in microsecond with 60 second window interval GNSS1, GNSS2, GNSS3 , so for starting only it will take 3 min to set this variable, It will be in 3 holdover on 80mhz hardware timer clock, then with PPS keep these 3 values updated and avg them out by dividing 3 then use this time to server as NTP. Keep updating these values 
- option to force update GNSS NAV-PVT time from dashboard
- GNSS UTC Epoch (µs) on dashboard rather than direct update have avg for 60 second GNSS NAV-PVT . issue with this is dashboard value will update every 60 second. Can have holdover with eps milis()

## Project Structure

- [src/main.cpp](src/main.cpp): main application loop, OLED rendering, GNSS/NTP integration, PPS hardware capture (MCPWM)
- [lib/GnssMaxM10S/GnssMaxM10S.h](lib/GnssMaxM10S/GnssMaxM10S.h): GNSS parser/clock interface — hardware tick source config, ns-resolution PPS time query
- [lib/GnssMaxM10S/GnssMaxM10S.cpp](lib/GnssMaxM10S/GnssMaxM10S.cpp): GNSS UART polling, NAV-PVT UTC-second anchoring, PPS-assisted time interpolation
- [lib/NmeaServer/NmeaServer.h](lib/NmeaServer/NmeaServer.h): TCP NMEA broadcast server (port 10110)
- [lib/NmeaServer/NmeaServer.cpp](lib/NmeaServer/NmeaServer.cpp): NMEA server implementation
- [lib/NtpServer/NtpServer.h](lib/NtpServer/NtpServer.h): GNSS-backed NTP server (UDP port 123)
- [lib/NtpServer/NtpServer.cpp](lib/NtpServer/NtpServer.cpp): NTP server implementation
- [lib/WiFiSetupPortal/WiFiSetupPortal.h](lib/WiFiSetupPortal/WiFiSetupPortal.h): captive portal + GNSS dashboard interface
- [lib/WiFiSetupPortal/WiFiSetupPortal.cpp](lib/WiFiSetupPortal/WiFiSetupPortal.cpp): portal, credential storage, dashboard pages
- [platformio.ini](platformio.ini): ESP32 PlatformIO environment and dependencies
