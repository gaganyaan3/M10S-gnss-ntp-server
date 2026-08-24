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

After first satellite-backed time is seen since boot, optional holdover keeps
NTP responses available even if live satellite fix is temporarily lost.

#### PPS-Disciplined Clock (hardware capture + EMA calibration)

Time interpolation between PPS edges uses the MCPWM 80 MHz hardware capture
counter. The measured tick interval between consecutive PPS edges is fed into
an **exponential moving average (EMA, α = 0.1)** that continuously disciplines
the nominal 80 MHz rate against the GPS module's own 1 Hz PPS edge — correcting
for the ESP32 crystal's absolute frequency error (typically tens of ppm).

New time-query API added to `GnssMaxM10S`:

| Method | Returns |
|---|---|
| `currentUtcEpochPps(uint32_t&)` | Whole-second UTC epoch via PPS |
| `currentUtcEpochMsPps(uint64_t&)` | Millisecond UTC epoch via PPS |
| `currentUtcEpochNsPps(uint64_t&)` | **Nanosecond** UTC epoch via PPS (new) |
| `ppsCalibratedTicksPerSecond()` | Live EMA-calibrated tick rate (Hz) |
| `configurePpsHardwareClock(fn, hz)` | Inject custom tick source + nominal rate |

Serial log format per PPS pulse:

```
PPS pulse #N at <ticks> ticks | rawS: 1.000000000 s | calibrated: 0.999999987 s | fq: 80000001.1 Hz
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
```
## Project Structure

- [src/main.cpp](src/main.cpp): main application loop, OLED rendering, GNSS/NTP integration, PPS hardware capture (MCPWM)
- [lib/GnssMaxM10S/GnssMaxM10S.h](lib/GnssMaxM10S/GnssMaxM10S.h): GNSS parser/clock interface — hardware tick source config, ns-resolution PPS time query
- [lib/GnssMaxM10S/GnssMaxM10S.cpp](lib/GnssMaxM10S/GnssMaxM10S.cpp): GNSS UART polling, PPS-assisted time interpolation with EMA-calibrated 80 MHz tick rate
- [lib/NmeaServer/NmeaServer.h](lib/NmeaServer/NmeaServer.h): TCP NMEA broadcast server (port 10110)
- [lib/NmeaServer/NmeaServer.cpp](lib/NmeaServer/NmeaServer.cpp): NMEA server implementation
- [lib/NtpServer/NtpServer.h](lib/NtpServer/NtpServer.h): GNSS-backed NTP server (UDP port 123)
- [lib/NtpServer/NtpServer.cpp](lib/NtpServer/NtpServer.cpp): NTP server implementation
- [lib/WiFiSetupPortal/WiFiSetupPortal.h](lib/WiFiSetupPortal/WiFiSetupPortal.h): captive portal + GNSS dashboard interface
- [lib/WiFiSetupPortal/WiFiSetupPortal.cpp](lib/WiFiSetupPortal/WiFiSetupPortal.cpp): portal, credential storage, dashboard pages
- [platformio.ini](platformio.ini): ESP32 PlatformIO environment and dependencies
