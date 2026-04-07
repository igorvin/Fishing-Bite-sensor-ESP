# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Flash Commands

All commands run via PlatformIO CLI (`pio`) from the project root.

**Sensor firmware** (root project):
```bash
# Build (default env: xiao_esp32c3)
pio run

# Build specific board
pio run -e xiao_esp32c3
pio run -e xiao_esp32c6
pio run -e seeed_xiao_esp32s3

# Upload firmware
pio run -e xiao_esp32c3 --target upload

# Upload filesystem (LittleFS) — run after firmware upload
pio run -e xiao_esp32c3 --target uploadfs

# Serial monitor
pio device monitor --baud 115200

# Clean build
pio run --target clean
```

**Hub firmware** (`ESP_HUB/` sub-project):
```bash
cd ESP_HUB
pio run -e xiao_esp32c3 --target upload
```

## Project Architecture

This is a **two-device ESP-NOW system**:

### Device 1 — Bite Sensor (`src/main.cpp`)
Mounted on a fishing rod. Detects bites via LSM6DS3 accelerometer and sends wireless alerts.

Key subsystems in `main.cpp` (single-file firmware):
- **IMU sampling loop** — reads LSM6DS3 every 10 ms, computes Δg (delta from calibrated baseline gravity magnitude), triggers short/continuous bite events
- **Auto-disarm / Smart-detection** — estimates pitch/roll from gravity vector; on ARM runs a 20-second learn phase (10s normal + 10s moved) to auto-derive tilt thresholds; suppresses bite alarms during learning
- **ESP-NOW TX** — sends `BitePacket` structs (`rodName`, `eventType`, `batteryPct`, `deltaG`) to a configured hub MAC address; no router required
- **Web config UI** — GyverDBFile (LittleFS `/config.db`) + SettingsGyver serves a Wi-Fi AP at `192.168.4.1`; all settings (sensitivity, ESP-NOW, LED brightness, language, etc.) persist across reboots
- **Deep sleep** — DISARMED: 3-min AP grace → sleep if no clients; ARMED: AP off after 2 min, ESP-NOW stays active; wakeup via EXT0 button
- **Battery monitor** — ADC via 100kΩ/100kΩ divider on VBAT_PIN, Li-ion curve → %; sends `eventType=3` ESP-NOW packet when below 20%
- **PWM outputs** — buzzer volume and LED brightness via `ledc` driving MOSFETs

### Device 2 — Hub / Beeper (`ESP_HUB/src/main.cpp`)
Central receiver. Displays alerts on a 1.3" SH1106 OLED, drives a vibration motor.

Key subsystems:
- **ESP-NOW RX** — always-on receiver, parses `BitePacket`
- **OLED display** — alert screen (rod name + event label + progress bar) → idle "WAITING..." + MAC; dims after 1 min, off after 2 min
- **Green LED heartbeat** — solid ON normally, blinks every 10s when OLED is off

### Shared packet format
```cpp
struct BitePacket {
  char    rodName[16];  // AP SSID / sensor name
  uint8_t eventType;    // 1=short, 2=continuous, 3=low battery
  uint8_t batteryPct;
  float   deltaG;
};
```

## Pin Mapping

Pins are defined as Arduino `D`-aliases for XIAO boards. Current mapping in `src/main.cpp` (C3 layout):

| Signal | Pin |
|---|---|
| LED Green | D7 |
| LED Red | D8 |
| Buzzer | D9 |
| Button (arm/wake) | D6 |
| Power ctrl | D0 |
| Button input | D1 |
| VBAT ADC | D2 |
| I²C SDA | D4 |
| I²C SCL | D5 |

> Pin assignments differ between ESP32-S3, C6, and C3 boards. Always cross-check `#define` block at the top of `main.cpp` when targeting a new board.

## LittleFS Filesystem

Settings persist in `/config.db` on LittleFS. After changing `board_build.filesystem` or on a fresh board, upload the filesystem partition separately with `--target uploadfs`. The GyverDB schema is defined inline in `main.cpp` — adding new keys requires a re-upload of firmware only (GyverDB handles schema migration automatically).

## Board Variants

The `platformio.ini` `default_envs` is `xiao_esp32c3`. The C6 env uses a pioarduino platform ZIP (not the standard espressif32 registry) to get newer IDF support. The S3 env uses standard espressif32.
