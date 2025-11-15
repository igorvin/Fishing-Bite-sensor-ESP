# 🎣 Fishing Bite Sensor (ESP32-S3 + LSM6DS3)

[Русская версия](./README_RU.md)

**Fishing Bite Sensor** is an intelligent motion/vibration detector for fishing rods, built on the **Seeed XIAO ESP32-S3** platform and an **LSM6DS3 accelerometer**.  
It detects bites using acceleration magnitude changes and notifies with **LEDs and sound**.  
The system includes a **web configuration UI** using **SettingsGyver**, supports **persistent settings**, and allows switching between **English / Russian** languages.

---
<p align="center">
  <img src="./img/Fishing Bite Sensor_en.png" width="300" alt="Fishing Bite Sensor Web EN">
  <img src="./img/Fishing Bite Sensor_en 2.png" width="300" alt="Fishing Bite Sensor Web EN 2">
</p>


## 🧠 Features

- **Motion Detection**
  - Detects vibration or movement using the **LSM6DS3 accelerometer** (via Adafruit LSM6DS library).
  - Short beep + red LED flash for brief vibration.
  - Continuous pulse signals when motion persists.

- **Arming / Disarming**
  - Controlled by a **long button press** (1 second).
  - **Green LED ON** → Armed.
  - **Green LED OFF** → Disarmed (power-saving).

- **Audio/Visual Alerts**
  - **Long beep** when armed.
  - **Two short beeps** when disarmed.
  - **Red LED + buzzer** pulse on detected motion.

- **Adjustable Buzzer Volume & LED Brightness**
  - **Buzzer volume** can be configured from 0–100 % via a slider in the web UI.
  - **Red LED brightness** (alert) and **Green LED brightness** (armed state) are independently adjustable (0–100 %) via sliders.
  - Implemented using ESP32 **PWM (ledc)** driving MOSFETs, as in the hardware schematic.

- **Deep Sleep Mode**
  - When disarmed, after a **3-minute grace period**,  
    if no Wi-Fi clients are connected, the board enters **deep sleep** to preserve battery life.
  - Wakes up when the button is pressed.

- **Configuration Web UI**
  - Built on **GyverDBFile** + **SettingsGyver**.
  - Access via the ESP’s **Wi-Fi Access Point**.
  - Allows configuration of:
    - Sensor name (AP SSID)
    - AP password
    - Sensitivity (Δg threshold)
    - Pulse duration & period
    - Continuous motion threshold
    - Armed/disarmed toggle
    - **Buzzer volume**
    - **Red/Green LED brightness**
    - Language selection (English / Русский)
  - Supports **persistent storage** via LittleFS.

- **Language Switching**
  - Full localization support for English and Russian.
  - Shows a message prompting to refresh the page after switching language.
  - Status area displays the current language.

- **Battery Monitoring**
  - Reads battery voltage via an ADC divider (100 kΩ / 100 kΩ + 100 nF).
  - Converts voltage to an approximate **battery percentage** and shows it in the Status section with a color indicator (green / yellow / red).

---

## 🧩 Hardware Components

| Component | Description | Notes |
|----------|-------------|-------|
| **ESP32-S3** | Seeed Studio XIAO ESP32-S3 | Main controller (Wi-Fi + BLE + low-power) |
| **Accelerometer** | LSM6DS3 (GY-LSM6DS3 module) | Detects vibrations and acceleration changes |
| **LEDs** | Green and Red + MOSFET drivers | Green = Armed, Red = Motion alert, brightness via PWM |
| **Buzzer** | Passive buzzer + MOSFET driver | Audio indication for events, volume via PWM |
| **Button** | Single momentary button | Long press = Arm/Disarm, wake from deep sleep |
| **Battery** | Li-ion / LiPo | Powers entire unit; ESP32-S3’s deep sleep minimizes consumption |
| **Voltage Divider** | 2×100 kΩ + 100 nF | For battery voltage measurement on ADC pin |

---

## 📦 Required Libraries

Make sure the following libraries are installed (PlatformIO or Arduino IDE):

| Library | Repository / Notes |
|--------|---------------------|
| **Adafruit LSM6DS** | `adafruit/Adafruit LSM6DS` – IMU driver (LSM6DS3/LSM6DSOX family) |
| **Adafruit Unified Sensor** | `adafruit/Adafruit Unified Sensor` – required by Adafruit IMU |
| **Adafruit BusIO** | `adafruit/Adafruit BusIO` – required by Adafruit IMU |
| [**SettingsGyver**](https://github.com/GyverLibs/Settings) | Web configuration UI |
| [**GyverDBFile (GyverDB)**](https://github.com/GyverLibs/GyverDB) | Persistent storage on LittleFS |
| [**ENCButton** (optional)](https://github.com/GyverLibs/EncButton) | For more advanced button handling (not required in current version) |
| [**GyverLibs Core**](https://github.com/GyverLibs/) | Base dependency collection |

---

## ⚙️ Pinout (Seeed XIAO ESP32-S3)

| Function | Pin | Notes |
|---------|-----|-------|
| **LED Green (PWM)** | D1 (GPIO2) | MOSFET gate, brightness via PWM |
| **LED Red (PWM)** | D2 (GPIO3) | MOSFET gate, brightness via PWM |
| **Buzzer (PWM)** | D8 (GPIO9) | MOSFET gate, volume via PWM |
| **Button** | D4 (GPIO4) | Active LOW, wake from deep sleep |
| **I²C SDA** | D4 (GPIO4) | Connected to GY-LSM6DS3 SDA (per schematic) |
| **I²C SCL** | D5 (GPIO5) | Connected to GY-LSM6DS3 SCL (per schematic) |
| **VBAT sense** | D0 (GPIO1 / ADC1_CH0) | Through 100 kΩ / 100 kΩ divider + 100 nF to GND |
| **Power** | 3.3 V | From regulator / battery system |
| **Ground** | GND | Common ground |

*(If you wire SDA/SCL differently, adjust in code or board schematic accordingly.)*

---

## 🌐 Wi-Fi Configuration

- The ESP creates an **Access Point** using the configured sensor name (SSID) and password.
- Default SSID: `BiteSensor`
- Default password: *(empty)*
- You can open the web UI by visiting `http://192.168.4.1`.

---

## 🖥️ Web Interface Overview

| Section | Controls |
|--------|----------|
| **General** | Language dropdown (English / Русский), **Buzzer volume slider**, **Red LED brightness slider**, **Green LED brightness slider** |
| **Sensor** | Sensitivity (Δg), Pulse Duration, Pulse Period, Continuous Threshold, Armed/Disarmed switch |
| **Wi-Fi Settings** | AP Name (SSID), Password, Save & Restart (reboots to apply Wi-Fi changes) |
| **Status** | Armed state, Accelerometer status, Active language, Battery level (%, V, color), Δg value, Alerts count, Uptime (seconds) |

---

## 💤 Deep Sleep Logic

- When **Disarmed**:
  - Keeps the AP active for a **3-minute configuration grace period**.
  - If **no Wi-Fi clients** are connected after that period, automatically enters **deep sleep**.
- Wake-up trigger: **Button press** (active LOW, EXT0 wake).

---

## 📘 Default Behavior Summary

| State | LED | Buzzer | Action |
|-------|-----|--------|--------|
| **Armed** | Green ON (brightness from slider) | 1 long beep (volume from slider) | Ready for motion detection |
| **Disarmed** | Green OFF | 2 short beeps | Enters deep sleep after grace period if no clients |
| **Motion Detected (short)** | Red blink (brightness from slider) | Short beep | Quick vibration |
| **Continuous Motion** | Red pulsing | Repeated beeps | Sustained vibration |

---

## 🔋 Power Efficiency

- Deep Sleep current: ~10–20 µA (depends on board and peripherals).
- Active mode: ~60–80 mA (Wi-Fi enabled, IMU active).
- PWM-driven LEDs and buzzer allow reducing brightness/volume to save extra power at night.

---

## 🧪 Testing

1. Power the board → it starts in **Disarmed** mode.  
2. Long press the button → **Armed** (green LED ON).  
3. Shake the accelerometer or rod → buzzer + red LED pulse.  
4. Adjust buzzer volume and LED brightness in the **General** tab of the web UI and test again.  
5. Long press again → **Disarmed** → two short beeps → after 3 minutes (if no clients) the device goes to deep sleep.

---

## 🧠 Developer Notes

- Built and tested with:
  - PlatformIO
  - ESP32 Arduino Core 2.x

Example `platformio.ini`:

```ini
[platformio]
default_envs = seeed_xiao_esp32s3

[env:seeed_xiao_esp32s3]
platform  = espressif32
board     = seeed_xiao_esp32s3
framework = arduino

monitor_speed = 115200
upload_speed  = 921600

; Use LittleFS for filesystem (config.db, etc.)
board_build.filesystem = littlefs

; Native USB CDC on XIAO ESP32-S3
build_flags =
  -D ARDUINO_USB_MODE=1
  -D ARDUINO_USB_CDC_ON_BOOT=1

lib_deps =
  gyverlibs/GyverDB               ; GyverDB + GyverDBFile
  gyverlibs/Settings              ; SettingsGyver / SettingsESP
  adafruit/Adafruit LSM6DS        ; IMU (LSM6DS3 / LSM6DS33 / LSM6DSOX)
  adafruit/Adafruit Unified Sensor
  adafruit/Adafruit BusIO
