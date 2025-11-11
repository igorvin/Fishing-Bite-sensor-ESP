# 🎣 Fishing Bite Sensor (ESP32-S3 + BMI160)

[Русская версия](./README_RU.md)

**Fishing Bite Sensor** is an intelligent motion/vibration detector for fishing rods, built on the **Seeed XIAO ESP32-S3** platform and a **BMI160 accelerometer**.  
It detects bites using acceleration magnitude changes and notifies with **LEDs and sound**.  
The system includes a **web configuration UI** using **SettingsGyver**, supports **persistent settings**, and allows switching between **English / Russian** languages.

---
<p align="center">
  <img src="./img/Fishing Bite Sensor_en.png" width="300" alt="Fishing Bite Sensor Web EN">
  <img src="./img/Fishing Bite Sensor_en 2.png" width="300" alt="Fishing Bite Sensor Web EN 2">
</p>


## 🧠 Features

- **Motion Detection**
  - Detects vibration or movement using the **BMI160 accelerometer**.
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

- **Deep Sleep Mode**
  - When disarmed, after a 3-minute grace period (or immediately if no Wi-Fi clients are connected),  
    the board enters **deep sleep** to preserve battery life.
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
    - Language selection (English / Русский)
  - Supports **persistent storage** via LittleFS.

- **Language Switching**
  - Full localization support for English and Russian.
  - Shows a message prompting to refresh the page after switching language.
  - Real-time status badge displays current language.

---

## 🧩 Hardware Components

| Component | Description | Notes |
|------------|--------------|-------|
| **ESP32-S3** | Seeed Studio XIAO ESP32-S3 | Main controller (Wi-Fi + BLE + low-power) |
| **Accelerometer** | BMI160 (DFRobot_BMI160 library) | Detects vibrations and acceleration changes |
| **LEDs** | Green and Red | Green = Armed, Red = Motion alert |
| **Buzzer** | Passive buzzer | Audio indication for events |
| **Button** | Single momentary button | Long press = Arm/Disarm |
| **Battery** | Li-ion / LiPo | Powers entire unit; ESP32-S3’s deep sleep minimizes consumption |

---

## 📦 Required Libraries

Make sure the following libraries are installed (PlatformIO or Arduino IDE):

| Library | Repository |
|----------|-------------|
| [**DFRobot_BMI160**](https://github.com/DFRobot/DFRobot_BMI160) | Accelerometer driver |
| [**SettingsGyver**](https://github.com/GyverLibs/Settings) | Web configuration UI |
| [**GyverDBFile**](https://github.com/GyverLibs/GyverDB) | Persistent storage (LittleFS) |
| [**ENCButton** (optional)](https://github.com/GyverLibs/EncButton) | For advanced button management |
| [**GyverLibs Core**](https://github.com/GyverLibs/) | Base dependency |

---

## ⚙️ Pinout (Seeed XIAO ESP32-S3)

| Function | Pin |
|-----------|-----|
| **LED Green** | D5 |
| **LED Red** | D6 |
| **Buzzer** | D8 |
| **Button** | D4 |
| **I²C SDA/SCL** | Internal (auto) |
| **Power** | 3.3V |
| **Ground** | GND |

---

## 🌐 Wi-Fi Configuration

- The ESP creates an **Access Point** using the configured sensor name (SSID) and password.
- Default SSID: `BiteSensor`
- Default password: *(empty)*
- You can open the web UI by visiting `http://192.168.4.1`.

---

## 🖥️ Web Interface Overview

| Section | Controls |
|----------|-----------|
| **General** | Language dropdown (English / Русский) |
| **Sensor** | Sensitivity (Δg), Pulse Duration, Pulse Period, Continuous Threshold, Armed Switch |
| **Wi-Fi Settings** | AP Name (SSID), Password, Save & Restart |
| **Status** | Real-time LEDs: Armed state, Accelerometer status, Active language, Δg value, Alerts count, Uptime |

---

## 💤 Deep Sleep Logic

- When disarmed:
  - Keeps the AP active for 3 minutes (configuration grace period).
  - If no Wi-Fi clients are connected, automatically enters **deep sleep**.
- Wake-up trigger: **Button press** (active LOW).

---

## 📘 Default Behavior Summary

| State | LED | Buzzer | Action |
|--------|-----|--------|--------|
| **Armed** | Green ON | 1 long beep | Ready for motion detection |
| **Disarmed** | Green OFF | 2 short beeps | Goes to deep sleep |
| **Motion Detected (short)** | Red blink | Short beep | Quick vibration |
| **Continuous Motion** | Red pulsing | Repeated beeps | Sustained vibration |

---

## 🔋 Power Efficiency

- Deep Sleep current: ~10–20 µA (depends on board)
- Active mode: ~60–80 mA (Wi-Fi enabled)
- Ideal for battery-powered, portable fishing setups.

---

## 🧪 Testing

1. Power the board → it starts in **Disarmed** mode.  
2. Long press the button → **Armed** (green LED ON).  
3. Shake the accelerometer or rod → buzzer + red LED pulse.  
4. Long press again → **Disarmed** → two short beeps → goes to sleep.

---

## 🧠 Developer Notes

- Built and tested with:
  - PlatformIO / Arduino Core v2.0.14+
  - C++17 enabled for fold-expression support
- Recommended `platformio.ini`:
  ```ini
  [env:seeed_xiao_esp32s3]
  platform = espressif32
  board = seeed_xiao_esp32s3
  framework = arduino
  build_unflags = -std=gnu++11
  build_flags   = -std=gnu++17
  monitor_speed = 115200
  lib_deps =
    DFRobot_BMI160
    GyverLibs/Settings
    GyverLibs/GyverDB
    GyverLibs/EncButton

📄 License
MIT License
Created by Igor Vinokur (2025)
