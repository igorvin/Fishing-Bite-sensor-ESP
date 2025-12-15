# 🎣 Fishing Bite Sensor (ESP32-S3 + LSM6DS3)

[Русская версия](./README_RU.md)

**Fishing Bite Sensor** is an intelligent motion/vibration detector mounted on fishing rods, built on the **Seeed XIAO ESP32-S3** platform and an **LSM6DS3 accelerometer**.  
It detects bites using acceleration magnitude changes and notifies with **LEDs and sound**, and now can also send **wireless alerts** to a central **ESP-NOW Hub (Beeper)**.

The system includes a **web configuration UI** (SettingsGyver), supports **persistent settings**, allows switching between **English / Russian**, and uses **ESP-NOW** to report bite events and low battery to the hub.

---
<p align="center">
  <img src="./img/Fishing Bite Sensor_en.png" width="300" alt="Fishing Bite Sensor Web EN">
  <img src="./img/Fishing Bite Sensor_en 2.png" width="300" alt="Fishing Bite Sensor Web EN 2">
</p>

---

## 🧠 Features

- **Motion Detection**
  - Detects vibration or movement using the **LSM6DS3 accelerometer** (via Adafruit LSM6DS library).
  - Uses Δg (difference from baseline gravity) to detect bites.
  - Short pulse (buzzer + red LED) for brief vibration.
  - Repeated pulses for continuous motion.
  - **Automatic baseline calibration**:
    - On power-up.
    - When switching to **DISARMED** mode.
    - On demand via web UI (“Calibrate baseline & test alarm”).

- **ESP-NOW Wireless Alerts**
  - Sends bite events to a central **ESP-NOW Hub** (Beeper) without any Wi-Fi router.
  - **Configurable:** ESP-NOW enable/disable in the web UI.
  - Hub MAC address configurable as `AA:BB:CC:DD:EE:FF`.
  - Sends structured packets with:
    - Rod name (ID)
    - Event type (short, continuous, low battery)
    - Battery percentage
    - Δg (vibration intensity).

- **Low-Battery Warning (ESP-NOW)**
  - Periodically measures battery voltage and calculates %.
  - When battery drops below threshold, sends a **low-battery ESP-NOW event** to the hub.
  - Uses hysteresis to avoid spam (reports once per low-battery state).

- **Arming / Disarming**
  - Controlled by a **long button press** (~1 second).
  - **Green LED ON** → Armed (ready to detect bites).
  - **Green LED OFF** → Disarmed (configuration/sleep mode).
  - Web UI also has an **Armed/Disarmed toggle** (software).

- **Audio/Visual Alerts**
  - **Long beep** when entering ARMED mode.
  - **Two short beeps** when entering DISARMED mode.
  - **Red LED + buzzer** pulse on detected motion (short or continuous event).
  - Test mode from web UI:
    - Recalibrates baseline.
    - Makes a short local buzzer + red LED pulse.
    - Does **not** send ESP-NOW packet (quiet bench test).

- **Adjustable Buzzer Volume & LED Brightness**
  - **Buzzer volume** configurable from 0–100 % in the web UI.
  - **Red LED brightness** (alert) and **Green LED brightness** (armed state) are independently adjustable (0–100 %).
  - Implemented using ESP32 **PWM (ledc)** driving MOSFETs.

- **🧠 Auto-disarm (Anti-noise / Rod-moved detection)**
  - Prevents unnecessary alarm noise when the whole rod/stand is lifted, moved, or tilted (e.g. you pick up the rod).
  - Works by estimating rod orientation (pitch/roll) from the accelerometer gravity vector.
  - When enabled, if rod angle changes above threshold for a short hold time → the device automatically switches to **DISARMED**.
  - Includes **Smart-detection** mode that learns your real angles automatically each time you ARM.

- **🧠 Smart-detection + Auto calibration (Learn mode)**
  - **Default: Smart-detection ON**
  - Every time you ARM the sensor, it runs a short automatic learning sequence:
    - **10 seconds NORMAL**: keep rod in the usual “armed / resting” position.
    - **10 seconds MOVED**: move rod to the “disarm position” (lift and/or tilt).
  - During this 20-second learn window:
    - **Bite alarms are suppressed** (no buzzer noise, no false bite events).
  - After learning completes, the firmware auto-derives thresholds:
    - `UP (deg)` is derived from max pitch deviation
    - `TILT (deg)` is derived from max roll deviation
  - If no meaningful move is detected during the MOVED phase, the system falls back to safe defaults:
    - **UP = 25°**, **TILT = 35°**.

- **Deep Sleep Mode**
  - **DISARMED:**
    - Keeps AP active for a **3-minute grace period**.
    - If no Wi-Fi clients are connected → enters **deep sleep** to preserve battery.
  - Wakeup from deep sleep via **button press** (EXT0 wake).
  - **ARMED:**
    - AP is active only for the first **2 minutes** for quick configuration, then automatically turned off.
    - ESP-NOW can remain active with Wi-Fi radio in minimal mode.

- **Configuration Web UI**
  - Built on **GyverDBFile** + **SettingsGyver** with **LittleFS** storage.
  - Access via the ESP’s **Wi-Fi Access Point**.
  - Allows configuration of:
    - Sensor name (also **rod ID** and AP SSID).
    - AP password.
    - Sensitivity (Δg threshold).
    - Pulse duration & period.
    - Continuous motion threshold.
    - Armed/disarmed toggle.
    - **Auto-disarm (master switch)**.
    - **Smart-detection switch** (default ON).
      - If Smart OFF → manual thresholds appear:
        - **Auto-disarm UP (deg)**
        - **Auto-disarm TILT (deg)**
    - **Buzzer volume**.
    - **Red/Green LED brightness**.
    - **Language**: English / Русский.
    - **ESP-NOW enable switch**.
    - **Hub MAC address** (for ESP-NOW peer).
    - **Calibrate baseline & test alarm** button.

- **Language Switching**
  - Full localization for English and Russian.
  - Language stored in settings and applied to all UI labels.
  - Shows a notification to refresh the page after changing language.
  - Status area displays the currently active language.

- **Battery Monitoring**
  - Reads battery voltage via an ADC divider (100 kΩ / 100 kΩ + 100 nF).
  - Converts voltage to approximate **battery percentage** using a Li-ion curve.
  - Displays both **voltage** and **%** in the Status section.
  - LED color indicator: green / yellow / red depending on remaining charge.

---

<p align="center">
  <img src="./img/Auto Disarm functionality.png" width="300" alt="Auto Disarm functionality">
  </p>
  
## 🧠 Auto-disarm (Smart) — User Flow

This is the recommended usage (Smart-detection ON):

1. **ARM** the sensor (long press).
2. **Wait 10 seconds** (NORMAL learn phase): keep rod in resting position.
3. **Move the rod for 10 seconds** (MOVED learn phase):
   - Lift it upward (simulate taking rod out of holder), and/or
   - Tilt sideways (simulate changing the stand angle).
4. After that, Smart-detection becomes active and bite detection continues normally.

✅ Result: no manual tuning, and far fewer “false alarm noises” when moving the rod/stand.

---

## 🧠 Manual Auto-disarm Mode (if Smart-detection = OFF)

If Smart-detection is turned **OFF** in web UI, two manual lines appear:

- **Auto-disarm UP (deg)**
- **Auto-disarm TILT (deg)**

Notes:
- `0` disables that axis (0 = OFF).
- Manual mode still captures a baseline automatically once motion is stable after ARM.

---

## 📡 ESP-NOW Bite Packet Format

The sensor sends the following structure to the Hub:

```cpp
struct BitePacket {
  char    rodName[16];   // Rod identifier
  uint8_t eventType;     // 1 = short, 2 = continuous, 3 = low battery
  uint8_t batteryPct;    // Rod battery percentage 0–100%
  float   deltaG;        // Vibration intensity (Δg)
};

---

## 📡 Bite Packet Fields

### `rodName`
- Sensor name / rod ID (from web UI, also AP SSID).

### `eventType`
- `1` – Short/first vibration  
- `2` – Continuous vibration (longer motion)  
- `3` – Low-battery warning  

### `batteryPct`
- Battery level **0–100 %**.

### `deltaG`
- Difference from baseline magnitude, used to quantify bite intensity.

✅ ESP-NOW functionality uses the built-in ESP32 **`esp_now`** API — no extra library required.

---

## 🧩 Hardware Components

| Component          | Description                     | Notes                                                   |
|-------------------|----------------------------------|---------------------------------------------------------|
| **ESP32-S3**      | Seeed Studio XIAO ESP32-S3       | Main controller (Wi-Fi, BLE, ESP-NOW, deep sleep)       |
| **Accelerometer** | LSM6DS3 (GY-LSM6DS3 module)      | Detects vibrations and acceleration magnitude changes + tilt |
| **LEDs**          | Green and Red + MOSFET drivers   | Green = Armed, Red = Motion alert, brightness via PWM   |
| **Buzzer**        | Passive buzzer + MOSFET driver   | Audio indication for events, volume via PWM             |
| **Button**        | Single momentary button          | Long press = Arm/Disarm, also wake from deep sleep      |
| **Battery**       | Li-ion / LiPo                    | Powers the unit; deep sleep minimizes consumption       |
| **Voltage Divider** | 2×100 kΩ + 100 nF              | For battery voltage measurement on ADC pin              |

---

## 📦 Required Libraries

Make sure the following libraries are installed (PlatformIO or Arduino IDE):

| Library                       | Repository / Notes |
|------------------------------|--------------------|
| **Adafruit LSM6DS**          | `adafruit/Adafruit LSM6DS` – IMU driver (LSM6DS3/LSM6DSOX family) |
| **Adafruit Unified Sensor**  | `adafruit/Adafruit Unified Sensor` – required by Adafruit IMU |
| **Adafruit BusIO**           | `adafruit/Adafruit BusIO` – required by Adafruit IMU |
| **SettingsGyver**            | Web configuration UI builder |
| **GyverDBFile (GyverDB)**    | Persistent storage on LittleFS |
| **ENCButton (optional)**     | For more advanced button handling (not required in current firmware) |
| **GyverLibs Core**           | Base dependency collection |

> ESP-NOW is part of the **ESP32 Arduino core**, no separate library is required.

---

## ⚙️ Pinout (Seeed XIAO ESP32-S3)

| Function            | Pin                    | Notes |
|--------------------|------------------------|-------|
| **LED Green (PWM)**| D1 (GPIO2)             | MOSFET gate, brightness via PWM |
| **LED Red (PWM)**  | D2 (GPIO3)             | MOSFET gate, brightness via PWM |
| **Buzzer (PWM)**   | D8 (GPIO9)             | MOSFET gate, volume via PWM |
| **Button**         | D4 (GPIO4)             | Active LOW, wake from deep sleep (EXT0) |
| **I²C SDA**        | D4 (GPIO4)             | Connected to GY-LSM6DS3 SDA (per schematic) |
| **I²C SCL**        | D5 (GPIO5)             | Connected to GY-LSM6DS3 SCL (per schematic) |
| **VBAT sense**     | D0 (GPIO1 / ADC1_CH0)  | Through 100 kΩ / 100 kΩ divider + 100 nF to GND |
| **Power**          | 3.3 V                  | From regulator / battery system |
| **Ground**         | GND                    | Common ground |

> Adjust SDA/SCL in code if your PCB uses different pins.

---

## 🌐 Wi-Fi Configuration

The ESP creates an **Access Point** using the configured Sensor name as SSID and configured password.

- **Default SSID:** `BiteSensor`  
- **Default password:** *(empty)*  
- Open web UI at: `http://192.168.4.1`

### AP Behavior in Modes

#### DISARMED
- AP is always ON during the **3-minute grace period**.
- If no client remains connected → device enters **deep sleep**.

#### ARMED
- AP remains ON for **2 minutes** after arming for quick adjustments.
- After that it is disabled to save power; **ESP-NOW can still operate**.

---

## 🖥️ Web Interface Overview

| Section         | Controls / Info |
|----------------|------------------|
| **General**     | Language dropdown (English / Русский), Buzzer volume slider, Red LED brightness slider, Green LED brightness slider, “Calibrate baseline & test alarm” button |
| **Sensor**      | Sensitivity (Δg), Short pulse duration, Pulse period, Continuous motion threshold, Armed/Disarmed switch, Auto-disarm switch, Smart-detection switch *(manual UP/TILT degrees appear only when Smart-detection is OFF)* |
| **ESP-NOW**     | ESP-NOW enable switch, Hub MAC address input |
| **Wi-Fi Settings** | Sensor name (AP SSID), AP password, Save & Restart button (reboots to apply new SSID/password) |
| **Status**      | Armed state, Accelerometer status, Active language, Battery level (%, V, color), Δg current value, Alerts counter, Uptime (seconds) |

---

## 💤 Deep Sleep Logic

### In DISARMED mode
- AP is active for **3 minutes** after boot or disarm.
- If `WiFi.softAPgetStationNum() == 0` and grace period expired → enter **deep sleep**.

### Wake up from deep sleep
- Button press (active-LOW, EXT0 wake).

### In ARMED mode
- AP is turned off after **2 minutes**.
- ESP-NOW stays available (if enabled), or Wi-Fi can be completely turned off if ESP-NOW is disabled.

---

## 📘 Default Behavior Summary

| State | LED(s) | Buzzer | Notes |
|------|--------|--------|------|
| **Power-up / DISARMED** | Green OFF | 2 short beeps on disarm | AP ON, can configure |
| **ARMED** | Green ON (PWM) | 1 long beep | Motion detection active |
| **ARMED (Smart Learn)** | Green ON (PWM) | short beeps at learn markers | First 20s: 10s normal + 10s moved; bite alarms suppressed |
| **Short vibration** | Red flash (PWM) | Short beep | Sends ESP-NOW `eventType = 1` |
| **Continuous vibration** | Repeated red flashes | Periodic beeps | Sends ESP-NOW `eventType = 2` |
| **Rod moved / picked up** | Green OFF | Two short beeps (disarm) | Auto-disarm triggers to avoid noise |
| **Low battery (threshold)** | Status shows low %, red color | (Optional) same as normal use | Sends ESP-NOW `eventType = 3` |
| **DISARMED + idle** | Green OFF | Silent | After 3 min → deep sleep |

---

## 🔋 Power Efficiency

- **Deep sleep current:** depends on XIAO ESP32-S3 and peripherals (~tens of µA typical).
- **Active mode:** controlled by CPU frequency (80 MHz) and Wi-Fi/ESP-NOW usage.
- PWM brightness and buzzer volume can be reduced at night to save power and noise.

---

## 🧪 Testing Scenario

1. Power the board → it starts (according to stored state), usually in **DISARMED** with AP active.
2. Open Wi-Fi list on phone, connect to sensor AP (e.g., `BiteSensor`).
3. Open `http://192.168.4.1` → configure:
   - Name, password, sensitivity, ESP-NOW, Hub MAC, etc.
4. Press “Calibrate baseline & test alarm”:
   - Sensor recalibrates baseline.
   - Short local beep + red LED flash.
5. Long-press button → **ARMED**:
   - Green LED ON, long beep.
   - If Smart-detection ON:
     - Wait 10s (NORMAL), then move rod for 10s (MOVED).
     - Bite alarms suppressed during these 20 seconds.
   - AP available for 2 minutes, then turns off.
6. Move/shake the rod:
   - Short or continuous pulses on buzzer + red LED.
   - ESP-NOW packets sent to hub.
7. Pick up the rod / tilt the stand strongly:
   - Auto-disarm triggers → device switches to DISARMED to prevent noise.
8. Long-press again → **DISARMED**:
   - Two short beeps, AP ON again.
   - After 3 minutes of inactivity → deep sleep.


## 🧠 Developer Notes

### Built and tested with:

   - PlatformIO

   - ESP32 Arduino Core 2.x (XIAO ESP32-S3)

### Uses LittleFS for persistent DB (/config.db).

  - Example platformio.ini:

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
