# 🎣 Fishing Bite Sensor – ESP-NOW HUB (Beeper)

Central receiver module (Beeper) for wireless fishing rod sensors using ESP-NOW on **Seeed Studio XIAO ESP32-S3**.

---

## 📌 Overview

The **ESP-NOW Hub** is the central unit of a multi-rod wireless bite detection system.  
Each fishing rod has an ESP32-based vibration sensor that sends an instant alert using **ESP-NOW** when motion is detected or when the battery is low.

The Hub receives these packets and displays:

- The **Rod ID** in large text  
- The **Hub's own MAC address**  
- The **Hub battery level** using an MP3-style 5-bar icon  
- **Alert type**:
  - Short vibration
  - Continuous vibration
  - Low-battery warning from the rod

The entire system is designed to operate from battery power with **high efficiency**, **low latency**, and **zero Wi-Fi infrastructure**.

---

## ✨ Features

### 🧲 ESP-NOW Communication

- Direct **point-to-point** wireless communication (no router/AP required)  
- Ultra-low latency (typically 0–10 ms)  
- Supports **multiple rod sensors** sending to a single Hub  
- Simple fixed packet format for easy decoding and expansion

---

### 📟 OLED Display (SSD1306 128×64)

Two main display states:

#### 🔔 Alert Mode

Triggered when a bite/alert packet is received:

- **Rod name (Rod ID)** is shown in the center in large font  
- **Alert stays on screen for a few seconds**  
- Hub MAC is shown in the top-left corner  
- Hub battery icon shown in the top-right corner  
- Optional differentiation by event type:
  - Short vibration
  - Continuous vibration
  - Low-battery from rod (can be displayed as a special message or icon)

#### 💤 Idle Mode

When no recent alerts:

- Shows `READY (ESP-NOW Hub)` or similar idle text  
- Shows Hub MAC address  
- Battery indicator updated periodically  
- Display is dimmed or turned off after a timeout to save power

---

### 🔋 Battery Monitoring (Hub)

- Hub is powered from a single-cell **Li-Ion 3.7V** battery  
- Voltage is measured via ADC and a **resistive divider**  
- Voltage is converted to approximate **battery percentage**  
- A **5-segment MP3-style icon** shows charge level:
  - 0–1 bars: low battery  
  - 2–3 bars: medium  
  - 4–5 bars: high  
- Voltage-to-% mapping is adjustable in firmware for calibration to your specific cell and load

---

### ⚡ Power Saving

The Hub is optimized for long battery life:

- CPU frequency lowered to **80 MHz**  
- Wi-Fi **Modem-Sleep / Light-Sleep** enabled where possible  
- OLED backlight / panel is:
  - **Dimmed** after ~30 seconds of inactivity  
  - **Turned off** after ~60 seconds of inactivity  
- Display wakes up immediately when:
  - New ESP-NOW packet (bite or low-battery) arrives  
  - A button is pressed (if you add one later)

---

## 📡 Bite Packet Format

Rod sensors send the following packet over ESP-NOW:

```cpp
struct BitePacket {
  char    rodName[16];   // Rod identifier / sensor name
  uint8_t eventType;     // 1 = short vibration
                         // 2 = continuous vibration
                         // 3 = low-battery warning (from rod)
  uint8_t batteryPct;    // Rod battery percentage 0–100%
  float   deltaG;        // Vibration intensity (Δg)
};
```

----

##🔎 Field Details

# rodName[16]
Human-readable name of the rod / sensor.
Usually matches the AP SSID / sensor name configured on the rod device.

# eventType
Encodes the type of event the rod reports:
- 1 – Short bite / first trigger
- 2 – Continuous vibration / prolonged motion
- 3 – Low battery on the rod device (no bite, just status)
# The Hub can:
Show different messages or icons based on eventType
For example:
- 1 – flash screen once
- 2 – keep flashing or beep multiple times
- 3 – show "LOW BAT" plus rod name

# batteryPct
Battery level of the rod sensor, already mapped to 0–100%.
Can be used by the Hub to:
- Show per-rod battery status
- Decide when to display warnings or log maintenance

# deltaG
Last measured vibration intensity in Δg (difference from baseline gravity).
Useful for debugging, tuning sensitivity, or showing how strong the bite was.

## 🔧 Hardware Summary

Hub MCU: Seeed Studio XIAO ESP32-S3
Display: 0.96" SSD1306 OLED, 128×64, I²C
Power: 3.7V Li-Ion cell (e.g., 18650 or flat LiPo)
Battery sensing:
 - Voltage divider (e.g., 2×100k + 100nF to GND)
 - Connected to ADC-capable pin of XIAO ESP32-S3
Optional beeper / LED:
 - Can be added to signal alerts with sound and/or light
 -Driven via transistor/MOSFET from a GPIO pin

##🔄 System Architecture
# 1.Rod Sensor (ESP32 + accelerometer)
     Measures vibration via IMU (e.g., LSM6DS3 / BMI160 / etc.)
     Detects short and continuous bites based on Δg and time
     Monitors its own battery
     Sends BitePacket via ESP-NOW to the Hub’s MAC

# 2.ESP-NOW Hub (this project)
  - Listens for BitePacket on ESP-NOW
  - On packet reception:
     - Parses rodName, eventType, batteryPct, deltaG
     - Updates OLED display accordingly
     - Optionally drives a buzzer/LED

##🧪 Testing & Debugging

Use Serial Monitor (115200 baud) to:
 - Print received packets
 - Show parsed fields (rod name, event type, battery, Δg)
 - Verify that low-battery alerts (eventType = 3) are received correctly
Use one rod sensor first to test range and reliability
Add more rods once basic communication is stable

📁 Project Structure (recommended)

hub/
  - src/main.cpp – ESP-NOW Hub firmware
  -  platformio.ini – PlatformIO config for XIAO ESP32-S3
  - README.md – This documentation

