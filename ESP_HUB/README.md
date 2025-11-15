# 🎣 Fishing Bite Sensor – Rod Unit (ESP-NOW Sensor)

Wireless **vibration-based bite detector** for fishing rods using **Seeed Studio XIAO ESP32-S3**, an accelerometer (e.g. LSM6DS3/BMI160), and **ESP-NOW** for instant communication with the central ESP-NOW Hub.

This device attaches to the fishing rod and detects both **short taps** and **continuous tension**, sending alerts with ultra-low latency and very low power consumption.

---

## 📌 Overview

The Rod Sensor continuously monitors vibrations from the fishing rod using an IMU (accelerometer).  
When motion is detected, it sends a structured packet via **ESP-NOW** to the central **Hub (Beeper)**.

The sensor supports:

- Short vibration detection  
- Continuous vibration detection  
- Low-battery warnings  
- Baseline auto-calibration  
- Test mode from the web UI  
- Deep-sleep when disarmed  
- Built-in Wi-Fi AP for configuration  

Designed to run for **days on a single battery charge** thanks to aggressive sleep logic and power optimization.

---

## ✨ Features

### 🧲 Vibration Detection (IMU-based)

- Works with **LSM6DS3** or **BMI160**  
- Measures Δg relative to a dynamic gravity baseline  
- Auto-baseline calibration when DISARMED  
- User-configurable parameters:
  - Sensitivity (Δg threshold)  
  - Short pulse duration  
  - Continuous vibration threshold  

---

### 📡 ESP-NOW Communication

Direct peer-to-peer communication with the Hub:

- No router required  
- Instant alert delivery (typically 0–10 ms)  
- Rod name included in every packet  
- Configurable Hub MAC via web UI  

The Rod Sensor sends the following **event types**:

| Event Type | Meaning               |
|------------|-----------------------|
| **1**      | Short / first vibration |
| **2**      | Continuous vibration  |
| **3**      | Low battery warning   |

---

### 🔋 Battery Monitoring

- Battery voltage is read using ADC + a 100k/100k divider and 100nF capacitor  
- Voltage is converted to approximate battery percentage (0–100%)  
- Status is periodically updated internally  
- When battery falls below a configured threshold, the sensor sends a **low-battery ESP-NOW event (eventType = 3)**  

---

### 🎛 Web Interface (Settings)

Available only when **DISARMED** via the built-in Access Point (AP).

Configurable options include:

- **Sensor name** (also used as `rodName` in packets / AP SSID)  
- **AP password**  
- **Sensitivity (Δg threshold)**  
- **Short pulse length** (red LED & buzzer)  
- **Continuous vibration threshold** (ms)  
- **LED brightness** (red & green)  
- **Buzzer volume**  
- **Language**: English / Русский  
- **ESP-NOW enable/disable**  
- **Hub MAC address** (AA:BB:CC:DD:EE:FF format)  
- **Test Mode button**:
  - Recalibrates baseline  
  - Generates a local short red LED + buzzer pulse  
  - Does **not** send ESP-NOW packets (for quiet testing)  

Built using:

- **SettingsGyver**  
- **GyverDBFile**  
- **LittleFS**  

---

### 🛡 Armed / Disarmed Modes

#### 🔓 DISARMED mode

- Web UI (settings portal) is active via Wi-Fi AP  
- Baseline auto-calibration is performed when entering DISARMED  
- After **3 minutes** with no clients connected → device enters **deep sleep**  
- LEDs and buzzer are OFF  
- **Long-press button** → switch to ARMED mode  

#### 🔒 ARMED mode

- Vibration detection logic is active  
- ESP-NOW is enabled if configured  
- Wi-Fi AP is ON only for the first **2 minutes** after arming, then turned OFF to save power  
- CPU frequency is reduced and Wi-Fi is mostly off for power saving  
- **Long-press button** → switch back to DISARMED mode  

---

### 🔕 Buzzer & LED Behavior

- All high-current loads (buzzer, LEDs) are driven via MOSFETs and controlled with PWM  
- **Long beep** when entering ARMED mode  
- **Two short beeps** when entering DISARMED mode  
- **Red LED flashes** on vibration alarm (short or continuous)  
- **Green LED ON** whenever the sensor is ARMED  

---

## ⚡ Power Saving Features

To maximize battery life:

- CPU frequency reduced to **80 MHz**  
- Wi-Fi disabled in ARMED mode after initial configuration window  
- Deep sleep in DISARMED mode when inactive  
- Efficient accelerometer sampling and Δg calculation  
- No OLED on sensor → very low standby current  

---

## 📡 Bite Packet Format

The Rod Sensor sends the following structure to the Hub via ESP-NOW:

```cpp
struct BitePacket {
  char    rodName[16];   // Rod identifier
  uint8_t eventType;     // 1 = short, 2 = continuous, 3 = low battery
  uint8_t batteryPct;    // Rod battery level %
  float   deltaG;        // Measured vibration intensity (Δg)
};

---
## Field Summary

rodName
Unique name of the rod/sensor, configured in the web UI. Also used as AP SSID.

eventType
Encodes the alert type:

1 – Short/first vibration
2 – Continuous vibration / prolonged motion
3 – Low battery warning

batteryPct
Battery level mapped to 0–100%.

deltaG
Last measured vibration intensity (difference from baseline gravity).

---

## 🔧 Hardware Summary

MCU: Seeed Studio XIAO ESP32-S3
Accelerometer: LSM6DS3 or BMI160 (I²C)
Power: 3.7V Li-Ion or LiPo battery (e.g., 18650 or flat cell)
Buzzer: driven via N-MOSFET with PWM from ESP32 pin
LEDs: Green & Red LEDs driven via N-MOSFETs (PWM capable pins)
Button: momentary push-button, long-press for ARM/DISARM
Battery ADC: 100k/100k resistor divider + 100nF capacitor to an ADC pin


---

## 🧪 Test Mode (Web UI)

When you press the “Calibrate baseline & test alarm” button in the web interface:
The sensor performs baseline recalibration (re-reads gravity vector and sets a new quiet level).
It triggers a local short alarm:
Brief buzzer beep
Short red LED flash
No ESP-NOW packet is sent → useful for quiet/bench testing without waking the Hub.

---

## 🔄 Auto Baseline Recalibration

Whenever the sensor transitions to DISARMED:
It reads ~50 IMU samples over a short window.
Computes the average magnitude of acceleration in g.
Stores this value as baselineMagG (quiet reference).
This significantly reduces false triggers when:
The position or angle of the rod changes.
The rod is moved to a different holder.
Boat tilt or wave direction changes.
The sensor is bumped during setup before arming.