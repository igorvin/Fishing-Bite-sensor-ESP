# 🎣 Fishing Bite Sensor – ESP-NOW HUB

Central receiver module for wireless fishing rod sensors using **ESP-NOW** on **Seeed Studio XIAO ESP32-S3**.

---

## 📌 Overview

The **ESP-NOW Hub** is the central unit of a multi-rod wireless bite detection system.
Each fishing rod has its own ESP32-based sensor with an accelerometer / vibration detection logic. When a bite is detected, the rod sends an instant packet using **ESP-NOW**.

The Hub receives these packets and provides immediate local indication using:

- **OLED display**
- **Green status LED**
- **Vibration motor**

The Hub is designed for:

- **battery-powered outdoor use**
- **low latency**
- **low power consumption**
- **operation without Wi-Fi router or internet**

---

## ✨ Main Functionality

### 🧲 ESP-NOW Communication

- Direct wireless communication with rod sensors
- No router or access point required
- Very low latency
- Supports multiple rod sensors sending to one Hub
- Receives alerts even when the OLED display is off

---

### 📟 OLED Display

The Hub uses a **128×64 I2C OLED** display.
In the current firmware the display is intended for **SH1106** modules and supports auto-detection of common I2C addresses.

#### Alert Screen
When a packet is received, the Hub shows:

- **Rod name / rod ID** in large centered text
- **Event label** in the header
- **Hub battery icon**
- **Progress bar** showing the alert display timeout

#### Idle Screen
When there is no active alert, the Hub shows:

- `WAITING...`
- Hub MAC address
- Battery icon

#### OLED Power Saving
To save battery:

- after **1 minute** of inactivity → display goes to **dim mode**
- after **2 minutes** of inactivity → display turns **OFF**
- on any new ESP-NOW alert → display wakes immediately

> ESP-NOW remains active even when the display is off.

---

### 🟢 Green Status LED

A green LED is used as a simple “alive” indicator.

Behavior:

- **Normal mode / dim mode** → LED is **solid ON**
- **Display OFF mode** → LED **blinks once every 10 seconds**

This allows the user to confirm that the Hub is still powered and operating even when the OLED is off.

---

### 📳 Vibration Motor Alert

A miniature vibration motor is connected to the Hub and activated when an alert is received.

Motor behavior depends on the event type:

- **eventType = 1** → short vibration pulse
- **eventType = 2** → longer vibration pulse
- **eventType = 3** → double short pulse for low-battery warning from a rod

The motor is driven through a **MOSFET transistor stage** and must **not** be connected directly to the ESP32 GPIO.

---

### 🔋 Battery Monitoring (Hub)

The Hub measures its own battery voltage using an ADC input and resistor divider.

Current hardware concept:

- single-cell **Li-Ion / 18650** battery
- resistor divider: **100k / 100k**
- ADC filtering capacitor recommended near the XIAO ADC input

Battery information is shown on-screen using a **5-segment battery icon**.

---

## 📡 Packet Format

The Hub expects rod sensors to send the following structure:

```cpp
struct BitePacket {
  char    rodName[16];
  uint8_t eventType;     // 1 = BITE, 2 = RUN, 3 = LOW BAT
  uint8_t batteryPct;    // rod battery percentage
  float   deltaG;        // vibration intensity
};
```

### Field Description

#### `rodName`
Human-readable rod or sensor name.

#### `eventType`
Defines the meaning of the alert:

- `1` = short bite / first trigger
- `2` = continuous run / prolonged motion
- `3` = low battery warning from rod sensor

#### `batteryPct`
Battery level of the rod sensor.

#### `deltaG`
Measured vibration level used for debugging and tuning.

---

## ⚙️ Alert Handling Logic

When the Hub receives an ESP-NOW packet:

1. Packet is decoded
2. OLED wakes if needed
3. Rod name and event type are shown on screen
4. Vibration motor pattern is triggered
5. Alert stays visible for a few seconds
6. Hub returns to idle screen

---

## ⚡ Power Saving Strategy

The Hub keeps **ESP-NOW active at all times**.
It does **not** use deep sleep in normal operation, because deep sleep would stop ESP-NOW reception.

Power saving is achieved by:

- lowering CPU frequency
- enabling Wi-Fi sleep where possible
- dimming the OLED after inactivity
- fully turning off the OLED after longer inactivity
- using LED heartbeat indication instead of leaving the display on

This allows the Hub to remain responsive while still reducing battery consumption.

---

## 🔧 Hardware Summary

### Main Parts

- **Seeed Studio XIAO ESP32-S3**
- **SH1106 OLED display, 128×64, I2C**
- **Single-cell Li-Ion / 18650 battery**
- **Green status LED**
- **Mini vibration motor (7 mm type)**
- **AO3400A MOSFET drivers** for LED and motor

### Recommended Supporting Components

- flyback diode across motor
- bulk capacitor near motor supply
- ceramic capacitor near motor supply
- resistor divider for battery ADC
- ADC filter capacitor near XIAO ADC input

---

## 🧩 Driver Hardware Notes

### Vibration Motor Driver

Recommended implementation:

- motor powered from battery rail
- low-side MOSFET switching
- flyback diode across motor
- bulk + ceramic capacitor near motor

### LED Driver

- low-side MOSFET switching
- current-limiting resistor for the LED

---

## 🧪 Testing & Debugging

Use the serial monitor to verify:

- Hub MAC address
- received rod name
- event type
- rod battery percentage
- `deltaG` value

Suggested test sequence:

1. Power the Hub
2. Confirm OLED shows idle screen
3. Wait until OLED turns off and confirm green LED heartbeat
4. Trigger a rod sensor
5. Confirm OLED wakes instantly
6. Confirm vibration motor activates
7. Confirm correct rod name is shown

---

## 📁 Project Structure

```text
hub/
  src/main.cpp        # ESP-NOW Hub firmware
  platformio.ini      # PlatformIO configuration
  README.md           # Project documentation
```

---

## 🛠 Current Functional Summary

The current ESP-HUB firmware supports:

- ESP-NOW packet reception
- OLED alert screen with rod name
- idle screen with battery and MAC
- battery monitoring
- OLED dim mode after inactivity
- OLED off mode after inactivity
- green LED normal/alive indication
- green LED heartbeat when display is off
- vibration motor alert on incoming bite events
- different motor patterns by event type

---

## 🚀 Planned / Optional Future Improvements

- button-based wake or menu handling
- per-rod alert history
- configurable motor patterns
- sound buzzer output
- wireless settings interface
- battery warning for the Hub itself

---

## 👤 Project

Fishing Bite Sensor – ESP-NOW HUB
Designed for portable fishing bite notification with local visual and vibration alerts.
