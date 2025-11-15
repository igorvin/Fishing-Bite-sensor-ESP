# 🎣 Fishing Bite Sensor – ESP-NOW HUB (Beeper) 
Central receiver module(Beeper) for wireless fishing rod sensors using ESP-NOW on **Seeed Studio XIAO ESP32-S3**.

---

## 📌 Overview

The **ESP-NOW Hub** is the central unit of a multi-rod wireless bite detection system.  
Each fishing rod has an ESP32-based vibration sensor that sends an instant alert using **ESP-NOW** when motion is detected.

The Hub receives these alerts and displays:

- The **Rod ID** in large text  
- The **Hub's own MAC address**  
- The **Hub battery level** using an MP3-style 5-bar icon  
- Alert type (short or continuous vibration)

The entire system is designed to operate from battery power with **high efficiency**, **low latency**, and **zero Wi-Fi infrastructure**.

---

## ✨ Features

### 🧲 ESP-NOW Communication  
- Direct P2P wireless communication without router  
- Ultra-low latency (0–10 ms)  
- Compatible with multiple rod sensors

### 📟 OLED Display (SSD1306 128×64)  
Two display modes:

#### **Alert Mode**
- Rod name centered in large font  
- Display stays on for 5 seconds  
- Hub MAC shown in top-left  
- Battery icon shown in top-right

#### **Idle Mode**
- Shows “READY (ESP-NOW Hub)”  
- Shows Hub MAC  
- Battery indicator updated periodically

### 🔋 Battery Monitoring  
- Reads Li-Ion 3.7V battery through ADC and voltage divider  
- Converts voltage to percentage  
- Shows 5-segment MP3-style battery icon  
- Adjustable voltage curve for calibration

### ⚡ Power Saving  
- CPU frequency lowered to **80 MHz**  
- Wi-Fi **Modem-Sleep** enabled  
- OLED auto-dimming after 30 seconds  
- OLED auto-off after 60 seconds  
- Display wakes immediately on new alert

---

## 📡 Bite Packet Format

Rod sensors send the following structure:

```cpp
struct BitePacket {
  char    rodName[16];   // Rod identifier
  uint8_t eventType;     // 1 = short vibration, 2 = continuous
  uint8_t batteryPct;    // Rod battery percentage 0–100%
  float   deltaG;        // Vibration intensity
};
