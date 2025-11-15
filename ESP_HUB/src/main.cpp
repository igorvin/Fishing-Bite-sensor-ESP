/*******************************************************
  ESP-NOW Hub for Fishing Bite Sensor
  - Receives BitePacket from rod nodes (ESP32)
  - Shows rodName (rod ID) full-screen on OLED
  - Shows HUB MAC + battery % with MP3-style icon
  - Battery friendly: CPU 80 MHz, WiFi sleep, OLED dim/off
********************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =======================
// Display config (SSD1306 128x64 I2C)
// =======================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1      // shared reset
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// =======================
// Battery ADC config (HUB)
// Adjust pin & divider to your board
// =======================
#define VBAT_PIN        34        // <--- CHANGE if your ADC pin is different
const float    VBAT_VREF      = 3.30f;  // ADC reference
const float    VBAT_DIV       = 2.00f;  // 100k/100k divider
const float    VBAT_CAL       = 1.00f;  // calibration factor
const uint16_t VBAT_ADC_MAX   = 4095;   // 12-bit ADC
const uint16_t VBAT_SAMPLES   = 16;
const uint32_t VBAT_PERIOD_MS = 5000;   // read every 5 s

float    g_vbat_V   = 0.0f;
int      g_battPct  = 0;
uint32_t lastBatMs  = 0;

// =======================
// Packet from sensor (must match rod firmware)
// =======================
struct __attribute__((packed)) BitePacket {
  char    rodName[16];
  uint8_t eventType;     // 1 = short / first, 2 = continuous
  uint8_t batteryPct;    // rod battery, 0..100 %
  float   deltaG;        // Δg at moment of event
};

// =======================
// State
// =======================
volatile bool       g_hasAlert = false;
volatile BitePacket g_lastPkt;

uint32_t lastAlertMs = 0;
const uint32_t ALERT_HOLD_MS = 5000;  // show rod ID for 5 sec

String g_macStr;   // hub MAC string

// OLED power state
enum OledState : uint8_t { OLED_ON, OLED_DIM, OLED_OFF };
OledState g_oledState = OLED_ON;

const uint32_t OLED_DIM_TIMEOUT_MS = 30000;  // 30 s → dim
const uint32_t OLED_OFF_TIMEOUT_MS = 60000;  // 60 s → off
uint32_t lastActivityMs = 0;

// =======================
// Battery helpers
// =======================
float readVbat() {
  uint32_t acc = 0;
  analogRead(VBAT_PIN);        // dummy read to settle
  for (int i = 0; i < VBAT_SAMPLES; i++) {
    acc += analogRead(VBAT_PIN);
  }
  float adc = acc / float(VBAT_SAMPLES);
  float v = (adc / VBAT_ADC_MAX) * VBAT_VREF * VBAT_DIV * VBAT_CAL;
  return v;
}

// Rough Li-ion % curve (similar to rod firmware)
int vbatToPercent(float v) {
  if      (v >= 4.18f) return 100;
  else if (v >= 4.10f) return 90;
  else if (v >= 3.98f) return 80;
  else if (v >= 3.85f) return 70;
  else if (v >= 3.80f) return 60;
  else if (v >= 3.75f) return 50;
  else if (v >= 3.70f) return 40;
  else if (v >= 3.63f) return 30;
  else if (v >= 3.55f) return 20;
  else if (v >= 3.45f) return 10;
  else                 return 0;
}

// =======================
// ESP-NOW callback
// =======================
void onEspNowRecv(const uint8_t *mac, const uint8_t *data, int len) {
  (void)mac;
  if (len != sizeof(BitePacket)) return;
  memcpy((void*)&g_lastPkt, data, sizeof(BitePacket));
  g_hasAlert = true;
}

// =======================
// OLED power helpers
// =======================
void wakeDisplay() {
  if (g_oledState == OLED_OFF) {
    display.ssd1306_command(SSD1306_DISPLAYON);
  }
  display.dim(false);        // normal brightness
  g_oledState = OLED_ON;
  lastActivityMs = millis();
}

void maybeDimOrOff() {
  uint32_t now = millis();
  uint32_t idle = now - lastActivityMs;

  if (g_oledState == OLED_ON && idle > OLED_DIM_TIMEOUT_MS) {
    display.dim(true);       // lower brightness
    g_oledState = OLED_DIM;
  }

  if (g_oledState != OLED_OFF && idle > OLED_OFF_TIMEOUT_MS) {
    display.ssd1306_command(SSD1306_DISPLAYOFF);
    g_oledState = OLED_OFF;
  }
}

// =======================
// MP3-style battery icon (like your reference picture)
// =======================
void drawBatteryIconFancy(int pct) {
  // Icon position (top-right corner)
  const int x = SCREEN_WIDTH - 24;   // total width ~21 px
  const int y = 0;

  // Battery body
  const int bodyW = 18;
  const int bodyH = 10;

  // Terminal
  const int termW = 3;
  const int termH = 4;

  // ---- Outline: double rectangle ----
  display.drawRect(x,     y,     bodyW,     bodyH,     SSD1306_WHITE);
  display.drawRect(x + 1, y + 1, bodyW - 2, bodyH - 2, SSD1306_WHITE);

  // ---- Terminal ----
  int termX = x + bodyW;
  int termY = y + (bodyH - termH) / 2;
  display.fillRect(termX, termY, termW, termH, SSD1306_WHITE);

  // ---- Inner fill area ----
  int fillX = x + 3;
  int fillY = y + 3;
  int fillW = bodyW - 6;    // inside width
  int fillH = bodyH - 6;

  const int bars = 5;       // 5 segments, like your image
  int filledBars = (pct * bars) / 100;
  if (filledBars < 0)       filledBars = 0;
  if (filledBars > bars)    filledBars = bars;

  for (int i = 0; i < bars; i++) {
    int bx = fillX + i * (fillW / bars);
    int bw = (fillW / bars) - 1;

    if (bw < 1) bw = 1;

    if (i < filledBars) {
      display.fillRect(bx, fillY, bw, fillH, SSD1306_WHITE);
    } else {
      display.drawRect(bx, fillY, bw, fillH, SSD1306_WHITE);
    }
  }
}

// =======================
// Display helpers
// =======================
void showAlertScreen(const String& rodName) {
  wakeDisplay();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // Top-left: HUB MAC
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("HUB:");
  display.setCursor(0, 10);
  display.print(g_macStr);

  // Top-right: battery icon
  drawBatteryIconFancy(g_battPct);

  // Center: big rod ID
  display.setTextSize(3);
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(rodName, 0, 0, &x1, &y1, &w, &h);

  int16_t x = (SCREEN_WIDTH - w) / 2;
  int16_t y = (SCREEN_HEIGHT - h) / 2 + 8;  // небольшой сдвиг вниз

  display.setCursor(x, y);
  display.print(rodName);
  display.display();
}

void showIdle() {
  wakeDisplay();

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print("READY (ESP-NOW Hub)");

  display.setCursor(0, 16);
  display.print("MAC:");
  display.setCursor(0, 26);
  display.print(g_macStr);

  // Top-right: battery icon
  drawBatteryIconFancy(g_battPct);

  display.display();
}

// =======================
// Setup
// =======================
void setup() {
  Serial.begin(115200);
  delay(100);

#ifdef ESP32
  // Lower CPU frequency for power saving
  setCpuFrequencyMhz(80);
#endif

  // Init ADC for battery
  pinMode(VBAT_PIN, INPUT);

  // Init display
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // I2C address 0x3C
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.clearDisplay();
  display.display();

  // WiFi + ESP-NOW
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);       // modem sleep to save power

  g_macStr = WiFi.macAddress();
  Serial.print("Hub STA MAC: ");
  Serial.println(g_macStr);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    for(;;);
  }

  esp_now_register_recv_cb(onEspNowRecv);

  // First battery reading
  g_vbat_V  = readVbat();
  g_battPct = vbatToPercent(g_vbat_V);
  lastBatMs = millis();

  lastActivityMs = millis();
  g_oledState = OLED_ON;

  showIdle();
}

// =======================
// Main loop
// =======================
void loop() {
  uint32_t now = millis();

  // Battery refresh
  if (now - lastBatMs >= VBAT_PERIOD_MS) {
    lastBatMs = now;
    g_vbat_V  = readVbat();
    g_battPct = vbatToPercent(g_vbat_V);

    // If no active alert and display is not fully off – refresh idle to update icon
    if (lastAlertMs == 0 && g_oledState != OLED_OFF) {
      showIdle();
    }
  }

  // New ESP-NOW alert?
  if (g_hasAlert) {
    noInterrupts();
    BitePacket pkt = g_lastPkt;
    g_hasAlert = false;
    interrupts();

    String rod = String(pkt.rodName);
    rod.trim();
    if (rod.length() == 0) rod = "UNKNOWN";

    Serial.print("Alert from: "); Serial.print(rod);
    Serial.print("  type=");      Serial.print(pkt.eventType);
    Serial.print("  batt=");      Serial.print(pkt.batteryPct);
    Serial.print("%  dG=");       Serial.println(pkt.deltaG, 3);

    showAlertScreen(rod);
    lastAlertMs = now;
  }

  // After timeout, go back to idle screen
  if (lastAlertMs != 0 && now - lastAlertMs > ALERT_HOLD_MS) {
    showIdle();
    lastAlertMs = 0;
  }

  // Handle OLED dim/off timers
  maybeDimOrOff();

  // Small delay to reduce wakeups
  delay(10);
}
