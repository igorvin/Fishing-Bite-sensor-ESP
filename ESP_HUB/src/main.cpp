/*******************************************************
  ESP-NOW Hub for Fishing Bite Sensor (SH1106 OLED version)
  UI v2:
    - Status bar (HUB + dot + battery)
    - Alert screen: rod name centered + event label
    - One-time invert flash on new alert
    - Progress bar during ALERT_HOLD_MS
    - Idle screen: clean "WAITING..." and MAC (small)

  Notes:
    - 1.3" I2C OLED is often SH1106 (132x64 internal RAM)
    - Auto-detect OLED addr 0x3C/0x3D (7-bit)
********************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_idf_version.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// =======================
// Display config (SH1106 128x64 I2C)
// =======================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// SH1106 contrast values (0..255)
static const uint8_t OLED_CONTRAST_ON  = 0xFF;
static const uint8_t OLED_CONTRAST_DIM = 0x20; // tune 0x10..0x40

// =======================
// I2C pins (XIAO family)
// =======================
#if defined(ARDUINO_XIAO_ESP32S3) || defined(ARDUINO_XIAO_ESP32C6) || defined(ARDUINO_XIAO_ESP32C3)
  #define I2C_SDA_PIN   D4
  #define I2C_SCL_PIN   D5
#else
  #define I2C_SDA_PIN   SDA
  #define I2C_SCL_PIN   SCL
#endif

// =======================
// Battery ADC config (HUB)
// =======================
#define VBAT_PIN        A0
const float    VBAT_VREF      = 3.30f;
const float    VBAT_DIV       = 2.00f;   // 100k/100k divider
const float    VBAT_CAL       = 1.00f;
const uint16_t VBAT_ADC_MAX   = 4095;
const uint16_t VBAT_SAMPLES   = 16;
const uint32_t VBAT_PERIOD_MS = 5000;

float    g_vbat_V   = 0.0f;
int      g_battPct  = 0;
uint32_t lastBatMs  = 0;

// =======================
// Packet from sensor
// =======================
struct __attribute__((packed)) BitePacket {
  char    rodName[16];
  uint8_t eventType;     // 1=BITE, 2=RUN, 3=LOW BAT
  uint8_t batteryPct;
  float   deltaG;
};

volatile bool       g_hasAlert = false;
volatile BitePacket g_lastPkt;

// =======================
// State
// =======================
uint32_t lastAlertMs = 0;
const uint32_t ALERT_HOLD_MS = 5000;

String  g_macStr;
uint8_t g_oledAddr = 0x3C;

uint8_t g_lastEventType = 0;
String  g_lastRodShown;

// progress bar refresh throttle
uint32_t lastProgressDrawMs = 0;
const uint32_t PROGRESS_REFRESH_MS = 200;

// OLED power state
enum OledState : uint8_t { OLED_ON, OLED_DIM, OLED_OFF };
OledState g_oledState = OLED_ON;

const uint32_t OLED_DIM_TIMEOUT_MS = 30000;
const uint32_t OLED_OFF_TIMEOUT_MS = 60000;
uint32_t lastActivityMs = 0;

// =======================
// I2C helpers
// =======================
static bool i2cProbe(uint8_t addr) {
  Wire.beginTransmission(addr);
  return (Wire.endTransmission() == 0);
}

static uint8_t detectOLEDAddress() {
  if (i2cProbe(0x3C)) return 0x3C;
  if (i2cProbe(0x3D)) return 0x3D;
  return 0;
}

// =======================
// Battery helpers
// =======================
float readVbat() {
  uint32_t acc = 0;
  analogRead(VBAT_PIN);
  for (int i = 0; i < (int)VBAT_SAMPLES; i++) acc += analogRead(VBAT_PIN);
  float adc = acc / float(VBAT_SAMPLES);
  return (adc / VBAT_ADC_MAX) * VBAT_VREF * VBAT_DIV * VBAT_CAL;
}

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
// ESP-NOW callback (IDF4 vs IDF5 compatible)
// =======================
#if ESP_IDF_VERSION_MAJOR >= 5
void onEspNowRecv(const esp_now_recv_info* info, const uint8_t* data, int len) {
  (void)info;
  if (len != (int)sizeof(BitePacket)) return;
  memcpy((void*)&g_lastPkt, data, sizeof(BitePacket));
  g_hasAlert = true;
}
#else
void onEspNowRecv(const uint8_t* mac, const uint8_t* data, int len) {
  (void)mac;
  if (len != (int)sizeof(BitePacket)) return;
  memcpy((void*)&g_lastPkt, data, sizeof(BitePacket));
  g_hasAlert = true;
}
#endif

// =======================
// OLED low-level helpers (contrast + power)
// =======================
static void oledSetContrast(uint8_t val) {
  display.oled_command(0x81);
  display.oled_command(val);
}

static void oledOn()  { display.oled_command(SH110X_DISPLAYON); }
static void oledOff() { display.oled_command(SH110X_DISPLAYOFF); }

// =======================
// OLED power helpers
// =======================
void wakeDisplay() {
  if (g_oledState == OLED_OFF) oledOn();
  oledSetContrast(OLED_CONTRAST_ON);
  g_oledState = OLED_ON;
  lastActivityMs = millis();
}

void maybeDimOrOff() {
  uint32_t now = millis();
  uint32_t idle = now - lastActivityMs;

  if (g_oledState == OLED_ON && idle > OLED_DIM_TIMEOUT_MS) {
    oledSetContrast(OLED_CONTRAST_DIM);
    g_oledState = OLED_DIM;
  }
  if (g_oledState != OLED_OFF && idle > OLED_OFF_TIMEOUT_MS) {
    oledOff();
    g_oledState = OLED_OFF;
  }
}

// =======================
// Battery icon (top-right)
// =======================
void drawBatteryIconFancy(int pct) {
  const int x = SCREEN_WIDTH - 24;
  const int y = 1;

  const int bodyW = 18;
  const int bodyH = 10;
  const int termW = 3;
  const int termH = 4;

  display.drawRect(x, y, bodyW, bodyH, SH110X_WHITE);
  display.drawRect(x + 1, y + 1, bodyW - 2, bodyH - 2, SH110X_WHITE);

  int termX = x + bodyW;
  int termY = y + (bodyH - termH) / 2;
  display.fillRect(termX, termY, termW, termH, SH110X_WHITE);

  int fillX = x + 3;
  int fillY = y + 3;
  int fillW = bodyW - 6;
  int fillH = bodyH - 6;

  const int bars = 5;
  int filledBars = (pct * bars) / 100;
  if (filledBars < 0) filledBars = 0;
  if (filledBars > bars) filledBars = bars;

  for (int i = 0; i < bars; i++) {
    int bx = fillX + i * (fillW / bars);
    int bw = (fillW / bars) - 1;
    if (bw < 1) bw = 1;

    if (i < filledBars) display.fillRect(bx, fillY, bw, fillH, SH110X_WHITE);
    else                display.drawRect(bx, fillY, bw, fillH, SH110X_WHITE);
  }
}

// =======================
// UI helpers
// =======================
const char* eventLabel(uint8_t type) {
  switch (type) {
    case 1: return "BITE";
    case 2: return "RUN!";
    case 3: return "LOW BAT";
    default: return "";
  }
}

// Top status bar height
static const int STATUS_H = 14;

void drawStatusBar(bool connected) {
  // divider line
  display.drawFastHLine(0, STATUS_H - 1, SCREEN_WIDTH, SH110X_WHITE);

  display.setTextWrap(false);
  display.setTextSize(1);
  display.setCursor(0, 2);
  display.print("HUB");

  // connection dot
  if (connected) display.fillCircle(24, 6, 2, SH110X_WHITE);
  else           display.drawCircle(24, 6, 2, SH110X_WHITE);

  drawBatteryIconFancy(g_battPct);
}

// progress bar at bottom
void drawAlertProgress(uint32_t elapsedMs) {
  const int barH = 4;
  const int y = SCREEN_HEIGHT - barH;

  // clear bar area only
  display.fillRect(0, y, SCREEN_WIDTH, barH, SH110X_BLACK);

  uint32_t e = (elapsedMs > ALERT_HOLD_MS) ? ALERT_HOLD_MS : elapsedMs;
  int w = (int)((uint64_t)e * SCREEN_WIDTH / ALERT_HOLD_MS);
  if (w < 0) w = 0;
  if (w > SCREEN_WIDTH) w = SCREEN_WIDTH;

  display.fillRect(0, y, w, barH, SH110X_WHITE);
}

// =======================
// Screens
// =======================
void showIdle() {
  wakeDisplay();

  display.clearDisplay();
  display.display();

  display.setTextColor(SH110X_WHITE);
  display.setTextWrap(false);

  // Status bar
  drawStatusBar(true);

  // ---- Big center text ----
  display.setTextSize(2);
  const char* msg = "WAITING";
  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);

  int16_t x = (SCREEN_WIDTH - (int)w) / 2;
  if (x < 0) x = 0;

  int16_t yBig = STATUS_H + 10;
  display.setCursor(x, yBig);
  display.print(msg);

  // ---- Line A: MAC (full, single line) ----
  display.setTextSize(1);
  display.setCursor(0, 42);
  display.print(g_macStr);   // e.g. 94:A9:90:60:F7:94

  // ---- Line B: ESP-NOW ACTIVE ----
  const char* st = "ESP-NOW ACTIVE";
  display.getTextBounds(st, 0, 0, &x1, &y1, &w, &h);
  int16_t xs = (SCREEN_WIDTH - (int)w) / 2;
  if (xs < 0) xs = 0;

  display.setCursor(xs, 54);
  display.print(st);

  display.display();
}



void showAlertScreen(const String& rodNameIn, uint8_t evtType, bool doFlash) {
  wakeDisplay();

  // HARD clear (flush)
  display.clearDisplay();
  display.display();

  display.setTextColor(SH110X_WHITE);
  display.setTextWrap(false);

  // status bar (no MAC here)
  drawStatusBar(true);

  // optional flash (only once per new alert)
  if (doFlash) {
    display.invertDisplay(true);
    delay(120);
    display.invertDisplay(false);
  }

  String rod = rodNameIn;
  rod.trim();
  if (rod.length() == 0) rod = "UNKNOWN";

  const int contentTop = STATUS_H + 2;

  // ----- Rod name formatting -----
  if (rod.length() <= 6) {
    display.setTextSize(3);
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(rod, 0, 0, &x1, &y1, &w, &h);
    int16_t x = (SCREEN_WIDTH - (int)w) / 2; if (x < 0) x = 0;
    int16_t y = contentTop + 6;
    display.setCursor(x, y);
    display.print(rod);

  } else if (rod.length() <= 10) {
    display.setTextSize(2);
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(rod, 0, 0, &x1, &y1, &w, &h);
    int16_t x = (SCREEN_WIDTH - (int)w) / 2; if (x < 0) x = 0;
    int16_t y = contentTop + 10;
    display.setCursor(x, y);
    display.print(rod);

  } else {
    display.setTextSize(2);
    if (rod.length() > 16) rod = rod.substring(0, 16);

    String l1 = rod.substring(0, 8); l1.trim();
    String l2 = rod.substring(8);    l2.trim();

    int16_t x1, y1; uint16_t w1, h1;
    display.getTextBounds(l1, 0, 0, &x1, &y1, &w1, &h1);
    int16_t xLine1 = (SCREEN_WIDTH - (int)w1) / 2; if (xLine1 < 0) xLine1 = 0;

    uint16_t w2, h2;
    display.getTextBounds(l2, 0, 0, &x1, &y1, &w2, &h2);
    int16_t xLine2 = (SCREEN_WIDTH - (int)w2) / 2; if (xLine2 < 0) xLine2 = 0;

    int16_t yLine1 = contentTop + 6;
    int16_t yLine2 = yLine1 + 18;

    display.setCursor(xLine1, yLine1); display.print(l1);
    display.setCursor(xLine2, yLine2); display.print(l2);
  }

  // ----- Event label (below rod) -----
  const char* label = eventLabel(evtType);
  if (label && label[0]) {
    display.setTextSize(2);
    int16_t x1, y1; uint16_t w, h;
    display.getTextBounds(label, 0, 0, &x1, &y1, &w, &h);
    int16_t x = (SCREEN_WIDTH - (int)w) / 2; if (x < 0) x = 0;
    display.setCursor(x, STATUS_H + 44);
    display.print(label);
  }

  // initial progress bar (0%)
  drawAlertProgress(0);

  display.display();
}

// =======================
// Setup / Loop
// =======================
void setup() {
  Serial.begin(115200);
  delay(150);

#if defined(ESP32)
  setCpuFrequencyMhz(80);
#endif

  pinMode(VBAT_PIN, INPUT);
  analogReadResolution(12);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(20);

  g_oledAddr = detectOLEDAddress();
  if (!g_oledAddr) {
    Serial.println("❌ OLED not detected at 0x3C/0x3D. Check wiring + ADD_SELECT.");
    for(;;);
  }

  if (!display.begin(g_oledAddr, true)) {
    Serial.println("❌ SH1106 init failed");
    for(;;);
  }

  oledSetContrast(OLED_CONTRAST_ON);
  oledOn();

  display.clearDisplay();
  display.display();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);

  g_macStr = WiFi.macAddress();
  Serial.print("Hub STA MAC: ");
  Serial.println(g_macStr);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    for(;;);
  }
  esp_now_register_recv_cb(onEspNowRecv);

  g_vbat_V  = readVbat();
  g_battPct = vbatToPercent(g_vbat_V);
  lastBatMs = millis();

  lastActivityMs = millis();
  g_oledState = OLED_ON;

  showIdle();
}

void loop() {
  uint32_t now = millis();

  // Battery refresh
  if (now - lastBatMs >= VBAT_PERIOD_MS) {
    lastBatMs = now;
    g_vbat_V  = readVbat();
    g_battPct = vbatToPercent(g_vbat_V);

    // refresh idle to update battery icon (only if not in alert)
    if (lastAlertMs == 0 && g_oledState != OLED_OFF) {
      showIdle();
    }
  }

  // New ESP-NOW alert?
  if (g_hasAlert) {
    BitePacket pkt;
    noInterrupts();
    memcpy(&pkt, (const void*)&g_lastPkt, sizeof(BitePacket));
    g_hasAlert = false;
    interrupts();

    String rod = String(pkt.rodName);
    rod.trim();
    if (!rod.length()) rod = "UNKNOWN";

    Serial.print("Alert from: "); Serial.print(rod);
    Serial.print(" type=");       Serial.print(pkt.eventType);
    Serial.print(" batt=");       Serial.print(pkt.batteryPct);
    Serial.print("% dG=");        Serial.println(pkt.deltaG, 3);

    bool flash = true; // always flash on new alert
    g_lastRodShown = rod;
    g_lastEventType = pkt.eventType;

    showAlertScreen(rod, pkt.eventType, flash);
    lastAlertMs = now;
    lastProgressDrawMs = 0;
  }

  // While alert is active: update progress bar occasionally
  if (lastAlertMs != 0) {
    uint32_t elapsed = now - lastAlertMs;

    if (now - lastProgressDrawMs >= PROGRESS_REFRESH_MS) {
      lastProgressDrawMs = now;

      // draw only progress region in buffer, then flush
      drawAlertProgress(elapsed);
      display.display();
    }

    // After hold timeout -> back to idle
    if (elapsed > ALERT_HOLD_MS) {
      showIdle();
      lastAlertMs = 0;
      lastProgressDrawMs = 0;
    }
  }

  maybeDimOrOff();
  delay(10);
}
