/*******************************************************
  ESP-NOW Hub for Fishing Bite Sensor (SH1106 OLED version)
  Board: Seeed XIAO ESP32-S3

  Features:
    - ESP-NOW receiver always active
    - Alert screen: rod name centered + event label
    - Progress bar during ALERT_HOLD_MS
    - Idle screen: "WAITING..." + MAC
    - OLED dims after 1 min inactivity
    - OLED turns OFF after 2 min inactivity
    - Green LED solid ON in normal mode
    - Green LED heartbeat blink every 10 sec in sleep mode
      (sleep mode here = OLED_OFF, ESP-NOW still active)
    - Vibration motor alert on bite

  Notes:
    - 1.3" I2C OLED is often SH1106
    - Auto-detect OLED addr 0x3C/0x3D
********************************************************/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_idf_version.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>

// =======================
// Display config
// =======================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

Adafruit_SH1106G display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// SH1106 contrast values
static const uint8_t OLED_CONTRAST_ON  = 0xFF;
static const uint8_t OLED_CONTRAST_DIM = 0x20;   // tune if needed

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
// Battery ADC config
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
// Status LED
// =======================
#define LED_GREEN_PIN         D1       // <-- change if needed
const uint32_t LED_BLINK_PERIOD_MS = 10000;   // blink every 10 sec
const uint32_t LED_BLINK_ON_MS     = 80;      // short heartbeat blink

uint32_t lastLedBlinkMs   = 0;
uint32_t ledBlinkStartMs  = 0;
bool     ledBlinkActive   = false;

// If your LED is active LOW, invert these two functions
void ledOn()  { digitalWrite(LED_GREEN_PIN, HIGH); }
void ledOff() { digitalWrite(LED_GREEN_PIN, LOW);  }

// =======================
// Vibration motor
// =======================
#define MOTOR_PIN              D2      // <-- change if needed

const uint32_t MOTOR_BITE_MS   = 300;  // eventType 1
const uint32_t MOTOR_RUN_MS    = 800;  // eventType 2
const uint32_t MOTOR_LOWBAT_MS = 150;  // eventType 3
const uint32_t MOTOR_GAP_MS    = 120;  // gap for double pulse on low batt

bool     motorActive = false;
uint32_t motorOffMs = 0;

bool     motorSecondPulsePending = false;
uint32_t motorSecondPulseStartMs = 0;

// If motor driver transistor is active LOW, invert these functions
void motorOn()  { digitalWrite(MOTOR_PIN, HIGH); }
void motorOff() { digitalWrite(MOTOR_PIN, LOW);  }

void startMotorPulse(uint32_t durationMs) {
  motorOn();
  motorActive = true;
  motorOffMs = millis() + durationMs;
}

void startMotorPattern(uint8_t eventType) {
  motorSecondPulsePending = false;

  switch (eventType) {
    case 1:   // BITE
      startMotorPulse(MOTOR_BITE_MS);
      break;

    case 2:   // RUN
      startMotorPulse(MOTOR_RUN_MS);
      break;

    case 3:   // LOW BAT
      startMotorPulse(MOTOR_LOWBAT_MS);
      motorSecondPulsePending = true;
      motorSecondPulseStartMs = millis() + MOTOR_LOWBAT_MS + MOTOR_GAP_MS;
      break;

    default:
      startMotorPulse(MOTOR_BITE_MS);
      break;
  }
}

void handleMotor(uint32_t now) {
  if (motorActive && (int32_t)(now - motorOffMs) >= 0) {
    motorOff();
    motorActive = false;
  }

  if (motorSecondPulsePending && (int32_t)(now - motorSecondPulseStartMs) >= 0) {
    motorSecondPulsePending = false;
    startMotorPulse(MOTOR_LOWBAT_MS);
  }
}

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

// Progress bar refresh throttle
uint32_t lastProgressDrawMs = 0;
const uint32_t PROGRESS_REFRESH_MS = 200;

// OLED power state
enum OledState : uint8_t { OLED_ON, OLED_DIM, OLED_OFF };
OledState g_oledState = OLED_ON;

// Inactivity timers
const uint32_t OLED_DIM_TIMEOUT_MS  = 60000;    // 1 minute
const uint32_t OLED_OFF_TIMEOUT_MS  = 120000;   // 2 minutes
uint32_t lastActivityMs = 0;

// Top header height
static const int STATUS_H = 18;

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
  analogRead(VBAT_PIN);  // dummy read
  for (int i = 0; i < (int)VBAT_SAMPLES; i++) {
    acc += analogRead(VBAT_PIN);
  }
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
// ESP-NOW callback
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
// OLED low-level helpers
// =======================
static void oledSetContrast(uint8_t val) {
  display.oled_command(0x81);
  display.oled_command(val);
}

static void oledOn()  { display.oled_command(SH110X_DISPLAYON); }
static void oledOff() { display.oled_command(SH110X_DISPLAYOFF); }

// =======================
// Activity helpers
// =======================
void markActivity() {
  lastActivityMs = millis();
}

void wakeDisplay() {
  if (g_oledState == OLED_OFF) {
    oledOn();
    delay(5);
  }
  oledSetContrast(OLED_CONTRAST_ON);
  g_oledState = OLED_ON;
}

void maybeDimOrOff() {
  uint32_t now = millis();
  uint32_t idle = now - lastActivityMs;

  if (g_oledState == OLED_ON && idle >= OLED_DIM_TIMEOUT_MS) {
    oledSetContrast(OLED_CONTRAST_DIM);
    g_oledState = OLED_DIM;
  }

  if (g_oledState != OLED_OFF && idle >= OLED_OFF_TIMEOUT_MS) {
    oledOff();
    g_oledState = OLED_OFF;
  }
}

// =======================
// LED behavior
// =======================
void handleStatusLed(uint32_t now) {
  // Normal / dim display -> LED solid ON
  if (g_oledState == OLED_ON || g_oledState == OLED_DIM) {
    ledOn();
    ledBlinkActive = false;
    return;
  }

  // OLED_OFF -> heartbeat blink every 10 sec
  if (g_oledState == OLED_OFF) {
    if (!ledBlinkActive && (now - lastLedBlinkMs >= LED_BLINK_PERIOD_MS)) {
      ledOn();
      ledBlinkActive = true;
      ledBlinkStartMs = now;
      lastLedBlinkMs = now;
    }

    if (ledBlinkActive && (now - ledBlinkStartMs >= LED_BLINK_ON_MS)) {
      ledOff();
      ledBlinkActive = false;
    }
  }
}

// =======================
// Battery icon
// =======================
void drawBatteryIconFancy(int pct, uint16_t color) {
  const int x     = SCREEN_WIDTH - 24;
  const int y     = 4;
  const int bodyW = 18;
  const int bodyH = 10;
  const int termW = 3;
  const int termH = 4;

  display.drawRect(x, y, bodyW, bodyH, color);
  display.fillRect(x + bodyW, y + (bodyH - termH) / 2, termW, termH, color);

  const int fillX = x + 2;
  const int fillY = y + 2;
  const int fillW = bodyW - 4;
  const int fillH = bodyH - 4;
  const int bars  = 5;

  int filled = constrain((pct * bars) / 100, 0, bars);
  for (int i = 0; i < bars; i++) {
    int bx = fillX + i * (fillW / bars);
    int bw = (fillW / bars) - 1;
    if (bw < 1) bw = 1;

    if (i < filled) display.fillRect(bx, fillY, bw, fillH, color);
    else            display.drawRect(bx, fillY, bw, fillH, color);
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

void drawIdleHeader() {
  display.fillRect(0, 0, SCREEN_WIDTH, STATUS_H, SH110X_WHITE);

  display.setTextColor(SH110X_BLACK);
  display.setTextWrap(false);
  display.setTextSize(1);
  display.setCursor(3, 5);
  display.print("HUB");

  drawBatteryIconFancy(g_battPct, SH110X_BLACK);

  display.setTextColor(SH110X_WHITE);
}

void drawAlertHeader(uint8_t evtType) {
  display.fillRect(0, 0, SCREEN_WIDTH, STATUS_H, SH110X_WHITE);

  char label[24];
  snprintf(label, sizeof(label), "!! %s !!", eventLabel(evtType));

  display.setTextColor(SH110X_BLACK);
  display.setTextWrap(false);
  display.setTextSize(1);

  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(label, 0, 0, &x1, &y1, &w, &h);

  int16_t x = (SCREEN_WIDTH - (int)w) / 2;
  if (x < 0) x = 0;

  display.setCursor(x, 5);
  display.print(label);

  drawBatteryIconFancy(g_battPct, SH110X_BLACK);

  display.setTextColor(SH110X_WHITE);
}

void drawAlertProgress(uint32_t elapsedMs) {
  const int barH = 8;
  const int y = SCREEN_HEIGHT - barH;

  display.fillRect(0, y, SCREEN_WIDTH, barH, SH110X_BLACK);

  uint32_t e = (elapsedMs > ALERT_HOLD_MS) ? ALERT_HOLD_MS : elapsedMs;
  int w = (int)((uint64_t)e * SCREEN_WIDTH / ALERT_HOLD_MS);
  w = constrain(w, 0, SCREEN_WIDTH);

  display.fillRect(0, y, w, barH, SH110X_WHITE);
}

// =======================
// Screens
// =======================
void showIdle(bool resetInactivity = false) {
  wakeDisplay();
  if (resetInactivity) markActivity();

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextWrap(false);

  drawIdleHeader();

  display.setTextSize(2);
  const char* msg = "WAITING...";
  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(msg, 0, 0, &x1, &y1, &w, &h);

  int16_t x = (SCREEN_WIDTH - (int)w) / 2;
  if (x < 0) x = 0;

  display.setCursor(x, STATUS_H + 10);
  display.print(msg);

  display.setTextSize(1);
  display.setCursor(0, 48);
  display.print(g_macStr);

  display.display();
}

void showAlertScreen(const String& rodNameIn, uint8_t evtType) {
  wakeDisplay();
  markActivity();

  display.clearDisplay();
  display.setTextColor(SH110X_WHITE);
  display.setTextWrap(false);

  drawAlertHeader(evtType);

  String rod = rodNameIn;
  rod.trim();
  if (rod.length() == 0) rod = "UNKNOWN";

  const int PROGRESS_H = 8;
  const int contentTop = STATUS_H + 2;
  const int contentH   = (SCREEN_HEIGHT - PROGRESS_H) - contentTop;

  uint8_t textSize;
  if (rod.length() <= 6) {
    textSize = 3;
  } else {
    if (rod.length() > 10) rod = rod.substring(0, 10);
    textSize = 2;
  }

  display.setTextSize(textSize);

  int16_t x1, y1;
  uint16_t w, h;
  display.getTextBounds(rod.c_str(), 0, 0, &x1, &y1, &w, &h);

  int16_t x = (SCREEN_WIDTH - (int)w) / 2;
  if (x < 0) x = 0;

  int16_t y = contentTop + (contentH - (int)h) / 2;
  display.setCursor(x, y);
  display.print(rod);

  drawAlertProgress(0);
  display.display();
}

// =======================
// Setup
// =======================
void setup() {
  Serial.begin(115200);
  delay(150);

#if defined(ESP32)
  setCpuFrequencyMhz(80);
#endif

  pinMode(LED_GREEN_PIN, OUTPUT);
  ledOff();

  pinMode(MOTOR_PIN, OUTPUT);
  motorOff();

  pinMode(VBAT_PIN, INPUT);
  analogReadResolution(12);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  delay(20);

  g_oledAddr = detectOLEDAddress();
  if (!g_oledAddr) {
    Serial.println("OLED not detected at 0x3C/0x3D");
    while (true) delay(1000);
  }

  if (!display.begin(g_oledAddr, true)) {
    Serial.println("SH1106 init failed");
    while (true) delay(1000);
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
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onEspNowRecv);

  g_vbat_V  = readVbat();
  g_battPct = vbatToPercent(g_vbat_V);
  lastBatMs = millis();

  markActivity();
  g_oledState = OLED_ON;

  showIdle(true);
  ledOn();
}

// =======================
// Loop
// =======================
void loop() {
  uint32_t now = millis();

  // Battery refresh
  if (now - lastBatMs >= VBAT_PERIOD_MS) {
    lastBatMs = now;
    g_vbat_V  = readVbat();
    g_battPct = vbatToPercent(g_vbat_V);

    // Refresh idle screen only when OLED is fully ON.
    // This prevents dim mode from being canceled every 5 sec.
    if (lastAlertMs == 0 && g_oledState == OLED_ON) {
      showIdle(false);
    }
  }

  // New ESP-NOW alert
  if (g_hasAlert) {
    BitePacket pkt;

    noInterrupts();
    memcpy(&pkt, (const void*)&g_lastPkt, sizeof(BitePacket));
    g_hasAlert = false;
    interrupts();

    String rod = String(pkt.rodName);
    rod.trim();
    if (!rod.length()) rod = "UNKNOWN";

    Serial.print("Alert from: ");
    Serial.print(rod);
    Serial.print(" type=");
    Serial.print(pkt.eventType);
    Serial.print(" batt=");
    Serial.print(pkt.batteryPct);
    Serial.print("% dG=");
    Serial.println(pkt.deltaG, 3);

    g_lastRodShown  = rod;
    g_lastEventType = pkt.eventType;

    showAlertScreen(rod, pkt.eventType);
    startMotorPattern(pkt.eventType);

    lastAlertMs = now;
    lastProgressDrawMs = 0;
  }

  // While alert is active
  if (lastAlertMs != 0) {
    uint32_t elapsed = now - lastAlertMs;

    if (g_oledState != OLED_OFF && (now - lastProgressDrawMs >= PROGRESS_REFRESH_MS)) {
      lastProgressDrawMs = now;
      drawAlertProgress(elapsed);
      display.display();
    }

    if (elapsed > ALERT_HOLD_MS) {
      lastAlertMs = 0;
      lastProgressDrawMs = 0;
      showIdle(true);
    }
  }

  maybeDimOrOff();
  handleStatusLed(now);
  handleMotor(now);

  delay(10);
}