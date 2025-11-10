#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <DFRobot_BMI160.h>

#include <LittleFS.h>
#include <GyverDBFile.h>
#include <SettingsGyver.h>

// ==============================
//            PINS (XIAO ESP32-S3)
// ==============================
#define LED_GREEN   5
#define LED_RED     6
#define BUZZER      8
#define BUTTON      4        // Active-LOW; for EXT0 wake choose an RTC-capable pin

#define I2C_SDA_PIN -1
#define I2C_SCL_PIN -1

// ==============================
//        SENSOR / ENGINE CONST
// ==============================
const int8_t   I2C_ADDR              = 0x69;
const float    G_PER_LSB             = 1.0f / 16384.0f;
const uint8_t  TRIGGER_SAMPLES       = 3;
const uint16_t LONG_PRESS_MS         = 1000;
const uint16_t DEBOUNCE_MS           = 30;
const uint16_t SAMPLE_INTERVAL_MS    = 10;

// ==============================
//             STATE
// ==============================
DFRobot_BMI160 bmi160;
bool imuOK = false;
bool armed = false;

uint32_t lastSample = 0;
uint8_t  consecutiveTriggers = 0;
float    baselineMagG = 1.0f;

bool     inMotion = false;
uint32_t motionStartMs = 0;

bool     pulseOn = false;
uint32_t pulseOffAt = 0;
uint32_t lastPulseStart = 0;

// Button FSM
bool     btnPrev = HIGH;
uint32_t btnLastChange = 0;
bool     btnIsDown = false;
uint32_t btnDownAt = 0;
bool     longFired = false;

// Live telemetry
volatile float    g_lastDelta = 0.0f;   // last |mag - baseline|
volatile uint32_t g_alerts    = 0;      // short pulse count

// ==============================
//      SETTINGS + DATABASE
// ==============================
// Define keys namespace "kk" for DB-backed widgets
DB_KEYS(
  kk,
  conf,
  name,
  deltaG,
  shortPulse,
  pulsePeriod,
  contThresh,
  armedSw
);

// Live (non-DB) widget IDs (hashed)
#define H_stateLbl     H(stateLbl)
#define H_deltaLbl     H(deltaLbl)
#define H_alertsLbl    H(alertsLbl)
#define H_uptimeLbl    H(uptimeLbl)
#define H_stateLED     H(stateLED)

GyverDBFile   db(&LittleFS, "/config.db");
SettingsGyver sett("Fishing Bite Sensor", &db);

// Runtime config (mirrors DB)
String   cfg_name         = "BiteSensor";
float    cfg_deltaG       = 0.15f;
uint16_t cfg_shortPulseMs = 120;
uint16_t cfg_pulsePeriod  = 300;
uint16_t cfg_contThresh   = 250;
bool     cfg_armedSw      = false;   // UI toggle mirrors physical state

// Small clamp helpers
static inline uint16_t clamp16(uint16_t v, uint16_t lo, uint16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// ==============================
//         LOW-LEVEL HELPERS
// ==============================
inline float mag3(float x, float y, float z) { return sqrtf(x*x + y*y + z*z); }

inline void startPulse(uint32_t now) {
  pulseOn = true;
  lastPulseStart = now;
  pulseOffAt = now + cfg_shortPulseMs;
  digitalWrite(LED_RED, HIGH);
  tone(BUZZER, 3000);
  g_alerts++;  // count alerts
}

inline void stopPulse() {
  pulseOn = false;
  noTone(BUZZER);
  digitalWrite(LED_RED, LOW);
}

inline void updateGreenLED() {
  digitalWrite(LED_GREEN, armed ? HIGH : LOW);
}

// Unified state setter to keep button & web in sync
void setArmed(bool v, bool deepSleepOnDisarm) {
  armed = v;
  cfg_armedSw = v;
  db.set(kk::armedSw, v);   // mirror to DB so UI reflects it
  updateGreenLED();

  if (v) {
    tone(BUZZER, 3000); delay(600); noTone(BUZZER);              // long beep
  } else {
    tone(BUZZER, 3000); delay(150); noTone(BUZZER); delay(80);   // two short
    tone(BUZZER, 3000); delay(150); noTone(BUZZER);
    if (deepSleepOnDisarm) {
      noTone(BUZZER);
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN, LOW);
      pinMode(BUTTON, INPUT_PULLUP);
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
      // Preferred: EXT0 (BUTTON must be RTC-capable). Else use EXT1 if needed.
      esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON, 0);
      // Alternative if BUTTON isn't RTC-capable:
      // esp_sleep_enable_ext1_wakeup(1ULL << BUTTON, ESP_EXT1_WAKEUP_ALL_LOW);
      esp_deep_sleep_start();
    }
  }
}

void suspendUntilLongPressToArm() {
  noTone(BUZZER);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);

  pinMode(BUTTON, INPUT_PULLUP);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON, 0); // LOW wake
  esp_deep_sleep_start();
}

void beepShort() { tone(BUZZER, 3000); delay(150); noTone(BUZZER); delay(80); }
void beepLong()  { tone(BUZZER, 3000); delay(600); noTone(BUZZER); }

// ==============================
//         SETTINGS UI BUILD
// ==============================
void build(sets::Builder& b) {
  using namespace sets;

  {
    Group g(b, "Sensor");
    b.Input( kk::name,       "Name");
    b.Number(kk::deltaG,     "Sensitivity Δg (0.02..1.0)");
    b.Number(kk::shortPulse, "Short pulse (ms)");
    b.Number(kk::pulsePeriod,"Pulse period (ms)");
    b.Number(kk::contThresh, "Continuous threshold (ms)");
    b.Switch(kk::armedSw,    "Armed (web toggle)");
  }

  {
    Group g(b, "Status");           // live (non-DB) widgets
    b.LED(   H_stateLED);
    b.Label( H_stateLbl,  "State");
    b.Label( H_deltaLbl,  "Δg");
    b.Label( H_alertsLbl, "Alerts");
    b.Label( H_uptimeLbl, "Uptime (s)");
  }
}

// Apply web changes → runtime + push live status
void update(sets::Updater& u) {
  // Pull the latest from DB (Entry → toXxx())
  if (auto e = db.get(kk::name))        cfg_name         = e.toString();
  if (auto e = db.get(kk::deltaG))      cfg_deltaG       = (float)e.toFloat();
  if (auto e = db.get(kk::shortPulse))  cfg_shortPulseMs = (uint16_t)e.toInt();
  if (auto e = db.get(kk::pulsePeriod)) cfg_pulsePeriod  = (uint16_t)e.toInt();
  if (auto e = db.get(kk::contThresh))  cfg_contThresh   = (uint16_t)e.toInt();
  if (auto e = db.get(kk::armedSw))     cfg_armedSw      = e.toBool();

  // Sanitize
  cfg_deltaG       = clampf(cfg_deltaG, 0.02f, 1.0f);
  cfg_shortPulseMs = clamp16(cfg_shortPulseMs, 40, 1000);
  cfg_pulsePeriod  = clamp16(cfg_pulsePeriod,  100, 2000);
  cfg_contThresh   = clamp16(cfg_contThresh,   50, 2000);

  // If user toggled armed via web, reflect exactly like button:
  if (cfg_armedSw != armed) {
    setArmed(cfg_armedSw, /*deepSleepOnDisarm=*/!cfg_armedSw);
  }

  // ---- live status to UI (safe String casting per docs warning) ----
  u.update(H_stateLbl,  armed ? String("ARMED") : String("DISARMED"));
  u.update(H_deltaLbl,  String(g_lastDelta, 3));
  u.update(H_alertsLbl, (int)g_alerts);
  u.update(H_uptimeLbl, (uint32_t)(millis() / 1000));
  u.updateColor(H_stateLED, armed ? sets::Colors::Aqua : sets::Colors::Pink);
}

// ==============================
//         BUTTON HANDLER
// ==============================
static void handleButton(uint32_t now) {
  int raw = digitalRead(BUTTON); // active-LOW
  if (raw != btnPrev && (now - btnLastChange) >= DEBOUNCE_MS) {
    btnPrev = raw;
    btnLastChange = now;

    if (raw == LOW) { btnIsDown = true; btnDownAt = now; longFired = false; }
    else btnIsDown = false;
  }

  // Long-press keeps previous behavior 1:1
  if (btnIsDown && !longFired && (now - btnDownAt) >= LONG_PRESS_MS) {
    longFired = true;
    if (armed) setArmed(false, /*deepSleepOnDisarm=*/true);
    else       setArmed(true,  /*deepSleepOnDisarm=*/false);
  }
}

// ==============================
//             SETUP
// ==============================
void setup() {
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED,   OUTPUT);
  pinMode(BUZZER,    OUTPUT);
  pinMode(BUTTON,    INPUT_PULLUP);

  Serial.begin(115200);
  delay(100);
  Serial.println();

#ifdef ESP32
  LittleFS.begin(true);   // format on first run
#else
  LittleFS.begin();
#endif
  db.begin();

  // Initialize defaults (first run only)
  db.init(kk::name,        cfg_name);
  db.init(kk::deltaG,      cfg_deltaG);
  db.init(kk::shortPulse,  cfg_shortPulseMs);
  db.init(kk::pulsePeriod, cfg_pulsePeriod);
  db.init(kk::contThresh,  cfg_contThresh);
  db.init(kk::armedSw,     false);

  // Load persisted config to RAM
  if (auto e = db.get(kk::name))        cfg_name         = e.toString();
  if (auto e = db.get(kk::deltaG))      cfg_deltaG       = (float)e.toFloat();
  if (auto e = db.get(kk::shortPulse))  cfg_shortPulseMs = (uint16_t)e.toInt();
  if (auto e = db.get(kk::pulsePeriod)) cfg_pulsePeriod  = (uint16_t)e.toInt();
  if (auto e = db.get(kk::contThresh))  cfg_contThresh   = (uint16_t)e.toInt();
  if (auto e = db.get(kk::armedSw))     cfg_armedSw      = e.toBool();

  // ======== WIFI AP ========
  uint8_t mac[6]; WiFi.macAddress(mac);
  char ssid[40];
  snprintf(ssid, sizeof(ssid), "%s-%02X%02X", cfg_name.c_str(), mac[4], mac[5]);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, "");   // open AP; set a password if desired
  delay(300);
  Serial.print("AP SSID: "); Serial.println(ssid);
  Serial.print("AP IP:   "); Serial.println(WiFi.softAPIP());

  // ======== SETTINGS WEB ========
  sett.begin();
  sett.onBuild(build);
  sett.onUpdate(update);

  // ======== I2C + BMI160 ========
  if (I2C_SDA_PIN >= 0 && I2C_SCL_PIN >= 0) Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  else Wire.begin();
  Wire.setClock(100000);

  if (bmi160.softReset() != BMI160_OK) Serial.println("BMI160 reset FAILED");
  else if (bmi160.I2cInit(I2C_ADDR) != BMI160_OK) Serial.println("BMI160 I2C init FAILED");
  else {
    imuOK = true;
    Serial.printf("BMI160 init OK @ 0x%02X\n", I2C_ADDR);
  }

  if (imuOK) {
    float sum = 0;
    for (int i = 0; i < 50; i++) {
      int16_t acc[3] = {0};
      if (bmi160.getAccelData(acc) == 0) {
        sum += mag3(acc[0]*G_PER_LSB, acc[1]*G_PER_LSB, acc[2]*G_PER_LSB);
      }
      delay(10);
    }
    baselineMagG = sum / 50.0f;
  }

  // Initial armed state from DB
  setArmed(cfg_armedSw, /*deepSleepOnDisarm=*/false);

  Serial.println("Open http://192.168.4.1 for Settings");
  Serial.println("Default: DISARMED (hold button to arm)");
  delay(DEBOUNCE_MS);

  // Boot-time long-press to ARM (original UX)
  if (digitalRead(BUTTON) == LOW) {
    uint32_t t0 = millis();
    while (digitalRead(BUTTON) == LOW) {
      if (millis() - t0 >= LONG_PRESS_MS) {
        setArmed(true, /*deepSleepOnDisarm=*/false);
        return;
      }
      delay(5);
    }
  }

  // If still disarmed -> suspend until long press
  if (!armed) suspendUntilLongPressToArm();
}

// ==============================
//              LOOP
// ==============================
void loop() {
  // Serve Settings UI and flush DB
  sett.tick();
  db.tick();

  const uint32_t now = millis();

  // Full button functionality preserved
  handleButton(now);
  updateGreenLED();

  if (!armed) return;

  // ---- Motion sampling ----
  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;

    if (imuOK) {
      int16_t acc[3] = {0};
      if (bmi160.getAccelData(acc) == 0) {
        float gx = acc[0] * G_PER_LSB;
        float gy = acc[1] * G_PER_LSB;
        float gz = acc[2] * G_PER_LSB;
        float mag = mag3(gx, gy, gz);

        // drift tracking
        baselineMagG = baselineMagG * 0.999f + mag * 0.001f;
        float delta = fabsf(mag - baselineMagG);
        g_lastDelta = delta;

        if (delta >= cfg_deltaG) {
          if (consecutiveTriggers < 255) consecutiveTriggers++;
        } else if (consecutiveTriggers > 0) {
          consecutiveTriggers--;
        }

        bool motionNow = (consecutiveTriggers >= TRIGGER_SAMPLES);
        if (motionNow && !inMotion) {
          inMotion = true;
          motionStartMs = now;
          startPulse(now);
        } else if (!motionNow && inMotion) {
          inMotion = false;
        }
      }
    }
  }

  // ---- Pulse engine ----
  if (pulseOn && (int32_t)(now - pulseOffAt) >= 0) stopPulse();
  if (inMotion) {
    uint32_t motionDur = now - motionStartMs;
    if (motionDur >= cfg_contThresh) {
      if (!pulseOn && (now - lastPulseStart) >= cfg_pulsePeriod) {
        startPulse(now);
      }
    }
  }
}
