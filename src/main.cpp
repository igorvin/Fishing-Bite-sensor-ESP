/*******************************************************
  Fishing Bite Sensor – ESP32-S3 (Seeed XIAO)
  - Accelerometer: BMI160 (DFRobot_BMI160)
  - UI/Settings:   GyverDBFile + SettingsGyver
  - Storage:       LittleFS
  - Features:
      * ARMED/DISARMED (long-press button toggles)
      * Green LED on when ARMED
      * Red LED + short buzzer pulses on motion
      * Long beep when ARMED, two short when DISARMED
      * DISARMED → ultra-low-power deep sleep after grace
      * AP SSID = Sensor Name (persistent, save+reboot)
********************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <DFRobot_BMI160.h>

#include <LittleFS.h>
#include <GyverDBFile.h>
#include <SettingsGyver.h>   // Using SettingsGyver in the "SettingsESP pattern"

// ==============================
//            PINS (XIAO ESP32-S3)
// ==============================
#define LED_GREEN   5
#define LED_RED     6
#define BUZZER      8
#define BUTTON      4        // Active-LOW; if using EXT0 wake, must be RTC-capable

// If your board routes I2C to specific pins, set them here.
// Leave as -1 to let Wire use the default mapping.
#define I2C_SDA_PIN -1
#define I2C_SCL_PIN -1

// ==============================
/*        SENSOR / ENGINE CONST
   G_PER_LSB assumes ±2g range (1/16384 g/LSB typical for BMI160 at ±2g).
*/
const int8_t   I2C_ADDR              = 0x69;
const float    G_PER_LSB             = 1.0f / 16384.0f;
const uint8_t  TRIGGER_SAMPLES       = 3;        // samples over threshold before "motion"
const uint16_t LONG_PRESS_MS         = 1000;
const uint16_t DEBOUNCE_MS           = 30;
const uint16_t SAMPLE_INTERVAL_MS    = 10;

// Keep AP alive on boot while DISARMED so UI can load
const uint32_t CONFIG_GRACE_MS       = 3UL * 60UL * 1000UL;  // 3 minutes

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

// Boot timestamp (for config grace window)
uint32_t bootMs = 0;

// ==============================
//      SETTINGS + DATABASE
// ==============================
// DB keys (persistent)
DB_KEYS(
  kk,
  // config & runtime
  name,          // Sensor name (ALSO AP SSID)
  deltaG,        // Sensitivity Δg
  shortPulse,    // Short pulse, ms
  pulsePeriod,   // Pulse period, ms
  contThresh,    // Continuous threshold, ms
  armedSw,       // Armed switch
  // Wi-Fi
  apPass,        // AP password (>=8 WPA2, empty=open)
  // actions
  saveRB         // Save & Restart button
);

// Live (non-DB) widget IDs (hashed)
#define H_stateLbl     H(stateLbl)
#define H_deltaLbl     H(deltaLbl)
#define H_alertsLbl    H(alertsLbl)
#define H_uptimeLbl    H(uptimeLbl)
#define H_stateLED     H(stateLED)
#define H_accelLED     H(accelLED)
#define H_accelLbl     H(accelLbl)

GyverDBFile   db(&LittleFS, "/config.db");
SettingsGyver sett("Fishing Bite Sensor", &db);

// Runtime config (mirrors DB)
String   cfg_name         = "BiteSensor";  // ALSO used as AP SSID
float    cfg_deltaG       = 0.15f;
uint16_t cfg_shortPulseMs = 120;
uint16_t cfg_pulsePeriod  = 300;
uint16_t cfg_contThresh   = 250;
bool     cfg_armedSw      = false;

String   cfg_apPass       = "";            // empty = open AP

// ==============================
// Small clamp helpers
// ==============================
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

/**
 * @brief Compute vector magnitude for 3D acceleration.
 */
inline float mag3(float x, float y, float z) { return sqrtf(x*x + y*y + z*z); }

/**
 * @brief Begin a short alert pulse: red LED ON + buzzer tone for cfg_shortPulseMs.
 *        Increments global alert counter.
 */
inline void startPulse(uint32_t now) {
  pulseOn = true;
  lastPulseStart = now;
  pulseOffAt = now + cfg_shortPulseMs;
  digitalWrite(LED_RED, HIGH);
  tone(BUZZER, 3000);
  g_alerts++;  // count alerts
}

/**
 * @brief Stop the current alert pulse: buzzer off, red LED off.
 */
inline void stopPulse() {
  pulseOn = false;
  noTone(BUZZER);
  digitalWrite(LED_RED, LOW);
}

/**
 * @brief Update green LED according to armed state (ON when armed).
 */
inline void updateGreenLED() {
  digitalWrite(LED_GREEN, armed ? HIGH : LOW);
}

/**
 * @brief Bring up the Access Point with the given SSID/password.
 *        Empty password → open AP.
 */
void configureAP(const String& ssid, const String& pass) {
  WiFi.softAPdisconnect(true);
  delay(100);
  if (pass.length() >= 8) WiFi.softAP(ssid.c_str(), pass.c_str());
  else                    WiFi.softAP(ssid.c_str(), "");  // open AP
  delay(200);
  Serial.print("AP SSID now: "); Serial.println(ssid);
  Serial.print("AP IP: ");       Serial.println(WiFi.softAPIP());
}

/**
 * @brief Central setter for armed state (syncs UI switch + plays tones).
 *        If disarming with deepSleepOnDisarm=true → enter deep sleep.
 */
void setArmed(bool v, bool deepSleepOnDisarm) {
  armed = v;
  cfg_armedSw = v;
  db.set(kk::armedSw, v);   // mirror to DB so UI reflects it
  updateGreenLED();

  if (v) {
    // Long beep when armed
    tone(BUZZER, 3000); delay(600); noTone(BUZZER);
  } else {
    // Two short beeps when disarmed
    tone(BUZZER, 3000); delay(150); noTone(BUZZER); delay(80);
    tone(BUZZER, 3000); delay(150); noTone(BUZZER);

    if (deepSleepOnDisarm) {
      // Power down outputs and go to deep sleep
      noTone(BUZZER);
      digitalWrite(LED_RED, LOW);
      digitalWrite(LED_GREEN, LOW);
      pinMode(BUTTON, INPUT_PULLUP);

      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
      // Preferred: EXT0 (BUTTON must be RTC-capable). Wake when pressed (LOW).
      esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON, 0);

      // Alternative if BUTTON is not RTC-capable on your board:
      // esp_sleep_enable_ext1_wakeup(1ULL << BUTTON, ESP_EXT1_WAKEUP_ALL_LOW);

      esp_deep_sleep_start();
    }
  }
}

/**
 * @brief Enter deep sleep while DISARMED; wake on long press later.
 *        (Used after the grace period and no AP clients.)
 */
void suspendUntilLongPressToArm() {
  noTone(BUZZER);
  digitalWrite(LED_RED, LOW);
  digitalWrite(LED_GREEN, LOW);

  pinMode(BUTTON, INPUT_PULLUP);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON, 0); // LOW wake

  // Alternative if BUTTON isn't RTC-capable:
  // esp_sleep_enable_ext1_wakeup(1ULL << BUTTON, ESP_EXT1_WAKEUP_ALL_LOW);

  esp_deep_sleep_start();
}

/**
 * @brief Convenience tones.
 */
void beepShort() { tone(BUZZER, 3000); delay(150); noTone(BUZZER); delay(80); }
void beepLong()  { tone(BUZZER, 3000); delay(600); noTone(BUZZER); }

// ==============================
//         SETTINGS UI BUILD
// ==============================
bool saveReboot_f = false;

/**
 * @brief Build SettingsGyver UI: sensor tuning, Wi-Fi/AP config, and status.
 */
void build(sets::Builder& b) {
  using namespace sets;

  {
    Group g(b, "Sensor");

    // Sliders with IDs + labels + units + AnyPtr(...)
    b.Slider(kk::deltaG,     "Sensitivity Δg",          0.02f, 1.00f, 0.01f,  "",   AnyPtr(&cfg_deltaG));
    b.Slider(kk::shortPulse, "Short pulse",             40.0f, 1000.0f, 10.0f, "ms", AnyPtr(&cfg_shortPulseMs));
    b.Slider(kk::pulsePeriod,"Pulse period",            100.0f, 2000.0f, 10.0f, "ms", AnyPtr(&cfg_pulsePeriod));
    b.Slider(kk::contThresh, "Continuous threshold",    50.0f, 2000.0f, 10.0f, "ms", AnyPtr(&cfg_contThresh));

    b.Switch(kk::armedSw, "Armed (web toggle)");
  }

  {
    Group g(b, "Wireless settings");
    // Sensor name (ALSO AP SSID)
    b.Input(kk::name, "Sensor name (also AP SSID)");
    b.Pass( kk::apPass, "AP password (Min 8 chars)");

    // Button → handled in update() with db.update() + ESP.restart()
    if (b.Button(kk::saveRB, "Save & Restart")) saveReboot_f = true;
  }

  {
    Group g(b, "Status");           // live (non-DB) widgets
    b.LED(  H_stateLED);
    b.Label(H_stateLbl,  "State");

    b.LED(  H_accelLED);
    b.Label(H_accelLbl,  "Accelerometer");

    b.Label(H_deltaLbl,  "Δg");
    b.Label(H_alertsLbl, "Alerts");
    b.Label(H_uptimeLbl, "Uptime (s)");
  }
}

/**
 * @brief Persist UI changes, apply Armed switch, handle Save&Restart,
 *        and push live status back to the UI.
 */
void update(sets::Updater& u) {
  // Persist sliders to DB (manual write for variable-bound sliders)
  db.set(kk::deltaG,      cfg_deltaG);
  db.set(kk::shortPulse,  (int)cfg_shortPulseMs);
  db.set(kk::pulsePeriod, (int)cfg_pulsePeriod);
  db.set(kk::contThresh,  (int)cfg_contThresh);

  // Read DB-backed toggles and name (Inputs write to DB automatically)
  if (auto e = db.get(kk::armedSw)) cfg_armedSw = e.toBool();
  if (auto e = db.get(kk::name))    cfg_name    = e.toString();

  // Sanitize sliders
  cfg_deltaG       = clampf(cfg_deltaG, 0.02f, 1.0f);
  cfg_shortPulseMs = clamp16(cfg_shortPulseMs, 40, 1000);
  cfg_pulsePeriod  = clamp16(cfg_pulsePeriod,  100, 2000);
  cfg_contThresh   = clamp16(cfg_contThresh,   50, 2000);

  // Apply armed switch
  if (cfg_armedSw != armed) {
    setArmed(cfg_armedSw, /*deepSleepOnDisarm=*/!cfg_armedSw);
  }

  // ===== Save & Restart (apply name=SSID and password) with IMMEDIATE DB FLUSH =====
  if (saveReboot_f) {
    saveReboot_f = false;

    // The latest text inputs are already in DB (because Input widgets write there)
    String newName = db.get(kk::name).toString();
    String newPass = db.get(kk::apPass).toString();
    newName.trim();

    // Validate SSID (name) & password
    if (newName.length() == 0 || newName.length() > 32) {
      u.alert("Sensor name / SSID must be 1..32 characters.");
    } else if (!newPass.isEmpty() && newPass.length() < 8) {
      u.alert("Password must be at least 8 characters or empty (open AP).");
    } else {
      // Optionally normalize name (e.g., remove CR/LF)
      db.set(kk::name, newName);
      db.set(kk::apPass, newPass);

      // Force-flush DB NOW (critical to persist before restart)
      db.update();

      // Optional: show notice; then restart
      u.notice("Saved. Rebooting to apply AP SSID/password...");
      ESP.restart();
    }
  }

  // ---- live status to UI ----
  u.update(H_stateLbl,  armed ? String("ARMED") : String("DISARMED"));
  u.updateColor(H_stateLED, armed ? sets::Colors::Aqua : sets::Colors::Pink);

  // Accelerometer init state LED + label
  u.update(H_accelLbl, imuOK ? String("OK") : String("ERROR"));
  u.updateColor(H_accelLED, imuOK ? sets::Colors::Green : sets::Colors::Red);

  u.update(H_deltaLbl,  String(g_lastDelta, 3));
  u.update(H_alertsLbl, (int)g_alerts);
  u.update(H_uptimeLbl, (uint32_t)(millis() / 1000));
}

// ==============================
//         BUTTON HANDLER
// ==============================

/**
 * @brief Debounced button handler with long-press action to toggle armed state.
 */
static void handleButton(uint32_t now) {
  int raw = digitalRead(BUTTON); // active-LOW
  if (raw != btnPrev && (now - btnLastChange) >= DEBOUNCE_MS) {
    btnPrev = raw;
    btnLastChange = now;

    if (raw == LOW) { btnIsDown = true; btnDownAt = now; longFired = false; }
    else btnIsDown = false;
  }

  // Long-press toggles arm/disarm
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

  // --- CRITICAL ORDER: mount FS + start DB BEFORE SettingsGyver ---
#ifdef ESP32
  LittleFS.begin(true);   // format on first run
#else
  LittleFS.begin();
#endif
  db.begin();

  // Initialize defaults (first run only)
  db.init(kk::name,        cfg_name);
  db.init(kk::deltaG,      cfg_deltaG);
  db.init(kk::shortPulse,  (int)cfg_shortPulseMs);
  db.init(kk::pulsePeriod, (int)cfg_pulsePeriod);
  db.init(kk::contThresh,  (int)cfg_contThresh);
  db.init(kk::armedSw,     false);
  db.init(kk::apPass,      cfg_apPass);

  // Load persisted config fresh from database (always latest)
  cfg_name         = db.get(kk::name).toString();
  cfg_deltaG       = db.get(kk::deltaG).toFloat();
  cfg_shortPulseMs = db.get(kk::shortPulse).toInt();
  cfg_pulsePeriod  = db.get(kk::pulsePeriod).toInt();
  cfg_contThresh   = db.get(kk::contThresh).toInt();
  cfg_armedSw      = db.get(kk::armedSw).toBool();
  cfg_apPass       = db.get(kk::apPass).toString();

  // WiFi mode first (like SettingsESP example), THEN start SettingsGyver
  WiFi.mode(WIFI_AP_STA);
  // Optionally set station/AP hostname to match sensor name
  WiFi.setHostname(cfg_name.c_str());

  // Start Settings AFTER FS/DB/config are ready
  sett.begin();
  sett.onBuild(build);
  sett.onUpdate(update);

  // Bring up the AP using the freshly loaded SSID/pass
  WiFi.softAPdisconnect(true);
  configureAP(cfg_name, cfg_apPass);
  Serial.print("Loaded SSID: "); Serial.println(cfg_name);
  Serial.print("AP pass: ");     Serial.println(cfg_apPass.isEmpty() ? String("<open>") : String("<hidden>"));
  Serial.print("AP IP: ");       Serial.println(WiFi.softAPIP());

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

  // Establish initial baseline g magnitude
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

  // Apply initial ARMED state from DB (plays tone pattern)
  setArmed(cfg_armedSw, /*deepSleepOnDisarm=*/false);

  Serial.println("Open http://192.168.4.1 for Settings");
  Serial.println("Default: DISARMED (hold button to arm)");

  // Boot-time long-press to ARM (quality-of-life feature)
  delay(DEBOUNCE_MS);
  if (digitalRead(BUTTON) == LOW) {
    uint32_t t0 = millis();
    while (digitalRead(BUTTON) == LOW) {
      if (millis() - t0 >= LONG_PRESS_MS) {
        setArmed(true, /*deepSleepOnDisarm=*/false);
        break;
      }
      delay(5);
    }
  }

  // Start config-grace timer (do not sleep at the end of setup)
  bootMs = millis();
}

// ==============================
//              LOOP
// ==============================
void loop() {
  // Serve Settings UI and flush DB internals
  sett.tick();
  // db.tick();  // not required with SettingsGyver

  const uint32_t now = millis();

  // Full button functionality preserved
  handleButton(now);
  updateGreenLED();

  // If DISARMED → stay awake during grace or while a client is connected; else sleep
  if (!armed) {
    const bool clientConnected =
      (WiFi.getMode() == WIFI_AP || WiFi.getMode() == WIFI_AP_STA) &&
      (WiFi.softAPgetStationNum() > 0);
    const bool inGrace = (now - bootMs) < CONFIG_GRACE_MS;

    if (!clientConnected && !inGrace) {
      suspendUntilLongPressToArm();
    }
    return;  // disarmed but awake: keep serving UI
  }

  // ---- Motion sampling (ARMED) ----
  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;

    if (imuOK) {
      int16_t acc[3] = {0};
      if (bmi160.getAccelData(acc) == 0) {
        float gx = acc[0] * G_PER_LSB;
        float gy = acc[1] * G_PER_LSB;
        float gz = acc[2] * G_PER_LSB;
        float mag = mag3(gx, gy, gz);

        // Slow baseline drift tracking (EWMA)
        baselineMagG = baselineMagG * 0.999f + mag * 0.001f;
        float delta = fabsf(mag - baselineMagG);
        g_lastDelta = delta;

        // Trigger count filter
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
    // After continuous motion for cfg_contThresh, continue periodic pulses
    if (motionDur >= cfg_contThresh) {
      if (!pulseOn && (now - lastPulseStart) >= cfg_pulsePeriod) {
        startPulse(now);
      }
    }
  }
}
