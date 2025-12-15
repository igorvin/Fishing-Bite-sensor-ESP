/*******************************************************
  Fishing Bite Sensor – ESP32 (Seeed XIAO ESP32-S3 / ESP32-C6 / ESP32-C3)
  - Accelerometer: LSM6DS3 (Adafruit LSM6DS library)
  - UI/Settings:   GyverDBFile + SettingsGyver
  - Storage:       LittleFS
  - ESP-NOW:       Sends alarms to central ESP hub
  - Features:
      * Soft power-latch:
          - Q4 AO3407 + Q5 AO3400A + SW2 (PWR_KEY)
          - MCU holds power via PWR_CTRL pin
          - Long press PWR_KEY (~3 s) -> clean power OFF
      * ARMED/DISARMED (long-press BUTTON, SW1)
      * Green LED = armed (PWM brightness)
      * Red LED + buzzer pulses on motion (PWM volume/brightness)
      * Long beep when armed, two short when disarmed
      * DISARMED → deep-sleep after grace period
      * Persistent AP SSID/password via Settings
      * Language dropdown: English / Русский
      * Power save: CPU at 80 MHz, Wi-Fi AP OFF in ARMED after 2 min
      * Battery monitor: % + voltage via ADC divider (100k/100k + 100nF)
      * Low-battery alarm over ESP-NOW (eventType = 3)
      * Web "Calibrate baseline & test alarm" button
      * Automatic baseline recalibration on disarm

  - NEW: Auto-disarm (anti-noise) + Smart learning
      * Auto-disarm (master) default ON
      * Smart-detection default ON:
          - Every time you ARM, it runs an automatic learn:
              10s learn NORMAL + 10s learn MOVED
          - During learn (first 20s) bite alarms are suppressed
          - After learn it auto-derives thresholds:
              UP  = clamp(0.6 * maxPitchDev, 15..60) deg
              TILT= clamp(0.6 * maxRollDev , 15..70) deg
            If no meaningful move detected -> fallback defaults 25/35 deg
      * If Smart-detection OFF -> manual thresholds shown in UI
********************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_idf_version.h>
#include <esp_sleep.h>
#if ESP_IDF_VERSION_MAJOR >= 5
  #include <esp_wifi_types.h>
#endif

#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS3.h>

#include <LittleFS.h>
#include <GyverDBFile.h>
#include <SettingsGyver.h>

#if !defined(ARDUINO_XIAO_ESP32S3) && !defined(ARDUINO_XIAO_ESP32C6) && !defined(ARDUINO_XIAO_ESP32C3)
#warning "This sketch is tuned for Seeed XIAO ESP32-S3 / ESP32-C6 / ESP32-C3. Check pin mappings if using another board."
#endif

// ==============================
//            PINS (Example: XIAO ESP32-C3 style)
// ==============================
#define LED_GREEN   D7
#define LED_RED     D8
#define BUZZER      D9
#define BUTTON      D6

#define PIN_PWR_CTRL D0
#define PIN_BTN_IN   D1

#define VBAT_PIN    D2

#define I2C_SDA_PIN D4
#define I2C_SCL_PIN D5

// ==============================
// LSM6DS3 CONFIG
// ==============================
const uint8_t LSM6_ADDR         = 0x6B;
const float   MS2_PER_G         = 9.80665f;

const uint8_t  TRIGGER_SAMPLES    = 3;
const uint16_t LONG_PRESS_MS      = 1000;
const uint16_t DEBOUNCE_MS        = 30;
const uint16_t SAMPLE_INTERVAL_MS = 10;

// --- Power button (BTN_IN) timings ---
const uint32_t PWR_DEBOUNCE_MS    = 50;
const uint32_t PWR_LONG_PRESS_MS  = 3000;

// Time windows
const uint32_t DISARMED_AP_GRACE_MS = 3UL * 60UL * 1000UL;
const uint32_t ARMED_AP_WINDOW_MS   = 2UL * 60UL * 1000UL;

// Battery read config
const float    VBAT_VREF          = 3.30f;
const float    VBAT_DIV           = 2.00f;
const float    VBAT_CAL           = 1.00f;
const uint16_t VBAT_SAMPLES       = 16;
const uint16_t VBAT_ADC_MAX       = 4095;
const uint32_t VBAT_PERIOD_MS     = 5000;

// Low-battery ESP-NOW alert
const int      LOW_BATT_THRESHOLD_PCT = 20;
const int      LOW_BATT_HYSTERESIS    = 5;

// ==============================
// STATE
// ==============================
Adafruit_LSM6DS3 lsm6;
bool imuOK = false, armed = false;
uint32_t lastSample = 0, motionStartMs = 0, bootMs = 0;
uint8_t  consecutiveTriggers = 0;
float    baselineMagG = 1.0f;
bool     inMotion = false, pulseOn = false;
uint32_t pulseOffAt = 0, lastPulseStart = 0;

// BUTTON (SW1) FSM – ARM/DISARM
bool btnPrev = HIGH, btnIsDown = false, longFired = false;
uint32_t btnLastChange = 0, btnDownAt = 0;

// POWER BUTTON (SW2 / BTN_IN) FSM – POWER OFF
const bool PWRBTN_ACTIVE_LEVEL = LOW;
bool     pwrBtnStableState   = !PWRBTN_ACTIVE_LEVEL;
bool     pwrBtnPrevStable    = !PWRBTN_ACTIVE_LEVEL;
uint32_t pwrBtnLastChangeMs  = 0;
uint32_t pwrBtnPressStartMs  = 0;
bool     pwrIgnoreFirstPress = true;

// telemetry
volatile float    g_lastDelta = 0.0f;
volatile uint32_t g_alerts    = 0;

// ARMED config window timer
uint32_t wifiArmStart = 0;
bool     wifiApActiveAfterArm = false;

// Battery telemetry
float    g_vbat_V = 0.0f;
int      g_batt_pct = 0;
uint32_t lastVbatMs = 0;
bool     lowBattReported = false;

// ==============================
// ESP-NOW STATE
// ==============================
struct __attribute__((packed)) BitePacket {
  char    rodName[16];
  uint8_t eventType;     // 1 short/first, 2 continuous, 3 low battery
  uint8_t batteryPct;    // 0..100
  float   deltaG;
};

bool     cfg_espNowEn   = false;
String   cfg_hubMacStr  = "";
bool     espNowEnabled  = false;
bool     espNowInited   = false;
bool     hubMacValid    = false;
uint8_t  hubMacAddr[6]  = {0};

// ==============================
// SETTINGS + DATABASE
// ==============================
// NOTE: GyverDB macro supports up to 15 keys; keep count <= 15
DB_KEYS(
  kk,
  name, deltaG, shortPulse, pulsePeriod, contThresh,
  armedSw, langIdx, testAlarm, apPass, saveRB,
  buzVol, ledRedLvl, ledGreenLvl,
  espNowEn, hubMac
);

// UI holders (LEDs/labels)
#define H_stateLbl    H(stateLbl)
#define H_deltaLbl    H(deltaLbl)
#define H_alertsLbl   H(alertsLbl)
#define H_uptimeLbl   H(uptimeLbl)
#define H_stateLED    H(stateLED)
#define H_accelLED    H(accelLED)
#define H_accelLbl    H(accelLbl)
#define H_langLED     H(langLED)
#define H_langLbl     H(langLbl)
#define H_langHintLbl H(langHintLbl)
#define H_battLED     H(battLED)
#define H_battLbl     H(battLbl)

GyverDBFile   db(&LittleFS, "/config.db");
SettingsGyver sett("Fishing Bite Sensor", &db);

// ==============================
// LOCALIZATION
// ==============================
struct LangPack {
  const char* grpGeneral;
  const char* grpSensor;
  const char* grpWiFi;
  const char* grpStatus;
  const char* grpEspNow;

  const char* lblLanguage;
  const char* langHint;
  const char* lblSensitivity;
  const char* lblShortPulse;
  const char* lblPulsePeriod;
  const char* lblContThresh;
  const char* lblArmedSwitch;

  const char* lblName;
  const char* lblApPass;
  const char* lblSaveRestart;

  const char* lblState;
  const char* lblAccel;
  const char* lblDeltaG;
  const char* lblAlerts;
  const char* lblUptime;
  const char* lblBattery;

  const char* lblEspNowEnable;
  const char* lblHubMac;
  const char* lblTestAlarm;

  const char* valArmed;
  const char* valDisarmed;
  const char* valAccelOK;
  const char* valAccelErr;

  const char* msgNameBad;
  const char* msgPassBad;
  const char* msgSavedReboot;
  const char* msgLangChanged;
  const char* msgHubMacBad;
};

const LangPack LANG_EN = {
  "General","Sensor","Wireless settings","Status","ESP-NOW",
  "Language","(English / Русский)",
  "Sensitivity \xCE\x94g","Short pulse","Pulse period","Continuous threshold","Armed (web toggle)",
  "Sensor name (AP SSID)","AP password (Min 8 chars.)","Save & Restart",
  "State","Accelerometer","\xCE\x94g","Alerts","Uptime (s)",
  "Battery",
  "Enable ESP-NOW","Hub MAC (AA:BB:CC:DD:EE:FF)","Calibrate baseline & test alarm",
  "ARMED","DISARMED","OK","ERROR",
  "Sensor name / SSID (1..32 chars.)",
  "Password (min 8 chars.)",
  "Saved. Rebooting to apply AP SSID/password...",
  "Language changed. Refresh the page.",
  "Hub MAC format invalid"
};

const LangPack LANG_RU = {
  "Общие","Датчик","Настройки Wi-Fi","Статус","ESP-NOW",
  "Язык","(Русский / English)",
  "Чувствительность Δg","Короткий импульс","Период импульсов","Порог непрерывности","Охрана (веб-переключатель)",
  "Имя датчика (SSID AP)","Пароль AP (мин. 8 симв.)","Сохранить и перезагрузить",
  "Состояние","Акселерометр","Δg","Срабатывания","Время работы (с)",
  "Батарея",
  "Включить ESP-NOW","MAC хаба (AA:BB:CC:DD:EE:FF)","Калибровка базы + тест",
  "ОХРАНА","СНЯТО","ОК","ОШИБКА",
  "Имя датчика / SSID 1–32 символов",
  "Пароль (мин. 8 символов)",
  "Сохранено. Перезагрузка для применения SSID/пароля...",
  "Язык изменён. Обновите страницу.",
  "Неверный формат MAC хаба"
};

String   cfg_name="BiteSensor", cfg_apPass="";
float    cfg_deltaG=0.15f;
uint16_t cfg_shortPulseMs=120, cfg_pulsePeriod=300, cfg_contThresh=250;
bool     cfg_armedSw=false;

// Language control
int      cfg_langIdx=0;      // 0=EN, 1=RU
bool     cfg_langRu=false;
int      g_prevLangIdx=-1;

// Output settings
uint16_t cfg_buzVolPct   = 100;
uint16_t cfg_ledRedPct   = 100;
uint16_t cfg_ledGreenPct = 100;

inline const LangPack& L(){ return cfg_langRu?LANG_RU:LANG_EN; }

// ==============================
// AUTO-DISARM CONFIG + LEARNING (stored with string keys)
// ==============================
// Master enable
const char* DBK_AUTO_DISARM_EN       = "autoDisarmEn";

// Smart/manual
const char* DBK_AUTO_SMART_EN        = "autoDisarmSmart";
const char* DBK_AUTO_DISARM_UP_DEG   = "autoDisarmUpDeg";
const char* DBK_AUTO_DISARM_TILT_DEG = "autoDisarmTiltDeg";

// Learned results (used only in Smart mode)
const char* DBK_LEARN_UP_DEG         = "autoLearnUpDeg";
const char* DBK_LEARN_TILT_DEG       = "autoLearnTiltDeg";

// Defaults for Smart fallback
const float SMART_UP_DEFAULT_DEG     = 25.0f;
const float SMART_TILT_DEFAULT_DEG   = 35.0f;

// UI/config variables
bool  cfg_autoDisarmEn     = true;   // default ON
bool  cfg_autoSmart        = true;   // default ON
float cfg_autoDisarmUpDeg  = 25.0f;  // manual if Smart OFF
float cfg_autoDisarmTiltDeg= 35.0f;  // manual if Smart OFF

// learned thresholds for Smart ON (derived each arm)
float cfg_learnUpDeg       = SMART_UP_DEFAULT_DEG;
float cfg_learnTiltDeg     = SMART_TILT_DEFAULT_DEG;

// Orientation math
struct Vec3 { float x,y,z; };
static Vec3 gravF = {0,0,1};

static bool  oriBaseValid = false;
static float basePitchDeg = 0.0f;
static float baseRollDeg  = 0.0f;
static uint32_t moveOverSince = 0;

// gravity filter + gate
const float    MOVE_GRAV_ALPHA = 0.94f;
const uint32_t MOVE_HOLD_MS    = 120;
const float    MOVE_MAG_MIN_G  = 0.75f;
const float    MOVE_MAG_MAX_G  = 1.25f;

// Learn timing (10s + 10s)
enum LearnState : uint8_t { LEARN_NONE=0, LEARN_NORMAL=1, LEARN_MOVED=2 };
static LearnState g_learnState = LEARN_NONE;
static uint32_t   g_learnStartMs = 0;
static uint32_t   g_phaseStartMs = 0;

static Vec3  g_sumN = {0,0,0};
static uint32_t g_cntN = 0;

static float g_maxPitchDev = 0.0f;
static float g_maxRollDev  = 0.0f;

// ==============================
// BUZZER + LED PWM HELPERS (ESP32 family)
// ==============================
#ifdef ESP32
  #define BUZZER_LEDC_CH 0
  #define LED_GREEN_CH   1
  #define LED_RED_CH     2

  const uint8_t BUZZER_BITS = 10;   // 0..1023
  const uint8_t LED_BITS    = 8;    // 0..255

  static bool pwmReady = false;

  inline void pwmWriteCh(uint8_t ch, uint32_t duty){
#if ESP_IDF_VERSION_MAJOR >= 5
    ledcWriteChannel(ch, duty);
#else
    ledcWrite(ch, duty);
#endif
  }

  void pwmInit() {
#if ESP_IDF_VERSION_MAJOR >= 5
    ledcAttachChannel(BUZZER, 3000, BUZZER_BITS, BUZZER_LEDC_CH);
    ledcWriteChannel(BUZZER_LEDC_CH, 0);

    ledcAttachChannel(LED_GREEN, 1000, LED_BITS, LED_GREEN_CH);
    ledcAttachChannel(LED_RED,   1000, LED_BITS, LED_RED_CH);
    ledcWriteChannel(LED_GREEN_CH, 0);
    ledcWriteChannel(LED_RED_CH,   0);
#else
    ledcSetup(BUZZER_LEDC_CH, 3000, BUZZER_BITS);
    ledcAttachPin(BUZZER, BUZZER_LEDC_CH);
    pwmWriteCh(BUZZER_LEDC_CH, 0);

    ledcSetup(LED_GREEN_CH, 1000, LED_BITS);
    ledcSetup(LED_RED_CH,   1000, LED_BITS);
    ledcAttachPin(LED_GREEN, LED_GREEN_CH);
    ledcAttachPin(LED_RED,   LED_RED_CH);
    ledcWrite(LED_GREEN_CH, 0);
    ledcWrite(LED_RED_CH,   0);
#endif
    pwmReady = true;
  }

  inline uint16_t buzzerDuty(){
    if (cfg_buzVolPct > 100) cfg_buzVolPct = 100;
    return (uint16_t)((cfg_buzVolPct * ((1 << BUZZER_BITS) - 1)) / 100);
  }

  inline uint8_t ledDuty(uint16_t pct){
    if (pct > 100) pct = 100;
    return (uint8_t)((pct * ((1 << LED_BITS) - 1)) / 100);
  }

  inline void buzzStart(){
    if (!pwmReady) return;
    pwmWriteCh(BUZZER_LEDC_CH, buzzerDuty());
  }

  inline void buzzStop(){
    if (!pwmReady) return;
    pwmWriteCh(BUZZER_LEDC_CH, 0);
  }

  inline void setGreenLED(bool on){
    if (!pwmReady) return;
    pwmWriteCh(LED_GREEN_CH, on ? ledDuty(cfg_ledGreenPct) : 0);
  }

  inline void setRedLED(bool on){
    if (!pwmReady) return;
    pwmWriteCh(LED_RED_CH, on ? ledDuty(cfg_ledRedPct) : 0);
  }
#else
  void pwmInit() {}
  inline void buzzStart(){ digitalWrite(BUZZER, HIGH); }
  inline void buzzStop(){  digitalWrite(BUZZER, LOW);  }
  inline void setGreenLED(bool on){ digitalWrite(LED_GREEN, on ? HIGH : LOW); }
  inline void setRedLED(bool on){   digitalWrite(LED_RED,   on ? HIGH : LOW); }
#endif

inline void longBeep(){ buzzStart(); delay(600); buzzStop(); }
inline void shortBeep(){ buzzStart(); delay(150); buzzStop(); }

// ==============================
// HELPERS
// ==============================
inline float mag3(float x,float y,float z){return sqrtf(x*x+y*y+z*z);}
static inline uint16_t clamp16(uint16_t v,uint16_t lo,uint16_t hi){return v<lo?lo:(v>hi?hi:v);}
static inline float clampf(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}

static inline float vMag(const Vec3& v){ return sqrtf(v.x*v.x + v.y*v.y + v.z*v.z); }
static inline float rad2deg(float r){ return r * 57.2957795f; }

static void gravityToPitchRoll(const Vec3& g, float& pitchDeg, float& rollDeg) {
  pitchDeg = rad2deg(atan2f(-g.x, sqrtf(g.y*g.y + g.z*g.z)));
  rollDeg  = rad2deg(atan2f(g.y, g.z));
}

// ==============================
// Battery read + mapping
// ==============================
float readVbat() {
  uint32_t acc = 0;
  analogRead(VBAT_PIN);                    // dummy
  for (int i=0;i<VBAT_SAMPLES;i++) acc += analogRead(VBAT_PIN);
  float adc = acc / float(VBAT_SAMPLES);
  float v = (adc / VBAT_ADC_MAX) * VBAT_VREF * VBAT_DIV * VBAT_CAL;
  return v;
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

// ==============================
// Wi-Fi MODES
// ==============================
void configureAP(const String& ssid,const String& pass){
  WiFi.softAPdisconnect(true);
  delay(50);
  if (pass.length()>=8) WiFi.softAP(ssid.c_str(),pass.c_str());
  else                  WiFi.softAP(ssid.c_str(),"");
  delay(100);
  Serial.printf("AP SSID: %s\n",ssid.c_str());
  Serial.println(WiFi.softAPIP());
}

void wifiEnterConfigMode() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.setHostname(cfg_name.c_str());
  configureAP(cfg_name, cfg_apPass);
  WiFi.setSleep(true);
}

void wifiAfterArmWindow() {
  WiFi.softAPdisconnect(true);
  Serial.println("AP disabled after ARM window");

  if (!espNowEnabled || !hubMacValid) {
    WiFi.disconnect(true, true);
    delay(50);
    WiFi.mode(WIFI_OFF);
    Serial.println("WiFi fully OFF (no ESP-NOW)");
  }
}

// ==============================
// ESP-NOW HELPERS
// ==============================
bool parseMac(const String& mac, uint8_t out[6]) {
  if (mac.length() != 17) return false;
  int vals[6];
  if (sscanf(mac.c_str(), "%x:%x:%x:%x:%x:%x",
             &vals[0], &vals[1], &vals[2],
             &vals[3], &vals[4], &vals[5]) != 6) return false;
  for (int i=0;i<6;i++) out[i] = (uint8_t)vals[i];
  return true;
}

#if ESP_IDF_VERSION_MAJOR >= 5
void onEspNowSent(const wifi_tx_info_t *info, esp_now_send_status_t status) { (void)info; (void)status; }
#else
void onEspNowSent(const uint8_t *mac, esp_now_send_status_t status) { (void)mac; (void)status; }
#endif

void espNowBegin() {
  if (!espNowEnabled || espNowInited || !hubMacValid) return;

  if (WiFi.getMode() == WIFI_OFF) WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  espNowInited = true;
  esp_now_register_send_cb(onEspNowSent);

  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, hubMacAddr, 6);
  peer.channel = 0;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) Serial.println("ESP-NOW add peer failed");
  else                                   Serial.println("ESP-NOW peer added");
}

void espNowEnd() {
  if (!espNowInited) return;
  esp_now_deinit();
  espNowInited = false;
  Serial.println("ESP-NOW deinit");
}

void sendBitePacket(uint8_t eventType) {
  if (!espNowEnabled || !espNowInited || !hubMacValid) return;

  BitePacket pkt{};
  strlcpy(pkt.rodName, cfg_name.c_str(), sizeof(pkt.rodName));
  pkt.eventType  = eventType;
  pkt.batteryPct = (uint8_t) constrain(g_batt_pct, 0, 100);
  pkt.deltaG     = g_lastDelta;

  esp_now_send(hubMacAddr, (uint8_t*)&pkt, sizeof(pkt));
}

// ==============================
// ALERT PULSES / LEDS
// ==============================
inline void startPulse(uint32_t now, uint8_t eventType){
  pulseOn        = true;
  lastPulseStart = now;
  pulseOffAt     = now + cfg_shortPulseMs;
  setRedLED(true);
  buzzStart();
  g_alerts = g_alerts + 1;
  sendBitePacket(eventType);
}

inline void startTestPulse() {
  uint32_t now = millis();
  pulseOn        = true;
  lastPulseStart = now;
  pulseOffAt     = now + cfg_shortPulseMs;
  setRedLED(true);
  buzzStart();
}

inline void stopPulse(){
  pulseOn=false;
  buzzStop();
  setRedLED(false);
}

inline void updateGreenLED(){ setGreenLED(armed); }

// ==============================
// BASELINE RECALIBRATION
// ==============================
void recalibrateBaseline() {
  if (!imuOK) return;
  float sum = 0.0f;
  int   cnt = 0;
  for (int i=0;i<50;i++) {
    sensors_event_t accel, gyro, temp;
    if (lsm6.getEvent(&accel, &gyro, &temp)) {
      float gx = accel.acceleration.x / MS2_PER_G;
      float gy = accel.acceleration.y / MS2_PER_G;
      float gz = accel.acceleration.z / MS2_PER_G;
      sum += mag3(gx, gy, gz);
      cnt++;
    }
    delay(10);
  }
  if (cnt > 0) {
    baselineMagG = sum / (float)cnt;
    Serial.print("Baseline recalibrated: ");
    Serial.println(baselineMagG, 4);
  }
}

// ==============================
// SOFT-LATCH POWER CONTROL HELPERS
// ==============================
inline void holdPower()    { digitalWrite(PIN_PWR_CTRL, HIGH); }
inline void releasePower() { digitalWrite(PIN_PWR_CTRL, LOW);  }

void requestPowerOff() {
  Serial.println("Power off requested, shutting down...");
  db.update();
  delay(100);
  releasePower();
  while (true) delay(100);
}

bool readPwrButtonDebounced() {
  uint32_t now = millis();
  bool raw = digitalRead(PIN_BTN_IN);

  if (raw != pwrBtnStableState) {
    if (now - pwrBtnLastChangeMs >= PWR_DEBOUNCE_MS) {
      pwrBtnPrevStable   = pwrBtnStableState;
      pwrBtnStableState  = raw;
      pwrBtnLastChangeMs = now;
    }
  } else {
    pwrBtnLastChangeMs = now;
  }
  return pwrBtnStableState;
}

void handlePowerButton() {
  uint32_t now = millis();
  bool level  = readPwrButtonDebounced();
  bool pressed = (level == PWRBTN_ACTIVE_LEVEL);

  if (pwrIgnoreFirstPress) {
    if (!pressed) {
      pwrIgnoreFirstPress = false;
      Serial.println("Power button first release detected, power-off detection armed.");
    }
    return;
  }

  if (pressed) {
    if (pwrBtnPrevStable != PWRBTN_ACTIVE_LEVEL) {
      pwrBtnPressStartMs = now;
    } else if (now - pwrBtnPressStartMs >= PWR_LONG_PRESS_MS) {
      Serial.println("Power button long press -> power off");
      requestPowerOff();
    }
  }

  pwrBtnPrevStable = level;
}

// ==============================
// AUTO-DISARM: learning + trigger
// ==============================
static inline bool accelMagOk(const Vec3& gNow) {
  float m = vMag(gNow);
  return (m >= MOVE_MAG_MIN_G && m <= MOVE_MAG_MAX_G);
}

static void learnStart(uint32_t now) {
  g_learnState   = LEARN_NORMAL;
  g_learnStartMs = now;
  g_phaseStartMs = now;

  g_sumN = {0,0,0};
  g_cntN = 0;

  g_maxPitchDev = 0.0f;
  g_maxRollDev  = 0.0f;

  oriBaseValid = false;
  moveOverSince = 0;

  shortBeep();
  Serial.println("AUTO-LEARN: started (10s NORMAL + 10s MOVED)");
}

static void learnFinish(bool movedMeaningful) {
  if (movedMeaningful) {
    cfg_learnUpDeg   = clampf(0.60f * g_maxPitchDev, 15.0f, 60.0f);
    cfg_learnTiltDeg = clampf(0.60f * g_maxRollDev,  15.0f, 70.0f);
    Serial.printf("AUTO-LEARN: derived UP=%.1f TILT=%.1f (maxP=%.1f maxR=%.1f)\n",
                  cfg_learnUpDeg, cfg_learnTiltDeg, g_maxPitchDev, g_maxRollDev);
  } else {
    cfg_learnUpDeg   = SMART_UP_DEFAULT_DEG;
    cfg_learnTiltDeg = SMART_TILT_DEFAULT_DEG;
    Serial.println("AUTO-LEARN: no meaningful moved detected -> fallback UP=25 TILT=35");
  }

  db.set(DBK_LEARN_UP_DEG,   cfg_learnUpDeg);
  db.set(DBK_LEARN_TILT_DEG, cfg_learnTiltDeg);

  shortBeep(); delay(80); shortBeep();
  g_learnState = LEARN_NONE;
}

static void learnTick(uint32_t now, const Vec3& gNow, float dDelta) {
  bool stable = accelMagOk(gNow) && (dDelta < (cfg_deltaG * 0.60f));

  if (g_learnState == LEARN_NORMAL) {
    if (stable) {
      g_sumN.x += gNow.x;
      g_sumN.y += gNow.y;
      g_sumN.z += gNow.z;
      g_cntN++;
    }

    if (now - g_phaseStartMs >= 10000UL) {
      Vec3 avg = gNow;
      if (g_cntN >= 30) {
        avg.x = g_sumN.x / (float)g_cntN;
        avg.y = g_sumN.y / (float)g_cntN;
        avg.z = g_sumN.z / (float)g_cntN;
      }

      gravityToPitchRoll(avg, basePitchDeg, baseRollDeg);
      oriBaseValid = true;
      moveOverSince = 0;

      g_learnState = LEARN_MOVED;
      g_phaseStartMs = now;

      Serial.printf("AUTO-LEARN: NORMAL done (cnt=%lu) basePitch=%.1f baseRoll=%.1f\n",
                    (unsigned long)g_cntN, basePitchDeg, baseRollDeg);
      shortBeep();
    }
    return;
  }

  if (g_learnState == LEARN_MOVED) {
    if (oriBaseValid && accelMagOk(gNow)) {
      float pitchDeg, rollDeg;
      gravityToPitchRoll(gNow, pitchDeg, rollDeg);

      float dP = fabsf(pitchDeg - basePitchDeg);
      float dR = fabsf(rollDeg  - baseRollDeg);

      if (dP > g_maxPitchDev) g_maxPitchDev = dP;
      if (dR > g_maxRollDev)  g_maxRollDev  = dR;
    }

    if (now - g_phaseStartMs >= 10000UL) {
      bool meaningful = (g_maxPitchDev >= 12.0f) || (g_maxRollDev >= 12.0f);
      learnFinish(meaningful);
    }
    return;
  }
}

static bool checkAutoDisarmMove(uint32_t now, const Vec3& gNow) {
  if (!cfg_autoDisarmEn) return false;
  if (!armed) return false;

  if (g_learnState != LEARN_NONE) return false;
  if (!oriBaseValid) return false;

  if (!accelMagOk(gNow)) {
    moveOverSince = 0;
    return false;
  }

  float upTh, tiltTh;
  if (cfg_autoSmart) {
    upTh   = cfg_learnUpDeg;
    tiltTh = cfg_learnTiltDeg;
  } else {
    upTh   = cfg_autoDisarmUpDeg;
    tiltTh = cfg_autoDisarmTiltDeg;
    if (upTh <= 0.0f && tiltTh <= 0.0f) return false;
  }

  float pitchDeg, rollDeg;
  gravityToPitchRoll(gNow, pitchDeg, rollDeg);

  float dPitch = fabsf(pitchDeg - basePitchDeg);
  float dRoll  = fabsf(rollDeg  - baseRollDeg);

  bool overUp   = (upTh   > 0.0f) && (dPitch >= upTh);
  bool overTilt = (tiltTh > 0.0f) && (dRoll  >= tiltTh);

  if (overUp || overTilt) {
    if (moveOverSince == 0) moveOverSince = now;
    if (now - moveOverSince >= MOVE_HOLD_MS) return true;
  } else {
    moveOverSince = 0;
  }
  return false;
}

// ==============================
// ARM/DISARM + SLEEP
// ==============================
void suspendUntilLongPressToArm(){
  setRedLED(false);
  setGreenLED(false);
  pinMode(BUTTON, INPUT_PULLUP);

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);

#if CONFIG_IDF_TARGET_ESP32C3
  uint64_t mask = 1ULL << (uint8_t)BUTTON;
  esp_deep_sleep_enable_gpio_wakeup(mask, ESP_GPIO_WAKEUP_GPIO_LOW);
#elif CONFIG_IDF_TARGET_ESP32C6
  uint64_t mask = 1ULL << (uint8_t)BUTTON;
  esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_LOW);
#else
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON, 0);
#endif

  esp_deep_sleep_start();
}

void setArmed(bool v,bool deepSleep){
  armed=v; cfg_armedSw=v; db.set(kk::armedSw,v); updateGreenLED();

  if(v){
    longBeep();

    stopPulse();
    inMotion = false;
    consecutiveTriggers = 0;

    wifiArmStart = millis();
    wifiApActiveAfterArm = true;

    if (espNowEnabled && hubMacValid) espNowBegin();

    oriBaseValid = false;
    moveOverSince = 0;

    if (cfg_autoDisarmEn && cfg_autoSmart) {
      learnStart(millis());
    }

  } else {
    recalibrateBaseline();

    espNowEnd();

    stopPulse();
    inMotion = false;
    consecutiveTriggers = 0;

    g_learnState = LEARN_NONE;

    shortBeep(); delay(80); shortBeep();
    wifiEnterConfigMode();

    if (deepSleep) suspendUntilLongPressToArm();
  }
}

// ==============================
// SETTINGS UI
// ==============================
bool saveReboot_f = false;
bool testAlarm_f  = false;

// UI-only IDs (must be size_t)
static const size_t UI_AUTO_DISARM_EN   = 1300;
static const size_t UI_AUTO_SMART       = 1301;
static const size_t UI_AUTO_UP          = 1302;
static const size_t UI_AUTO_TILT        = 1303;
static const size_t UI_AUTO_INFO        = 1304;

void build(sets::Builder& b){
  using namespace sets;

  // --- General group ---
  {
    Group g(b,L().grpGeneral);
    b.Select(kk::langIdx, L().lblLanguage, "English\nРусский", AnyPtr(&cfg_langIdx));
    b.Label(H_langHintLbl, L().langHint);

    const char* lblBuz = cfg_langRu ? "Громкость зуммера" : "Buzzer volume";
    const char* lblLR  = cfg_langRu ? "Яркость красного светодиода" : "Red LED brightness";
    const char* lblLG  = cfg_langRu ? "Яркость зелёного светодиода" : "Green LED brightness";

    b.Slider(kk::buzVol,     lblBuz, 0.0f, 100.0f, 1.0f, "%", AnyPtr(&cfg_buzVolPct));
    b.Slider(kk::ledRedLvl,  lblLR,  0.0f, 100.0f, 1.0f, "%", AnyPtr(&cfg_ledRedPct));
    b.Slider(kk::ledGreenLvl,lblLG,  0.0f, 100.0f, 1.0f, "%", AnyPtr(&cfg_ledGreenPct));

    if (b.Button(kk::testAlarm, L().lblTestAlarm)) testAlarm_f = true;
  }

  // --- Sensor group ---
  {
    Group g(b,L().grpSensor);

    b.Slider(kk::deltaG,      L().lblSensitivity, 0.02f, 1.0f, 0.01f, "",     AnyPtr(&cfg_deltaG));
    b.Slider(kk::shortPulse,  L().lblShortPulse,  40.0f, 1000.0f, 10.0f, "ms", AnyPtr(&cfg_shortPulseMs));
    b.Slider(kk::pulsePeriod, L().lblPulsePeriod, 100.0f, 2000.0f, 10.0f, "ms", AnyPtr(&cfg_pulsePeriod));
    b.Slider(kk::contThresh,  L().lblContThresh,  50.0f, 2000.0f, 10.0f, "ms", AnyPtr(&cfg_contThresh));
    b.Switch(kk::armedSw, L().lblArmedSwitch);

    const char* lblAuto = cfg_langRu ? "Автоснятие сигнализации" : "Auto-disarm";
    b.Switch(UI_AUTO_DISARM_EN, lblAuto, &cfg_autoDisarmEn);

    if (cfg_autoDisarmEn) {
      const char* lblSmart = "Smart-detection";
      b.Switch(UI_AUTO_SMART, lblSmart, &cfg_autoSmart);

      if (!cfg_autoSmart) {
        const char* lblUp   = cfg_langRu ? "Auto-disarm UP (градусы, 0=выкл.)"   : "Auto-disarm UP (deg, 0=OFF)";
        const char* lblTilt = cfg_langRu ? "Auto-disarm TILT (градусы, 0=выкл.)" : "Auto-disarm TILT (deg, 0=OFF)";
        b.Slider(UI_AUTO_UP,   lblUp,   0.0f, 90.0f, 1.0f, "deg", AnyPtr(&cfg_autoDisarmUpDeg));
        b.Slider(UI_AUTO_TILT, lblTilt, 0.0f, 90.0f, 1.0f, "deg", AnyPtr(&cfg_autoDisarmTiltDeg));
      } else {
        String info = String("Learn each ARM: 10s+10s, using UP=") + String(cfg_learnUpDeg,0) +
                      "°, TILT=" + String(cfg_learnTiltDeg,0) + "°";
        // ✅ FIX: Label must use numeric ID, not "smartInfo" string
        b.Label(UI_AUTO_INFO, info);
      }
    }
  }

  // --- ESP-NOW group ---
  {
    Group g(b, L().grpEspNow);
    b.Switch(kk::espNowEn, L().lblEspNowEnable);
    b.Input(kk::hubMac, L().lblHubMac);
  }

  // --- Wi-Fi group ---
  {
    Group g(b,L().grpWiFi);
    b.Input(kk::name,  L().lblName);
    b.Pass (kk::apPass,L().lblApPass);
    if (b.Button(kk::saveRB, L().lblSaveRestart)) saveReboot_f = true;
  }

  // --- Status group ---
  {
    Group g(b,L().grpStatus);
    b.Label(H_stateLbl,  L().lblState);  b.LED(H_stateLED);
    b.Label(H_accelLbl,  L().lblAccel);  b.LED(H_accelLED);
    b.LED(H_langLED);                    b.Label(H_langLbl,   L().lblLanguage);
    b.LED(H_battLED);                    b.Label(H_battLbl,   L().lblBattery);
    b.Label(H_deltaLbl,   L().lblDeltaG);
    b.Label(H_alertsLbl,  L().lblAlerts);
    b.Label(H_uptimeLbl,  L().lblUptime);
  }
}

void update(sets::Updater& u){
  // Persist core sliders
  db.set(kk::deltaG,      cfg_deltaG);
  db.set(kk::shortPulse,  (int)cfg_shortPulseMs);
  db.set(kk::pulsePeriod, (int)cfg_pulsePeriod);
  db.set(kk::contThresh,  (int)cfg_contThresh);
  db.set(kk::buzVol,      (int)cfg_buzVolPct);
  db.set(kk::ledRedLvl,   (int)cfg_ledRedPct);
  db.set(kk::ledGreenLvl, (int)cfg_ledGreenPct);

  // Persist auto-disarm switches + manual thresholds (string keys)
  cfg_autoDisarmUpDeg   = clampf(cfg_autoDisarmUpDeg,   0.0f, 90.0f);
  cfg_autoDisarmTiltDeg = clampf(cfg_autoDisarmTiltDeg, 0.0f, 90.0f);

  db.set(DBK_AUTO_DISARM_EN,        (int)(cfg_autoDisarmEn ? 1 : 0));
  db.set(DBK_AUTO_SMART_EN,         (int)(cfg_autoSmart ? 1 : 0));
  db.set(DBK_AUTO_DISARM_UP_DEG,    cfg_autoDisarmUpDeg);
  db.set(DBK_AUTO_DISARM_TILT_DEG,  cfg_autoDisarmTiltDeg);

  if (!cfg_autoDisarmEn) {
    oriBaseValid = false;
    moveOverSince = 0;
    g_learnState = LEARN_NONE;
  }

  // ESP-NOW settings
  if (auto e = db.get(kk::espNowEn))   cfg_espNowEn = e.toBool();
  if (auto e = db.get(kk::hubMac))     cfg_hubMacStr = e.toString();

  espNowEnabled = cfg_espNowEn;
  hubMacValid   = false;
  if (cfg_hubMacStr.length() > 0) {
    hubMacValid = parseMac(cfg_hubMacStr, hubMacAddr);
    if (!hubMacValid) u.alert(L().msgHubMacBad);
  }

  // Armed switch (web toggle)
  if (auto e = db.get(kk::armedSw)) cfg_armedSw = e.toBool();

  // Language index change detection
  if (cfg_langIdx != g_prevLangIdx) {
    g_prevLangIdx = cfg_langIdx;
    cfg_langRu = (cfg_langIdx == 1);
    db.set(kk::langIdx, cfg_langIdx);
    u.notice(L().msgLangChanged);
  } else {
    cfg_langRu = (cfg_langIdx == 1);
  }

  // Name/SSID/pass
  if (auto e = db.get(kk::name))   cfg_name   = e.toString();
  if (auto e = db.get(kk::apPass)) cfg_apPass = e.toString();

  // Sanitize
  cfg_deltaG       = clampf(cfg_deltaG, 0.02f, 1.0f);
  cfg_shortPulseMs = clamp16(cfg_shortPulseMs, 40, 1000);
  cfg_pulsePeriod  = clamp16(cfg_pulsePeriod,  100, 2000);
  cfg_contThresh   = clamp16(cfg_contThresh,   50, 2000);
  cfg_buzVolPct    = clamp16(cfg_buzVolPct,    0, 100);
  cfg_ledRedPct    = clamp16(cfg_ledRedPct,    0, 100);
  cfg_ledGreenPct  = clamp16(cfg_ledGreenPct,  0, 100);

  // Apply armed switch (no immediate deep sleep)
  if (cfg_armedSw != armed) setArmed(cfg_armedSw, false);

  // Save & restart for Wi-Fi creds / SSID
  if (saveReboot_f) {
    saveReboot_f = false;
    String newName = db.get(kk::name).toString();
    String newPass = db.get(kk::apPass).toString();
    newName.trim();
    if (newName.length() == 0 || newName.length() > 32) u.alert(L().msgNameBad);
    else if (!newPass.isEmpty() && newPass.length() < 8) u.alert(L().msgPassBad);
    else {
      db.set(kk::name, newName);
      db.set(kk::apPass, newPass);
      db.update();
      u.notice(L().msgSavedReboot);
      ESP.restart();
    }
  }

  // Web "Calibrate baseline & test alarm" button
  if (testAlarm_f) {
    testAlarm_f = false;
    recalibrateBaseline();
    startTestPulse();
  }

  // ---- Status ----
  u.update(H_stateLbl, armed ? String(L().valArmed) : String(L().valDisarmed));
  u.updateColor(H_stateLED, armed ? sets::Colors::Aqua : sets::Colors::Pink);

  u.update(H_accelLbl, imuOK ? String(L().valAccelOK) : String(L().valAccelErr));
  u.updateColor(H_accelLED, imuOK ? sets::Colors::Green : sets::Colors::Red);

  u.update(H_langLbl, cfg_langRu ? String("Русский") : String("English"));
  u.updateColor(H_langLED, cfg_langRu ? sets::Colors::Yellow : sets::Colors::Blue);

  // Battery
  {
    String txt = String(g_batt_pct) + "% (" + String(g_vbat_V, 2) + " V)";
    u.update(H_battLbl, txt);

    sets::Colors col =
      (g_batt_pct >= 60) ? sets::Colors::Green :
      (g_batt_pct >= 30) ? sets::Colors::Yellow :
                           sets::Colors::Red;

    u.updateColor(H_battLED, col);
  }

  u.update(H_deltaLbl,  String(g_lastDelta, 3));
  u.update(H_alertsLbl, (int)g_alerts);
  u.update(H_uptimeLbl, (uint32_t)(millis() / 1000));

  updateGreenLED();
}

// ==============================
// BUTTON HANDLER (SW1 – ARM/DISARM)
// ==============================
static void handleButton(uint32_t now){
  int raw = digitalRead(BUTTON);
  if (raw != btnPrev && (now - btnLastChange) >= DEBOUNCE_MS) {
    btnPrev = raw; btnLastChange = now;
    if (raw == LOW) { btnIsDown = true; btnDownAt = now; longFired = false; }
    else btnIsDown = false;
  }
  if (btnIsDown && !longFired && (now - btnDownAt) >= LONG_PRESS_MS) {
    longFired = true;
    if (armed) setArmed(false, false);
    else       setArmed(true,  false);
  }
}

// ==============================
// SETUP
// ==============================
void setup(){
  pinMode(PIN_PWR_CTRL, OUTPUT);
  holdPower();

  pinMode(PIN_BTN_IN, INPUT);

  pinMode(LED_GREEN,OUTPUT);
  pinMode(LED_RED,OUTPUT);
  pinMode(BUZZER,OUTPUT);
  pinMode(BUTTON,INPUT_PULLUP);
  pinMode(VBAT_PIN, INPUT);

  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("Fishing Bite Sensor starting...");

#ifdef ESP32
  setCpuFrequencyMhz(80);
#endif

  analogReadResolution(12);
  pwmInit();

#ifdef ESP32
  LittleFS.begin(true);
#else
  LittleFS.begin();
#endif
  db.begin();

  // Defaults (DB_KEYS)
  db.init(kk::name,       cfg_name);
  db.init(kk::deltaG,     cfg_deltaG);
  db.init(kk::shortPulse, (int)cfg_shortPulseMs);
  db.init(kk::pulsePeriod,(int)cfg_pulsePeriod);
  db.init(kk::contThresh, (int)cfg_contThresh);
  db.init(kk::armedSw,    false);
  db.init(kk::langIdx,    cfg_langIdx);
  db.init(kk::apPass,     cfg_apPass);
  db.init(kk::buzVol,      (int)cfg_buzVolPct);
  db.init(kk::ledRedLvl,   (int)cfg_ledRedPct);
  db.init(kk::ledGreenLvl, (int)cfg_ledGreenPct);
  db.init(kk::espNowEn,    false);
  db.init(kk::hubMac,      cfg_hubMacStr);
  db.init(kk::testAlarm,   0);

  // Defaults (string keys for auto-disarm)
  db.init(DBK_AUTO_DISARM_EN,        1);
  db.init(DBK_AUTO_SMART_EN,         1);
  db.init(DBK_AUTO_DISARM_UP_DEG,    cfg_autoDisarmUpDeg);
  db.init(DBK_AUTO_DISARM_TILT_DEG,  cfg_autoDisarmTiltDeg);
  db.init(DBK_LEARN_UP_DEG,          SMART_UP_DEFAULT_DEG);
  db.init(DBK_LEARN_TILT_DEG,        SMART_TILT_DEFAULT_DEG);

  // Load persisted (DB_KEYS)
  cfg_name         = db.get(kk::name).toString();
  cfg_deltaG       = db.get(kk::deltaG).toFloat();
  cfg_shortPulseMs = db.get(kk::shortPulse).toInt();
  cfg_pulsePeriod  = db.get(kk::pulsePeriod).toInt();
  cfg_contThresh   = db.get(kk::contThresh).toInt();
  cfg_armedSw      = db.get(kk::armedSw).toBool();
  cfg_langIdx      = db.get(kk::langIdx).toInt();
  cfg_langRu       = (cfg_langIdx == 1);
  cfg_apPass       = db.get(kk::apPass).toString();
  cfg_buzVolPct    = db.get(kk::buzVol).toInt();
  cfg_ledRedPct    = db.get(kk::ledRedLvl).toInt();
  cfg_ledGreenPct  = db.get(kk::ledGreenLvl).toInt();
  cfg_espNowEn     = db.get(kk::espNowEn).toBool();
  cfg_hubMacStr    = db.get(kk::hubMac).toString();

  // Load persisted (auto-disarm)
  cfg_autoDisarmEn     = (db.get(DBK_AUTO_DISARM_EN).toInt() != 0);
  cfg_autoSmart        = (db.get(DBK_AUTO_SMART_EN).toInt() != 0);
  cfg_autoDisarmUpDeg  = clampf(db.get(DBK_AUTO_DISARM_UP_DEG).toFloat(),   0.0f, 90.0f);
  cfg_autoDisarmTiltDeg= clampf(db.get(DBK_AUTO_DISARM_TILT_DEG).toFloat(), 0.0f, 90.0f);

  cfg_learnUpDeg       = clampf(db.get(DBK_LEARN_UP_DEG).toFloat(),   0.0f, 90.0f);
  cfg_learnTiltDeg     = clampf(db.get(DBK_LEARN_TILT_DEG).toFloat(), 0.0f, 90.0f);

  espNowEnabled = cfg_espNowEn;
  hubMacValid = false;
  if (cfg_hubMacStr.length() > 0) hubMacValid = parseMac(cfg_hubMacStr, hubMacAddr);

  wifiEnterConfigMode();

  sett.begin();
  sett.onBuild(build);
  sett.onUpdate(update);

  if (I2C_SDA_PIN>=0 && I2C_SCL_PIN>=0) Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  else Wire.begin();
  Wire.setClock(100000);

  if (!lsm6.begin_I2C(LSM6_ADDR)) {
    Serial.println("LSM6DS3 not found");
    imuOK = false;
  } else {
    imuOK = true;
    Serial.print("LSM6DS3 found @0x");
    Serial.println(LSM6_ADDR, HEX);
    lsm6.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6.setAccelDataRate(LSM6DS_RATE_104_HZ);
  }

  recalibrateBaseline();

  setArmed(cfg_armedSw,false);
  bootMs = millis();

  g_vbat_V   = readVbat();
  g_batt_pct = vbatToPercent(g_vbat_V);
  lastVbatMs = millis();

  uint32_t now = millis();
  bool raw = digitalRead(PIN_BTN_IN);
  pwrBtnStableState  = raw;
  pwrBtnPrevStable   = raw;
  pwrBtnLastChangeMs = now;
  pwrIgnoreFirstPress = true;
}

// ==============================
// LOOP
// ==============================
void loop(){
  sett.tick();
  uint32_t now = millis();

  handlePowerButton();
  handleButton(now);
  updateGreenLED();

  if (armed && wifiApActiveAfterArm && (now - wifiArmStart >= ARMED_AP_WINDOW_MS)) {
    wifiAfterArmWindow();
    wifiApActiveAfterArm = false;
  }

  if (!armed){
    bool clientConnected = (WiFi.softAPgetStationNum()>0);
    bool inGrace = (now - bootMs) < DISARMED_AP_GRACE_MS;
    if (!clientConnected && !inGrace) suspendUntilLongPressToArm();
  }

  if (now - lastVbatMs >= VBAT_PERIOD_MS) {
    lastVbatMs = now;
    g_vbat_V   = readVbat();
    g_batt_pct = vbatToPercent(g_vbat_V);

    if (armed && espNowEnabled && espNowInited && hubMacValid) {
      if (!lowBattReported && g_batt_pct <= LOW_BATT_THRESHOLD_PCT) {
        sendBitePacket(3);
        lowBattReported = true;
      } else if (lowBattReported && g_batt_pct > LOW_BATT_THRESHOLD_PCT + LOW_BATT_HYSTERESIS) {
        lowBattReported = false;
      }
    }
  }

  if (!armed) return;

  if (now - lastSample >= SAMPLE_INTERVAL_MS){
    lastSample = now;

    if (imuOK){
      sensors_event_t accel, gyro, temp;
      if (lsm6.getEvent(&accel, &gyro, &temp)) {

        float gx = accel.acceleration.x / MS2_PER_G;
        float gy = accel.acceleration.y / MS2_PER_G;
        float gz = accel.acceleration.z / MS2_PER_G;

        gravF.x = MOVE_GRAV_ALPHA*gravF.x + (1.0f - MOVE_GRAV_ALPHA)*gx;
        gravF.y = MOVE_GRAV_ALPHA*gravF.y + (1.0f - MOVE_GRAV_ALPHA)*gy;
        gravF.z = MOVE_GRAV_ALPHA*gravF.z + (1.0f - MOVE_GRAV_ALPHA)*gz;

        float mag = mag3(gx,gy,gz);
        baselineMagG = baselineMagG*0.999f + mag*0.001f;
        float d = fabsf(mag - baselineMagG);
        g_lastDelta = d;

        if (cfg_autoDisarmEn && cfg_autoSmart && (g_learnState != LEARN_NONE)) {
          learnTick(now, gravF, d);

          stopPulse();
          inMotion = false;
          consecutiveTriggers = 0;
          return;
        }

        if (cfg_autoDisarmEn && !cfg_autoSmart && !oriBaseValid && accelMagOk(gravF) && (d < (cfg_deltaG * 0.60f))) {
          gravityToPitchRoll(gravF, basePitchDeg, baseRollDeg);
          oriBaseValid = true;
          moveOverSince = 0;
          Serial.printf("AUTO-DISARM: baseline captured (manual) pitch=%.1f roll=%.1f\n", basePitchDeg, baseRollDeg);
        }

        if (cfg_autoDisarmEn && checkAutoDisarmMove(now, gravF)) {
          Serial.println("AUTO-DISARM: rod moved -> disarm");
          setArmed(false, false);
          return;
        }

        if (d >= cfg_deltaG){ if (consecutiveTriggers<255) consecutiveTriggers++; }
        else if (consecutiveTriggers>0) consecutiveTriggers--;

        bool motionNow = (consecutiveTriggers >= TRIGGER_SAMPLES);
        if (motionNow && !inMotion){
          inMotion = true; motionStartMs = now;
          startPulse(now, 1);
        } else if (!motionNow && inMotion){
          inMotion = false;
        }
      }
    }
  }

  if (pulseOn && (int32_t)(now - pulseOffAt) >= 0) stopPulse();

  if (inMotion){
    uint32_t motionDur = now - motionStartMs;
    if (motionDur >= cfg_contThresh){
      if (!pulseOn && (now - lastPulseStart) >= cfg_pulsePeriod){
        startPulse(now, 2);
      }
    }
  }
}
// ==============================