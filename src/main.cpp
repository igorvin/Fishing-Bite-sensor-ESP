/*******************************************************
  Fishing Bite Sensor – ESP32-S3 (Seeed XIAO)
  - Accelerometer: LSM6DS3 (Adafruit LSM6DS library)
  - UI/Settings:   GyverDBFile + SettingsGyver
  - Storage:       LittleFS
  - Features:
      * ARMED/DISARMED (long-press button)
      * Green LED = armed (with adjustable brightness)
      * Red LED + buzzer pulses on motion (volume & brightness adjustable)
      * Long beep when armed, two short when disarmed
      * DISARMED → deep-sleep after grace period
      * Persistent AP SSID/password via Settings
      * Language dropdown: English / Русский
      * Power save: Wi-Fi OFF when ARMED (after 3-min config window)
      * Battery monitor: % + voltage via ADC divider (100k/100k + 100nF)
********************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

// ---- LSM6DS3 (Adafruit LSM6DS) ----
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM6DS3TRC.h>   // for LSM6DS3; change to LSM6DS33 if needed

#include <LittleFS.h>
#include <GyverDBFile.h>
#include <SettingsGyver.h>

// ==============================
//            PINS (XIAO ESP32-S3)
// ==============================
// Header mapping reference (for clarity):
//   D0 -> GPIO1  (ADC1_CH0)   - used for VBAT sense
//   D1 -> GPIO2               - used for GREEN LED (MOSFET gate)
//   D2 -> GPIO3               - used for RED LED   (MOSFET gate)

#define LED_GREEN   2          // D1 / GPIO2
#define LED_RED     3          // D2 / GPIO3
#define BUZZER      8          // MOSFET gate to buzzer
#define BUTTON      4          // active-LOW (RTC-capable for EXT0)

// VBAT sense: D0 (GPIO1 / ADC1_CH0) on XIAO ESP32-S3 header
#define VBAT_PIN    1          // D0 / GPIO1 / ADC1_CH0

#define I2C_SDA_PIN -1
#define I2C_SCL_PIN -1

// ==============================
// LSM6DS3 CONFIG
// ==============================
const uint8_t LSM6_ADDR       = 0x6A;      // change to 0x6B if your board uses that
const float   MS2_PER_G       = 9.80665f;  // m/s^2 per g

const uint8_t  TRIGGER_SAMPLES    = 3;
const uint16_t LONG_PRESS_MS      = 1000;
const uint16_t DEBOUNCE_MS        = 30;
const uint16_t SAMPLE_INTERVAL_MS = 10;
const uint32_t CONFIG_GRACE_MS    = 3UL * 60UL * 1000UL;   // disarmed → AP + config grace
const uint32_t CONFIG_WINDOW_MS   = 3UL * 60UL * 1000UL;   // armed  → AP window before Wi-Fi OFF

// Battery read config
const float    VBAT_VREF          = 3.30f;   // ADC reference (3.3V)
const float    VBAT_DIV           = 2.00f;   // 100k / 100k divider
const float    VBAT_CAL           = 1.00f;   // calibration factor (tweak ±2%)
const uint16_t VBAT_SAMPLES       = 16;      // averaging
const uint16_t VBAT_ADC_MAX       = 4095;
const uint32_t VBAT_PERIOD_MS     = 5000;    // read every 5s

// ==============================
// STATE
// ==============================
Adafruit_LSM6DS3TRC lsm6;      // LSM6DS3/3TR-C instance (via Adafruit LSM6DS)
bool imuOK = false, armed = false;
uint32_t lastSample = 0, motionStartMs = 0, bootMs = 0;
uint8_t  consecutiveTriggers = 0;
float    baselineMagG = 1.0f;
bool     inMotion = false, pulseOn = false;
uint32_t pulseOffAt = 0, lastPulseStart = 0;

// button FSM
bool btnPrev = HIGH, btnIsDown = false, longFired = false;
uint32_t btnLastChange = 0, btnDownAt = 0;

// telemetry
volatile float    g_lastDelta = 0.0f;
volatile uint32_t g_alerts    = 0;

// ARMED config window timer
uint32_t wifiArmStart = 0;
bool     wifiStillOnAfterArm = false;

// Battery telemetry
float    g_vbat_V = 0.0f;
int      g_batt_pct = 0;
uint32_t lastVbatMs = 0;

// ==============================
// SETTINGS + DATABASE
// ==============================
DB_KEYS(
  kk,
  name, deltaG, shortPulse, pulsePeriod, contThresh,
  armedSw, langIdx, langRu, apPass, saveRB,
  buzVol, ledRedLvl, ledGreenLvl
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
  const char* grpGeneral; const char* grpSensor; const char* grpWiFi; const char* grpStatus;
  const char* lblLanguage; const char* langHint;
  const char* lblSensitivity; const char* lblShortPulse; const char* lblPulsePeriod;
  const char* lblContThresh; const char* lblArmedSwitch;
  const char* lblName; const char* lblApPass; const char* lblSaveRestart;
  const char* lblState; const char* lblAccel; const char* lblDeltaG;
  const char* lblAlerts; const char* lblUptime;
  const char* lblBattery;
  const char* valArmed; const char* valDisarmed; const char* valAccelOK; const char* valAccelErr;
  const char* msgNameBad; const char* msgPassBad; const char* msgSavedReboot; const char* msgLangChanged;
};

const LangPack LANG_EN = {
  "General","Sensor","Wireless settings","Status",
  "Language","(English / Русский)",
  "Sensitivity \xCE\x94g","Short pulse","Pulse period","Continuous threshold","Armed (web toggle)",
  "Sensor name (AP SSID)","AP password (Min 8 chars.)","Save & Restart",
  "State","Accelerometer","\xCE\x94g","Alerts","Uptime (s)",
  "Battery",
  "ARMED","DISARMED","OK","ERROR",
  "Sensor name / SSID (1..32 chars.)",
  "Password (min 8 chars.)",
  "Saved. Rebooting to apply AP SSID/password...",
  "Language changed. Refresh the page."
};

const LangPack LANG_RU = {
  u8"Общие",u8"Датчик",u8"Настройки Wi-Fi",u8"Статус",
  u8"Язык",u8"(Русский / English)",
  u8"Чувствительность Δg",u8"Короткий импульс",u8"Период импульсов",u8"Порог непрерывности",u8"Охрана (веб-переключатель)",
  u8"Имя датчика (SSID AP)",u8"Пароль AP (мин. 8 симв.)",u8"Сохранить и перезагрузить",
  u8"Состояние",u8"Акселерометр",u8"Δg",u8"Срабатывания",u8"Время работы (с)",
  u8"Батарея",
  u8"ОХРАНА",u8"СНЯТО",u8"ОК",u8"ОШИБКА",
  u8"Имя датчика / SSID 1–32 симв.",u8"Пароль (8 симв.).",
  u8"Сохранено. Перезагрузка для применения SSID/пароля...",
  u8"Язык изменён. Обновите страницу."
};

String   cfg_name="BiteSensor", cfg_apPass="";
float    cfg_deltaG=0.15f;
uint16_t cfg_shortPulseMs=120, cfg_pulsePeriod=300, cfg_contThresh=250;
bool     cfg_armedSw=false;

// Language control
int      cfg_langIdx=0;      // 0=EN, 1=RU
bool     cfg_langRu=false;
int      g_prevLangIdx=-1;

// New output settings (0..100 %)
uint16_t cfg_buzVolPct   = 100;   // buzzer volume %
uint16_t cfg_ledRedPct   = 100;   // red LED brightness %
uint16_t cfg_ledGreenPct = 100;   // green LED brightness %

inline const LangPack& L(){ return cfg_langRu?LANG_RU:LANG_EN; }

// ==============================
// BUZZER + LED PWM HELPERS (ESP32)
// ==============================
#ifdef ESP32
  #define BUZZER_LEDC_CH 0
  #define LED_GREEN_CH   1
  #define LED_RED_CH     2

  const uint8_t BUZZER_BITS = 10;   // 0..1023
  const uint8_t LED_BITS    = 8;    // 0..255

  static bool pwmReady = false;

  void pwmInit() {
    // Buzzer: 3 kHz, 10-bit
    ledcSetup(BUZZER_LEDC_CH, 3000, BUZZER_BITS);
    ledcAttachPin(BUZZER, BUZZER_LEDC_CH);
    ledcWrite(BUZZER_LEDC_CH, 0);

    // LEDs: 1 kHz, 8-bit
    ledcSetup(LED_GREEN_CH, 1000, LED_BITS);
    ledcSetup(LED_RED_CH,   1000, LED_BITS);
    ledcAttachPin(LED_GREEN, LED_GREEN_CH);
    ledcAttachPin(LED_RED,   LED_RED_CH);
    ledcWrite(LED_GREEN_CH, 0);
    ledcWrite(LED_RED_CH,   0);

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
    uint16_t d = buzzerDuty();
    ledcWrite(BUZZER_LEDC_CH, d);
  }

  inline void buzzStop(){
    if (!pwmReady) return;
    ledcWrite(BUZZER_LEDC_CH, 0);
  }

  inline void setGreenLED(bool on){
    if (!pwmReady) return;
    ledcWrite(LED_GREEN_CH, on ? ledDuty(cfg_ledGreenPct) : 0);
  }

  inline void setRedLED(bool on){
    if (!pwmReady) return;
    ledcWrite(LED_RED_CH, on ? ledDuty(cfg_ledRedPct) : 0);
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

// ==============================
// Battery read + mapping
// ==============================
float readVbat() {
  uint32_t acc = 0;
  analogRead(VBAT_PIN);                    // dummy to charge sampling cap
  for (int i=0;i<VBAT_SAMPLES;i++) acc += analogRead(VBAT_PIN);
  float adc = acc / float(VBAT_SAMPLES);
  float v = (adc / VBAT_ADC_MAX) * VBAT_VREF * VBAT_DIV * VBAT_CAL;
  return v;
}

// Rough Li-ion percentage curve (resting)
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
// Wi-Fi POWER MODES
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

// AP on for config (DISARMED or ARMED window)
void wifiEnterConfigMode() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.setHostname(cfg_name.c_str());
  configureAP(cfg_name, cfg_apPass);
  WiFi.setSleep(true);          // save power in AP idle
}

// Full Wi-Fi OFF for low power (ARMED after window)
void wifiEnterLowPowerMode() {
  WiFi.softAPdisconnect(true);
  WiFi.disconnect(true, true);
  delay(50);
  WiFi.mode(WIFI_OFF);
}

// ==============================
// ALERT PULSES / LEDS
// ==============================
inline void startPulse(uint32_t now){
  (void)now;
  pulseOn=true; lastPulseStart=now; pulseOffAt=now+cfg_shortPulseMs;
  setRedLED(true);
  buzzStart();
  g_alerts++;
}
inline void stopPulse(){
  pulseOn=false;
  buzzStop();
  setRedLED(false);
}

inline void updateGreenLED(){ setGreenLED(armed); }

// ==============================
// ARM/DISARM + SLEEP
// ==============================
void suspendUntilLongPressToArm(){
  setRedLED(false);
  setGreenLED(false);
  pinMode(BUTTON,INPUT_PULLUP);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON,0);
  esp_deep_sleep_start();
}

void setArmed(bool v,bool deepSleep){
  armed=v; cfg_armedSw=v; db.set(kk::armedSw,v); updateGreenLED();

  if(v){
    // Enter ARMED: long beep, keep AP ON for CONFIG_WINDOW_MS then turn Wi-Fi off
    longBeep();
    wifiArmStart = millis();
    wifiStillOnAfterArm = true;   // loop will switch to low-power after window
  } else {
    // Enter DISARMED: two short beeps, AP ON for user config, then
    // loop() will handle deep sleep after CONFIG_GRACE_MS
    shortBeep(); delay(80); shortBeep();
    wifiEnterConfigMode();

    if (deepSleep) {
      // optional immediate deep-sleep path (not used by button or web now)
      suspendUntilLongPressToArm();
    }
  }
}

// ==============================
// SETTINGS UI
// ==============================
bool saveReboot_f=false;

void build(sets::Builder& b){
  using namespace sets;

  // --- General group with language + output sliders ---
  {
    Group g(b,L().grpGeneral);
    b.Select(kk::langIdx, L().lblLanguage, "English\nРусский", AnyPtr(&cfg_langIdx));
    b.Label(H_langHintLbl, L().langHint);

    const char* lblBuz = cfg_langRu ? u8"Громкость буззера" : "Buzzer volume";
    const char* lblLR  = cfg_langRu ? u8"Яркость красного светодиода" : "Red LED brightness";
    const char* lblLG  = cfg_langRu ? u8"Яркость зелёного светодиода" : "Green LED brightness";

    b.Slider(kk::buzVol,    lblBuz, 0.0f, 100.0f, 1.0f, "%", AnyPtr(&cfg_buzVolPct));
    b.Slider(kk::ledRedLvl, lblLR,  0.0f, 100.0f, 1.0f, "%", AnyPtr(&cfg_ledRedPct));
    b.Slider(kk::ledGreenLvl,lblLG, 0.0f, 100.0f, 1.0f, "%", AnyPtr(&cfg_ledGreenPct));
  }

  // --- Sensor group ---
  {
    Group g(b,L().grpSensor);
    b.Slider(kk::deltaG,      L().lblSensitivity, 0.02f, 1.0f, 0.01f, "",     AnyPtr(&cfg_deltaG));
    b.Slider(kk::shortPulse,  L().lblShortPulse,  40.0f, 1000.0f, 10.0f, "ms", AnyPtr(&cfg_shortPulseMs));
    b.Slider(kk::pulsePeriod, L().lblPulsePeriod, 100.0f, 2000.0f, 10.0f, "ms", AnyPtr(&cfg_pulsePeriod));
    b.Slider(kk::contThresh,  L().lblContThresh,  50.0f, 2000.0f, 10.0f, "ms", AnyPtr(&cfg_contThresh));
    b.Switch(kk::armedSw, L().lblArmedSwitch);
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
    b.Label(H_accelLbl,  L().lblAccel);  b.LED(" ", H_accelLED);
    b.LED(H_langLED);                    b.Label(H_langLbl,   L().lblLanguage);
    b.LED(H_battLED);                    b.Label(H_battLbl,   L().lblBattery);
    b.Label(H_deltaLbl,   L().lblDeltaG);
    b.Label(H_alertsLbl,  L().lblAlerts);
    b.Label(H_uptimeLbl,  L().lblUptime);
  }
}

void update(sets::Updater& u){
  // Persist variable-bound sliders manually
  db.set(kk::deltaG,      cfg_deltaG);
  db.set(kk::shortPulse,  (int)cfg_shortPulseMs);
  db.set(kk::pulsePeriod, (int)cfg_pulsePeriod);
  db.set(kk::contThresh,  (int)cfg_contThresh);
  db.set(kk::buzVol,      (int)cfg_buzVolPct);
  db.set(kk::ledRedLvl,   (int)cfg_ledRedPct);
  db.set(kk::ledGreenLvl, (int)cfg_ledGreenPct);

  // Armed switch (web toggle)
  if (auto e = db.get(kk::armedSw)) cfg_armedSw = e.toBool();

  // Language index change detection from bound variable
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

  // Apply armed switch (also toggles Wi-Fi mode)
  // deepSleep=false: DISARMED always goes into grace-mode, not instant sleep
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

  // ---- Status ----
  u.update(H_stateLbl, armed ? String(L().valArmed) : String(L().valDisarmed));
  u.updateColor(H_stateLED, armed ? sets::Colors::Aqua : sets::Colors::Pink);

  u.update(H_accelLbl, imuOK ? String(L().valAccelOK) : String(L().valAccelErr));
  u.updateColor(H_accelLED, imuOK ? sets::Colors::Green : sets::Colors::Red);

  u.update(H_langLbl, cfg_langRu ? String("Русский") : String("English"));
  u.updateColor(H_langLED, cfg_langRu ? sets::Colors::Yellow : sets::Colors::Blue);

  // Battery line (text + LED color)
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

  // Apply new brightness immediately for green LED (state indicator)
  updateGreenLED();
}

// ==============================
// BUTTON HANDLER
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
    // Long press toggles armed <-> disarmed
    // DISARMED goes into grace-mode (no immediate sleep)
    if (armed) setArmed(false, false);
    else       setArmed(true,  false);
  }
}

// ==============================
// SETUP
// ==============================
void setup(){
  pinMode(LED_GREEN,OUTPUT); pinMode(LED_RED,OUTPUT);
  pinMode(BUZZER,OUTPUT);   pinMode(BUTTON,INPUT_PULLUP);
  pinMode(VBAT_PIN, INPUT);       // ADC input

  pwmInit();

  Serial.begin(115200); delay(100);

#ifdef ESP32
  LittleFS.begin(true);
#else
  LittleFS.begin();
#endif
  db.begin();

  // First-run defaults
  db.init(kk::name, cfg_name);
  db.init(kk::deltaG, cfg_deltaG);
  db.init(kk::shortPulse, (int)cfg_shortPulseMs);
  db.init(kk::pulsePeriod,(int)cfg_pulsePeriod);
  db.init(kk::contThresh,(int)cfg_contThresh);
  db.init(kk::armedSw, false);
  db.init(kk::langIdx, cfg_langIdx);   // new int index (0=EN,1=RU)
  db.init(kk::apPass,  cfg_apPass);
  db.init(kk::buzVol,      (int)cfg_buzVolPct);
  db.init(kk::ledRedLvl,   (int)cfg_ledRedPct);
  db.init(kk::ledGreenLvl, (int)cfg_ledGreenPct);

  // One-time migration from old bool key 'langRu' if present
  if (auto e = db.get(kk::langRu)) {
    bool legacyRu = e.toBool();
    db.set(kk::langIdx, legacyRu ? 1 : 0);
  }

  // Load persisted
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

  // --- Wi-Fi must be UP before SettingsGyver to avoid lwIP assert ---
  wifiEnterConfigMode();   // AP up now; ARMED path will shut off after 3 min

  // Settings UI
  sett.begin();
  sett.onBuild(build);
  sett.onUpdate(update);

  // I2C
  if (I2C_SDA_PIN>=0 && I2C_SCL_PIN>=0) Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  else Wire.begin();
  Wire.setClock(100000);

  // ---- LSM6DS3 init ----
  if (!lsm6.begin_I2C(LSM6_ADDR)) {   // pass address if needed
    Serial.println("LSM6DS3 not found");
    imuOK = false;
  } else {
    imuOK = true;
    Serial.print("LSM6DS3 found @0x");
    Serial.println(LSM6_ADDR, HEX);

    // Configure accelerometer (low range, reasonable data rate)
    lsm6.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
    lsm6.setAccelDataRate(LSM6DS_RATE_104_HZ);
  }

  // Baseline calibration
  if (imuOK){
    float sum=0;
    for(int i=0;i<50;i++){
      sensors_event_t accel, gyro, temp;
      if (lsm6.getEvent(&accel, &gyro, &temp)) {
        float gx = accel.acceleration.x / MS2_PER_G;
        float gy = accel.acceleration.y / MS2_PER_G;
        float gz = accel.acceleration.z / MS2_PER_G;
        sum += mag3(gx,gy,gz);
      }
      delay(10);
    }
    baselineMagG = sum/50.0f;
  }

  // Apply initial ARMED/DISARMED state and Wi-Fi mode
  setArmed(cfg_armedSw,false);
  bootMs = millis();

  // Prime a first battery reading
  g_vbat_V = readVbat();
  g_batt_pct = vbatToPercent(g_vbat_V);
  lastVbatMs = millis();
}

// ==============================
// LOOP
// ==============================
void loop(){
  sett.tick();
  uint32_t now = millis();

  handleButton(now);
  updateGreenLED();

  // ARMED: after 3 min window, turn Wi-Fi fully OFF
  if (armed && wifiStillOnAfterArm && (now - wifiArmStart >= CONFIG_WINDOW_MS)) {
    wifiEnterLowPowerMode();
    wifiStillOnAfterArm = false;
  }

  // DISARMED: keep AP for grace or while client connected, then deep sleep
  if (!armed){
    bool clientConnected = (WiFi.softAPgetStationNum()>0);
    bool inGrace = (now - bootMs) < CONFIG_GRACE_MS;
    if (!clientConnected && !inGrace) suspendUntilLongPressToArm();

    // still allow battery refresh while disarmed
  }

  // Battery refresh (every VBAT_PERIOD_MS)
  if (now - lastVbatMs >= VBAT_PERIOD_MS) {
    lastVbatMs = now;
    g_vbat_V = readVbat();
    g_batt_pct = vbatToPercent(g_vbat_V);
  }

  if (!armed) return;  // disarmed path handled above (sleep logic)

  // Motion sampling
  if (now - lastSample >= SAMPLE_INTERVAL_MS){
    lastSample = now;
    if (imuOK){
      sensors_event_t accel, gyro, temp;
      if (lsm6.getEvent(&accel, &gyro, &temp)) {
        // convert m/s^2 to g
        float gx = accel.acceleration.x / MS2_PER_G;
        float gy = accel.acceleration.y / MS2_PER_G;
        float gz = accel.acceleration.z / MS2_PER_G;

        float mag = mag3(gx,gy,gz);
        baselineMagG = baselineMagG*0.999f + mag*0.001f;
        float d = fabsf(mag - baselineMagG);
        g_lastDelta = d;

        if (d >= cfg_deltaG){ if (consecutiveTriggers<255) consecutiveTriggers++; }
        else if (consecutiveTriggers>0) consecutiveTriggers--;

        bool motionNow = (consecutiveTriggers >= TRIGGER_SAMPLES);
        if (motionNow && !inMotion){
          inMotion = true; motionStartMs = now; startPulse(now);
        } else if (!motionNow && inMotion){
          inMotion = false;
        }
      }
    }
  }

  // Pulse engine
  if (pulseOn && (int32_t)(now - pulseOffAt) >= 0) stopPulse();

  if (inMotion){
    uint32_t motionDur = now - motionStartMs;
    if (motionDur >= cfg_contThresh){
      if (!pulseOn && (now - lastPulseStart) >= cfg_pulsePeriod){
        startPulse(now);
      }
    }
  }
}
