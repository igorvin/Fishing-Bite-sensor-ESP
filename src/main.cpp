/*******************************************************
  Fishing Bite Sensor – ESP32-S3 (Seeed XIAO)
  - Accelerometer: BMI160 (DFRobot_BMI160)
  - UI/Settings:   GyverDBFile + SettingsGyver
  - Storage:       LittleFS
  - Features:
      * ARMED/DISARMED (long-press button)
      * Green LED = armed
      * Red LED + buzzer pulses on motion
      * Long beep when armed, two short when disarmed
      * DISARMED → deep-sleep after grace period
      * Persistent AP SSID/password via Settings
      * Language dropdown: English / Русский
********************************************************/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>

// --- fix BMI160 endian redefinition warning ---
#ifdef LITTLE_ENDIAN
  #undef LITTLE_ENDIAN
#endif
#include <DFRobot_BMI160.h>

#include <LittleFS.h>
#include <GyverDBFile.h>
#include <SettingsGyver.h>

// ==============================
//            PINS
// ==============================
#define LED_GREEN   5
#define LED_RED     6
#define BUZZER      8
#define BUTTON      4          // active-LOW (RTC-capable for EXT0)

#define I2C_SDA_PIN -1
#define I2C_SCL_PIN -1

// ==============================
const int8_t   I2C_ADDR           = 0x69;
const float    G_PER_LSB          = 1.0f / 16384.0f;
const uint8_t  TRIGGER_SAMPLES    = 3;
const uint16_t LONG_PRESS_MS      = 1000;
const uint16_t DEBOUNCE_MS        = 30;
const uint16_t SAMPLE_INTERVAL_MS = 10;
const uint32_t CONFIG_GRACE_MS    = 3UL * 60UL * 1000UL;   // 3 min

// ==============================
// STATE
// ==============================
DFRobot_BMI160 bmi160;
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

// ==============================
// SETTINGS + DATABASE
// ==============================
DB_KEYS(
  kk,
  name, deltaG, shortPulse, pulsePeriod, contThresh,
  armedSw, langRu, apPass, saveRB
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
#define H_langHintLbl H(langHintLbl)   // <— static hint label (instead of Note)

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
  const char* valArmed; const char* valDisarmed; const char* valAccelOK; const char* valAccelErr;
  const char* msgNameBad; const char* msgPassBad; const char* msgSavedReboot; const char* msgLangChanged;
};

const LangPack LANG_EN = {
  "General","Sensor","Wireless settings","Status",
  "Language","(English / Русский)",
  "Sensitivity \xCE\x94g","Short pulse","Pulse period","Continuous threshold","Armed (web toggle)",
  "Sensor name (AP SSID)","AP password (Min 8 chars.)","Save & Restart",
  "State","Accelerometer","\xCE\x94g","Alerts","Uptime (s)",
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
  u8"ОХРАНА",u8"СНЯТО",u8"ОК",u8"ОШИБКА",
  u8"Имя датчика / SSID 1–32 симв.",u8"Пароль (8 симв.).",
  u8"Сохранено. Перезагрузка для применения SSID/пароля...",
  u8"Язык изменён. Обновите страницу."
};

String   cfg_name="BiteSensor", cfg_apPass="";
float    cfg_deltaG=0.15f;
uint16_t cfg_shortPulseMs=120, cfg_pulsePeriod=300, cfg_contThresh=250;
bool     cfg_armedSw=false, cfg_langRu=false;

inline const LangPack& L(){ return cfg_langRu?LANG_RU:LANG_EN; }

// ==============================
// BUZZER HELPERS (ESP32-S3 safe)
// ==============================
#ifdef ESP32
  #ifndef BUZZER_LEDC_CH
    #define BUZZER_LEDC_CH 0
  #endif
  void buzzerInit() {
    ledcAttachPin(BUZZER, BUZZER_LEDC_CH);
    ledcWriteTone(BUZZER_LEDC_CH, 0);
  }
  inline void buzzStart(uint32_t fHz){ ledcWriteTone(BUZZER_LEDC_CH, fHz); }
  inline void buzzStop(){ ledcWriteTone(BUZZER_LEDC_CH, 0); }
#else
  void buzzerInit() {}
  inline void buzzStart(uint32_t fHz){ tone(BUZZER, fHz); }
  inline void buzzStop(){ noTone(BUZZER); }
#endif

inline void longBeep(){ buzzStart(3000); delay(600); buzzStop(); }
inline void shortBeep(){ buzzStart(3000); delay(150); buzzStop(); }

// ==============================
// HELPERS
// ==============================
inline float mag3(float x,float y,float z){return sqrtf(x*x+y*y+z*z);}
static inline uint16_t clamp16(uint16_t v,uint16_t lo,uint16_t hi){return v<lo?lo:(v>hi?hi:v);}
static inline float clampf(float v,float lo,float hi){return v<lo?lo:(v>hi?hi:v);}

inline void startPulse(uint32_t now){
  (void)now;
  pulseOn=true; lastPulseStart=now; pulseOffAt=now+cfg_shortPulseMs;
  digitalWrite(LED_RED,HIGH); buzzStart(3000); g_alerts++;
}
inline void stopPulse(){pulseOn=false; buzzStop(); digitalWrite(LED_RED,LOW);}
inline void updateGreenLED(){digitalWrite(LED_GREEN,armed?HIGH:LOW);}

void configureAP(const String& ssid,const String& pass){
  WiFi.softAPdisconnect(true); delay(100);
  if(pass.length()>=8) WiFi.softAP(ssid.c_str(),pass.c_str());
  else WiFi.softAP(ssid.c_str(),"");
  delay(200);
  Serial.printf("AP SSID: %s\n",ssid.c_str());
  Serial.println(WiFi.softAPIP());
}

// ==============================
// ARM/DISARM + SLEEP
// ==============================
void setArmed(bool v,bool deepSleep){
  armed=v; cfg_armedSw=v; db.set(kk::armedSw,v); updateGreenLED();
  if(v){ longBeep(); }
  else{
    shortBeep(); delay(80); shortBeep();
    if(deepSleep){
      digitalWrite(LED_RED,LOW); digitalWrite(LED_GREEN,LOW);
      pinMode(BUTTON,INPUT_PULLUP);
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
      esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON,0);
      // Alternative if BUTTON isn’t RTC-capable:
      // esp_sleep_enable_ext1_wakeup(1ULL << BUTTON, ESP_EXT1_WAKEUP_ALL_LOW);
      esp_deep_sleep_start();
    }
  }
}
void suspendUntilLongPressToArm(){
  digitalWrite(LED_RED,LOW); digitalWrite(LED_GREEN,LOW);
  pinMode(BUTTON,INPUT_PULLUP);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON,0);
  esp_deep_sleep_start();
}

// ==============================
// SETTINGS UI
// ==============================
bool saveReboot_f=false, langSeenThisUpdate=false;

void build(sets::Builder& b){
  using namespace sets;

  // --- General group with dropdown ---
  {
    Group g(b,L().grpGeneral);
    int langIdx = cfg_langRu ? 1 : 0;                 // 0=EN, 1=RU
    b.Select(kk::langRu, L().lblLanguage, "English\nРусский", AnyPtr(&langIdx));
    b.Label(H_langHintLbl, L().langHint);             // <— static text instead of Note()
  }

  // --- Sensor group ---
  {
    Group g(b,L().grpSensor);
    b.Slider(kk::deltaG,      L().lblSensitivity, 0.02f, 1.0f, 0.01f, "",   AnyPtr(&cfg_deltaG));
    b.Slider(kk::shortPulse,  L().lblShortPulse,  40.0f, 1000.0f, 10.0f, "ms", AnyPtr(&cfg_shortPulseMs));
    b.Slider(kk::pulsePeriod, L().lblPulsePeriod, 100.0f, 2000.0f, 10.0f, "ms", AnyPtr(&cfg_pulsePeriod));
    b.Slider(kk::contThresh,  L().lblContThresh,  50.0f, 2000.0f, 10.0f, "ms", AnyPtr(&cfg_contThresh));
    b.Switch(kk::armedSw, L().lblArmedSwitch);
  }

  // --- WiFi group ---
  {
    Group g(b,L().grpWiFi);
    b.Input(kk::name,  L().lblName);
    b.Pass (kk::apPass,L().lblApPass);
    if (b.Button(kk::saveRB, L().lblSaveRestart)) saveReboot_f = true;
  }

  // --- Status group ---
  {
    Group g(b,L().grpStatus);
    b.LED  (H_stateLED);  b.Label(H_stateLbl,  L().lblState);
    b.LED  (H_accelLED);  b.Label(H_accelLbl,  L().lblAccel);
    b.LED  (H_langLED);   b.Label(H_langLbl,   L().lblLanguage); // language badge
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

  // Armed switch
  if (auto e=db.get(kk::armedSw)) cfg_armedSw=e.toBool();

  // Language from dropdown (0/1 stored in DB). Any non-zero means RU.
  bool langPrev = cfg_langRu;
  if (auto e = db.get(kk::langRu)) {
    cfg_langRu = (e.toInt() != 0);   // 0 = EN, 1 = RU
  }
  if (cfg_langRu != langPrev && !langSeenThisUpdate) {
    langSeenThisUpdate = true;
    u.notice(L().msgLangChanged);
  }

  // Name/SSID/pass
  if (auto e=db.get(kk::name))   cfg_name = e.toString();
  if (auto e=db.get(kk::apPass)) cfg_apPass = e.toString();

  // Sanitize
  cfg_deltaG       = clampf(cfg_deltaG, 0.02f, 1.0f);
  cfg_shortPulseMs = clamp16(cfg_shortPulseMs, 40, 1000);
  cfg_pulsePeriod  = clamp16(cfg_pulsePeriod,  100, 2000);
  cfg_contThresh   = clamp16(cfg_contThresh,   50, 2000);

  // Apply armed switch
  if (cfg_armedSw != armed) setArmed(cfg_armedSw, !cfg_armedSw);

  // Save & restart for Wi-Fi creds / SSID
  if (saveReboot_f){
    saveReboot_f=false;
    String newName = db.get(kk::name).toString();
    String newPass = db.get(kk::apPass).toString();
    newName.trim();
    if (newName.length()==0 || newName.length()>32) u.alert(L().msgNameBad);
    else if (!newPass.isEmpty() && newPass.length()<8) u.alert(L().msgPassBad);
    else {
      db.set(kk::name,newName);
      db.set(kk::apPass,newPass);
      db.update();
      u.notice(L().msgSavedReboot);
      ESP.restart();
    }
  }

  // Live status labels/LEDs
  u.update(H_stateLbl,  armed ? String(L().valArmed) : String(L().valDisarmed));
  u.updateColor(H_stateLED, armed ? sets::Colors::Aqua : sets::Colors::Pink);

  u.update(H_accelLbl, imuOK ? String(L().valAccelOK) : String(L().valAccelErr));
  u.updateColor(H_accelLED, imuOK ? sets::Colors::Green : sets::Colors::Red);

  // language badge in Status
  u.update(H_langLbl, cfg_langRu ? String("Русский") : String("English"));
  u.updateColor(H_langLED, cfg_langRu ? sets::Colors::Yellow : sets::Colors::Blue);

  u.update(H_deltaLbl,  String(g_lastDelta, 3));
  u.update(H_alertsLbl, (int)g_alerts);
  u.update(H_uptimeLbl, (uint32_t)(millis()/1000));
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
    if (armed) setArmed(false, true);
    else       setArmed(true,  false);
  }
}

// ==============================
// SETUP
// ==============================
void setup(){
  pinMode(LED_GREEN,OUTPUT); pinMode(LED_RED,OUTPUT);
  pinMode(BUZZER,OUTPUT);   pinMode(BUTTON,INPUT_PULLUP);
  buzzerInit();

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
  db.init(kk::langRu,  cfg_langRu);
  db.init(kk::apPass,  cfg_apPass);

  // Load persisted
  cfg_name         = db.get(kk::name).toString();
  cfg_deltaG       = db.get(kk::deltaG).toFloat();
  cfg_shortPulseMs = db.get(kk::shortPulse).toInt();
  cfg_pulsePeriod  = db.get(kk::pulsePeriod).toInt();
  cfg_contThresh   = db.get(kk::contThresh).toInt();
  cfg_armedSw      = db.get(kk::armedSw).toBool();
  cfg_langRu       = db.get(kk::langRu).toBool();
  cfg_apPass       = db.get(kk::apPass).toString();

  // Wi-Fi + Settings UI
  WiFi.mode(WIFI_AP_STA);
  WiFi.setHostname(cfg_name.c_str());
  sett.begin();
  sett.onBuild(build);
  sett.onUpdate(update);

  // AP
  WiFi.softAPdisconnect(true);
  configureAP(cfg_name, cfg_apPass);

  // I2C + BMI160
  if (I2C_SDA_PIN>=0 && I2C_SCL_PIN>=0) Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  else Wire.begin();
  Wire.setClock(100000);

  if (bmi160.softReset()!=BMI160_OK) Serial.println("BMI160 reset FAILED");
  else if (bmi160.I2cInit(I2C_ADDR)!=BMI160_OK) Serial.println("BMI160 I2C init FAILED");
  else { imuOK=true; Serial.printf("BMI160 init OK @0x%02X\n", I2C_ADDR); }

  if (imuOK){
    float sum=0;
    for(int i=0;i<50;i++){
      int16_t a[3]={0};
      if(bmi160.getAccelData(a)==0)
        sum += mag3(a[0]*G_PER_LSB,a[1]*G_PER_LSB,a[2]*G_PER_LSB);
      delay(10);
    }
    baselineMagG = sum/50.0f;
  }

  setArmed(cfg_armedSw,false);
  bootMs = millis();
}

// ==============================
// LOOP
// ==============================
void loop(){
  sett.tick();
  uint32_t now = millis();

  handleButton(now);
  updateGreenLED();

  // Disarmed: keep AP for grace or when client connected, then deep sleep
  if (!armed){
    bool clientConnected = (WiFi.softAPgetStationNum()>0);
    bool inGrace = (now - bootMs) < CONFIG_GRACE_MS;
    if (!clientConnected && !inGrace) suspendUntilLongPressToArm();
    return;
  }

  // Motion sampling
  if (now - lastSample >= SAMPLE_INTERVAL_MS){
    lastSample = now;
    if (imuOK){
      int16_t a[3]={0};
      if (bmi160.getAccelData(a)==0){
        float gx=a[0]*G_PER_LSB, gy=a[1]*G_PER_LSB, gz=a[2]*G_PER_LSB;
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
