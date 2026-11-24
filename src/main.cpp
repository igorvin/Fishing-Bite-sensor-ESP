// Sensor bench release. Portable algorithms are shared with tests/replay.
#include <Arduino.h>
#include <atomic>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <esp_task_wdt.h>
#include <LittleFS.h>
#include <esp_partition.h>
#include <GyverDBFile.h>
#include <SettingsGyver.h>
#include <driver/gpio.h>
#include <soc/soc_caps.h>
#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED
#include <driver/rtc_io.h>
#endif
#include "BoardProfile.h"
#include "ImuDriver.h"
#include "SensorCore.h"
#include "CsvBuffer.h"
#include "ConfigDatabase.h"
#include "ImuSession.h"
#include "PowerButton.h"
#include "MobileUi.h"
#include "Feedback.h"
#include "EventLog.h"
#include "Sensitivity.h"
#include "../shared/StartupRadio.h"

// Keep original key hashes and 22-byte wire contract. New semantics use new keys.
DB_KEYS(kk,name,deltaG,shortPulse,pulsePeriod,contThresh,armedSw,langIdx,testAlarm,apPass,saveRB,buzVol,ledRedLvl,ledGreenLvl,espNowEn,hubMac);
struct __attribute__((packed)) BitePacket {char rodName[16];uint8_t eventType,batteryPct;float deltaG;};
static_assert(sizeof(BitePacket)==22,"Hub wire compatibility");
bite::ConfigDatabase<GyverDBFile> db(&LittleFS,"/config.db");
SettingsGyver sett("Fishing Bite Sensor - bench",&db);
bite::PowerButton button;
bite::LowBatteryGuard lowBattery;
ImuDriver imu;
bite::SensorCore sensor;
bite::ImuSession imuSession;
bite::Config draft;
bite::Pulse alarmPulse,beep;
bite::Pattern errorCue;
bite::ArmAssist armAssist;
bite::EventLog<32> journal;
bite::State loggedState=bite::State::Fault;
uint32_t heartbeatSince=0,settleLogAt=0;
// The long "armed" beep sounds when ARMED is actually reached after an explicit ARM request.
bool armBeepPending=false;
bite::CsvBuffer<2048> csvBuffer;
bite::Calibration previousCalibration=bite::Calibration::None;
bite::StateClock apClock,disarmedClock;
bool storageOK=false,pwmOK=false;
bool apActive=true,radioReady=false,radioEn=false,csv=false,armUi=false;
bool saveConfig=false,testRequest=false,networkRequest=false;
bool sleepAttempted=false;
// Simplified web UI (user decision 2026-11-24): sensitivity preset, volume, language and the
// network fields are the only settings; everything else is fixed to tested defaults.
uint8_t language=0,sensitivity=bite::defaultSensitivity;
uint16_t volume=100;
constexpr uint16_t pulseMs=120;  // alarm pulse; LEDs always at full brightness
String sensorName="BiteSensor",apPassword="",hubAddress="";
String draftName,draftPass,draftMac,notice="",sleepNotice="",radioNotice="";
uint32_t noticeSeq=0,noticeShown=0;
uint8_t peerMac[6]={0};
startup::Sensor registration;uint32_t sensorBoot=1;
startup::Ring<startup::Message,4> controlInbox;portMUX_TYPE controlMux=portMUX_INITIALIZER_UNLOCKED;
uint32_t controlTx=0,controlRx=0;char serialCommand[48]{};size_t serialLength=0;bool serialOverflow=false;
uint32_t lastBattery=0,lastRate=0,rateSamples=0;
uint32_t pulseCount=0,readErrors=0,noNew=0,csvDrops=0,sendErrors=0,saveNonce=0;
uint32_t rebootAt=0;
float sampleRate=0,vbat=0;int batteryPct=0;
std::atomic<uint32_t> txSuccess{0},txFail{0};
// Survive deep sleep (RTC slow memory); reported by the USB "status" command.
RTC_DATA_ATTR uint32_t rtcSleeps=0,rtcShortWakes=0;RTC_DATA_ATTR char rtcLastSleep[12]="none";
const char* wakeName="reset";
const char* tr(const char* en,const char* ru){return language?ru:en;}
// Important messages pop up in the web page (u.notice) and are echoed on USB.
void say(const String& text){notice=text;++noticeSeq;Serial.printf("SENSOR note: %s\n",text.c_str());}
// Journal entries are printed live on USB, except during CSV capture (they would corrupt the stream).
void note(uint32_t now,bite::Log type,uint8_t a=0,uint8_t b=0,uint8_t c=0,float v=0,float w=0){
    const auto& e=journal.add(now,type,a,b,c,v,w);
    if(csv)return;char line[200];bite::formatLog(e,line,sizeof(line));Serial.printf("SENSOR log %s\n",line);
}

namespace ui {
constexpr size_t lang=2001,arm=2002,vol=2014,name=2017,pass=2018,mac=2019,save=2021,test=2023,sensitivity=2025;
constexpr size_t state=2100,battery=2106,radioInfo=2110;
}

void stopOutputs(){alarmPulse.stop();beep.stop();errorCue.stop();}
void applyOutputs(uint32_t now){
    bool alarmOn=alarmPulse.tick(now),beepOn=beep.tick(now),cueOn=errorCue.tick(now);
    bool armed=sensor.state()==bite::State::Armed;
    // Red: alarm pulses and error cue; otherwise a short "awake" flash every 5 s in any state.
    // Deep sleep switches every output off (enterSleep), so the flashing stops only there.
    bool redOn=alarmOn||cueOn||bite::Heartbeat::on(now,heartbeatSince);
    if(!pwmOK)return;
    // Passive buzzer: 100% volume maps to 50% carrier duty, never DC.
    ledcWriteChannel(0,(alarmOn||beepOn||cueOn)?(uint32_t(volume)*512u/100u):0);
    // Green: solid when ARMED; blinking while an ARM request is still settling/calibrating/pending.
    auto st=sensor.state();
    bool arming=!armed&&(armAssist.pending()||(sensor.desiredArmed()&&(st==bite::State::Settling||st==bite::State::Calibrating)));
    bool greenOn=armed||(arming&&bite::elapsed(now,heartbeatSince)%500<250);
    ledcWriteChannel(1,greenOn?255:0);
    ledcWriteChannel(2,redOn?255:0);
}
void initOutputs(){
    // Separate frequencies/resolutions force separate timers in the pinned core.
    bool a=ledcAttachChannel(board::buzzer,3000,10,0);
    bool b=ledcAttachChannel(board::green,1000,8,1);
    bool c=ledcAttachChannel(board::red,1000,8,2);
    pwmOK=a&&b&&c;ledcWriteChannel(0,0);ledcWriteChannel(1,0);ledcWriteChannel(2,0);
}

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5,5,0)
void onSent(const wifi_tx_info_t*,esp_now_send_status_t s){if(s==ESP_NOW_SEND_SUCCESS)txSuccess.fetch_add(1,std::memory_order_relaxed);else txFail.fetch_add(1,std::memory_order_relaxed);}
#else
void onSent(const uint8_t*,esp_now_send_status_t s){if(s==ESP_NOW_SEND_SUCCESS)txSuccess.fetch_add(1,std::memory_order_relaxed);else txFail.fetch_add(1,std::memory_order_relaxed);}
#endif
void onControl(const esp_now_recv_info* info,const uint8_t* data,int len){
 startup::Message message;if(!info||len<0||!startup::decode(data,size_t(len),info->src_addr,message.packet))return;
 std::memcpy(message.mac.data(),info->src_addr,6);portENTER_CRITICAL(&controlMux);controlInbox.push(message);portEXIT_CRITICAL(&controlMux);
}
void radioEnd(){if(radioReady)esp_now_deinit();radioReady=false;registration.disable();portENTER_CRITICAL(&controlMux);controlInbox.clear();portEXIT_CRITICAL(&controlMux);}
bool reconcileRadio(){
    radioEnd();radioNotice="OFF";
    if(!radioEn)return true;
    uint8_t candidate[6];if(!bite::parseMac(hubAddress.c_str(),candidate)){radioNotice="invalid MAC";return false;}
    if(WiFi.getMode()==WIFI_OFF)WiFi.mode(WIFI_STA);
    if(esp_now_init()!=ESP_OK){radioNotice="init failed";return false;}
    if(esp_now_register_send_cb(onSent)!=ESP_OK||esp_now_register_recv_cb(onControl)!=ESP_OK){esp_now_deinit();radioNotice="callback failed";return false;}
    esp_now_peer_info_t p{};memcpy(p.peer_addr,candidate,6);p.channel=0;p.encrypt=false;p.ifidx=WIFI_IF_STA;
    if(esp_now_add_peer(&p)!=ESP_OK){esp_now_deinit();radioNotice="peer failed";return false;}
    memcpy(peerMac,candidate,6);registration.begin(peerMac,sensorBoot);radioReady=true;radioNotice="ready (MAC delivery only)";return true;
}
bool sendPacket(uint8_t type){
    if(!radioReady)return false;
    BitePacket packet{};startup::copyName(packet.rodName,sensorName.c_str());
    packet.eventType=type;packet.batteryPct=uint8_t(constrain(batteryPct,0,100));
    packet.deltaG=sensor.config().legacyMode?sensor.telemetry().signal.legacy:sensor.telemetry().signal.dynamic;
    esp_err_t result=esp_now_send(peerMac,reinterpret_cast<uint8_t*>(&packet),sizeof(packet));
    if(result!=ESP_OK){++sendErrors;return false;}return true;
}
void enterConfig(uint32_t now){
    WiFi.mode(WIFI_AP_STA);
    apActive=WiFi.softAP(sensorName.c_str(),apPassword.length()?apPassword.c_str():nullptr);
    WiFi.setSleep(true);apClock.enter(now);
}
void setArmed(bool value,uint32_t now,bite::Source source){
    stopOutputs();armAssist.cancel();bool was=sensor.desiredArmed();
    if(!sensor.setArmed(value,now)){armUi=false;say(tr("Cannot arm: sensor error","Нельзя включить охрану: ошибка датчика"));note(now,bite::Log::Arm,uint8_t(source),value,bite::Refused);return;}
    note(now,bite::Log::Arm,uint8_t(source),value,uint8_t(sensor.state()));
    armUi=sensor.desiredArmed();db.set(kk::armedSw,armUi);
    if(value&&!was){apClock.enter(now);armBeepPending=true;}
    if(!value){armBeepPending=false;disarmedClock.enter(now);sleepAttempted=false;beep.start(now,120);enterConfig(now);}
}
void updateBattery(){
    uint32_t mv=0;for(int i=0;i<8;++i)mv+=analogReadMilliVolts(board::battery);
    vbat=(mv/8.f)*.002f;
    if(vbat>=4.18f)batteryPct=100;else if(vbat>=4.10f)batteryPct=90;else if(vbat>=3.98f)batteryPct=80;
    else if(vbat>=3.85f)batteryPct=70;else if(vbat>=3.80f)batteryPct=60;else if(vbat>=3.75f)batteryPct=50;
    else if(vbat>=3.70f)batteryPct=40;else if(vbat>=3.63f)batteryPct=30;else if(vbat>=3.55f)batteryPct=20;
    else if(vbat>=3.45f)batteryPct=10;else batteryPct=0;
}
bool flushSettings(){
    if(!storageOK)return false;
    // Force a dirty marker so false from update is a genuine write failure.
    db.set("benchSave",++saveNonce);return db.update();
}
void serviceRegistration(uint32_t now){
 startup::Message message;
 for(unsigned i=0;i<4;++i){portENTER_CRITICAL(&controlMux);bool present=controlInbox.pop(message);portEXIT_CRITICAL(&controlMux);if(!present)break;
  if(registration.receive(message.mac.data(),message.packet)){++controlRx;if(registration.confirmed()){radioNotice="registered at startup (not live link)";Serial.printf("SENSOR registration confirmed node=%u\n",registration.node());}}
 }
 if(radioReady&&registration.due(now)){
  auto packet=registration.announce(now,sensorName.c_str(),startup::batteryKnown(vbat),uint8_t(constrain(batteryPct,0,100)));
  if(esp_now_send(peerMac,reinterpret_cast<const uint8_t*>(&packet),sizeof(packet))==ESP_OK)++controlTx;else ++sendErrors;
 }
}
bool enterSleep(const char* reason,bool force,uint32_t now);
const char* calibrationCode(bite::Calibration c){
    switch(c){case bite::Calibration::None:return "none";case bite::Calibration::Running:return "running";case bite::Calibration::Success:return "success";case bite::Calibration::Moving:return "rejected_movement";case bite::Calibration::SensorError:return "rejected_imu";}
    return "none";
}
void serviceSerial(uint32_t now){
 unsigned consumed=0;while(Serial.available()&&consumed++<64){char ch=char(Serial.read());
  if(ch=='\r'||ch=='\n'){
   serialCommand[serialLength]=0;
   if(serialOverflow)Serial.println("SENSOR error: command too long");
   else if(!std::strcmp(serialCommand,"status"))Serial.printf("SENSOR status radio=%u registered=%u attempts=%u node=%u control_rx=%lu control_tx=%lu state=%s imu=%u storage=%u pwm=%u calibrated=%u cal=%s want=%u arm_pending=%u cal_retries=%u board=%s wake=%s sleeps=%lu short_wakes=%lu last_sleep=%s button=%u vbat=%.2f sensitivity=%u noise=%.4f ambient=%.4f threshold=%.3f rate_hz=%.1f events=%lu i2c_errors=%lu csv_dropped=%lu\n",radioReady,registration.confirmed(),registration.attempts(),registration.node(),(unsigned long)controlRx,(unsigned long)controlTx,bite::stateName(sensor.state()),sensor.healthy(),storageOK,pwmOK,sensor.calibrated(),calibrationCode(sensor.calibration()),sensor.desiredArmed(),armAssist.pending(),armAssist.retries(),board::name,wakeName,(unsigned long)rtcSleeps,(unsigned long)rtcShortWakes,rtcLastSleep,digitalRead(board::button)==LOW,vbat,unsigned(sensitivity),sensor.telemetry().noiseG,sensor.telemetry().ambientG,sensor.telemetry().effectiveG,sampleRate,(unsigned long)sensor.telemetry().events,(unsigned long)readErrors,(unsigned long)csvDrops);
   else if(!std::strcmp(serialCommand,"log")){
    Serial.printf("SENSOR log: %u of %lu entries kept, oldest first\n",unsigned(journal.size()),(unsigned long)journal.total());
    for(size_t i=0;i<journal.size();++i){char line[200];bite::formatLog(journal.at(i),line,sizeof(line));Serial.printf("  %s\n",line);}
   }
   else if(!std::strcmp(serialCommand,"register")){
    if(sensor.desiredArmed())Serial.println("SENSOR error: disarm before register");
    else if(!radioReady)Serial.println("SENSOR error: radio disabled");
    else {sensorBoot=esp_random();if(!sensorBoot)sensorBoot=1;registration.begin(peerMac,sensorBoot);Serial.println("SENSOR diagnostic registration requested; no bite event");}
   }else if(!std::strncmp(serialCommand,"peer ",5)){
    uint8_t candidate[6];
    if(sensor.desiredArmed())Serial.println("SENSOR error: disarm before peer setup");
    else if(!storageOK||!bite::parseMac(serialCommand+5,candidate))Serial.println("SENSOR error: invalid peer or storage unavailable");
    else {
     db.set(kk::hubMac,String(serialCommand+5));db.set(kk::espNowEn,true);
     if(flushSettings()){hubAddress=serialCommand+5;radioEn=true;draftMac=hubAddress;reconcileRadio();Serial.println("SENSOR peer saved; radio enabled");}
     else {db.set(kk::hubMac,hubAddress);db.set(kk::espNowEn,radioEn);Serial.println("SENSOR error: peer save failed; live configuration unchanged");}
    }
   }else if(!std::strcmp(serialCommand,"sleep")){
    if(sensor.desiredArmed())Serial.println("SENSOR error: disarm before sleep");
    else if(!enterSleep("serial",false,now))Serial.printf("SENSOR sleep refused: %s\n",sleepNotice.c_str());
   }else if(!std::strcmp(serialCommand,"csv on")){
    // Full-rate capture for analysis (moved from the web UI). Live journal lines pause meanwhile.
    Serial.println("SENSOR csv on");csvBuffer.clear();csv=true;
    const char* header="ms,ax_g,ay_g,az_g,dynamic_g,legacy_g,noise_g,threshold_g,state,event,mode,impulse_g,ambient_g\n";csvBuffer.append(header,strlen(header));
   }else if(!std::strcmp(serialCommand,"csv off")){csv=false;csvBuffer.clear();Serial.println("SENSOR csv off");
   }else if(serialLength)Serial.println("SENSOR commands: status, log, csv on, csv off, register, sleep, peer AA:BB:CC:DD:EE:FF (disarmed only)");
   // same timestamp as the idle check in loop(): a later millis() would wrap elapsed()
   if(serialLength)disarmedClock.enter(now);
   serialLength=0;serialOverflow=false;
  }else if(serialLength+1<sizeof(serialCommand))serialCommand[serialLength++]=ch;else serialOverflow=true;
 }
}
void persistConfig(){
    language=language==1?1:0;sensitivity=bite::clampSensitivity(sensitivity);volume=constrain(volume,0,100);
    bite::Config next=bite::configForSensitivity(sensitivity);
    if(!sensor.configure(next)){say(tr("Invalid setting rejected","Неверная настройка отклонена"));return;}
    draft=next;
    // Only the settings still in the UI are written; older keys stay untouched in /config.db.
    db.set("benchSchema",2);db.set("sensitivity",int(sensitivity));
    db.set("dynamicThresholdG",draft.thresholdG);db.set("noiseMultiplier",draft.noiseMultiplier);
    db.set(kk::buzVol,int(volume));db.set(kk::langIdx,int(language));
    if(flushSettings())notice=tr("Saved","Сохранено");else say(tr("Save failed: settings are RAM only","Ошибка записи: настройки только в RAM"));
}
void loadConfig(){
    db.init(kk::name,sensorName);db.init(kk::apPass,apPassword);db.init(kk::hubMac,hubAddress);db.init(kk::espNowEn,false);
    db.init(kk::buzVol,100);db.init(kk::langIdx,0);
    db.init("calNoiseG",0.f);sensor.restoreNoise(db.get("calNoiseG").toFloat());
    db.init("dynamicThresholdG",.02f);db.init("sensitivity",-1);
    // Do not read a saved arm intent after boot: calibration and explicit ARM are required.
    sensorName=db.get(kk::name).toString();apPassword=db.get(kk::apPass).toString();hubAddress=db.get(kk::hubMac).toString();
    // A filled-in hub MAC enables the radio (the separate ESP-NOW switch was removed).
    radioEn=hubAddress.length()>0;
    if(!bite::validNetwork(sensorName.c_str(),apPassword.c_str())){sensorName="BiteSensor";apPassword="";say("Invalid saved AP settings; using recovery AP");}
    // Older configurations stored only a threshold: start from the closest preset.
    int stored=db.get("sensitivity").toInt();
    sensitivity=stored<0?bite::nearestSensitivity(db.get("dynamicThresholdG").toFloat()):bite::clampSensitivity(stored);
    draft=bite::configForSensitivity(sensitivity);sensor.configure(draft);
    volume=constrain(db.get(kk::buzVol).toInt(),0,100);language=db.get(kk::langIdx).toInt()==1?1:0;
    saveNonce=db.get("benchSave").toInt();draftName=sensorName;draftPass=apPassword;draftMac=hubAddress;
}
String stateText(){
    auto st=sensor.state();bool arming=armAssist.pending()||(sensor.desiredArmed()&&(st==bite::State::Settling||st==bite::State::Calibrating));
    if(st==bite::State::Armed)return tr("ARMED","На охране");
    if(arming)return tr("Arming…","Включается…");
    if(st==bite::State::MotionSuppressed)return tr("Paused: rod is being moved","Пауза: удилище двигают");
    if(st==bite::State::Fault)return tr("Sensor error","Ошибка датчика");
    if(st==bite::State::Calibrating)return tr("Calibrating…","Калибровка…");
    return tr("Off","Выключена");
}
void build(sets::Builder& b){
    using namespace sets;
    b.Custom("BenchLayout",2199);
    {Group g(b,tr("Sensor","Датчик"));
        b.Label(ui::state,tr("State","Состояние"));
        b.Label(ui::battery,tr("Battery","Батарея"));
        b.Label(ui::radioInfo,tr("Hub","Хаб"));
    }
    {Group g(b,tr("Control","Управление"));
        if(b.Switch(ui::arm,tr("Armed","Охрана"),&armUi))setArmed(armUi,millis(),bite::Source::Web);
        if(b.Select(ui::sensitivity,tr("Sensitivity","Чувствительность"),tr("Low\nBelow medium\nMedium\nHigh\nVery high","Низкая\nНиже средней\nСредняя\nВысокая\nОчень высокая"),AnyPtr(&sensitivity)))saveConfig=true;
        if(b.Slider(ui::vol,tr("Volume","Громкость"),0,100,5,"%",AnyPtr(&volume)))saveConfig=true;
        if(b.Button(ui::test,tr("Test signal","Проверить сигнал")))testRequest=true;
    }
    {Menu m(b,tr("Settings","Настройки"));
        {Group g(b,tr("Rod and hub","Удилище и хаб"));
            b.Input(ui::name,tr("Rod name","Имя удилища"),AnyPtr(&draftName));
            b.Pass(ui::pass,tr("Wi-Fi password","Пароль Wi-Fi"),AnyPtr(&draftPass));
            b.Input(ui::mac,tr("Hub MAC","MAC хаба"),AnyPtr(&draftMac));
            b.Paragraph("",tr("The rod name is the Wi-Fi name and is shown on the hub. The hub shows its MAC for 20 s after power-on. Password: empty or 8+ characters.",
                              "Имя удилища — это имя Wi-Fi, оно же видно на экране хаба. MAC хаба показан на его экране 20 секунд после включения. Пароль: пустой или от 8 символов."));
            if(b.Button(ui::save,tr("Save and restart","Сохранить и перезапустить")))networkRequest=true;
        }
        {Group g(b,tr("Language","Язык"));
            if(b.Select(ui::lang,"Language / Язык","English\nРусский",AnyPtr(&language))){saveConfig=true;b.reload();}
        }
    }
}
void update(sets::Updater& u){
    u.update(ui::state,stateText());
    u.update(ui::arm,sensor.desiredArmed());
    u.update(ui::battery,vbat>=2.5f&&vbat<=4.5f?"≈"+String(batteryPct)+"%":String(tr("no reading","нет данных")));
    u.update(ui::radioInfo,!radioEn?tr("Not set up: enter its MAC in Settings","Не настроен: впишите MAC в Настройках")
        :!radioReady?tr("Radio error","Ошибка радио")
        :registration.confirmed()?tr("Answered at power-on","Ответил при включении"):tr("Waiting for the hub","Ждём ответа хаба"));
    if(noticeShown!=noticeSeq){noticeShown=noticeSeq;if(notice.length())u.notice(notice);}
}

bool armWake(){
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);esp_err_t result=ESP_ERR_INVALID_ARG;
    gpio_num_t pin=gpio_num_t(board::button);
#if CONFIG_IDF_TARGET_ESP32C3
    // Low-level GPIO wake; the sleep driver enables the internal pull-up of a wake pin.
    gpio_pullup_en(pin);gpio_pulldown_dis(pin);
    result=esp_deep_sleep_enable_gpio_wakeup(1ULL<<board::button,ESP_GPIO_WAKEUP_GPIO_LOW);
#elif CONFIG_IDF_TARGET_ESP32C6
    result=esp_sleep_enable_ext1_wakeup(1ULL<<board::button,ESP_EXT1_WAKEUP_ANY_LOW);
#else
    result=esp_sleep_enable_ext0_wakeup(pin,0);
#endif
#if SOC_RTCIO_INPUT_OUTPUT_SUPPORTED && !CONFIG_IDF_TARGET_ESP32C3
    // The bench has no external pull-up; v4 also has R3 10k.
    if(result==ESP_OK){rtc_gpio_pullup_en(pin);rtc_gpio_pulldown_dis(pin);}
#endif
    return result==ESP_OK;
}
// force: button / low battery always sleep; idle and serial sleep wait for a successful settings flush.
bool enterSleep(const char* reason,bool force,uint32_t now){
    sleepAttempted=true;
    if(!board::wakeAllowed()){sleepNotice=tr("Deep sleep blocked: button pin cannot wake this board","Глубокий сон запрещён: вывод кнопки не будит эту плату");return false;}
    if(!button.released()){sleepAttempted=false;sleepNotice=tr("Sleep postponed: button is pressed","Сон отложен: кнопка нажата");return false;}
    if(sensor.desiredArmed()){sensor.setArmed(false,now);armUi=false;db.set(kk::armedSw,false);}
    armAssist.cancel();
    bool saved=flushSettings();
    if(!saved&&!force){sleepNotice=tr("Sleep postponed: settings save failed","Сон отложен: ошибка записи настроек");return false;}
    if(!armWake()){sleepNotice="Sleep blocked: wake API rejected pin";return false;}
    stopOutputs();applyOutputs(now);
    if(pwmOK){ledcWriteChannel(0,0);ledcWriteChannel(1,0);ledcWriteChannel(2,0);}
    imu.powerDown();
    radioEnd();WiFi.softAPdisconnect(true);WiFi.mode(WIFI_OFF);
    note(now,bite::Log::Sleep,uint8_t(!std::strcmp(reason,"idle")?bite::SleepReason::Idle:!std::strcmp(reason,"battery")?bite::SleepReason::Battery:!std::strcmp(reason,"serial")?bite::SleepReason::Serial:bite::SleepReason::Button));
    std::strncpy(rtcLastSleep,reason,sizeof(rtcLastSleep)-1);rtcLastSleep[sizeof(rtcLastSleep)-1]=0;++rtcSleeps;
    Serial.printf("SENSOR sleep reason=%s saved=%u; hold the button 1 s to wake\n",reason,saved);Serial.flush();delay(30);
    esp_deep_sleep_start();
    return false;
}

bool mountStorage(){
    if(LittleFS.begin(false))return true;
    // A fresh, fully erased partition contains no user data. Never auto-format a
    // nonblank partition after a mount failure: it may contain recoverable data.
    const esp_partition_t* part=esp_partition_find_first(ESP_PARTITION_TYPE_DATA,ESP_PARTITION_SUBTYPE_ANY,"spiffs");
    if(!part)part=esp_partition_find_first(ESP_PARTITION_TYPE_DATA,ESP_PARTITION_SUBTYPE_ANY,"littlefs");
    if(!part)return false;
    uint8_t block[1024];
    for(size_t offset=0;offset<part->size;offset+=sizeof(block)){
        size_t count=std::min<size_t>(sizeof(block),part->size-offset);
        if(esp_partition_read(part,offset,block,count)!=ESP_OK)return false;
        for(size_t i=0;i<count;++i)if(block[i]!=0xff)return false;
    }
    return LittleFS.format()&&LittleFS.begin(false);
}

void setup(){
    pinMode(board::button,INPUT_PULLUP);
    auto cause=esp_sleep_get_wakeup_cause();
    bool buttonWake=cause==ESP_SLEEP_WAKEUP_GPIO||cause==ESP_SLEEP_WAKEUP_EXT0||cause==ESP_SLEEP_WAKEUP_EXT1;
    wakeName=buttonWake?"button":cause==ESP_SLEEP_WAKEUP_UNDEFINED?"reset":"other";
    if(buttonWake){
        // Before radio/IMU start: a short touch goes straight back to sleep.
        bite::WakeConfirm confirm;confirm.begin(millis());bite::WakeResult r;
        while((r=confirm.update(digitalRead(board::button)==LOW,millis()))==bite::WakeResult::Waiting)delay(5);
        if(r==bite::WakeResult::Rejected&&armWake()){++rtcShortWakes;++rtcSleeps;esp_deep_sleep_start();}
    }
    pinMode(board::battery,INPUT);
    Serial.begin(115200);sensorBoot=esp_random();if(!sensorBoot)sensorBoot=1;setCpuFrequencyMhz(80);initOutputs();
    analogReadResolution(12);analogSetPinAttenuation(board::battery,ADC_11db);
    storageOK=mountStorage();if(storageOK)storageOK=db.begin();
    if(!storageOK)say("Storage error; RAM settings only (not formatting existing data)");
    loadConfig();Wire.begin(board::sda,board::scl);Wire.setClock(400000);Wire.setTimeOut(20);
    uint32_t now=millis();imuSession.begin(imu,sensor,now);now=millis();enterConfig(now);reconcileRadio();disarmedClock.enter(now);
    sleepNotice=board::wakeAllowed()?tr("Deep sleep: hold button 3 s; wake: hold 1 s","Сон: удержать кнопку 3 с; пробуждение: удержать 1 с"):tr("Button pin cannot deep-wake this board: sleep disabled","Вывод кнопки не будит эту плату: сон отключён");
    sett.setCustom(benchMobileJs,strlen_P(benchMobileJs));
    sett.begin();sett.onBuild(build);sett.onUpdate(update);
    updateBattery();lastBattery=lastRate=now;button.begin(digitalRead(board::button)==LOW,millis());
    if(buttonWake)beep.start(millis(),150);
    heartbeatSince=millis();
    note(millis(),bite::Log::Boot,uint8_t(buttonWake?bite::Wake::Button:cause==ESP_SLEEP_WAKEUP_UNDEFINED?bite::Wake::Reset:bite::Wake::Other));
    // Before the first IMU frame the core reports FAULT; start the journal from DISARMED.
    loggedState=bite::State::Disarmed;
    esp_task_wdt_config_t wc={.timeout_ms=30000,.idle_core_mask=0,.trigger_panic=true};
    if(esp_task_wdt_reconfigure(&wc)==ESP_ERR_INVALID_STATE)esp_task_wdt_init(&wc);
    esp_task_wdt_add(nullptr);
    Serial.printf("Bite sensor %s: XIAO, checked LSM6DS3, dynamic/legacy, wake=%s, no field guarantee\n",board::name,wakeName);
}
void loop(){
    esp_task_wdt_reset();sett.tick();uint32_t now=millis();
    auto action=button.update(digitalRead(board::button)==LOW,now);
    if(action!=bite::ButtonAction::None)note(now,bite::Log::Button,uint8_t(action));
    switch(action){
        case bite::ButtonAction::ArmCue:beep.start(now,60);break;
        case bite::ButtonAction::SleepCue:beep.start(now,400);break;
        case bite::ButtonAction::ToggleArm:
            // A second press while an ARM is pending cancels it, like a DISARM.
            if(armAssist.pending())setArmed(false,now,bite::Source::Button);
            // Not calibrated yet: a calibration started now would see the finger. Wait for release + 1 s.
            else if(!sensor.desiredArmed()&&sensor.healthy()&&!sensor.calibrated()){armAssist.defer(now);note(now,bite::Log::ArmDeferred);}
            else setArmed(!sensor.desiredArmed(),now,bite::Source::Button);
            break;
        case bite::ButtonAction::Sleep:enterSleep("button",true,now);break;
        default:break;
    }
    if(armAssist.due(now,button.released())){
        // Restart a calibration that may have begun while the finger was still on the board.
        if(sensor.state()==bite::State::Calibrating)sensor.beginCalibration(now);
        setArmed(true,now,bite::Source::Button);
    }
    serviceSerial(now);serviceRegistration(now);
    if(saveConfig){saveConfig=false;persistConfig();}
    if(networkRequest){networkRequest=false;uint8_t parsed[6];draftName.trim();draftMac.trim();bool draftRadio=draftMac.length()>0;
        if(!bite::validNetwork(draftName.c_str(),draftPass.c_str())||(draftRadio&&!bite::parseMac(draftMac.c_str(),parsed))){say(tr("Invalid rod name, password or hub MAC; nothing applied","Неверное имя удилища, пароль или MAC хаба; изменения не применены"));}
        else{db.set(kk::name,draftName);db.set(kk::apPass,draftPass);db.set(kk::hubMac,draftMac);db.set(kk::espNowEn,draftRadio);
            if(flushSettings()){sensorName=draftName;apPassword=draftPass;hubAddress=draftMac;radioEn=draftRadio;reconcileRadio();stopOutputs();rebootAt=now;say(tr("Saved; restarting","Сохранено; перезапуск"));}
            else{db.set(kk::name,sensorName);db.set(kk::apPass,apPassword);db.set(kk::hubMac,hubAddress);db.set(kk::espNowEn,radioEn);say(tr("Save failed; current settings kept","Ошибка записи; прежние настройки сохранены"));}}
    }
    if(rebootAt&&bite::elapsed(now,rebootAt)>=700)ESP.restart();
    if(testRequest){testRequest=false;if(sensor.state()==bite::State::Disarmed||sensor.state()==bite::State::Armed){alarmPulse.start(now,pulseMs);++pulseCount;}else say(tr("Test rejected: sensor not ready","Тест отклонён: датчик не готов"));}
    auto oldState=sensor.state();bool wantedBefore=sensor.desiredArmed();imuSession.supervise(imu,sensor,now);
    if(!imuSession.online())stopOutputs();
    if(imuSession.due(now)){
        bite::Vec3 sample;uint8_t event=0;auto result=imuSession.read(imu,sensor,now,sample,event);
        if(result==bite::ReadStatus::Fresh){
            if(event&&sensor.state()==bite::State::Armed){alarmPulse.start(now,pulseMs);++pulseCount;sendPacket(event);}
            if(csv){const auto& t=sensor.telemetry();char line[220];
                int n=snprintf(line,sizeof(line),"%lu,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%s,%u,%u,%.5f,%.5f\n",(unsigned long)now,sample.x,sample.y,sample.z,t.signal.dynamic,t.signal.legacy,t.noiseG,t.effectiveG,bite::stateName(sensor.state()),unsigned(event),0u,t.signal.impulse,t.ambientG);
                if(n<=0||n>=int(sizeof(line))||!csvBuffer.append(line,size_t(n)))++csvDrops;
            }
        }else if(result==bite::ReadStatus::TransportError){++readErrors;stopOutputs();}
        else ++noNew;
    }
    if(wantedBefore&&!sensor.desiredArmed()&&previousCalibration==bite::Calibration::Running&&sensor.calibration()==bite::Calibration::Moving){
        // Previously silent: the ARM request was dropped because the calibration saw movement.
        const auto& t=sensor.telemetry();note(now,bite::Log::ArmCancelled,uint8_t(t.calReject),0,0,t.calRejectValue);
        stopOutputs();errorCue.start(now,3,80,80);
        say(tr("ARM cancelled: the sensor was being moved during calibration. Leave it and arm again.","Охрана отменена: датчик двигали во время калибровки. Отпустите его и включите охрану снова."));
    }
    auto state=sensor.state();if(state!=oldState){
        if(state!=bite::State::Armed)alarmPulse.stop();
        if(state==bite::State::Disarmed){disarmedClock.enter(now);sleepAttempted=false;}
        if(state==bite::State::Fault){stopOutputs();armUi=false;db.set(kk::armedSw,false);disarmedClock.enter(now);sleepAttempted=false;enterConfig(now);}
    }
    armUi=sensor.desiredArmed();
    if(sensor.desiredArmed()&&apActive&&apClock.expired(now,120000)){WiFi.softAPdisconnect(false);WiFi.mode(WIFI_STA);apActive=false;}
    if(state==bite::State::Disarmed&&!sensor.desiredArmed()&&!csv&&!sleepAttempted&&disarmedClock.expired(now,180000)&&WiFi.softAPgetStationNum()==0)enterSleep("idle",false,now);
    if(bite::elapsed(now,lastRate)>=1000){sampleRate=(sensor.telemetry().samples-rateSamples)*1000.f/bite::elapsed(now,lastRate);rateSamples=sensor.telemetry().samples;lastRate=now;}
    static uint32_t lastLowAttempt=0;static bool lowSeen=false;
    if(bite::elapsed(now,lastBattery)>=5000){lastBattery=now;updateBattery();
        if(batteryPct>25)lowSeen=false;
        if(startup::batteryKnown(vbat)&&sensor.desiredArmed()&&batteryPct<=20&&(!lowSeen||bite::elapsed(now,lastLowAttempt)>=60000)){lastLowAttempt=now;lowSeen=sendPacket(3);}
        // Cell protection: three readings below 3.30 V -> tell the hub (if armed) and sleep.
        if(lowBattery.update(startup::batteryKnown(vbat),vbat)){if(sensor.desiredArmed())sendPacket(3);delay(50);enterSleep("battery",true,now);}}
    if(state==bite::State::Fault||state==bite::State::Calibrating||state==bite::State::MotionSuppressed||state==bite::State::Settling)alarmPulse.stop();
    auto calibration=sensor.calibration();
    if(calibration!=previousCalibration){
        const auto& t=sensor.telemetry();
        if(calibration==bite::Calibration::Success){
            armAssist.calibrated();note(now,bite::Log::Calibration,uint8_t(calibration),0,0,t.noiseG);
            db.set("calNoiseG",t.noiseG);
            notice=flushSettings()?tr("Noise calibration saved","Калибровка шума сохранена"):tr("Calibration valid in RAM; save failed","Калибровка в RAM; ошибка сохранения");
        }else if(calibration==bite::Calibration::Moving){armAssist.rejected(now);note(now,bite::Log::Calibration,uint8_t(calibration),uint8_t(t.calReject),0,t.calRejectValue);}
        else if(calibration==bite::Calibration::SensorError)note(now,bite::Log::Calibration,uint8_t(calibration));
    }
    previousCalibration=calibration;
    // Boot/wake calibration often sees the hand that pressed RESET or the button: retry it
    // a few times once untouched. This only calibrates; it never arms.
    bool retryEligible=sensor.state()==bite::State::Disarmed&&!sensor.desiredArmed()&&sensor.healthy()&&!sensor.calibrated()&&calibration==bite::Calibration::Moving&&!armAssist.pending();
    if(armAssist.retryDue(now,retryEligible,button.released())){sensor.beginCalibration(now);note(now,bite::Log::CalRetry,armAssist.retries(),bite::ArmAssist::maxRetries);}
    if(!sensor.desiredArmed())armBeepPending=false;
    if(armBeepPending&&sensor.state()==bite::State::Armed){armBeepPending=false;beep.start(now,300);}
    // Explain a long SETTLING: every 3 s, how often stillness was broken and by what.
    if(sensor.state()==bite::State::Settling){
        if(loggedState!=bite::State::Settling)settleLogAt=now;
        else if(bite::elapsed(now,settleLogAt)>=3000){const auto& t=sensor.telemetry();settleLogAt=now;
            if(t.settleResets)note(now,bite::Log::Settling,uint8_t(std::min<uint32_t>(t.settleResets,255)),t.settleCause,uint8_t(std::min(255.f,t.settleLimit*1000.f+.5f)),t.settleValue);}
    }
    if(sensor.state()!=loggedState){
        note(now,bite::Log::State,uint8_t(loggedState),uint8_t(sensor.state()));
        if(sensor.state()==bite::State::MotionSuppressed){const auto& t=sensor.telemetry();note(now,bite::Log::Suppressed,0,0,0,t.suppressPitchDeg,t.suppressRollDeg);}
        if(sensor.state()==bite::State::Armed){
            // Running impulse ambient and the threshold it implies right now (it keeps adapting).
            const auto& c=sensor.config();float ambient=sensor.telemetry().ambientG;bool raised=!c.legacyMode&&ambient*c.noiseMultiplier>c.thresholdG;
            note(now,bite::Log::Armed,raised,0,0,ambient,c.legacyMode?c.legacyThresholdG:std::max(c.thresholdG,ambient*c.noiseMultiplier));
        }
        loggedState=sensor.state();
    }
    if(csv&&csvBuffer.size()){char chunk[128];int available=Serial.availableForWrite();if(available>0){size_t n=csvBuffer.peek(chunk,std::min<size_t>(sizeof(chunk),size_t(available)));csvBuffer.consume(Serial.write(reinterpret_cast<const uint8_t*>(chunk),n));}}
    applyOutputs(now);
}
