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
#include <EncButton.h>
#include "BoardProfile.h"
#include "ImuDriver.h"
#include "SensorCore.h"
#include "CsvBuffer.h"
#include "ConfigDatabase.h"
#include "ImuSession.h"
#include "MobileUi.h"
#include "../shared/StartupRadio.h"

// Keep original key hashes and 22-byte wire contract. New semantics use new keys.
DB_KEYS(kk,name,deltaG,shortPulse,pulsePeriod,contThresh,armedSw,langIdx,testAlarm,apPass,saveRB,buzVol,ledRedLvl,ledGreenLvl,espNowEn,hubMac);
struct __attribute__((packed)) BitePacket {char rodName[16];uint8_t eventType,batteryPct;float deltaG;};
static_assert(sizeof(BitePacket)==22,"Hub wire compatibility");
bite::ConfigDatabase<GyverDBFile> db(&LittleFS,"/config.db");
SettingsGyver sett("Fishing Bite Sensor - bench",&db);
Button button(board::button);
ImuDriver imu;
bite::SensorCore sensor;
bite::ImuSession imuSession;
bite::Config draft;
bite::Pulse alarmPulse,beep;
bite::CsvBuffer<2048> csvBuffer;
bite::Calibration previousCalibration=bite::Calibration::None;
bite::StateClock apClock,disarmedClock;
bool storageOK=false,pwmOK=false,showDetails=false;
bool apActive=true,radioReady=false,radioEn=false,csv=false,armUi=false;
bool saveConfig=false,calRequest=false,testRequest=false,networkRequest=false;
bool shutdownPending=false,sleepAttempted=false;
uint8_t page=0,language=0,mode=0;
uint16_t pulseMs=120,volume=100,greenLevel=100,redLevel=100;
String sensorName="BiteSensor",apPassword="",hubAddress="";
String draftName,draftPass,draftMac,notice="",sleepNotice="",radioNotice="";
bool draftRadio=false;
uint8_t peerMac[6]={0};
startup::Sensor registration;uint32_t sensorBoot=1;
startup::Ring<startup::Message,4> controlInbox;portMUX_TYPE controlMux=portMUX_INITIALIZER_UNLOCKED;
uint32_t controlTx=0,controlRx=0;char serialCommand[48]{};size_t serialLength=0;bool serialOverflow=false;
uint32_t lastBattery=0,lastRate=0,rateSamples=0;
uint32_t pulseCount=0,readErrors=0,noNew=0,csvDrops=0,sendErrors=0,saveNonce=0;
uint32_t rebootAt=0,shutdownAt=0;
float sampleRate=0,vbat=0;int batteryPct=0;
std::atomic<uint32_t> txSuccess{0},txFail{0};
bool powerPrev=true,powerStable=true,powerReleased=false;
uint32_t powerChanged=0,powerPressedAt=0;
const char* tr(const char* en,const char* ru){return language?ru:en;}

namespace ui {
constexpr size_t menu=2000,lang=2001,arm=2002,algorithm=2003,threshold=2004,noise=2005,legacy=2006;
constexpr size_t pulse=2007,period=2008,continuous=2009,suppress=2010,up=2011,tilt=2012,hold=2013;
constexpr size_t vol=2014,green=2015,red=2016,name=2017,pass=2018,mac=2019,radio=2020,save=2021,cal=2022,test=2023,capture=2024;
constexpr size_t state=2100,health=2101,xyz=2102,signals=2103,noiseInfo=2104,calInfo=2105,battery=2106,stats=2107,timing=2108,sleep=2109,radioInfo=2110,message=2111,details=2112,radioDetails=2113,batteryRaw=2114;
}

void stopOutputs(){alarmPulse.stop();beep.stop();}
void applyOutputs(uint32_t now){
    bool alarmOn=alarmPulse.tick(now),beepOn=beep.tick(now);
    if(!pwmOK)return;
    // Passive buzzer: 100% volume maps to 50% carrier duty, never DC.
    ledcWriteChannel(0,(alarmOn||beepOn)?(uint32_t(volume)*512u/100u):0);
    ledcWriteChannel(1,sensor.state()==bite::State::Armed?uint32_t(greenLevel)*255/100:0);
    ledcWriteChannel(2,alarmOn?uint32_t(redLevel)*255/100:0);
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
void setArmed(bool value,uint32_t now){
    stopOutputs();bool was=sensor.desiredArmed();
    if(!sensor.setArmed(value,now)){armUi=false;notice=tr("Cannot arm: IMU fault","Нельзя включить охрану: ошибка IMU");return;}
    armUi=sensor.desiredArmed();db.set(kk::armedSw,armUi);
    if(value&&!was){apClock.enter(now);if(sensor.state()!=bite::State::Calibrating)beep.start(now,300);}
    if(!value){disarmedClock.enter(now);sleepAttempted=false;beep.start(now,120);enterConfig(now);}
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
void serviceSerial(){
 unsigned consumed=0;while(Serial.available()&&consumed++<64){char ch=char(Serial.read());
  if(ch=='\r'||ch=='\n'){
   serialCommand[serialLength]=0;
   if(serialOverflow)Serial.println("SENSOR error: command too long");
   else if(!std::strcmp(serialCommand,"status"))Serial.printf("SENSOR status radio=%u registered=%u attempts=%u node=%u control_rx=%lu control_tx=%lu state=%s imu=%u\n",radioReady,registration.confirmed(),registration.attempts(),registration.node(),(unsigned long)controlRx,(unsigned long)controlTx,bite::stateName(sensor.state()),sensor.healthy());
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
     if(flushSettings()){hubAddress=serialCommand+5;radioEn=true;draftMac=hubAddress;draftRadio=true;reconcileRadio();Serial.println("SENSOR peer saved; radio enabled");}
     else {db.set(kk::hubMac,hubAddress);db.set(kk::espNowEn,radioEn);Serial.println("SENSOR error: peer save failed; live configuration unchanged");}
    }
   }else if(serialLength)Serial.println("SENSOR commands: status, register, peer AA:BB:CC:DD:EE:FF (disarmed only)");
   serialLength=0;serialOverflow=false;
  }else if(serialLength+1<sizeof(serialCommand))serialCommand[serialLength++]=ch;else serialOverflow=true;
 }
}
void persistConfig(){
    mode=mode==1?1:0;language=language==1?1:0;draft.legacyMode=mode==1;
    if(!sensor.configure(draft)){draft=sensor.config();mode=draft.legacyMode;notice=tr("Invalid setting rejected","Неверная настройка отклонена");return;}
    pulseMs=constrain(pulseMs,40,1000);volume=constrain(volume,0,100);greenLevel=constrain(greenLevel,0,100);redLevel=constrain(redLevel,0,100);
    db.set("benchSchema",1);db.set("dynamicThresholdG",draft.thresholdG);db.set("noiseMultiplier",draft.noiseMultiplier);
    db.set("detectorMode",int(mode));db.set(kk::deltaG,draft.legacyThresholdG);
    db.set(kk::shortPulse,int(pulseMs));db.set(kk::pulsePeriod,int(draft.repeatMs));db.set(kk::contThresh,int(draft.continuousMs));
    db.set("autoDisarmEn",draft.suppression);db.set("autoUpDeg",draft.upDeg);db.set("autoTiltDeg",draft.tiltDeg);db.set("autoHoldMs",int(draft.suppressHoldMs));
    db.set(kk::buzVol,int(volume));db.set(kk::ledGreenLvl,int(greenLevel));db.set(kk::ledRedLvl,int(redLevel));db.set(kk::langIdx,int(language));
    notice=flushSettings()?tr("Saved","Сохранено"):tr("Save failed: settings are RAM only","Ошибка записи: настройки только в RAM");
}
void loadConfig(){
    db.init(kk::name,sensorName);db.init(kk::apPass,apPassword);db.init(kk::hubMac,hubAddress);db.init(kk::espNowEn,false);
    db.init(kk::deltaG,.15f);db.init(kk::shortPulse,120);db.init(kk::pulsePeriod,300);db.init(kk::contThresh,250);
    db.init(kk::buzVol,100);db.init(kk::ledGreenLvl,100);db.init(kk::ledRedLvl,100);db.init(kk::langIdx,0);
    db.init("calNoiseG",0.f);sensor.restoreNoise(db.get("calNoiseG").toFloat());
    db.init("dynamicThresholdG",.02f);db.init("noiseMultiplier",4.f);db.init("detectorMode",0);
    db.init("autoDisarmEn",true);db.init("autoUpDeg",25.f);db.init("autoTiltDeg",35.f);db.init("autoHoldMs",120);
    // Do not read a saved arm intent after boot: calibration and explicit ARM are required.
    sensorName=db.get(kk::name).toString();apPassword=db.get(kk::apPass).toString();hubAddress=db.get(kk::hubMac).toString();radioEn=db.get(kk::espNowEn).toBool();
    if(!bite::validNetwork(sensorName.c_str(),apPassword.c_str())){sensorName="BiteSensor";apPassword="";notice="Invalid saved AP settings; using recovery AP";}
    draft.thresholdG=db.get("dynamicThresholdG").toFloat();draft.legacyThresholdG=db.get(kk::deltaG).toFloat();draft.noiseMultiplier=db.get("noiseMultiplier").toFloat();
    mode=db.get("detectorMode").toInt()==1?1:0;draft.legacyMode=mode==1;
    draft.repeatMs=db.get(kk::pulsePeriod).toInt();draft.continuousMs=db.get(kk::contThresh).toInt();
    draft.suppression=db.get("autoDisarmEn").toBool();draft.upDeg=db.get("autoUpDeg").toFloat();draft.tiltDeg=db.get("autoTiltDeg").toFloat();draft.suppressHoldMs=db.get("autoHoldMs").toInt();
    if(!sensor.configure(draft)){draft=bite::Config{};sensor.configure(draft);mode=0;notice="Invalid saved detector settings; defaults in RAM";}
    pulseMs=constrain(db.get(kk::shortPulse).toInt(),40,1000);volume=constrain(db.get(kk::buzVol).toInt(),0,100);
    greenLevel=constrain(db.get(kk::ledGreenLvl).toInt(),0,100);redLevel=constrain(db.get(kk::ledRedLvl).toInt(),0,100);language=db.get(kk::langIdx).toInt()==1?1:0;
    saveNonce=db.get("benchSave").toInt();draftName=sensorName;draftPass=apPassword;draftMac=hubAddress;draftRadio=radioEn;
}

void build(sets::Builder& b){
    using namespace sets;
    if(page>2)page=0;
    b.Custom("BenchLayout",2199);
    {Group g(b,tr("Menu","Меню"));if(b.Select(ui::menu,tr("Page","Страница"),tr("Status\nSensor\nSettings","Статус\nДатчик\nНастройки"),AnyPtr(&page)))b.reload();}
    if(page==0){
        {Group g(b,tr("Status","Статус"));
            b.Label(ui::state,tr("State","Состояние"));
            b.Label(ui::calInfo,tr("Calibration","Калибровка"));
            b.Label(ui::battery,tr("Battery (approx.)","Батарея (≈)"));
            b.Label(ui::radioInfo,"ESP-NOW");
        }
        {Group g(b,tr("Controls","Управление"));
            if(b.Switch(ui::arm,tr("Armed","Охрана"),&armUi))setArmed(armUi,millis());
            if(b.Button(ui::cal,tr("Calibrate · keep still 2 s","Калибровка · не трогать 2 с")))calRequest=true;
            if(b.Button(ui::test,tr("Test sound / LED","Тест звука / LED")))testRequest=true;
            if(b.Switch(ui::capture,"Serial CSV",&csv)){csvBuffer.clear();if(csv){const char* header="ms,ax_g,ay_g,az_g,dynamic_g,legacy_g,noise_g,threshold_g,state,event,mode\n";csvBuffer.append(header,strlen(header));}}
        }
        if(b.Switch(ui::details,tr("Details","Подробности"),&showDetails))b.reload();
        if(showDetails){Group g(b,tr("Technical details","Технические данные"));
            b.Label(ui::health,tr("IMU / PWM / storage","IMU / PWM / память"));
            b.Label(ui::xyz,"XYZ, g");b.Label(ui::signals,tr("Dynamic / legacy, g","Dynamic / Legacy, g"));
            b.Label(ui::noiseInfo,tr("Noise / threshold, g","Шум / порог, g"));
            b.Label(ui::batteryRaw,tr("Battery ADC","Батарея: АЦП"));
            b.Label(ui::stats,tr("Events / pulses / suppressed ms","События / импульсы / подавление, мс"));
            b.Label(ui::timing,tr("Hz / gaps / I2C errors","Гц / пропуски / ошибки I2C"));
            b.Label(ui::sleep,tr("Sleep","Сон"));b.Label(ui::radioDetails,tr("ESP-NOW counters","ESP-NOW: счётчики"));
            b.Label(ui::message,tr("Last message","Последнее сообщение"));
            b.Paragraph("",tr("Serial CSV: fresh samples, up to 104 Hz. Test sound does not send radio.","Serial CSV: новые выборки, до 104 Гц. Тест звука не отправляет радио."));
        }
    }
    if(page==1){Group g(b,tr("Detection","Детекция"));
        b.Paragraph("",tr("Lower threshold = more sensitive. Noise protection may raise the effective threshold.","Меньше порог — выше чувствительность. Защита от шума может повысить рабочий порог."));
        if(b.Select(ui::algorithm,tr("Algorithm","Алгоритм"),"Dynamic XYZ\nLegacy magnitude",AnyPtr(&mode))){saveConfig=true;b.reload();}
        if(mode==0){if(b.Slider(ui::threshold,tr("Threshold, g","Порог, g"),.002f,.5f,.001f,"g",AnyPtr(&draft.thresholdG)))saveConfig=true;
            if(b.Slider(ui::noise,tr("Noise multiplier","Защита от шума, ×"),1,10,.5f,"",AnyPtr(&draft.noiseMultiplier)))saveConfig=true;
        }else if(b.Slider(ui::legacy,tr("Legacy threshold","Порог Legacy"),.02f,1,.01f,"g",AnyPtr(&draft.legacyThresholdG)))saveConfig=true;
        if(b.Slider(ui::pulse,tr("Alarm pulse","Импульс"),40,1000,10,"ms",AnyPtr(&pulseMs)))saveConfig=true;
        if(b.Slider(ui::period,tr("Repeat period","Повтор сигнала"),100,2000,10,"ms",AnyPtr(&draft.repeatMs)))saveConfig=true;
        if(b.Slider(ui::continuous,tr("Continuous delay","Непрерывность"),50,2000,10,"ms",AnyPtr(&draft.continuousMs)))saveConfig=true;
        if(b.Switch(ui::suppress,tr("Motion suppression","Подавлять перемещение"),&draft.suppression)){saveConfig=true;b.reload();}
        if(draft.suppression){
            b.Paragraph("",tr("Suppress alarms, then resume when still. Angle 0 = disabled.","Подавлять тревоги и возобновлять после стабилизации. Угол 0 — отключён."));
            if(b.Slider(ui::up,tr("Pitch limit","Наклон UP"),0,90,1,"deg",AnyPtr(&draft.upDeg)))saveConfig=true;
            if(b.Slider(ui::tilt,tr("Roll limit","Крен TILT"),0,90,1,"deg",AnyPtr(&draft.tiltDeg)))saveConfig=true;
            if(b.Slider(ui::hold,tr("Motion hold","Удержание"),50,2000,10,"ms",AnyPtr(&draft.suppressHoldMs)))saveConfig=true;
        }
    }
    if(page==2){Group g(b,tr("Settings","Настройки"));
        if(b.Select(ui::lang,"Language / Язык","English\nРусский",AnyPtr(&language))){saveConfig=true;b.reload();}
        if(b.Slider(ui::vol,tr("Buzzer volume","Громкость зуммера"),0,100,1,"%",AnyPtr(&volume)))saveConfig=true;
        if(b.Slider(ui::green,tr("Green LED","Зелёный LED"),0,100,1,"%",AnyPtr(&greenLevel)))saveConfig=true;
        if(b.Slider(ui::red,tr("Red LED","Красный LED"),0,100,1,"%",AnyPtr(&redLevel)))saveConfig=true;
        b.Input(ui::name,tr("AP name","Имя Wi-Fi"),AnyPtr(&draftName));
        b.Pass(ui::pass,tr("AP password","Пароль Wi-Fi"),AnyPtr(&draftPass));
        b.Switch(ui::radio,tr("ESP-NOW enabled","ESP-NOW включён"),&draftRadio);b.Input(ui::mac,tr("Hub unicast MAC","Unicast MAC хаба"),AnyPtr(&draftMac));
        if(b.Button(ui::save,tr("Save network and restart","Сохранить сеть и перезапустить")))networkRequest=true;
    }
}
void update(sets::Updater& u){
    const auto& t=sensor.telemetry();
    u.update(ui::state,benchStateText(sensor.state(),language));
    u.update(ui::arm,sensor.desiredArmed());
    u.update(ui::health,String(sensor.healthy()?"IMU OK":"IMU FAULT")+" / PWM "+(pwmOK?"OK":"ERROR")+" / FS "+(storageOK?"OK":"ERROR"));
    u.updateColor(ui::health,sensor.healthy()&&pwmOK?sets::Colors::Green:sets::Colors::Red);
    u.update(ui::xyz,String(t.accel.x,4)+", "+String(t.accel.y,4)+", "+String(t.accel.z,4));
    u.update(ui::signals,String(t.signal.dynamic,4)+" / "+String(t.signal.legacy,4));
    u.update(ui::noiseInfo,String(t.noiseG,4)+" / "+String(t.effectiveG,4)+(t.effectiveG>sensor.config().thresholdG&&!mode?tr(" (noise raised threshold)"," (порог повышен шумом)"):""));
    u.update(ui::calInfo,benchCalibrationText(sensor.calibration(),t.calibrationProgress,language));
    u.update(ui::battery,vbat>=2.5f&&vbat<=4.5f?String(vbat,2)+" V / ≈"+String(batteryPct)+"%":String(tr("No valid reading","Нет достоверного замера")));
    u.update(ui::batteryRaw,String(vbat,2)+" V / ≈"+String(batteryPct)+"%");
    u.update(ui::stats,String(t.events)+" / "+String(pulseCount)+" / "+String(t.suppressedMs));
    u.update(ui::timing,String(sampleRate,1)+" Hz / "+String(t.gaps)+" / "+String(readErrors)+"; CSV dropped="+String(csvDrops));
    u.update(ui::sleep,sleepNotice);
    u.update(ui::radioInfo,!radioEn?tr("Off","Выключен"):radioReady?(registration.confirmed()?tr("Registered at startup","Регистрация при старте подтверждена"):tr("Ready; hub unconfirmed","Готов; хаб не подтверждён")):tr("Error — see details","Ошибка — см. подробности"));
    u.update(ui::radioDetails,radioNotice+" TX="+String(txSuccess.load(std::memory_order_relaxed))+" fail="+String(txFail.load(std::memory_order_relaxed)+sendErrors));u.update(ui::message,notice);
}

void attemptSleep(uint32_t now){
    sleepAttempted=true;
    if(!board::wakeAllowed()){sleepNotice=tr("Deep sleep blocked: D6 is not wake-capable; wiring unchanged","Глубокий сон запрещён: D6 не поддерживает пробуждение; схема не изменена");Serial.println(sleepNotice);return;}
    if(!flushSettings()){sleepNotice="Sleep blocked: storage flush failed";return;}
    esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);esp_err_t result=ESP_ERR_INVALID_ARG;
#if CONFIG_IDF_TARGET_ESP32C3
    result=esp_deep_sleep_enable_gpio_wakeup(1ULL<<board::button,ESP_GPIO_WAKEUP_GPIO_LOW);
#elif CONFIG_IDF_TARGET_ESP32C6
    result=esp_sleep_enable_ext1_wakeup(1ULL<<board::button,ESP_EXT1_WAKEUP_ANY_LOW);
#else
    result=esp_sleep_enable_ext0_wakeup(gpio_num_t(board::button),0);
#endif
    if(result!=ESP_OK){sleepNotice="Sleep blocked: wake API rejected pin";return;}
    stopOutputs();applyOutputs(now);radioEnd();esp_deep_sleep_start();
}
void handlePower(uint32_t now){
    bool raw=digitalRead(board::powerButton);
    if(raw!=powerPrev){powerPrev=raw;powerChanged=now;}
    if(raw!=powerStable&&bite::elapsed(now,powerChanged)>=50){powerStable=raw;if(!raw)powerPressedAt=now;}
    if(powerStable)powerReleased=true;
    if(powerReleased&&!powerStable&&bite::elapsed(now,powerPressedAt)>=3000&&!shutdownPending){
        stopOutputs();sensor.setArmed(false,now);flushSettings();shutdownPending=true;shutdownAt=now;notice="Power release requested";
    }
    if(shutdownPending&&bite::elapsed(now,shutdownAt)>=100){digitalWrite(board::power,LOW);}
    // Under USB the latch may not cut power. Stay disarmed and keep feeding watchdog.
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
    pinMode(board::power,OUTPUT);digitalWrite(board::power,HIGH);pinMode(board::powerButton,INPUT_PULLUP);
    pinMode(board::button,INPUT_PULLUP);button.setHoldTimeout(1000);pinMode(board::battery,INPUT);
    Serial.begin(115200);sensorBoot=esp_random();if(!sensorBoot)sensorBoot=1;setCpuFrequencyMhz(80);initOutputs();
    analogReadResolution(12);analogSetPinAttenuation(board::battery,ADC_11db);
    storageOK=mountStorage();if(storageOK)storageOK=db.begin();
    if(!storageOK)notice="Storage error; RAM settings only (not formatting existing data)";
    loadConfig();Wire.begin(board::sda,board::scl);Wire.setClock(400000);Wire.setTimeOut(20);
    uint32_t now=millis();imuSession.begin(imu,sensor,now);now=millis();enterConfig(now);reconcileRadio();disarmedClock.enter(now);
    sleepNotice=board::wakeAllowed()?"Wake pin accepted; power latch still requires hardware test":"D6 cannot deep-wake this board: automatic deep sleep disabled";
    sett.setCustom(benchMobileJs,strlen_P(benchMobileJs));
    sett.begin();sett.onBuild(build);sett.onUpdate(update);
    updateBattery();lastBattery=lastRate=now;powerStable=powerPrev=digitalRead(board::powerButton);powerChanged=now;
    esp_task_wdt_config_t wc={.timeout_ms=30000,.idle_core_mask=0,.trigger_panic=true};
    if(esp_task_wdt_reconfigure(&wc)==ESP_ERR_INVALID_STATE)esp_task_wdt_init(&wc);
    esp_task_wdt_add(nullptr);
    Serial.println("Bite sensor bench: XIAO, checked LSM6DS3, dynamic/legacy, no field guarantee");
}
void loop(){
    esp_task_wdt_reset();sett.tick();uint32_t now=millis();
    button.tick();if(button.hold())setArmed(!sensor.desiredArmed(),now);
    serviceSerial();serviceRegistration(now);
    handlePower(now);if(shutdownPending){stopOutputs();applyOutputs(now);return;}
    if(saveConfig){saveConfig=false;persistConfig();}
    if(networkRequest){networkRequest=false;uint8_t parsed[6];draftName.trim();
        if(!bite::validNetwork(draftName.c_str(),draftPass.c_str())||(draftRadio&&!bite::parseMac(draftMac.c_str(),parsed))){notice=tr("Invalid AP credentials or hub MAC; nothing applied","Неверное имя/пароль AP или MAC; изменения не применены");}
        else{db.set(kk::name,draftName);db.set(kk::apPass,draftPass);db.set(kk::hubMac,draftMac);db.set(kk::espNowEn,draftRadio);
            if(flushSettings()){sensorName=draftName;apPassword=draftPass;hubAddress=draftMac;radioEn=draftRadio;reconcileRadio();stopOutputs();rebootAt=now;notice="Saved; restarting";}
            else{db.set(kk::name,sensorName);db.set(kk::apPass,apPassword);db.set(kk::hubMac,hubAddress);db.set(kk::espNowEn,radioEn);notice="Network save failed; current network kept";}}
    }
    if(rebootAt&&bite::elapsed(now,rebootAt)>=700)ESP.restart();
    if(calRequest){calRequest=false;stopOutputs();if(!sensor.beginCalibration(now))notice="Calibration unavailable: IMU fault";}
    if(testRequest){testRequest=false;if(sensor.state()==bite::State::Disarmed||sensor.state()==bite::State::Armed){alarmPulse.start(now,pulseMs);++pulseCount;}else notice=tr("Test rejected: sensor not ready","Тест отклонён: датчик не готов");}
    auto oldState=sensor.state();imuSession.supervise(imu,sensor,now);
    if(!imuSession.online())stopOutputs();
    if(imuSession.due(now)){
        bite::Vec3 sample;uint8_t event=0;auto result=imuSession.read(imu,sensor,now,sample,event);
        if(result==bite::ReadStatus::Fresh){
            if(event&&sensor.state()==bite::State::Armed){alarmPulse.start(now,pulseMs);++pulseCount;sendPacket(event);}
            if(csv){const auto& t=sensor.telemetry();char line[220];
                int n=snprintf(line,sizeof(line),"%lu,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%s,%u,%u\n",(unsigned long)now,sample.x,sample.y,sample.z,t.signal.dynamic,t.signal.legacy,t.noiseG,t.effectiveG,bite::stateName(sensor.state()),unsigned(event),unsigned(mode));
                if(n<=0||n>=int(sizeof(line))||!csvBuffer.append(line,size_t(n)))++csvDrops;
            }
        }else if(result==bite::ReadStatus::TransportError){++readErrors;stopOutputs();}
        else ++noNew;
    }
    auto state=sensor.state();if(state!=oldState){
        if(state!=bite::State::Armed)alarmPulse.stop();
        if(state==bite::State::Disarmed){disarmedClock.enter(now);sleepAttempted=false;}
        if(state==bite::State::Fault){stopOutputs();armUi=false;db.set(kk::armedSw,false);disarmedClock.enter(now);sleepAttempted=false;enterConfig(now);}
    }
    armUi=sensor.desiredArmed();
    if(sensor.desiredArmed()&&apActive&&apClock.expired(now,120000)){WiFi.softAPdisconnect(false);WiFi.mode(WIFI_STA);apActive=false;}
    if(state==bite::State::Disarmed&&!sensor.desiredArmed()&&!csv&&!sleepAttempted&&disarmedClock.expired(now,180000)&&WiFi.softAPgetStationNum()==0)attemptSleep(now);
    if(bite::elapsed(now,lastRate)>=1000){sampleRate=(sensor.telemetry().samples-rateSamples)*1000.f/bite::elapsed(now,lastRate);rateSamples=sensor.telemetry().samples;lastRate=now;}
    static uint32_t lastLowAttempt=0;static bool lowSeen=false;
    if(bite::elapsed(now,lastBattery)>=5000){lastBattery=now;updateBattery();
        if(batteryPct>25)lowSeen=false;
        if(startup::batteryKnown(vbat)&&sensor.desiredArmed()&&batteryPct<=20&&(!lowSeen||bite::elapsed(now,lastLowAttempt)>=60000)){lastLowAttempt=now;lowSeen=sendPacket(3);}}
    if(state==bite::State::Fault||state==bite::State::Calibrating||state==bite::State::MotionSuppressed||state==bite::State::Settling)alarmPulse.stop();
    auto calibration=sensor.calibration();
    if(calibration==bite::Calibration::Success&&previousCalibration!=calibration){
        db.set("calNoiseG",sensor.telemetry().noiseG);
        notice=flushSettings()?tr("Noise calibration saved","Калибровка шума сохранена"):tr("Calibration valid in RAM; save failed","Калибровка в RAM; ошибка сохранения");
    }
    previousCalibration=calibration;
    if(csv&&csvBuffer.size()){char chunk[128];int available=Serial.availableForWrite();if(available>0){size_t n=csvBuffer.peek(chunk,std::min<size_t>(sizeof(chunk),size_t(available)));csvBuffer.consume(Serial.write(reinterpret_cast<const uint8_t*>(chunk),n));}}
    applyOutputs(now);
}
