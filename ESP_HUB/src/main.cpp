#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "HubCore.h"
#include "HubRender.h"

// Existing hub wiring. No menu button is assigned; confirmed buttonless hub.
constexpr int LedPin=D1,MotorPin=D2,BatteryPin=A0;
constexpr uint8_t RadioChannel=1; // Existing sensor softAP defaults to channel 1; verify on the physical pair.
Adafruit_SH1106G oled(128,64,&Wire,-1);
hub::Model model;
hub::Motor motor;
hub::Ring<hub::Event,hub::RxDepth> inbox;
portMUX_TYPE inboxMux=portMUX_INITIALIZER_UNLOCKED;
uint32_t received=0,invalid=0,dropped=0;
bool radioReady=false,oledReady=false,frameValid=false,powerKnown=false;
uint8_t oledAddress=0;
hub::Power appliedPower=hub::Power::Off;
hub::Frame lastFrame;
uint32_t oledAttemptAt=0,radioAttemptAt=0,lastBatteryAt=0,lastDrawAt=0;
bool oledAttempted=false,radioAttempted=false;
int hubBattery=-1;
float hubVoltage=0;
char hubMac[18]{};
char command[40]{};size_t commandLength=0;

void onReceive(const esp_now_recv_info* info,const uint8_t* data,int len){
 hub::Event event;
 bool valid=info&&len>=0&&hub::decode(data,size_t(len),info->src_addr,millis(),event);
 portENTER_CRITICAL(&inboxMux);
 if(!valid)++invalid;
 else {++received;if(!inbox.push(event))++dropped;}
 portEXIT_CRITICAL(&inboxMux);
}
bool popEvent(hub::Event& event){portENTER_CRITICAL(&inboxMux);bool ok=inbox.pop(event);portEXIT_CRITICAL(&inboxMux);return ok;}
void beginRadio(uint32_t now){
 if(radioReady||(radioAttempted&&hub::elapsed(now,radioAttemptAt)<2000))return;
 radioAttempted=true;radioAttemptAt=now;
 WiFi.mode(WIFI_STA);WiFi.setSleep(false);
 if(esp_wifi_set_channel(RadioChannel,WIFI_SECOND_CHAN_NONE)!=ESP_OK)return;
 if(esp_now_init()!=ESP_OK)return;
 if(esp_now_register_recv_cb(onReceive)!=ESP_OK){esp_now_deinit();return;}
 radioReady=true;
 String mac=WiFi.macAddress();mac.toCharArray(hubMac,sizeof(hubMac));
 Serial.print("Hub MAC: ");Serial.println(hubMac);
}
bool probe(uint8_t address){Wire.beginTransmission(address);return Wire.endTransmission()==0;}
void beginOled(uint32_t now){
 if(oledReady||(oledAttempted&&hub::elapsed(now,oledAttemptAt)<2000))return;
 oledAttempted=true;oledAttemptAt=now;
 oledAddress=probe(0x3c)?0x3c:probe(0x3d)?0x3d:0;
 if(!oledAddress||!oled.begin(oledAddress,true))return;
 oledReady=true;frameValid=false;powerKnown=false;
}
void readBattery(){
 uint32_t sum=0;analogReadMilliVolts(BatteryPin);
 for(int i=0;i<16;++i)sum+=analogReadMilliVolts(BatteryPin);
 // Existing 100k/100k divider. Unknown without a valid single-cell voltage;
 // this does not prove a battery is attached or establish remaining runtime.
 hubVoltage=float(sum)/16.f*.002f;hubBattery=hub::batteryPercent(hubVoltage);
}
void printHelp(){Serial.println("Commands: list, history, service, help. Pages advance automatically; no buttons required.");}
void serialCommands(uint32_t now){
 unsigned consumed=0;
 while(Serial.available()&&consumed++<48){char c=char(Serial.read());
  if(c=='\r'||c=='\n'){
   command[commandLength]=0;
   if(!std::strcmp(command,"list"))model.select(hub::Screen::List,0,now);
   else if(!std::strcmp(command,"history"))model.select(hub::Screen::History,0,now);
   else if(!std::strcmp(command,"service"))model.select(hub::Screen::Service,0,now);
   else if(commandLength)printHelp();
   commandLength=0;
  }else if(commandLength+1<sizeof(command))command[commandLength++]=c;
 }
}
void updateOled(uint32_t now){
 if(!oledReady)return;
 hub::Power power=model.power(now);
 if(!powerKnown||power!=appliedPower){
  if(!probe(oledAddress)){oledReady=false;oledAttemptAt=now;return;}
  if(power==hub::Power::Off)oled.oled_command(SH110X_DISPLAYOFF);
  else {oled.oled_command(SH110X_DISPLAYON);oled.oled_command(0x81);oled.oled_command(power==hub::Power::Dim?0x20:0xff);}
  appliedPower=power;powerKnown=true;frameValid=false;
 }
 if(power==hub::Power::Off||(frameValid&&hub::elapsed(now,lastDrawAt)<200))return;
 lastDrawAt=now;
 hub::Diagnostics d;d.radio=radioReady;d.oled=oledReady;d.voltage=hubVoltage;std::memcpy(d.mac,hubMac,sizeof(d.mac));
 portENTER_CRITICAL(&inboxMux);d.rx=received;d.invalid=invalid;d.dropped=dropped;portEXIT_CRITICAL(&inboxMux);
 hub::Frame frame=hub::render(model,now,hubBattery,d);
 if(frameValid&&frame.bytes==lastFrame.bytes)return;
 if(!probe(oledAddress)){oledReady=false;oledAttemptAt=now;return;}
 oled.clearDisplay();oled.drawBitmap(0,0,frame.bytes.data(),128,64,SH110X_WHITE);oled.display();
 // display() has no transport return status, so re-probe before trusting state.
 if(!probe(oledAddress)){oledReady=false;oledAttemptAt=now;return;}
 lastFrame=frame;frameValid=true;
}
void setup(){
 pinMode(LedPin,OUTPUT);pinMode(MotorPin,OUTPUT);digitalWrite(MotorPin,LOW);
 pinMode(BatteryPin,INPUT);analogReadResolution(12);analogSetPinAttenuation(BatteryPin,ADC_11db);
 Serial.begin(115200);setCpuFrequencyMhz(80);
 uint32_t now=millis();model.begin(now);
 // Receiver starts before any OLED operation; missing OLED cannot gate reception.
 beginRadio(now);
 Wire.begin(D4,D5);Wire.setClock(400000);Wire.setTimeOut(10);
 readBattery();lastBatteryAt=now;printHelp();
}
void loop(){
 uint32_t now=millis();
 bool motorOn=motor.tick(now);digitalWrite(MotorPin,motorOn?HIGH:LOW);
 model.tick(now);
 // Drain a bounded burst and choose one highest-priority motor request.
 uint8_t motorRequest=0;
 for(size_t i=0;i<hub::RxDepth;++i){hub::Event event;if(!popEvent(event))break;
  hub::Ingest result=model.ingest(event,now);
  if(result.newEpisode)frameValid=false;
  if(result.motor&&(!motorRequest||(motorRequest==3&&result.type!=3)))motorRequest=result.type;
 }
 motorOn=motor.tick(millis());digitalWrite(MotorPin,motorOn?HIGH:LOW);
 serialCommands(now);
 // No slow I2C operation runs during a motor pattern, including its off gap.
 // Otherwise a failed bus could delay switching the motor off.
 if(!motor.busy()){
  beginRadio(now);beginOled(now);
  if(hub::elapsed(now,lastBatteryAt)>=5000){readBattery();lastBatteryAt=now;}
  updateOled(now);
 }
 // Paint a newly awakened alert before energising the motor. No I2C transfer
 // can then stretch an already running motor pulse.
 if(motorRequest)motor.start(motorRequest,millis());
 digitalWrite(MotorPin,motor.tick(millis())?HIGH:LOW);
 bool sleeping=model.power(now)==hub::Power::Off;
 digitalWrite(LedPin,!sleeping||(now%10000)<80?HIGH:LOW);
 delay(1);
}
