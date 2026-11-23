#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include "HubCore.h"
#include "HubRender.h"
#include "DriverSession.h"
#include <new>

// Existing hub wiring plus one button on D3 to GND (internal pull-up, added 2026-11-23):
// short press = status screen / close a screen, hold 1.5 s = hub MAC for 30 s.
constexpr int LedPin=D1,MotorPin=D2,BatteryPin=A0,ButtonPin=D3;
constexpr uint8_t RadioChannel=1; // Existing sensor softAP defaults to channel 1; verify on the physical pair.
hub::DriverSession<Adafruit_SH1106G> oled;
hub::Model model;
hub::Motor motor;
hub::Button button;
struct Incoming {bool control=false;hub::Event event{};startup::Message message{};};
hub::Ring<Incoming,hub::RxDepth> inbox;
startup::Discovery discovery;bool discoveryStarted=false;uint32_t controlReceived=0,controlSent=0;
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
 Incoming incoming;bool valid=false;
 if(info&&len>=0){incoming.control=size_t(len)==sizeof(startup::Packet);
  if(incoming.control){valid=startup::decode(data,size_t(len),info->src_addr,incoming.message.packet);std::memcpy(incoming.message.mac.data(),info->src_addr,6);}
  else valid=hub::decode(data,size_t(len),info->src_addr,millis(),incoming.event);
 }
 portENTER_CRITICAL(&inboxMux);
 if(!valid)++invalid;else{++received;if(!inbox.push(incoming))++dropped;}
 portEXIT_CRITICAL(&inboxMux);
}
bool popEvent(Incoming& event){portENTER_CRITICAL(&inboxMux);bool ok=inbox.pop(event);portEXIT_CRITICAL(&inboxMux);return ok;}
bool sendControl(const uint8_t* mac,const startup::Packet& packet){
 if(!radioReady)return false;
 if(!esp_now_is_peer_exist(mac)){esp_now_peer_info_t peer{};std::memcpy(peer.peer_addr,mac,6);peer.channel=RadioChannel;peer.ifidx=WIFI_IF_STA;if(esp_now_add_peer(&peer)!=ESP_OK)return false;}
 if(esp_now_send(mac,reinterpret_cast<const uint8_t*>(&packet),sizeof(packet))!=ESP_OK)return false;++controlSent;return true;
}
void beginRadio(uint32_t now){
 if(radioReady||(radioAttempted&&hub::elapsed(now,radioAttemptAt)<2000))return;
 radioAttempted=true;radioAttemptAt=now;
 WiFi.mode(WIFI_STA);WiFi.setSleep(false);
 // The station MAC is known once STA mode is on, even if ESP-NOW then fails.
 if(!hubMac[0]){String mac=WiFi.macAddress();mac.toCharArray(hubMac,sizeof(hubMac));}
 if(esp_wifi_set_channel(RadioChannel,WIFI_SECOND_CHAN_NONE)!=ESP_OK)return;
 if(esp_now_init()!=ESP_OK)return;
 if(esp_now_register_recv_cb(onReceive)!=ESP_OK){esp_now_deinit();return;}
 radioReady=true;
 if(!discoveryStarted){discovery.begin(esp_random(),now);discoveryStarted=true;}
 Serial.print("Hub MAC: ");Serial.println(hubMac);
}
bool probe(uint8_t address){Wire.beginTransmission(address);return Wire.endTransmission()==0;}
void beginOled(uint32_t now){
 if(oledReady||(oledAttempted&&hub::elapsed(now,oledAttemptAt)<10000))return;
 oledAttempted=true;oledAttemptAt=now;
 oledAddress=probe(0x3c)?0x3c:probe(0x3d)?0x3d:0;
 if(!oledAddress||!oled.begin(
  [](){return new(std::nothrow) Adafruit_SH1106G(128,64,&Wire,-1);},
  [](Adafruit_SH1106G& driver){return driver.begin(oledAddress,true);}))return;
 oledReady=true;frameValid=false;powerKnown=false;
}
void readBattery(){
 uint32_t sum=0;analogReadMilliVolts(BatteryPin);
 for(int i=0;i<16;++i)sum+=analogReadMilliVolts(BatteryPin);
 // Existing 100k/100k divider. Unknown without a valid single-cell voltage;
 // this does not prove a battery is attached or establish remaining runtime.
 hubVoltage=float(sum)/16.f*.002f;hubBattery=hub::batteryPercent(hubVoltage);
}
// The boot "Hub MAC" line is usually lost: native USB re-enumerates after reset, so
// the monitor reconnects too late. help/status/mac repeat it on demand.
void printHelp(){Serial.printf("Hub MAC: %s\nCommands: summary, list, history, service, mac, motor, status, help. Button D3: short press = status screen, hold 1.5 s = MAC for 30 s.\n",hubMac[0]?hubMac:"unknown");}
void serialCommands(uint32_t now){
 unsigned consumed=0;
 while(Serial.available()&&consumed++<48){char c=char(Serial.read());
  if(c=='\r'||c=='\n'){
   command[commandLength]=0;
   if(!std::strcmp(command,"summary"))model.select(hub::Screen::Summary,0,now);
   else if(!std::strcmp(command,"motor"))Serial.println(motor.start(1,now)?"HUB motor test: 300 ms pulse on D2":"HUB motor busy; try again");
   else if(!std::strcmp(command,"mac")){Serial.printf("Hub MAC: %s\n",hubMac[0]?hubMac:"unknown");model.select(hub::Screen::Mac,0,now);}
   else if(!std::strcmp(command,"status")){Serial.printf("HUB status mac=%s radio=%u oled=%u rods=%u history=%u control_rx=%lu control_tx=%lu screen=%s\n",hubMac[0]?hubMac:"unknown",radioReady,oledReady,unsigned(model.rodCount()),unsigned(model.historyCount()),(unsigned long)controlReceived,(unsigned long)controlSent,model.power(now)==hub::Power::Off?"OFF":"ON");}
   else if(!std::strcmp(command,"list")||!std::strncmp(command,"list ",5)){
    char* end=nullptr;unsigned long page=command[4]?std::strtoul(command+5,&end,10):1;
    if(page&&page<=std::max<size_t>(1,(model.rodCount()+2)/3)&&(!end||!*end))model.select(hub::Screen::List,page-1,now);else Serial.println("HUB error: invalid list page");
   }
   else if(!std::strcmp(command,"history")||!std::strncmp(command,"history ",8)){
    char* end=nullptr;unsigned long page=command[7]?std::strtoul(command+8,&end,10):1;
    if(page&&page<=std::max<size_t>(1,(model.historyCount()+2)/3)&&(!end||!*end))model.select(hub::Screen::History,page-1,now);else Serial.println("HUB error: invalid history page");
   }
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
  if(power==hub::Power::Off)oled->oled_command(SH110X_DISPLAYOFF);
  else {oled->oled_command(SH110X_DISPLAYON);oled->oled_command(0x81);oled->oled_command(power==hub::Power::Dim?0x20:0xff);}
  appliedPower=power;powerKnown=true;frameValid=false;
 }
 if(power==hub::Power::Off||(frameValid&&hub::elapsed(now,lastDrawAt)<200))return;
 lastDrawAt=now;
 hub::Diagnostics d;d.radio=radioReady;d.oled=oledReady;d.voltage=hubVoltage;std::memcpy(d.mac,hubMac,sizeof(d.mac));
 portENTER_CRITICAL(&inboxMux);d.rx=received;d.invalid=invalid;d.dropped=dropped;portEXIT_CRITICAL(&inboxMux);
 hub::Frame frame=hub::render(model,now,hubBattery,d);
 if(frameValid&&frame.bytes==lastFrame.bytes)return;
 if(!probe(oledAddress)){oledReady=false;oledAttemptAt=now;return;}
 oled->clearDisplay();oled->drawBitmap(0,0,frame.bytes.data(),128,64,SH110X_WHITE);oled->display();
 // display() has no transport return status, so re-probe before trusting state.
 if(!probe(oledAddress)){oledReady=false;oledAttemptAt=now;return;}
 lastFrame=frame;frameValid=true;
}
void setup(){
 pinMode(LedPin,OUTPUT);pinMode(MotorPin,OUTPUT);digitalWrite(MotorPin,LOW);pinMode(ButtonPin,INPUT_PULLUP);
 pinMode(BatteryPin,INPUT);analogReadResolution(12);analogSetPinAttenuation(BatteryPin,ADC_11db);
 Serial.begin(115200);setCpuFrequencyMhz(80);
 uint32_t now=millis();model.begin(now);
 // Receiver starts before any OLED operation; missing OLED cannot gate reception.
 beginRadio(now);
 Wire.begin(D4,D5);Wire.setClock(400000);Wire.setTimeOut(10);
 readBattery();lastBatteryAt=now;button.begin(digitalRead(ButtonPin)==LOW,millis());printHelp();
}
void loop(){
 uint32_t now=millis();
 bool motorOn=motor.tick(now);digitalWrite(MotorPin,motorOn?HIGH:LOW);
 model.tick(now);
 switch(button.update(digitalRead(ButtonPin)==LOW,now)){
  case hub::ButtonEvent::LongPress:Serial.printf("HUB button: MAC %s shown for 30 s\n",hubMac[0]?hubMac:"unknown");model.select(hub::Screen::Mac,0,now);break;
  case hub::ButtonEvent::ShortPress:if(model.manual())model.close(now);else model.select(hub::Screen::Summary,0,now);break;
  default:break;
 }
 // Drain a bounded burst and choose one highest-priority motor request.
 uint8_t motorRequest=0;bool batteryRequested=false;
 for(size_t i=0;i<hub::RxDepth;++i){Incoming incoming;if(!popEvent(incoming))break;
  if(incoming.control){const auto& message=incoming.message;
   if(discovery.accepts(message.packet)){
    int id=model.registerRod(message.mac.data(),message.packet.name,message.packet.flags&startup::BatteryValid,message.packet.battery,now);
    if(id>=0){++controlReceived;sendControl(message.mac.data(),discovery.ack(message.packet,uint8_t(id+1)));frameValid=false;Serial.printf("HUB registration node=%u count=%u\n",unsigned(id+1),unsigned(model.rodCount()));}
   }continue;
  }
  hub::Ingest result=model.ingest(incoming.event,now);
  if(result.newEpisode){frameValid=false;batteryRequested=true;}
  if(result.motor&&(!motorRequest||(motorRequest==3&&result.type!=3)))motorRequest=result.type;
 }
 motorOn=motor.tick(millis());digitalWrite(MotorPin,motorOn?HIGH:LOW);
 serialCommands(now);
 // No slow I2C operation runs during a motor pattern, including its off gap.
 // Otherwise a failed bus could delay switching the motor off.
 if(!motor.busy()){
  beginRadio(now);beginOled(now);
  if(batteryRequested||(model.power(now)!=hub::Power::Off&&hub::elapsed(now,lastBatteryAt)>=5000)){readBattery();lastBatteryAt=now;}
  if(discovery.due(now)){const uint8_t broadcast[6]={255,255,255,255,255,255};sendControl(broadcast,discovery.take());}
  updateOled(now);
 }
 // Paint a newly awakened alert before energising the motor. No I2C transfer
 // can then stretch an already running motor pulse.
 if(motorRequest)motor.start(motorRequest,millis());
 digitalWrite(MotorPin,motor.tick(millis())?HIGH:LOW);
 bool sleeping=model.power(now)==hub::Power::Off;
 digitalWrite(LedPin,!sleeping?HIGH:LOW);
 delay(1);
}
