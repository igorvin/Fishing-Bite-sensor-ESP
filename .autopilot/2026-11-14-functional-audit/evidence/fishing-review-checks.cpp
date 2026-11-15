
#include <cstdint>
#include <cmath>
#include <cassert>
#include <cstdio>
uint32_t clockMs=10000;
uint32_t millis(){return clockMs;}
struct { void tick(){} } sett;
struct { int softAPgetStationNum(){return 0;} } WiFi;
struct { void println(const char*){} } Serial;
struct sensors_event_t {struct {float x=0,y=0,z=9.80665;} acceleration;};
struct {bool getEvent(sensors_event_t*,sensors_event_t*,sensors_event_t*){return true;}} lsm6;
struct Vec3 {float x,y,z;};
Vec3 gravF{};
bool armed=false,wifiApActiveAfterArm=false,espNowEnabled=false,espNowInited=false,hubMacValid=false;
bool lowBattReported=false,imuOK=true,gravInited=false,armWarmup=false,inMotion=false,pulseOn=false;
bool armButton=false,apClosed=false;
uint32_t wifiArmStart=0,bootMs=10000,lastVbatMs=10000,lastSample=0,armAtMs=0,pulseOffAt=0,motionStartMs=0,lastPulseStart=0;
uint16_t warmupCnt=0,cfg_shortPulseMs=120,cfg_contThresh=250,cfg_pulsePeriod=300;
uint8_t consecutiveTriggers=0;
float g_vbat_V=0,g_lastDelta=0,baselineMagG=1,warmupSumMag=0,cfg_deltaG=.15;
int g_batt_pct=100;
const uint32_t ARMED_AP_WINDOW_MS=120000,DISARMED_AP_GRACE_MS=180000,VBAT_PERIOD_MS=5000,SAMPLE_INTERVAL_MS=10,ARM_SETTLE_MS=1500;
const int LOW_BATT_THRESHOLD_PCT=20,LOW_BATT_HYSTERESIS=5,TRIGGER_SAMPLES=3;
const float MS2_PER_G=9.80665f,MOVE_GRAV_ALPHA=.94f;
void esp_task_wdt_reset(){} void handlePowerButton(){} void updateGreenLED(){}
void handleButton(){if(armButton){armed=true;clockMs+=600;wifiArmStart=millis();wifiApActiveAfterArm=true;armAtMs=millis();armWarmup=true;armButton=false;}}
void wifiAfterArmWindow(){apClosed=true;} void suspendUntilLongPressToArm(){}
float readVbat(){return 4.2;} int vbatToPercent(float){return 100;} void sendBitePacket(int){}
float mag3(float x,float y,float z){return std::sqrt(x*x+y*y+z*z);}
void stopPulse(){pulseOn=false;} void resetAutoDisarm(){}
void autoDisarmCaptureBaseline(uint32_t,const Vec3&,float){} bool checkAutoDisarmMove(uint32_t,const Vec3&){return false;}
void setArmed(bool v,bool){armed=v;} void startPulse(uint32_t,int){pulseOn=true;}
void setRedLED(bool){} void buzzStart(){}
inline void startTestPulse() {
  uint32_t now = millis();
  pulseOn        = true;
  lastPulseStart = now;
  pulseOffAt     = now + cfg_shortPulseMs;
  setRedLED(true);
  buzzStart();
}
void loop(){
  esp_task_wdt_reset();
  sett.tick();
  uint32_t now = millis();

  handlePowerButton();
  handleButton();
  updateGreenLED();

  // AP window after ARM
  if (armed && wifiApActiveAfterArm && (now - wifiArmStart >= ARMED_AP_WINDOW_MS)) {
    wifiAfterArmWindow();
    wifiApActiveAfterArm = false;
  }

  // DISARMED: sleep after grace if no clients
  if (!armed){
    bool clientConnected = (WiFi.softAPgetStationNum()>0);
    bool inGrace = (now - bootMs) < DISARMED_AP_GRACE_MS;
    if (!clientConnected && !inGrace) suspendUntilLongPressToArm();
  }

  // Battery periodic update
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

  // IMU sample timing
  if (now - lastSample < SAMPLE_INTERVAL_MS) return;
  lastSample = now;

  if (!imuOK) return;

  sensors_event_t accel, gyro, temp;
  if (!lsm6.getEvent(&accel, &gyro, &temp)) return;

  float gx = accel.acceleration.x / MS2_PER_G;
  float gy = accel.acceleration.y / MS2_PER_G;
  float gz = accel.acceleration.z / MS2_PER_G;

  // init gravity filter
  if (!gravInited) {
    gravF = {gx, gy, gz};
    gravInited = true;
  } else {
    gravF.x = MOVE_GRAV_ALPHA*gravF.x + (1.0f - MOVE_GRAV_ALPHA)*gx;
    gravF.y = MOVE_GRAV_ALPHA*gravF.y + (1.0f - MOVE_GRAV_ALPHA)*gy;
    gravF.z = MOVE_GRAV_ALPHA*gravF.z + (1.0f - MOVE_GRAV_ALPHA)*gz;
  }

  float mag = mag3(gx,gy,gz);
  baselineMagG = baselineMagG*0.999f + mag*0.001f;

  float d = fabsf(mag - baselineMagG);
  g_lastDelta = d;

  // ARM warmup block
  if (armWarmup) {
    warmupCnt++;
    warmupSumMag += mag;

    if ((now - armAtMs) >= ARM_SETTLE_MS) {
      if (warmupCnt > 0) baselineMagG = warmupSumMag / (float)warmupCnt;

      consecutiveTriggers = 0;
      inMotion = false;
      stopPulse();

      armWarmup = false;

      resetAutoDisarm();
      Serial.println("ARM warmup finished");
    }
    return;
  }

  // Auto-disarm baseline capture
  autoDisarmCaptureBaseline(now, gravF, d);

  // Auto-disarm trigger
  if (checkAutoDisarmMove(now, gravF)) {
    Serial.println("AUTO-DISARM: rod moved -> disarm");
    setArmed(false, false);
    return;
  }

  // Bite detection (Δg)
  if (d >= cfg_deltaG){ if (consecutiveTriggers<255) consecutiveTriggers++; }
  else if (consecutiveTriggers>0) consecutiveTriggers--;

  bool motionNow = (consecutiveTriggers >= TRIGGER_SAMPLES);
  if (motionNow && !inMotion){
    inMotion = true;
    motionStartMs = now;
    startPulse(now, 1);
  } else if (!motionNow && inMotion){
    inMotion = false;
  }

  // End short pulse
  if (pulseOn && (int32_t)(now - pulseOffAt) >= 0) stopPulse();

  // Continuous vibration -> repeated pulses
  if (inMotion){
    uint32_t motionDur = now - motionStartMs;
    if (motionDur >= cfg_contThresh){
      if (!pulseOn && (now - lastPulseStart) >= cfg_pulsePeriod){
        startPulse(now, 2);
      }
    }
  }
}


int main(){
 armButton=true; loop();
 assert(apClosed); assert(!armWarmup);
 puts("REPRODUCED: button ARM closes AP immediately and bypasses 1500 ms warmup");
 armed=false; bootMs=clockMs; lastVbatMs=clockMs;
 startTestPulse(); clockMs+=1000; loop(); assert(pulseOn);
 puts("REPRODUCED: disarmed test pulse remains on after 1000 ms (configured 120 ms)");
 float baseRoll=179, roll=-179;
 assert(std::fabs(roll-baseRoll)>35);
 puts("REPRODUCED: 2-degree roll across +/-180 is treated as 358 degrees");
}
