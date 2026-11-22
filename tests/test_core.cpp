#include "SensorCore.h"
#include "ImuRead.h"
#include "CsvBuffer.h"
#include "ConfigDatabase.h"
#include "ImuSession.h"
#include "PowerButton.h"
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <functional>
#include <string>
#include <vector>
#include <stdexcept>
using namespace bite;
void require(bool ok,const char* name){if(!ok)throw std::runtime_error(name);}
uint32_t calibrate(SensorCore& c,uint32_t t=1000){c.update({0,0,1},t);require(c.beginCalibration(t),"calibration starts after valid sample");for(int i=0;i<201;++i){t+=10;c.update({0,0,1},t);}require(c.calibrated(),"quiet calibration completes");return t;}
uint32_t arm(SensorCore& c){Config cfg;c.configure(cfg);uint32_t t=calibrate(c);require(c.setArmed(true,t),"arm accepted");for(int i=0;i<150;++i){t+=10;c.update({0,0,1},t);}require(c.state()==State::Armed,"settled to armed");return t;}
struct FakeIo {bool statusOk=true,burstOk=true;uint8_t status=1;int bursts=0;bool read(uint8_t reg,uint8_t* out,size_t n){if(reg==0x1e){*out=status;return statusOk;}++bursts;if(!burstOk)return false;for(size_t i=0;i<n;++i)out[i]=0;out[4]=0x09;out[5]=0x40;return true;}};
struct FakeStorage {std::string bytes="corrupt existing bytes";bool exists=true,readable=false;int writes=0;};
// Only file IO and library scheduling are faked; the production guard is under test.
struct FakeFileDb {
    FakeStorage* fs;bool _update=false;uint32_t now=0,dirtyAt=0;
    explicit FakeFileDb(FakeStorage* f):fs(f){}
    ~FakeFileDb(){update();}
    bool begin(){if(!fs->exists){fs->exists=true;fs->bytes="";return true;}return fs->readable;}
    void set(){_update=true;}
    bool update(){if(!_update)return false;_update=false;++fs->writes;fs->bytes="saved settings";return true;}
    virtual bool tick(){if(_update&&!dirtyAt)dirtyAt=now;if(dirtyAt&&now-dirtyAt>=10000)return update();return false;}
};
struct SessionIo : FakeIo {int begins=0;bool begin(){++begins;return true;}ReadStatus sample(Vec3& a){return readAccel(*this,a);}};
int main(){
std::vector<std::pair<const char*,std::function<void()>>> tests={
{"storage-load-failure-preserves-bytes",[]{FakeStorage fs;auto original=fs.bytes;{ConfigDatabase<FakeFileDb> db(&fs);require(!db.begin(),"existing corrupt DB fails load");db.set();db.now=1;FakeFileDb* settings=&db;settings->tick();db.set();require(!db.update(),"RAM-only explicit save must be blocked");db.set();db.now=12002;settings->tick();require(fs.bytes==original&&fs.writes==0,"UI/calibration and delayed Settings tick preserve file");db.set();}require(fs.bytes==original&&fs.writes==0,"base destructor cannot save RAM defaults");}},
{"first-frame-timeout-retry-recovery",[]{for(uint32_t start:{100u,UINT32_MAX-80}){SensorCore c;SessionIo io;io.status=0;ImuSession session;session.begin(io,c,start);for(uint32_t dt=10;dt<200;dt+=10){Vec3 a;uint8_t event;session.supervise(io,c,start+dt);if(session.due(start+dt))session.read(io,c,start+dt,a,event);if(dt<150)require(io.begins==1&&session.online(),"normal DRDY wait must not reinit");}require(!session.online()&&io.begins==1&&c.state()==State::Fault,"no first frame has bounded deadline");session.supervise(io,c,start+2149);require(io.begins==1,"retry remains sparse");session.supervise(io,c,start+2150);require(io.begins==2&&session.online(),"retry successful begin after cooldown");io.status=1;Vec3 a;uint8_t event;session.read(io,c,start+2160,a,event);require(c.state()==State::Calibrating&&!c.desiredArmed(),"recovery must calibrate without arming");for(uint32_t dt=2170;dt<=4170;dt+=10){session.supervise(io,c,start+dt);if(session.due(start+dt))session.read(io,c,start+dt,a,event);}require(c.calibrated()&&c.state()==State::Disarmed&&!c.desiredArmed(),"recovery completes disarmed");}}},
{"storage-failed-load-auto-write",[]{FakeStorage fs;auto original=fs.bytes;{ConfigDatabase<FakeFileDb> db(&fs);require(!db.begin(),"load fails");FakeFileDb* settings=&db;db.set();db.now=1;settings->tick();db.set();db.now=12002;settings->tick();require(fs.bytes==original&&fs.writes==0,"library auto-write cannot replace corrupt bytes");}require(fs.bytes==original,"destruction preserves corrupt bytes");}},
{"storage-new-db-still-saves",[]{FakeStorage fs;fs.exists=false;ConfigDatabase<FakeFileDb> db(&fs);require(db.begin()&&fs.exists,"absent database is created");db.set();require(db.update()&&fs.writes==1,"new DB explicit save stays enabled");db.set();db.now=1;FakeFileDb* settings=&db;settings->tick();db.now=12002;settings->tick();require(fs.writes==2,"new DB Settings auto save stays enabled");}},
{"weak-axis",[]{Features f;for(uint32_t t=0;t<2000;t+=10)f.update({0,0,1},t);auto s=f.update({.1f,0,1},2000);require(s.dynamic>.09f,"weak transverse impulse preserves direction");require(s.legacy<.006f,"legacy magnitude blind spot");}},
{"first-armed-impulse",[]{SensorCore c;uint32_t t=arm(c);uint8_t event=0;for(int i=0;i<5;++i){t+=10;event|=c.update({.1f,0,1},t);}require(event==1,"first impulse after settle must not become gravity baseline");}},
{"dynamic-versus-legacy",[]{SensorCore d,l;auto t=arm(d);arm(l);Config cfg=l.config();cfg.legacyMode=true;l.configure(cfg);int de=0,le=0;for(int i=0;i<10;++i){t+=10;de+=d.update({.1f,0,1},t)==1;le+=l.update({.1f,0,1},t)==1;}require(de==1&&le==0,"same transverse input dynamic triggers once, legacy misses");}},
{"stationary-calibration",[]{SensorCore c;calibrate(c);require(c.calibration()==Calibration::Success&&c.telemetry().noiseG<.0001f,"quiet outcome and noise");}},
{"moving-calibration",[]{SensorCore c;c.update({0,0,1},100);c.beginCalibration(100);c.update({.08f,0,1},110);require(c.calibration()==Calibration::Moving&&!c.calibrated()&&c.state()==State::Disarmed,"movement rejects calibration");}},
{"noisy-calibration",[]{SensorCore c;uint32_t t=calibrate(c);c.beginCalibration(t);for(int i=0;i<205;++i){t+=10;c.update({i%2?.01f:-.01f,0,1},t);}require(c.telemetry().noiseG>.009f,"noise measured");c.setArmed(true,t);for(int i=0;i<160;++i){t+=10;c.update({0,0,1},t);}require(c.telemetry().effectiveG>.035f,"noise raises effective threshold");}},
{"manual-disarm",[]{SensorCore c;auto t=arm(c);c.setArmed(false,t);for(int i=0;i<1000;++i){t+=10;c.update({0,.5f,.866f},t);}require(c.state()==State::Disarmed&&!c.desiredArmed(),"manual disarm cannot auto resume");}},
{"suppression-new-position",[]{SensorCore c;auto t=arm(c);for(int i=0;i<100;++i){t+=10;c.update({0,.8660254f,.5f},t);}require(c.state()==State::MotionSuppressed,"rod movement suppresses");for(int i=0;i<400;++i){t+=10;auto e=c.update({0,.8660254f,.5f},t);require(e==0,"no alarm while resuming at stable new position");}require(c.state()==State::Armed&&c.desiredArmed(),"resume without returning to old angle");}},
{"fault-cannot-arm",[]{SensorCore c;require(!c.setArmed(true,10),"no IMU cannot arm");auto t=arm(c);c.fault(t);require(c.state()==State::Fault&&!c.desiredArmed(),"fault clears desired arm");c.update({0,0,1},t+10);require(c.state()==State::Disarmed&&!c.desiredArmed(),"recovery never silently arms");}},
{"gap-fault",[]{SensorCore c;auto t=arm(c);c.tick(t+151);require(c.state()==State::Fault&&c.telemetry().gaps==1,"no new sample faults");}},
{"gap-count-once",[]{SensorCore c;auto t=arm(c);c.tick(t+200);c.update({0,0,1},t+210);require(c.telemetry().gaps==1,"one interruption must not be counted twice by tick and fresh read");}},
{"bad-sample",[]{SensorCore c;auto t=arm(c);c.update({std::numeric_limits<float>::quiet_NaN(),0,1},t+10);require(!c.healthy(),"NaN cannot be valid IMU");}},
{"release-no-tail",[]{SensorCore c;auto t=arm(c);Config cfg=c.config();cfg.suppression=false;c.configure(cfg);for(int i=0;i<400;++i){t+=10;c.update({.2f,0,1},t);}int late=0;for(int i=0;i<100;++i){t+=10;auto e=c.update({0,0,1},t);if(i>10)late+=e;}require(late==0,"motion counter does not keep triggering for seconds after rest");}},
{"consecutive-attack",[]{SensorCore c;auto t=arm(c);int events=0;for(int i=0;i<20;++i){t+=10;events+=c.update({i%2?.1f:0,0,1},t);}require(events==0,"isolated threshold crossings do not accumulate");}},
{"pulse-in-disarmed-and-rollover",[]{Pulse p;p.start(UINT32_MAX-50,120);require(p.tick(UINT32_MAX-1),"pulse initially on");require(!p.tick(70),"pulse expires across millis rollover regardless sensor state");}},
{"state-entry-grace",[]{StateClock c;c.enter(400000);require(!c.expired(400001,180000),"grace starts on disarm, not boot");require(c.expired(580000,180000),"grace ends after full interval");}},
{"angle-wrap",[]{require(angleDifference(179,-179)<2.01f,"wrap is two degrees");}},
{"wake-pin-validation",[]{require(!wakePinAllowed(3,21)&&!wakePinAllowed(6,16)&&!wakePinAllowed(33,43),"D6 invalid for all current boards");require(wakePinAllowed(3,4)&&wakePinAllowed(33,4),"valid range accepted");}},
{"wake-pin-d1",[]{require(wakePinAllowed(3,3)&&wakePinAllowed(33,2)&&wakePinAllowed(6,1),"D1 (C3 GPIO3, S3 GPIO2, C6 GPIO1) can deep-wake");}},
{"button-arm-hold",[]{PowerButton b;b.begin(false,0);uint32_t t=0;int cues=0,toggles=0,sleeps=0;
  for(;t<=1600;t+=10){auto a=b.update(true,t);cues+=a==ButtonAction::ArmCue;}
  for(int i=0;i<10;++i){t+=10;auto a=b.update(false,t);toggles+=a==ButtonAction::ToggleArm;sleeps+=a==ButtonAction::Sleep;}
  require(cues==1&&toggles==1&&sleeps==0,"1-3 s hold toggles ARM once, on release");}},
{"button-sleep-hold",[]{PowerButton b;b.begin(false,0);uint32_t t=0;int arm=0,sleepCue=0,toggles=0,sleeps=0;
  for(;t<=3600;t+=10){auto a=b.update(true,t);arm+=a==ButtonAction::ArmCue;sleepCue+=a==ButtonAction::SleepCue;require(a!=ButtonAction::Sleep,"no sleep while still pressed");}
  for(int i=0;i<10;++i){t+=10;auto a=b.update(false,t);toggles+=a==ButtonAction::ToggleArm;sleeps+=a==ButtonAction::Sleep;}
  require(arm==1&&sleepCue==1&&sleeps==1&&toggles==0,">=3 s hold sleeps on release and does not toggle ARM");}},
{"button-short-and-bounce",[]{PowerButton b;b.begin(false,0);uint32_t t=0;int any=0;
  for(int i=0;i<40;++i){t+=10;any+=b.update(i%2==0,t)!=ButtonAction::None;}
  for(int i=0;i<30;++i){t+=10;any+=b.update(true,t)!=ButtonAction::None;}
  for(int i=0;i<20;++i){t+=10;any+=b.update(false,t)!=ButtonAction::None;}
  require(any==0&&b.released(),"bounce and short press do nothing");}},
{"button-wake-press-ignored",[]{PowerButton b;b.begin(true,0);uint32_t t=0;int any=0;
  for(;t<=5000;t+=10)any+=b.update(true,t)!=ButtonAction::None;
  for(int i=0;i<10;++i){t+=10;any+=b.update(false,t)!=ButtonAction::None;}
  require(any==0,"press held from boot is not ARM or sleep");
  for(int i=0;i<150;++i){t+=10;any+=b.update(true,t)==ButtonAction::ArmCue;}
  require(any==1,"next real press works");}},
{"button-rollover",[]{PowerButton b;uint32_t t=UINT32_MAX-500;b.begin(false,t);int toggles=0;
  for(int i=0;i<150;++i){t+=10;b.update(true,t);}for(int i=0;i<10;++i){t+=10;toggles+=b.update(false,t)==ButtonAction::ToggleArm;}
  require(toggles==1,"hold timing spans millis rollover");}},
{"wake-confirm",[]{WakeConfirm w;w.begin(0);WakeResult r=WakeResult::Waiting;uint32_t t=0;
  for(;t<1000&&r==WakeResult::Waiting;t+=10)r=w.update(true,t);require(r==WakeResult::Waiting,"not confirmed before 1 s");
  r=w.update(true,1000);require(r==WakeResult::Confirmed,"held 1 s confirms");
  w.begin(0);r=w.update(true,0);for(t=300;t<400&&r==WakeResult::Waiting;t+=10)r=w.update(false,t);require(r==WakeResult::Rejected,"brief touch goes back to sleep");
  w.begin(0);w.update(true,500);require(w.update(false,510)==WakeResult::Waiting&&w.update(true,520)==WakeResult::Waiting&&w.update(true,1000)==WakeResult::Confirmed,"contact bounce tolerated");}},
{"low-battery-guard",[]{LowBatteryGuard g;require(!g.update(true,3.2f)&&!g.update(true,3.2f),"two low readings not enough");require(!g.update(true,3.5f),"recovery resets");
  g.update(true,3.2f);g.update(true,3.2f);require(g.update(true,3.29f),"three consecutive low readings trip");
  LowBatteryGuard u;for(int i=0;i<10;++i)require(!u.update(false,0),"unknown voltage (USB only) never trips");}},
{"config-validation",[]{SensorCore c;Config cfg;cfg.thresholdG=.001f;require(!c.configure(cfg)&&c.config().thresholdG==.02f,"bad config cannot replace last good");cfg.thresholdG=std::numeric_limits<float>::infinity();require(!c.configure(cfg),"infinite threshold rejected");}},
{"network-validation",[]{uint8_t out[6]={9,9,9,9,9,9};require(!parseMac("100:0:00:00:00:00",out)&&out[0]==9,"overflow MAC rejected without modifying output");require(parseMac("02:aa:BB:00:11:22",out),"valid unicast MAC");require(!parseMac("ff:ff:ff:ff:ff:ff",out),"multicast rejected");require(!validNetwork("","")&&!validNetwork("sensor","short"),"bad AP settings rejected");require(validNetwork("sensor",""),"intentional open AP allowed");}},
{"checked-transaction",[]{FakeIo io;Vec3 out{9,9,9};io.burstOk=false;require(readAccel(io,out)==ReadStatus::TransportError&&out.x==9,"failed actual burst cannot produce valid sample");io.burstOk=true;io.status=0;require(readAccel(io,out)==ReadStatus::NoNewData&&io.bursts==1,"no data-ready means no stale burst");io.status=1;require(readAccel(io,out)==ReadStatus::Fresh&&out.z>.99f&&out.z<1.01f,"checked burst units are g");}},
{"calibration-fault",[]{SensorCore c;auto t=calibrate(c);c.beginCalibration(t);c.fault(t+10);require(c.calibration()==Calibration::SensorError&&!c.desiredArmed(),"fault rejects calibration and intent");}},
{"csv-buffer-backpressure",[]{CsvBuffer<16> q;require(q.append("first\n",6),"enqueue full record");require(!q.append("too-long-record\n",16)&&q.size()==6,"overflow drops whole record");char out[16];require(q.peek(out,3)==3&&out[0]=='f',"partial drain");q.consume(3);require(q.append("second\n",7),"wrap-around enqueue");auto n=q.peek(out,16);require(std::string(out,n)=="st\nsecond\n","record order after wrap");}},
{"persisted-noise-validation",[]{SensorCore c;require(c.restoreNoise(.01f)&&c.telemetry().noiseG==.01f,"restore noise result");require(!c.restoreNoise(.2f)&&c.telemetry().noiseG==.01f,"reject invalid noise without replacing");require(!c.calibrated()&&!c.setArmed(true,0),"stored noise alone never proves current IMU ready");}},
{"timestamp-rollover",[]{SensorCore c;uint32_t t=UINT32_MAX-1000;c.update({0,0,1},t);c.beginCalibration(t);for(int i=0;i<201;++i){t+=10;c.update({0,0,1},t);}require(c.calibrated(),"calibration spans rollover");}}
};
int failures=0;for(auto& test:tests){try{test.second();std::printf("PASS: %s\n",test.first);}catch(const std::exception& e){++failures;std::fprintf(stderr,"FAIL: %s: %s\n",test.first,e.what());}}
std::printf("%zu tests, %d failed\n",tests.size(),failures);return failures?1:0;
}
