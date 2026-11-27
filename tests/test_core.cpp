#include "SensorCore.h"
#include "ImuRead.h"
#include "CsvBuffer.h"
#include "ConfigDatabase.h"
#include "ImuSession.h"
#include "PowerButton.h"
#include "Feedback.h"
#include "EventLog.h"
#include "Sensitivity.h"
#include <cstring>
#include <fstream>
#include <sstream>
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
// Replays a field capture through the production core with a web preset; skips the start
// (the 2026-11-24 captures begin with a reboot) and arms 3 s later, like tools/replay.cpp.
struct FieldResult {size_t rows=0,armedRows=0,alarms=0;};
FieldResult replayField(const char* name,uint8_t preset,uint32_t skipMs=4000){
    std::string path=std::string(__FILE__);path=path.substr(0,path.find_last_of('/'))+"/fixtures/field/"+name;
    std::ifstream in(path);require(bool(in),"field fixture present");
    SensorCore c;require(c.configure(configForSensitivity(preset)),"preset applies");
    FieldResult r;std::string line;bool seen=false,started=false,armed=false;uint32_t origin=0,first=0;
    while(std::getline(in,line)){
        if(line.empty()||line[0]<'0'||line[0]>'9')continue;
        std::stringstream ss(line);std::string f[4];for(auto& x:f)std::getline(ss,x,',');
        uint32_t now=uint32_t(std::stoul(f[0]));Vec3 a{std::stof(f[1]),std::stof(f[2]),std::stof(f[3])};
        if(!seen){seen=true;origin=now;}
        ++r.rows;if(elapsed(now,origin)<skipMs)continue;
        if(!started){c.update(a,now);c.beginCalibration(now);first=now;started=true;}
        if(!armed&&c.calibrated()&&elapsed(now,first)>=3000){c.setArmed(true,now);armed=true;}
        auto e=c.update(a,now);r.alarms+=e!=0;r.armedRows+=c.state()==State::Armed;
    }
    return r;
}
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
{"moving-calibration",[]{SensorCore c;c.update({0,0,1},100);c.beginCalibration(100);c.update({.08f,0,1},110);require(c.state()==State::Calibrating,"a small sway sample is not handling");c.update({.9f,0,.436f},120);require(c.calibration()==Calibration::Moving&&!c.calibrated()&&c.state()==State::Disarmed&&c.telemetry().calReject==CalReject::Deviation,"a handling jolt rejects calibration");}},
{"noisy-calibration",[]{SensorCore c;uint32_t t=calibrate(c);c.beginCalibration(t);for(int i=0;i<205;++i){t+=10;c.update({i%2?.01f:-.01f,0,1},t);}require(c.calibrated()&&c.telemetry().noiseG>.009f,"steady vibration is accepted and measured");
  c.setArmed(true,t);for(int i=0;i<300;++i){t+=10;c.update({i%2?.01f:-.01f,0,1},t);}require(c.state()==State::Armed&&c.telemetry().effectiveG>.035f,"sharp vibration raises the running threshold");
  c.setArmed(false,t);c.setArmed(true,t);for(int i=0;i<1300;++i){t+=10;c.update({0,0,1},t);}require(c.state()==State::Armed&&c.telemetry().effectiveG<.021f,"about 10 s after the vibration stops the threshold is back at the floor");}},
{"manual-disarm",[]{SensorCore c;auto t=arm(c);c.setArmed(false,t);for(int i=0;i<1000;++i){t+=10;c.update({0,.5f,.866f},t);}require(c.state()==State::Disarmed&&!c.desiredArmed(),"manual disarm cannot auto resume");}},
{"suppression-new-position",[]{SensorCore c;auto t=arm(c);for(int i=0;i<100;++i){t+=10;c.update({0,.8660254f,.5f},t);}require(c.state()==State::MotionSuppressed,"rod movement suppresses");for(int i=0;i<400;++i){t+=10;auto e=c.update({0,.8660254f,.5f},t);require(e==0,"no alarm while resuming at stable new position");}require(c.state()==State::Armed&&c.desiredArmed(),"resume without returning to old angle");}},
{"fault-cannot-arm",[]{SensorCore c;require(!c.setArmed(true,10),"no IMU cannot arm");auto t=arm(c);c.fault(t);require(c.state()==State::Fault&&!c.desiredArmed(),"fault clears desired arm");c.update({0,0,1},t+10);require(c.state()==State::Disarmed&&!c.desiredArmed(),"recovery never silently arms");}},
{"gap-fault",[]{SensorCore c;auto t=arm(c);c.tick(t+151);require(c.state()==State::Fault&&c.telemetry().gaps==1,"no new sample faults");}},
{"gap-count-once",[]{SensorCore c;auto t=arm(c);c.tick(t+200);c.update({0,0,1},t+210);require(c.telemetry().gaps==1,"one interruption must not be counted twice by tick and fresh read");}},
{"bad-sample",[]{SensorCore c;auto t=arm(c);c.update({std::numeric_limits<float>::quiet_NaN(),0,1},t+10);require(!c.healthy(),"NaN cannot be valid IMU");}},
{"release-no-tail",[]{SensorCore c;auto t=arm(c);Config cfg=c.config();cfg.suppression=false;c.configure(cfg);for(int i=0;i<400;++i){t+=10;c.update({.2f,0,1},t);}int late=0;for(int i=0;i<100;++i){t+=10;auto e=c.update({0,0,1},t);if(i>10)late+=e;}require(late==0,"motion counter does not keep triggering for seconds after rest");}},
{"single-sample-tap-alarms-once",[]{
  // Field taps last 1..2 samples; the impulse envelope keeps them long enough for the 20 ms attack.
  SensorCore c;auto t=arm(c);int ones=0,others=0;for(int i=0;i<60;++i){t+=10;auto e=c.update({i==10?.1f:0.f,0,1},t);ones+=e==1;others+=e==2;}
  require(ones==1&&others==0,"one isolated 0.1 g tap gives exactly one bite event");}},
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
  for(;t<1000&&r==WakeResult::Waiting;t+=10){r=w.update(true,t);}require(r==WakeResult::Waiting,"not confirmed before 1 s");
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
{"calibration-reject-reason",[]{
  SensorCore c;uint32_t t=1000;c.update({0,0,1},t);c.beginCalibration(t);t+=10;c.update({.9f,0,.436f},t);
  require(c.calibration()==Calibration::Moving&&c.telemetry().calReject==CalReject::Deviation&&c.telemetry().calRejectValue>.899f&&c.telemetry().calRejectValue<.901f,"a jolt above 0.80 g is rejected at once with its size");
  c.beginCalibration(t);require(c.telemetry().calReject==CalReject::None,"a new calibration clears the old reason");t+=10;c.update({0,0,.7f},t);
  require(c.telemetry().calReject==CalReject::Magnitude,"out-of-range magnitude is recorded");
  SensorCore d;t=1000;d.update({0,0,1},t);d.beginCalibration(t);for(int i=0;i<201;++i){t+=10;d.update({i<100?0.f:.06f,0,1},t);}
  require(d.calibration()==Calibration::Moving&&d.telemetry().calReject==CalReject::Drift&&d.telemetry().calRejectValue>.035f,"a position change between the halves is drift");
  SensorCore n;t=1000;n.update({0,0,1},t);n.beginCalibration(t);for(int i=0;i<201;++i){t+=10;n.update({(i%2)?.2f:-.2f,0,1},t);}
  require(n.calibration()==Calibration::Moving&&n.telemetry().calReject==CalReject::Noise&&n.telemetry().calRejectValue>.15f,"sway beyond 0.15 g SD is too strong");}},
{"button-arm-uncalibrated-reproduction",[]{
  // Field report: after RESET the button would not arm, web ARM worked, then the button worked.
  SensorCore c;Config cfg;c.configure(cfg);uint32_t t=1000;c.update({0,0,1},t);c.beginCalibration(t);
  for(int i=0;i<20;++i){t+=10;c.update({(i%2)?.45f:-.45f,0,1},t);}
  require(!c.calibrated()&&c.calibration()==Calibration::Moving,"a finger jolting the board during boot calibration rejects it");
  require(c.setArmed(true,t)&&c.state()==State::Calibrating,"button ARM then needs a calibration");
  for(int i=0;i<10;++i){t+=10;c.update({(i%2)?.45f:-.45f,0,1},t);}
  require(!c.desiredArmed()&&c.state()==State::Disarmed,"release jiggle silently cancels that ARM (core behaviour kept)");
  // ArmAssist: same button ARM, calibration starts only 1 s after release.
  ArmAssist assist;assist.defer(t);require(!assist.due(t+500,true),"not before the settle delay");
  require(!assist.due(t+900,false),"a press restarts the wait");require(!assist.due(t+1800,true),"wait counted from the last press");
  require(assist.due(t+1900,true)&&!assist.pending(),"due once, after release + settle");for(int i=0;i<190;++i){t+=10;c.update({0,0,1},t);}
  require(c.setArmed(true,t)&&c.state()==State::Calibrating,"deferred ARM starts calibration");
  for(int i=0;i<201;++i){t+=10;c.update({0,0,1},t);}
  for(int i=0;i<160;++i){t+=10;c.update({0,0,1},t);}
  require(c.calibrated()&&c.state()==State::Armed,"untouched calibration then arms");}},
{"boot-calibration-retry-bounded",[]{
  ArmAssist a;a.rejected(1000);
  require(!a.retryDue(2500,true,true),"no retry sooner than 2 s after rejection");
  require(!a.retryDue(3000,true,false),"no retry while the button is pressed");
  require(!a.retryDue(3000,false,true),"no retry when not eligible");
  require(a.retryDue(3000,true,true)&&a.retries()==1,"first retry");
  a.rejected(5000);require(a.retryDue(7000,true,true)&&a.retries()==2,"second retry");
  a.rejected(9000);require(a.retryDue(11000,true,true)&&a.retries()==3,"third retry");
  a.rejected(13000);require(!a.retryDue(20000,true,true),"bounded: at most three");
  a.calibrated();a.rejected(UINT32_MAX-500);require(!a.retryDue(1000,true,true)&&a.retryDue(1500,true,true),"success resets the budget; rollover safe");}},
{"armed-heartbeat-and-error-cue",[]{
  require(Heartbeat::on(5000,5000)&&Heartbeat::on(5049,5000)&&!Heartbeat::on(5050,5000)&&!Heartbeat::on(6000,5000)&&!Heartbeat::on(9999,5000)&&Heartbeat::on(10000,5000),"50 ms flash every 5 s from entering ARMED");
  uint32_t since=UINT32_MAX-20;require(Heartbeat::on(since,since)&&!Heartbeat::on(since+100,since)&&!Heartbeat::on(since+1000,since)&&Heartbeat::on(since+5000,since),"phase survives millis rollover");
  Pattern p;p.start(100,3,80,80);int on=0,edges=0;bool last=false;
  for(uint32_t t=100;t<700;++t){bool x=p.tick(t);on+=x;edges+=x&&!last;last=x;}
  require(on==240&&edges==3&&!p.active(),"three 80 ms beeps, then finished");}},
{"event-log-ring-and-text",[]{
  EventLog<4> log;for(uint8_t i=0;i<6;++i)log.add(i*10,Log::CalRetry,i,3);
  require(log.size()==4&&log.total()==6&&log.at(0).a==2&&log.newest().a==5&&log.newest(3).a==2,"keeps the newest entries in order");
  char line[160];LogEntry e{12345,Log::ArmCancelled,uint8_t(CalReject::Deviation),0,0,.25f};formatLog(e,line,sizeof(line));
  require(std::strstr(line,"12.345s")&&std::strstr(line,"ARM cancelled")&&std::strstr(line,"jolt 0.250 g peak-to-peak")&&std::strstr(line,"limit 0.20"),"cancel explains the reason");
  e=LogEntry{0,Log::Arm,uint8_t(Source::Web),1,uint8_t(State::Calibrating),0};formatLog(e,line,sizeof(line));
  require(std::strstr(line,"ARM from web -> CALIBRATING"),"arm request shows source and result");
  e=LogEntry{0,Log::Arm,uint8_t(Source::Button),1,Refused,0};formatLog(e,line,sizeof(line));require(std::strstr(line,"refused: IMU fault"),"refusal");
  e=LogEntry{0,Log::Button,uint8_t(ButtonAction::ToggleArm),0,0,0};formatLog(e,line,sizeof(line));require(std::strstr(line,"toggle ARM"),"button action");
  char tiny[8];int n=formatLog(e,tiny,sizeof(tiny));require(n>0&&std::strlen(tiny)==7,"truncates safely");}},
{"arm-in-wind-sway",[]{
  // Field log 2026-11-23: wind-like sway (jolts 0.008..0.012 g at 104 Hz) kept SETTLING forever.
  auto sway=[](uint32_t t,float amp){float ph=float(t%667)/667.f*6.2831853f;return Vec3{amp*std::sin(ph),.5f*amp*std::cos(ph),1};};
  SensorCore c;Config cfg;c.configure(cfg);uint32_t t=calibrate(c);require(c.setArmed(true,t)&&c.state()==State::Settling,"armed request settles first");
  uint32_t armedAt=0;for(int i=0;i<400&&!armedAt;++i){t+=10;c.update(sway(t,.03f),t);if(c.state()==State::Armed)armedAt=t;}
  require(armedAt&&c.telemetry().settleResets==0,"steady 1.5 Hz sway arms after one 1.5 s window");
  float noise=c.telemetry().noiseG;require(noise>.015f&&noise<.03f,"window SD measures the sway");
  uint8_t ev=0;for(int i=0;i<1000;++i){t+=10;ev|=c.update(sway(t,.03f),t);}require(ev==0&&c.telemetry().effectiveG<.03f,"10 s of slow sway: no alarm, and the impulse threshold is not inflated by it");
  for(int i=0;i<8;++i){t+=10;Vec3 a=sway(t,.03f);a.y+=(i<2?.05f:0.f);ev|=c.update(a,t);}require(ev==1,"a small sharp tap during the sway alarms");
  // Being handled, being moved, or sway too strong: not steady, cause recorded, no arming.
  SensorCore h;h.configure(cfg);t=calibrate(h);h.setArmed(true,t);for(int i=0;i<400;++i){t+=10;h.update((i%30==0)?Vec3{.9f,0,.436f}:Vec3{0,0,1},t);}
  require(h.state()==State::Settling&&h.telemetry().settleResets>0&&h.telemetry().settleCause==uint8_t(Unsteady::Range),"handling jolts");
  SensorCore m;m.configure(cfg);t=calibrate(m);m.setArmed(true,t);for(int i=0;i<400;++i){t+=10;m.update({.0008f*float(i),0,1},t);}
  require(m.state()==State::Settling&&m.telemetry().settleCause==uint8_t(Unsteady::Drift),"slowly moving to a new position");
  SensorCore w;w.configure(cfg);t=calibrate(w);w.setArmed(true,t);for(int i=0;i<400;++i){t+=10;w.update(sway(t,.3f),t);}
  require(w.state()==State::Settling&&w.telemetry().settleCause==uint8_t(Unsteady::Noise)&&w.telemetry().settleLimit==.15f,"sway too strong to guard");
  for(int i=0;i<320;++i){t+=10;w.update({0,0,1},t);}require(w.state()==State::Armed,"wind drops: arms");
  char line[200];LogEntry e{1000,Log::Settling,7,uint8_t(Unsteady::Drift),0,.05f};formatLog(e,line,sizeof(line));
  require(std::strstr(line,"still SETTLING")&&std::strstr(line,"7 windows")&&std::strstr(line,"drifted 0.050 g"),"journal text for settling");
  e=LogEntry{1000,Log::Armed,1,0,0,.02f,.08f};formatLog(e,line,sizeof(line));require(std::strstr(line,"threshold 0.080 g raised by wind"),"journal text for armed in wind");}},
{"sensitivity-presets",[]{
  require(sensitivityCount==5&&defaultSensitivity==3,"five presets, High by default");
  for(uint8_t i=0;i<sensitivityCount;++i){Config c=configForSensitivity(i);require(validConfig(c)&&!c.legacyMode,"every preset is a valid Dynamic configuration");
    if(i)require(c.thresholdG<sensitivityPresets[i-1].thresholdG&&c.noiseMultiplier<=sensitivityPresets[i-1].noiseMultiplier,"higher sensitivity never raises threshold or multiplier");
    Config d;require(c.repeatMs==d.repeatMs&&c.continuousMs==d.continuousMs&&c.suppression&&c.upDeg==d.upDeg&&c.tiltDeg==d.tiltDeg&&c.suppressHoldMs==d.suppressHoldMs,"removed settings use tested defaults");}
  require(nearestSensitivity(.02f)==3&&nearestSensitivity(.15f)==0&&nearestSensitivity(.002f)==4&&nearestSensitivity(std::numeric_limits<float>::quiet_NaN())==3,"old saved thresholds map to the closest preset");
  require(clampSensitivity(-1)==3&&clampSensitivity(9)==3&&clampSensitivity(0)==0,"out-of-range index falls back to the default");
  SensorCore c;require(c.configure(configForSensitivity(0)),"applies");uint32_t t=arm(c);(void)t;}},
{"jolt-limit-follows-sway",[]{
  // Field log 2026-11-24: wind with SD 0.03..0.05 g reached 0.20..0.21 g peak-to-peak and kept failing a fixed 0.20 g limit.
  require(SteadyWindow::rangeLimit(0)==.20f&&SteadyWindow::rangeLimit(.04f)>.239f&&SteadyWindow::rangeLimit(.2f)==.80f,"limit = max(0.20, 6 x SD), capped at 0.80");
  SensorCore c;Config cfg;c.configure(cfg);uint32_t t=calibrate(c);c.setArmed(true,t);
  for(int i=0;i<400&&c.state()!=State::Armed;++i){t+=10;float ph=float(t%700)/700.f*6.2831853f;c.update({.05f*std::sin(ph)+((t%370)<10?.12f:0.f),0,1},t);}
  require(c.state()==State::Armed&&c.telemetry().settleResets==0,"sway plus gusts, about 0.22 g peak-to-peak, arms in one window");
  SensorCore q;q.configure(cfg);t=calibrate(q);q.setArmed(true,t);for(int i=0;i<150;++i){t+=10;q.update({i==70?.25f:0.f,0,1},t);}
  require(q.state()==State::Settling&&q.telemetry().settleCause==uint8_t(Unsteady::Range)&&q.telemetry().settleValue>.249f,"one 0.25 g jolt in a quiet window is still handling");
  SensorCore s2;s2.configure(cfg);t=arm(s2);for(int i=0;i<60;++i){t+=10;s2.update({0,.6427876f,.7660444f},t);}
  require(s2.state()==State::MotionSuppressed&&s2.telemetry().suppressRollDeg>35.f&&s2.telemetry().suppressPitchDeg<1.f,"the suppression reason is recorded");
  char line[200];LogEntry e{0,Log::Suppressed,0,0,0,27.3f,4.1f};formatLog(e,line,sizeof(line));require(std::strstr(line,"tilted 27.3 deg up/down, 4.1 deg roll"),"journal text for suppression");}},
{"field-2026-11-24-wind-and-bites",[]{
  // docs/field-analysis-2026-11-24.md: strong wind (~5 Hz, +-0.3 g) must arm without false alarms,
  // while the simulated light bites in calm still alarm.
  for(uint8_t p=0;p<sensitivityCount;++p){auto w=replayField("wind.csv",p);require(w.armedRows>9000&&w.alarms==0,"wind: armed for most of 2 min with no alarm at every preset");}
  auto calm=replayField("calm_bites.csv",defaultSensitivity);require(calm.armedRows>4500&&calm.alarms>=30,"calm: the simulated light bites alarm at High");}},
{"frequent-bites-keep-preset-threshold",[]{
  // 2026-11-24 log: a run of bites raised the ambient ~5x (Very high 0.009 -> 0.035 g) and the
  // next bites were missed. Bite samples no longer feed the ambient; the field calm run must
  // stay at the preset floor while armed, at Very high and High.
  for(uint8_t p:{uint8_t(4),defaultSensitivity}){
    std::string path=std::string(__FILE__);path=path.substr(0,path.find_last_of('/'))+"/fixtures/field/calm_bites.csv";
    std::ifstream in(path);require(bool(in),"field fixture present");
    SensorCore c;Config cfg=configForSensitivity(p);require(c.configure(cfg),"preset applies");
    std::string line;bool seen=false,started=false,armed=false;uint32_t origin=0,first=0;float worst=0;
    while(std::getline(in,line)){
      if(line.empty()||line[0]<'0'||line[0]>'9')continue;
      std::stringstream ss(line);std::string f[4];for(auto& x:f)std::getline(ss,x,',');
      uint32_t now=uint32_t(std::stoul(f[0]));Vec3 a{std::stof(f[1]),std::stof(f[2]),std::stof(f[3])};
      if(!seen){seen=true;origin=now;}if(elapsed(now,origin)<4000)continue;
      if(!started){c.update(a,now);c.beginCalibration(now);first=now;started=true;}
      if(!armed&&c.calibrated()&&elapsed(now,first)>=3000){c.setArmed(true,now);armed=true;}
      c.update(a,now);if(c.state()==State::Armed)worst=std::max(worst,c.telemetry().effectiveG);
    }
    require(armed&&worst<=cfg.thresholdG*1.25f,"frequent bites do not raise the threshold above the preset floor");
  }
  // Synthetic: quiet, then 20 sharp taps 0.4 s apart; a light tap right after must still alarm.
  SensorCore c;Config cfg=configForSensitivity(4);c.configure(cfg);uint32_t t=calibrate(c);c.setArmed(true,t);
  for(int i=0;i<300;++i){t+=9;c.update({0,0,1},t);}require(c.state()==State::Armed,"armed");
  auto tap=[&](float g){uint8_t ev=0;for(int k=0;k<44;++k){t+=9;float r=k<20?g*std::exp(-k/6.f)*((k%3)-1):0;ev|=c.update({r,0,1},t);}return ev;};
  for(int i=0;i<20;++i)tap(.05f);
  require(c.telemetry().effectiveG<=cfg.thresholdG*1.25f,"after 8 s of taps the threshold stays at the floor");
  require(tap(.02f)!=0,"a light tap after the run still alarms");}},
{"onset-of-continuous-vibration-still-adapts",[]{
  // The bite exclusion must not freeze the ambient: steady sharp vibration (not intermittent like
  // bites) starting after arming stops alarming within ~10 s, as before the change.
  SensorCore c;Config cfg=configForSensitivity(4);c.configure(cfg);uint32_t t=calibrate(c);c.setArmed(true,t);
  for(int i=0;i<300;++i){t+=9;c.update({0,0,1},t);}require(c.state()==State::Armed,"armed");
  uint32_t start=t,lastAlarm=t;
  for(int i=0;i<2500;++i){t+=9;float v=.03f*std::sin(float(i)*2.f*3.14159265f*20.f*.009f);if(c.update({v,0,1},t))lastAlarm=t;}
  require(c.state()==State::Armed&&elapsed(lastAlarm,start)<10000,"continuous vibration raises the ambient and alarms stop");}},
{"timestamp-rollover",[]{SensorCore c;uint32_t t=UINT32_MAX-1000;c.update({0,0,1},t);c.beginCalibration(t);for(int i=0;i<201;++i){t+=10;c.update({0,0,1},t);}require(c.calibrated(),"calibration spans rollover");}}
};
int failures=0;for(auto& test:tests){try{test.second();std::printf("PASS: %s\n",test.first);}catch(const std::exception& e){++failures;std::fprintf(stderr,"FAIL: %s: %s\n",test.first,e.what());}}
std::printf("%zu tests, %d failed\n",tests.size(),failures);return failures?1:0;
}
