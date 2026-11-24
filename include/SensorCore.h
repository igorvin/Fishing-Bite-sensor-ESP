#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace bite {
inline uint32_t elapsed(uint32_t now,uint32_t since){return now-since;}
struct Vec3 { float x=0,y=0,z=1; };
inline Vec3 operator-(Vec3 a,Vec3 b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
inline float magnitude(Vec3 a){return std::sqrt(a.x*a.x+a.y*a.y+a.z*a.z);}
inline bool valid(Vec3 a){return std::isfinite(a.x)&&std::isfinite(a.y)&&std::isfinite(a.z)&&magnitude(a)<3.5f;}
inline float angleDifference(float a,float b){return std::fabs(std::remainder(a-b,360.f));}
inline float pitch(Vec3 a){return std::atan2(-a.x,std::sqrt(a.y*a.y+a.z*a.z))*57.2957795f;}
inline float roll(Vec3 a){return std::atan2(a.y,a.z)*57.2957795f;}
struct Signal {float dynamic=0,legacy=0;};
class Features {
    Vec3 gravity_{}; float base_=1; uint32_t last_=0; bool started_=false;
public:
    void reset(){started_=false;}
    // Start the gravity estimate from a measured mean instead of one (possibly swaying) sample.
    void seed(Vec3 g,uint32_t now){gravity_=g;base_=magnitude(g);last_=now;started_=true;}
    Signal update(Vec3 a,uint32_t now,bool freeze=false){
        if(!started_){gravity_=a;base_=magnitude(a);last_=now;started_=true;return {};}
        float dt=std::min<uint32_t>(elapsed(now,last_),100u);last_=now;
        Signal s{magnitude(a-gravity_),std::fabs(magnitude(a)-base_)};
        if(!freeze){float k=dt/(500.f+dt);gravity_.x+=k*(a.x-gravity_.x);gravity_.y+=k*(a.y-gravity_.y);gravity_.z+=k*(a.z-gravity_.z);base_+=(dt/(10000.f+dt))*(magnitude(a)-base_);}
        return s;
    }
};
struct Config {
    float thresholdG=.02f, legacyThresholdG=.15f, noiseMultiplier=4;
    float upDeg=25, tiltDeg=35;
    uint32_t attackMs=20,releaseMs=40,continuousMs=250,repeatMs=300;
    uint32_t settleMs=1500,quietMs=1500,suppressHoldMs=120,maxGapMs=150;
    bool legacyMode=false,suppression=true;
};
inline bool validConfig(const Config& c){
    return std::isfinite(c.thresholdG)&&c.thresholdG>=.002f&&c.thresholdG<=1.f&&
        std::isfinite(c.legacyThresholdG)&&c.legacyThresholdG>=.02f&&c.legacyThresholdG<=1.f&&
        std::isfinite(c.noiseMultiplier)&&c.noiseMultiplier>=1&&c.noiseMultiplier<=10&&
        std::isfinite(c.upDeg)&&c.upDeg>=0&&c.upDeg<=90&&std::isfinite(c.tiltDeg)&&c.tiltDeg>=0&&c.tiltDeg<=90&&
        c.attackMs>=10&&c.attackMs<=1000&&c.releaseMs>=10&&c.releaseMs<=1000&&
        c.continuousMs>=50&&c.continuousMs<=2000&&c.repeatMs>=100&&c.repeatMs<=2000&&
        c.settleMs>=200&&c.settleMs<=10000&&c.quietMs>=200&&c.quietMs<=10000&&
        c.suppressHoldMs>=50&&c.suppressHoldMs<=2000&&c.maxGapMs>=50&&c.maxGapMs<=1000;
}
enum class State:uint8_t {Disarmed,Settling,Armed,MotionSuppressed,Fault,Calibrating};
inline const char* stateName(State s){switch(s){case State::Disarmed:return "DISARMED";case State::Settling:return "SETTLING";case State::Armed:return "ARMED";case State::MotionSuppressed:return "SUPPRESSED";case State::Fault:return "FAULT";case State::Calibrating:return "CALIBRATING";}return "FAULT";}
enum class Calibration:uint8_t {None,Running,Success,Moving,SensorError};
inline const char* calibrationName(Calibration c){switch(c){case Calibration::None:return "not calibrated";case Calibration::Running:return "keep still";case Calibration::Success:return "success";case Calibration::Moving:return "rejected: movement";case Calibration::SensorError:return "rejected: sensor error";}return "error";}
// Why the last calibration was rejected as movement (diagnostics only; limits unchanged).
// Why a window was not steady. Calibration, SETTLING and the return from SUPPRESSED share
// the same rules, so a rod swaying in wind around one position counts as steady.
enum class Unsteady:uint8_t {None,Magnitude,Range,Drift,Noise};
// Kept for the existing journal/telemetry names: Deviation = jolt range, plus Drift.
enum class CalReject:uint8_t {None,Magnitude,Deviation,Noise,Drift};
inline CalReject calRejectFrom(Unsteady u){switch(u){case Unsteady::Magnitude:return CalReject::Magnitude;case Unsteady::Range:return CalReject::Deviation;case Unsteady::Drift:return CalReject::Drift;case Unsteady::Noise:return CalReject::Noise;default:return CalReject::None;}}
// Steady = swaying is allowed, being handled or moved is not. Limits (2026-11-23, user: arm in wind):
//  every |a| within 0.85..1.15 g; peak-to-peak per axis <= max(0.20 g, 6 x SD), never above
//  0.30 g (2026-11-24 field log: wind with SD 0.03..0.05 g has 0.20..0.21 g peak-to-peak, so a
//  fixed 0.20 g kept rejecting it; a single jolt in a quiet window is still caught);
//  mean of the first and second half within 0.035 g (~2 deg, the rod is not being moved);
//  standard deviation <= 0.05 g (beyond that the threshold would exceed ~0.2 g).
// The window's standard deviation becomes the noise that raises the effective threshold.
class SteadyWindow {
    uint32_t start_=0,n_=0,nA_=0,nB_=0;Vec3 mean_{0,0,0},m2_{0,0,0},sumA_{0,0,0},sumB_{0,0,0},lo_{0,0,0},hi_{0,0,0};
    static Vec3 scaled(Vec3 v,float k){return {v.x*k,v.y*k,v.z*k};}
public:
    static constexpr float rangeMaxG=.20f,rangeSdFactor=6.f,rangeHardMaxG=.30f,driftMaxG=.035f,noiseMaxG=.05f;
    static float rangeLimit(float sd){return std::min(rangeHardMaxG,std::max(rangeMaxG,rangeSdFactor*sd));}
    static float limit(Unsteady u){return u==Unsteady::Range?rangeHardMaxG:u==Unsteady::Drift?driftMaxG:u==Unsteady::Noise?noiseMaxG:0.f;}
    void reset(uint32_t now){start_=now;n_=nA_=nB_=0;mean_=m2_=sumA_=sumB_=lo_=hi_=Vec3{0,0,0};}
    // Immediate failures (magnitude, jolt range); value receives the offending figure.
    Unsteady add(Vec3 a,uint32_t now,uint32_t windowMs,float& value){
        float m=magnitude(a);if(m<.85f||m>1.15f){value=m;return Unsteady::Magnitude;}
        if(!n_){lo_=hi_=a;}else{lo_={std::min(lo_.x,a.x),std::min(lo_.y,a.y),std::min(lo_.z,a.z)};hi_={std::max(hi_.x,a.x),std::max(hi_.y,a.y),std::max(hi_.z,a.z)};}
        ++n_;Vec3 d=a-mean_;float n=float(n_);mean_.x+=d.x/n;mean_.y+=d.y/n;mean_.z+=d.z/n;
        m2_.x+=d.x*(a.x-mean_.x);m2_.y+=d.y*(a.y-mean_.y);m2_.z+=d.z*(a.z-mean_.z);
        if(elapsed(now,start_)<windowMs/2){sumA_.x+=a.x;sumA_.y+=a.y;sumA_.z+=a.z;++nA_;}else{sumB_.x+=a.x;sumB_.y+=a.y;sumB_.z+=a.z;++nB_;}
        float range=this->range();
        if(range>rangeHardMaxG){value=range;return Unsteady::Range;}
        return Unsteady::None;
    }
    // Enough samples for a verdict: the window has elapsed with at least 50 Hz effective rate.
    bool full(uint32_t now,uint32_t windowMs)const{return elapsed(now,start_)>=windowMs&&n_>=std::max<uint32_t>(2,windowMs/20)&&nA_&&nB_;}
    Unsteady verdict(float& value)const{
        float drift=magnitude(scaled(sumA_,1.f/float(nA_))-scaled(sumB_,1.f/float(nB_)));
        if(!std::isfinite(drift)||drift>driftMaxG){value=drift;return Unsteady::Drift;}
        float sd=noise();if(!std::isfinite(sd)||sd>noiseMaxG){value=sd;return Unsteady::Noise;}
        float r=range();if(r>rangeLimit(sd)){value=r;return Unsteady::Range;}
        value=sd;return Unsteady::None;
    }
    float range()const{return n_?std::max(hi_.x-lo_.x,std::max(hi_.y-lo_.y,hi_.z-lo_.z)):0.f;}
    float noise()const{return n_>1?std::sqrt((m2_.x+m2_.y+m2_.z)/float(n_-1)):0.f;}
    Vec3 mean()const{return mean_;}
};
inline const char* calRejectName(CalReject r){switch(r){case CalReject::None:return "none";case CalReject::Magnitude:return "magnitude";case CalReject::Deviation:return "jolt";case CalReject::Noise:return "sway";case CalReject::Drift:return "drift";}return "none";}
struct Telemetry {
    Vec3 accel{};Signal signal{};float noiseG=0,effectiveG=.02f;
    uint32_t samples=0,errors=0,gaps=0,events=0,suppressedMs=0,lastSampleMs=0;
    uint8_t event=0,calibrationProgress=0;
    CalReject calReject=CalReject::None;float calRejectValue=0;
    // SETTLING diagnostics: windows that were not steady since SETTLING began and the last
    // cause (Unsteady) with its value and limit.
    uint32_t settleResets=0;uint8_t settleCause=0;float settleValue=0,settleLimit=0;
    // Why the last SUPPRESSED started: pitch and roll change from the armed position, degrees.
    float suppressPitchDeg=0,suppressRollDeg=0;
};
class Pulse {
    bool on_=false;uint32_t start_=0,duration_=0;
public:
    void start(uint32_t now,uint32_t duration){start_=now;duration_=duration;on_=duration>0;}
    void stop(){on_=false;}
    bool tick(uint32_t now){if(on_&&elapsed(now,start_)>=duration_)on_=false;return on_;}
    bool on()const{return on_;}
};
class StateClock {
    uint32_t entered_=0;
public:
    void enter(uint32_t now){entered_=now;}
    bool expired(uint32_t now,uint32_t duration)const{return elapsed(now,entered_)>=duration;}
};
class SensorCore {
    Config cfg_{};Features features_;State state_=State::Fault;Calibration cal_=Calibration::None;Telemetry t_{};
    bool healthy_=false,wanted_=false,calibrated_=false,started_=false;
    bool attacking_=false,releasing_=false,motion_=false,over_=false,quiet_=false;
    uint32_t stateAt_=0,attackAt_=0,releaseAt_=0,motionAt_=0,lastEventAt_=0,overAt_=0,quietAt_=0;
    Vec3 orientation_{},base_{};SteadyWindow window_;
    uint32_t calAt_=0,lastSuppressionTick_=0;
    void clearEvent(){motion_=attacking_=releasing_=false;t_.event=0;}
    void transition(State s,uint32_t now){state_=s;stateAt_=now;quiet_=over_=false;clearEvent();window_.reset(now);if(s==State::Settling){t_.settleResets=0;t_.settleCause=0;t_.settleValue=t_.settleLimit=0;}}
    void rejectCalibration(Calibration outcome,uint32_t now){cal_=outcome;wanted_=false;transition(State::Disarmed,now);}
public:
    const Config& config()const{return cfg_;}
    bool configure(const Config& c){if(!validConfig(c))return false;cfg_=c;clearEvent();return true;}
    const Telemetry& telemetry()const{return t_;}
    State state()const{return state_;} Calibration calibration()const{return cal_;}
    bool healthy()const{return healthy_;} bool desiredArmed()const{return wanted_;}
    bool calibrated()const{return calibrated_;}
    bool restoreNoise(float noise){if(!std::isfinite(noise)||noise<0||noise>.015f)return false;t_.noiseG=noise;return true;}
    uint32_t stateSince()const{return stateAt_;}
    bool setArmed(bool arm,uint32_t now){
        if(!arm){wanted_=false;if(state_==State::Calibrating)cal_=Calibration::None;transition(healthy_?State::Disarmed:State::Fault,now);return true;}
        if(!healthy_)return false;
        wanted_=true;if(state_==State::Calibrating)return true;
        if(!calibrated_)return beginCalibration(now);
        transition(State::Settling,now);features_.reset();return true;
    }
    bool beginCalibration(uint32_t now){
        if(!healthy_)return false;
        cal_=Calibration::Running;calAt_=now;
        t_.calReject=CalReject::None;t_.calRejectValue=0;
        t_.calibrationProgress=0;transition(State::Calibrating,now);
        // The sample the calibration starts from belongs to its window (as calFirst did before).
        if(started_){float ignored=0;window_.add(t_.accel,now,2000,ignored);}
        return true;
    }
    void fault(uint32_t now){
        ++t_.errors;healthy_=false;wanted_=false;calibrated_=false;
        if(state_==State::Calibrating)cal_=Calibration::SensorError;
        transition(State::Fault,now);features_.reset();
    }
    void tick(uint32_t now){
        t_.event=0;
        if(healthy_&&started_&&elapsed(now,t_.lastSampleMs)>cfg_.maxGapMs){++t_.gaps;fault(now);}
    }
    uint8_t update(Vec3 a,uint32_t now){
        t_.event=0;if(!valid(a)){fault(now);return 0;}
        uint32_t dt=started_?elapsed(now,t_.lastSampleMs):10;
        if(started_&&dt==0)return 0;
        if(started_&&healthy_&&dt>25){++t_.gaps;if(dt>cfg_.maxGapMs)fault(now);}
        bool recovering=!healthy_;healthy_=true;
        if(recovering){transition(State::Disarmed,now);features_.reset();orientation_=a;}
        started_=true;t_.lastSampleMs=now;t_.accel=a;++t_.samples;
        t_.signal=features_.update(a,now,state_==State::Armed&&motion_);
        float okMagnitude=magnitude(a);
        float k=std::min<uint32_t>(dt,100u)/(120.f+std::min<uint32_t>(dt,100u));
        orientation_.x+=k*(a.x-orientation_.x);orientation_.y+=k*(a.y-orientation_.y);orientation_.z+=k*(a.z-orientation_.z);
        t_.effectiveG=cfg_.legacyMode?cfg_.legacyThresholdG:std::max(cfg_.thresholdG,t_.noiseG*cfg_.noiseMultiplier);
        if(state_==State::Calibrating){
            t_.calibrationProgress=uint8_t(std::min<uint32_t>(100u,elapsed(now,calAt_)*100/2000));
            float value=0;Unsteady u=window_.add(a,now,2000,value);
            if(u==Unsteady::None&&window_.full(now,2000)){
                u=window_.verdict(value);
                if(u==Unsteady::None){
                    // Steady sway is accepted; its noise raises the threshold (see SteadyWindow).
                    t_.noiseG=window_.noise();cal_=Calibration::Success;calibrated_=true;t_.calibrationProgress=100;
                    features_.reset();base_=window_.mean();transition(wanted_?State::Settling:State::Disarmed,now);return 0;
                }
            }
            if(u!=Unsteady::None){t_.calReject=calRejectFrom(u);t_.calRejectValue=value;rejectCalibration(Calibration::Moving,now);}
            return 0;
        }
        if(state_==State::Disarmed||state_==State::Fault)return 0;
        if(state_==State::MotionSuppressed){
            t_.suppressedMs+=elapsed(now,lastSuppressionTick_);lastSuppressionTick_=now;
            float value=0;Unsteady u=window_.add(a,now,cfg_.quietMs,value);
            if(u==Unsteady::None&&window_.full(now,cfg_.quietMs))u=window_.verdict(value);
            else if(u==Unsteady::None)return 0;
            if(u==Unsteady::None){features_.reset();transition(State::Settling,now);}else window_.reset(now);
            return 0;
        }
        if(state_==State::Settling){
            float value=0;Unsteady u=window_.add(a,now,cfg_.settleMs,value);
            if(u==Unsteady::None){
                if(!window_.full(now,cfg_.settleMs))return 0;
                u=window_.verdict(value);
                if(u==Unsteady::None){
                    // Arm at the measured ambient level: in wind the effective threshold rises above the sway.
                    t_.noiseG=window_.noise();base_=window_.mean();features_.seed(window_.mean(),now);transition(State::Armed,now);return 0;
                }
            }
            ++t_.settleResets;t_.settleCause=uint8_t(u);t_.settleValue=value;t_.settleLimit=SteadyWindow::limit(u);
            stateAt_=now;window_.reset(now);
            return 0;
        }
        if(cfg_.suppression){
            bool moved=(cfg_.upDeg>0&&angleDifference(pitch(orientation_),pitch(base_))>=cfg_.upDeg)||
                (cfg_.tiltDeg>0&&angleDifference(roll(orientation_),roll(base_))>=cfg_.tiltDeg);
            if(moved&&okMagnitude>.75f&&okMagnitude<1.25f){
                if(!over_){over_=true;overAt_=now;}
                if(elapsed(now,overAt_)>=cfg_.suppressHoldMs){
                    t_.suppressPitchDeg=angleDifference(pitch(orientation_),pitch(base_));t_.suppressRollDeg=angleDifference(roll(orientation_),roll(base_));
                    transition(State::MotionSuppressed,now);lastSuppressionTick_=now;return 0;
                }
            }else over_=false;
        }
        float signal=cfg_.legacyMode?t_.signal.legacy:t_.signal.dynamic;
        if(!motion_){
            if(signal>=t_.effectiveG){if(!attacking_){attacking_=true;attackAt_=now;}
                if(elapsed(now,attackAt_)>=cfg_.attackMs){motion_=true;motionAt_=lastEventAt_=now;attacking_=false;t_.event=1;++t_.events;}}
            else attacking_=false;
        }else{
            if(signal<t_.effectiveG*.65f){if(!releasing_){releasing_=true;releaseAt_=now;}
                if(elapsed(now,releaseAt_)>=cfg_.releaseMs){clearEvent();return 0;}}
            else releasing_=false;
            if(!releasing_&&elapsed(now,motionAt_)>=cfg_.continuousMs&&elapsed(now,lastEventAt_)>=cfg_.repeatMs){lastEventAt_=now;t_.event=2;}
        }
        return t_.event;
    }
};
inline int hex(char c){return c>='0'&&c<='9'?c-'0':c>='A'&&c<='F'?c-'A'+10:c>='a'&&c<='f'?c-'a'+10:-1;}
inline bool parseMac(const char* s,uint8_t out[6]){
    if(!s||std::strlen(s)!=17)return false;
    uint8_t parsed[6];bool nonzero=false;
    for(int i=0;i<6;++i){int a=hex(s[i*3]),b=hex(s[i*3+1]);if(a<0||b<0||(i<5&&s[i*3+2]!=':'))return false;parsed[i]=uint8_t(a*16+b);nonzero|=parsed[i]!=0;}
    if(!nonzero||(parsed[0]&1))return false;
    std::memcpy(out,parsed,6);return true;
}
inline bool validNetwork(const char* name,const char* password){
    if(!name||!password)return false;
    auto n=std::strlen(name),p=std::strlen(password);
    return n>=1&&n<=32&&(p==0||(p>=8&&p<=63));
}
inline bool wakePinAllowed(uint8_t family,int pin){return pin>=0&&pin<=(family==3?5:family==6?7:21);}
}
