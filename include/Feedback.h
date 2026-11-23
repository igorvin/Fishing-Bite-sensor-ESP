#pragma once
#include <cstdint>
#include "SensorCore.h"
namespace bite {
// Visible "armed" heartbeat: a short red flash every 5 seconds (user choice; 1 s was too
// busy), phase-locked to the moment ARMED was entered (first flash immediately).
// Alarm pulses override it.
struct Heartbeat {
    static constexpr uint32_t periodMs=5000,onMs=50;
    static bool on(uint32_t now,uint32_t armedSince){return elapsed(now,armedSince)%periodMs<onMs;}
};

// Nonblocking repeated cue, e.g. three short beeps when an ARM request is cancelled.
class Pattern {
    uint32_t start_=0;uint16_t on_=0,off_=0;uint8_t count_=0;
public:
    void start(uint32_t now,uint8_t count,uint16_t onMs,uint16_t offMs){start_=now;count_=count;on_=onMs;off_=offMs;}
    void stop(){count_=0;}
    bool active()const{return count_!=0;}
    bool tick(uint32_t now){
        if(!count_)return false;
        uint32_t age=elapsed(now,start_),period=uint32_t(on_)+off_;
        if(!period||age>=period*count_){count_=0;return false;}
        return age%period<on_;
    }
};

// A finger on the board is movement for the 2 s quiet calibration. Holding RESET,
// the wake press, or the ARM press itself could therefore reject calibration, and a
// rejected calibration silently cancels the ARM request. ArmAssist keeps button
// actions away from calibration; it never arms by itself.
class ArmAssist {
public:
    static constexpr uint32_t settleMs=1000,retryMs=2000;
    static constexpr uint8_t maxRetries=3;
    // Button ARM while not calibrated: wait until the button has been released for settleMs.
    void defer(uint32_t now){pending_=true;at_=now;}
    void cancel(){pending_=false;}
    bool pending()const{return pending_;}
    bool due(uint32_t now,bool released){
        if(!pending_)return false;
        if(!released){at_=now;return false;}
        if(elapsed(now,at_)<settleMs)return false;
        pending_=false;return true;
    }
    // Boot/wake calibration rejected while never calibrated and disarmed: a bounded
    // number of quiet retries, each no sooner than retryMs after the last rejection.
    void rejected(uint32_t now){rejectedAt_=now;}
    void calibrated(){retries_=0;}
    bool retryDue(uint32_t now,bool eligible,bool released){
        if(!eligible||!released||retries_>=maxRetries||elapsed(now,rejectedAt_)<retryMs)return false;
        ++retries_;return true;
    }
    uint8_t retries()const{return retries_;}
private:
    bool pending_=false;uint32_t at_=0,rejectedAt_=0;uint8_t retries_=0;
};
}
