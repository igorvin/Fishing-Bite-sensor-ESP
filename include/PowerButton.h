#pragma once
#include <cstdint>
#include "SensorCore.h"
namespace bite {
// One active-LOW button handles ARM, deep sleep and wake. Portable: host tests use it.
//   hold >= 1 s, release before 3 s  -> toggle ARM (short cue beep at 1 s)
//   hold >= 3 s, then release        -> deep sleep (long cue beep at 3 s)
// Sleep happens only after release, otherwise the still-pressed pin would wake at once.
enum class ButtonAction:uint8_t{None,ArmCue,SleepCue,ToggleArm,Sleep};
class PowerButton {
public:
    static constexpr uint32_t debounceMs=50,armMs=1000,sleepMs=3000;
    // pressed at boot (the wake press) is ignored until it is released
    void begin(bool pressed,uint32_t now){raw_=stable_=locked_=pressed;armCue_=sleepCue_=false;changed_=pressedAt_=now;}
    ButtonAction update(bool pressed,uint32_t now){
        if(pressed!=raw_){raw_=pressed;changed_=now;}
        if(raw_!=stable_&&elapsed(now,changed_)>=debounceMs){
            stable_=raw_;
            if(stable_){pressedAt_=now;armCue_=sleepCue_=false;return ButtonAction::None;}
            bool wasLocked=locked_;locked_=false;
            if(wasLocked)return ButtonAction::None;
            if(sleepCue_)return ButtonAction::Sleep;
            if(armCue_)return ButtonAction::ToggleArm;
            return ButtonAction::None;
        }
        if(stable_&&!locked_){
            uint32_t held=elapsed(now,pressedAt_);
            if(!armCue_&&held>=armMs){armCue_=true;return ButtonAction::ArmCue;}
            if(armCue_&&!sleepCue_&&held>=sleepMs){sleepCue_=true;return ButtonAction::SleepCue;}
        }
        return ButtonAction::None;
    }
    bool released()const{return !stable_&&!raw_;}
private:
    bool raw_=false,stable_=false,locked_=false,armCue_=false,sleepCue_=false;
    uint32_t changed_=0,pressedAt_=0;
};

// After a button wake the press must last holdMs, so a brush in a bag goes straight back to sleep.
enum class WakeResult:uint8_t{Waiting,Confirmed,Rejected};
class WakeConfirm {
public:
    static constexpr uint32_t holdMs=1000,releaseMs=50;
    void begin(uint32_t now){start_=releasedAt_=now;released_=false;}
    WakeResult update(bool pressed,uint32_t now){
        if(pressed){released_=false;return elapsed(now,start_)>=holdMs?WakeResult::Confirmed:WakeResult::Waiting;}
        if(!released_){released_=true;releasedAt_=now;}
        return elapsed(now,releasedAt_)>=releaseMs?WakeResult::Rejected:WakeResult::Waiting;
    }
private:
    bool released_=false;uint32_t start_=0,releasedAt_=0;
};

// Deep sleep after several consecutive valid readings below the cut-off (one sag is ignored).
class LowBatteryGuard {
public:
    static constexpr float cutV=3.30f;static constexpr uint8_t needed=3;
    bool update(bool known,float volts){
        if(known&&volts<cutV){if(count_<needed)++count_;}else count_=0;
        return count_>=needed;
    }
private:
    uint8_t count_=0;
};
}
