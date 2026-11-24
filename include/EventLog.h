#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include "SensorCore.h"
#include "PowerButton.h"
namespace bite {
// Bounded RAM journal of what the sensor decided and why. Printed live on USB and
// dumped by the "log" command, so a failed ARM can be explained after the fact.
enum class Log:uint8_t {Boot,Button,Arm,ArmDeferred,ArmCancelled,State,Calibration,CalRetry,Sleep,Settling,Armed,Suppressed};
enum class Source:uint8_t {Button,Web,Serial};
enum class Wake:uint8_t {Reset,Button,Other};
enum class SleepReason:uint8_t {Button,Idle,Battery,Serial};
constexpr uint8_t Refused=255;
struct LogEntry {uint32_t ms=0;Log type=Log::Boot;uint8_t a=0,b=0,c=0;float v=0,w=0;};

inline const char* sourceName(uint8_t s){return s==uint8_t(Source::Web)?"web":s==uint8_t(Source::Serial)?"usb":"button";}
inline const char* wakeLogName(uint8_t w){return w==uint8_t(Wake::Button)?"button":w==uint8_t(Wake::Other)?"other":"reset";}
inline const char* sleepLogName(uint8_t r){return r==uint8_t(SleepReason::Idle)?"idle":r==uint8_t(SleepReason::Battery)?"battery":r==uint8_t(SleepReason::Serial)?"usb":"button";}
inline const char* buttonLogName(uint8_t a){
    switch(ButtonAction(a)){case ButtonAction::ArmCue:return "held 1 s (arm cue)";case ButtonAction::SleepCue:return "held 3 s (sleep cue)";
    case ButtonAction::ToggleArm:return "released: toggle ARM";case ButtonAction::Sleep:return "released: sleep";default:return "?";}
}
// The limit text mirrors SteadyWindow's rules (calibration, SETTLING, return from SUPPRESSED).
inline int unsteadyText(Unsteady u,float value,char* out,size_t n){
    switch(u){
    case Unsteady::Magnitude:return std::snprintf(out,n,"|a| %.3f g, allowed 0.85..1.15",double(value));
    case Unsteady::Range:return std::snprintf(out,n,"jolt %.3f g peak-to-peak, limit 0.20..0.30 (6x sway) (handled?)",double(value));
    case Unsteady::Drift:return std::snprintf(out,n,"position drifted %.3f g, limit %.3f (being moved?)",double(value),double(SteadyWindow::driftMaxG));
    case Unsteady::Noise:return std::snprintf(out,n,"sway %.4f g, limit %.2f (too strong)",double(value),double(SteadyWindow::noiseMaxG));
    default:return std::snprintf(out,n,"no reason recorded");
    }
}
inline int rejectText(uint8_t reason,float value,char* out,size_t n){
    switch(CalReject(reason)){
    case CalReject::Magnitude:return unsteadyText(Unsteady::Magnitude,value,out,n);
    case CalReject::Deviation:return unsteadyText(Unsteady::Range,value,out,n);
    case CalReject::Drift:return unsteadyText(Unsteady::Drift,value,out,n);
    case CalReject::Noise:return unsteadyText(Unsteady::Noise,value,out,n);
    default:return std::snprintf(out,n,"no reason recorded");
    }
}
inline int formatLog(const LogEntry& e,char* out,size_t n){
    char why[96];int head=std::snprintf(out,n,"%lu.%03lus ",(unsigned long)(e.ms/1000),(unsigned long)(e.ms%1000));
    if(head<0||size_t(head)>=n){return head;}
    out+=head;n-=size_t(head);int body=0;
    switch(e.type){
    case Log::Boot:body=std::snprintf(out,n,"boot wake=%s",wakeLogName(e.a));break;
    case Log::Button:body=std::snprintf(out,n,"button %s",buttonLogName(e.a));break;
    case Log::Arm:
        if(e.c==Refused)body=std::snprintf(out,n,"%s from %s refused: IMU fault",e.b?"ARM":"DISARM",sourceName(e.a));
        else body=std::snprintf(out,n,"%s from %s -> %s",e.b?"ARM":"DISARM",sourceName(e.a),stateName(State(e.c)));
        break;
    case Log::ArmDeferred:body=std::snprintf(out,n,"ARM deferred: not calibrated, calibrating 1 s after button release");break;
    case Log::ArmCancelled:rejectText(e.a,e.v,why,sizeof(why));body=std::snprintf(out,n,"ARM cancelled: calibration rejected (%s)",why);break;
    case Log::State:body=std::snprintf(out,n,"state %s -> %s",stateName(State(e.a)),stateName(State(e.b)));break;
    case Log::Calibration:
        if(Calibration(e.a)==Calibration::Success)body=std::snprintf(out,n,"calibration success, noise %.4f g%s",double(e.v),e.v>.015f?" (sway accepted)":"");
        else if(Calibration(e.a)==Calibration::Moving){rejectText(e.b,e.v,why,sizeof(why));body=std::snprintf(out,n,"calibration rejected: %s",why);}
        else body=std::snprintf(out,n,"calibration %s",calibrationName(Calibration(e.a)));
        break;
    case Log::CalRetry:body=std::snprintf(out,n,"calibration retry %u/%u (was rejected, not armed)",unsigned(e.a),unsigned(e.b));break;
    case Log::Sleep:body=std::snprintf(out,n,"sleep reason=%s",sleepLogName(e.a));break;
    // a = unsteady windows (capped 255), b = Unsteady cause, v = last value
    case Log::Settling:unsteadyText(Unsteady(e.b),e.v,why,sizeof(why));
        body=std::snprintf(out,n,"still SETTLING: %u%s windows of 1.5 s not steady, last: %s",unsigned(e.a),e.a==255?"+":"",why);break;
    // v = ambient sway (window SD), w = effective threshold
    // v = pitch change, w = roll change in degrees
    case Log::Suppressed:body=std::snprintf(out,n,"SUPPRESSED: rod tilted %.1f deg up/down, %.1f deg roll (limits 25/35)",double(e.v),double(e.w));break;
    case Log::Armed:body=std::snprintf(out,n,"ARMED: ambient %.4f g -> threshold %.3f g%s",double(e.v),double(e.w),e.v>.005f?" (raised for sway)":"");break;
    }
    return body<0?body:head+body;
}
template<size_t N> class EventLog {
    std::array<LogEntry,N> e_{};size_t head_=0,count_=0;uint32_t total_=0;
public:
    const LogEntry& add(uint32_t ms,Log type,uint8_t a=0,uint8_t b=0,uint8_t c=0,float v=0,float w=0){
        LogEntry& slot=e_[(head_+count_)%N];
        if(count_==N)head_=(head_+1)%N;else ++count_;
        slot=LogEntry{ms,type,a,b,c,v,w};++total_;return slot;
    }
    size_t size()const{return count_;}
    uint32_t total()const{return total_;}
    // 0 = oldest retained entry.
    const LogEntry& at(size_t i)const{return e_[(head_+i)%N];}
    const LogEntry& newest(size_t back=0)const{return e_[(head_+count_-1-back)%N];}
};
}
