#pragma once
#include "ImuRead.h"
namespace bite {
// Lifecycle shared by the firmware and fake transport regressions.
class ImuSession {
    // 150 ms matches the existing sample-gap budget; retry remains sparse.
    static constexpr uint32_t firstFrameMs=150,retryMs=2000;
    bool online_=false,initialCal_=true,waitingFirst_=true;
    uint32_t lastRetry_=0,lastRead_=0,beganAt_=0;
public:
    bool online()const{return online_;}
    template<class Driver> void begin(Driver& io,SensorCore& core,uint32_t now){
        online_=io.begin();lastRetry_=beganAt_=now;initialCal_=online_;waitingFirst_=true;
        if(!online_)core.fault(now);
    }
    template<class Driver> void supervise(Driver& io,SensorCore& core,uint32_t now){
        core.tick(now);
        if(online_&&((waitingFirst_&&elapsed(now,beganAt_)>=firstFrameMs)||(!waitingFirst_&&core.state()==State::Fault))){
            online_=false;lastRetry_=now;
            if(waitingFirst_)core.fault(now);
        }
        if(!online_&&elapsed(now,lastRetry_)>=retryMs)begin(io,core,now);
    }
    bool due(uint32_t now)const{return online_&&elapsed(now,lastRead_)>=9;}
    template<class Driver> ReadStatus read(Driver& io,SensorCore& core,uint32_t now,Vec3& sample,uint8_t& event){
        lastRead_=now;event=0;auto result=io.sample(sample);
        if(result==ReadStatus::Fresh){waitingFirst_=false;event=core.update(sample,now);if(initialCal_){initialCal_=false;core.beginCalibration(now);event=0;}}
        else if(result==ReadStatus::TransportError){online_=false;lastRetry_=now;core.fault(now);}
        return result;
    }
};
}
