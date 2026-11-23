#pragma once
#include "SensorCore.h"
namespace bite {
enum class ReadStatus:uint8_t {Fresh,NoNewData,TransportError};
// Transport::read must check actual I2C status and byte count, not an address probe.
template<class Transport> ReadStatus readAccel(Transport& io,Vec3& output){
    uint8_t status=0,raw[6];
    if(!io.read(0x1e,&status,1))return ReadStatus::TransportError;
    if(!(status&1))return ReadStatus::NoNewData;
    if(!io.read(0x28,raw,6))return ReadStatus::TransportError;
    auto axis=[&](int i){return float(int16_t(uint16_t(raw[i])|(uint16_t(raw[i+1])<<8)))*.000061f;};
    Vec3 candidate{axis(0),axis(2),axis(4)};
    if(!valid(candidate))return ReadStatus::TransportError;
    output=candidate;return ReadStatus::Fresh;
}
}
