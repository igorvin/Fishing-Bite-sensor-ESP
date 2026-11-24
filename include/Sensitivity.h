#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include "SensorCore.h"
namespace bite {
// Five sensitivity presets (user decision 2026-11-24). Since the impulse detector a preset sets
// the impulse floor (thresholdG) and the multiplier of the running ambient (noiseMultiplier):
// effective threshold = max(floor, ambient x multiplier). "High" = the values tuned on the
// 2026-11-24 field recordings (no false alarms in wind, 33/35 light bites in calm).
struct SensitivityPreset {float thresholdG,noiseMultiplier;const char* en;const char* ru;};
constexpr SensitivityPreset sensitivityPresets[]={
    {.030f,10.f,"Low","Низкая"},
    {.022f,8.f,"Below medium","Ниже средней"},
    {.016f,7.f,"Medium","Средняя"},
    {.012f,6.f,"High","Высокая"},
    {.009f,5.f,"Very high","Очень высокая"},
};
constexpr uint8_t sensitivityCount=sizeof(sensitivityPresets)/sizeof(sensitivityPresets[0]);
constexpr uint8_t defaultSensitivity=3;  // High
// Configurations from before the presets only stored a Dynamic threshold (old units): map it
// through the thresholds the presets had in those units, 0.050/0.035/0.025/0.018/0.012 g.
inline uint8_t nearestSensitivity(float oldDynamicThresholdG){
    constexpr float oldThresholds[]={.050f,.035f,.025f,.018f,.012f};
    if(!std::isfinite(oldDynamicThresholdG))return defaultSensitivity;
    uint8_t best=defaultSensitivity;float gap=1e9f;
    for(uint8_t i=0;i<sensitivityCount;++i){float d=std::fabs(oldThresholds[i]-oldDynamicThresholdG);if(d<gap){gap=d;best=i;}}
    return best;
}
inline uint8_t clampSensitivity(int index){return index<0||index>=sensitivityCount?defaultSensitivity:uint8_t(index);}
// Every setting that is no longer in the web UI is fixed to the tested defaults.
inline Config configForSensitivity(uint8_t index){
    Config c;const auto& p=sensitivityPresets[clampSensitivity(index)];
    c.thresholdG=p.thresholdG;c.noiseMultiplier=p.noiseMultiplier;c.legacyMode=false;
    return c;
}
}
