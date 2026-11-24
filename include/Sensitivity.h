#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include "SensorCore.h"
namespace bite {
// Five sensitivity presets replace the threshold (g) and noise-multiplier sliders
// (user decision 2026-11-24). A preset sets both, so it also matters in wind:
// effective threshold = max(thresholdG, ambient sway SD x noiseMultiplier).
struct SensitivityPreset {float thresholdG,noiseMultiplier;const char* en;const char* ru;};
constexpr SensitivityPreset sensitivityPresets[]={
    {.050f,6.f,"Low","Низкая"},
    {.035f,5.f,"Below medium","Ниже средней"},
    {.025f,4.f,"Medium","Средняя"},
    {.018f,4.f,"High","Высокая"},
    {.012f,3.f,"Very high","Очень высокая"},
};
constexpr uint8_t sensitivityCount=sizeof(sensitivityPresets)/sizeof(sensitivityPresets[0]);
constexpr uint8_t defaultSensitivity=3;  // High: closest to the former 0.020 g default
// Existing configurations only stored a threshold: pick the closest preset.
inline uint8_t nearestSensitivity(float thresholdG){
    if(!std::isfinite(thresholdG))return defaultSensitivity;
    uint8_t best=defaultSensitivity;float gap=1e9f;
    for(uint8_t i=0;i<sensitivityCount;++i){float d=std::fabs(sensitivityPresets[i].thresholdG-thresholdG);if(d<gap){gap=d;best=i;}}
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
