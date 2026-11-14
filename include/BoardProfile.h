#pragma once
#include <Arduino.h>
#include "SensorCore.h"
namespace board {
constexpr int green=D7,red=D8,buzzer=D9,button=D6,power=D0,powerButton=D1,battery=D2,sda=D4,scl=D5;
#if CONFIG_IDF_TARGET_ESP32C3
constexpr uint8_t family=3;
#elif CONFIG_IDF_TARGET_ESP32C6
constexpr uint8_t family=6;
#else
constexpr uint8_t family=33;
#endif
inline bool wakeAllowed(){return bite::wakePinAllowed(family,button);}
}
