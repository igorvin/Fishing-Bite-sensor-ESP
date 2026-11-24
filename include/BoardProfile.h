#pragma once
#include <Arduino.h>
#include "SensorCore.h"
namespace board {
#if defined(BITE_BOARD_V4)
// PCB v4 (kicad/kicad_bite): XIAO ESP32-S3, R3 10k external pull-up on BTN, LEDs straight
// from GPIO, buzzer via AO3400 (100k gate pull-down). D2/D6/D7 deliberately unused.
constexpr int green=D9,red=D8,buzzer=D10,button=D1,battery=D0,sda=D4,scl=D5,imuInt1=D3;
constexpr const char* name="PCB v4";
#elif defined(BITE_BOARD_S3_ZERO)
// Waveshare ESP32-S3-Zero (env s3_zero), added 2026-11-24. Raw GPIO numbers: the board's
// Arduino variant names D0..D16 in a different order than XIAO, so D-names are avoided.
// Only edge GPIO1..13 are used; GPIO3 (strapping), GPIO0 (BOOT), GPIO21 (on-board WS2812),
// GPIO19/20 (USB) and GPIO43/44 (UART0) are left free. Button GPIO6 is RTC-capable (deep wake).
// The board has no battery charger: VBAT divider (100k/100k) from the cell to GPIO7.
constexpr int sda=4,scl=5,button=6,battery=7,green=8,red=9,buzzer=10,imuInt1=-1;
constexpr const char* name="S3-Zero";
#else
// Bench: the user moved the single button from D6 to D1 on 2026-11-22 (D6 cannot deep-wake).
// D0 power latch is not wired; INT1 is not wired.
constexpr int green=D7,red=D8,buzzer=D9,button=D1,battery=D2,sda=D4,scl=D5,imuInt1=-1;
constexpr const char* name="bench";
#endif
#if CONFIG_IDF_TARGET_ESP32C3
constexpr uint8_t family=3;
#elif CONFIG_IDF_TARGET_ESP32C6
constexpr uint8_t family=6;
#else
constexpr uint8_t family=33;
#endif
inline bool wakeAllowed(){return bite::wakePinAllowed(family,button);}
}
