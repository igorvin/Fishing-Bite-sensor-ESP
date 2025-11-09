#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_BMI160.h>
#include <EncButton.h>
#include "esp_sleep.h"

// ==============================
//            PINS (EDIT ME)
// ==============================
#define LED_GREEN   5
#define LED_RED     6
#define BUZZER      8
#define BUTTON      4      // Button → GND (active-LOW, pull-up). Prefer RTC-capable for EXT0.

#define I2C_SDA_PIN -1
#define I2C_SCL_PIN -1

// ==============================
DFRobot_BMI160 bmi160;
EncButton eb(BUTTON);                 // ✅ EncButton v3: button-only

// ==============================
const int8_t   I2C_ADDR              = 0x69;
const float    G_PER_LSB             = 1.0f / 16384.0f;
const float    TRIGGER_DELTA_G       = 0.15f;
const uint8_t  TRIGGER_SAMPLES       = 3;
const uint16_t SAMPLE_INTERVAL_MS    = 10;

const uint16_t SHORT_PULSE_MS        = 120;
const uint16_t PULSE_PERIOD_MS       = 300;
const uint16_t CONTINUOUS_THRESH_MS  = 250;

const uint16_t LONG_PRESS_MS         = 1000;

// ==============================
bool imuOK = false;
bool armed = false;           // default DISARMED
bool inMotion = false;
bool pulseOn = false;

float    baselineMagG = 1.0f;
uint8_t  consecutiveTriggers = 0;
uint32_t lastSample = 0, motionStartMs = 0, pulseOffAt = 0, lastPulseStart = 0;

// ==============================
// helpers
inline float mag3(float x, float y, float z) { return sqrtf(x*x + y*y + z*z); }
inline void startPulse(uint32_t now) { pulseOn = true; lastPulseStart = now; pulseOffAt = now + SHORT_PULSE_MS; digitalWrite(LED_RED, HIGH); tone(BUZZER, 3000); }
inline void stopPulse() { pulseOn = false; noTone(BUZZER); digitalWrite(LED_RED, LOW); }
inline void updateGreenLED() { digitalWrite(LED_GREEN, armed ? HIGH : LOW); }
void beepShort() { tone(BUZZER, 3000); delay(150); noTone(BUZZER); delay(80); }
void beepLong()  { tone(BUZZER, 3000); delay(600); noTone(BUZZER); }

static void arm() {
  armed = true;
  updateGreenLED();
  beepLong();
  Serial.println(F("Armed"));
}

static void disarmAndSleep() {
  armed = false;
  updateGreenLED();
  if (pulseOn) stopPulse();
  inMotion = false;
  digitalWrite(LED_RED, LOW);
  noTone(BUZZER);
  Serial.println(F("Disarmed → deep sleep"));
  beepShort(); beepShort();

  // prepare wake by BUTTON LOW
  pinMode(BUTTON, INPUT_PULLUP);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  // EXT0 (single pin). BUTTON must be RTC-capable:
  esp_sleep_enable_ext0_wakeup((gpio_num_t)BUTTON, 0);
  // Or use EXT1 if needed:
  // esp_sleep_enable_ext1_wakeup(1ULL << BUTTON, ESP_EXT1_WAKEUP_ALL_LOW);

  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_RED, LOW);
  noTone(BUZZER);

  esp_deep_sleep_start(); // restart after wake
}

// ==============================
// setup / loop
void setup() {
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_RED,   OUTPUT);
  pinMode(BUZZER,    OUTPUT);
  pinMode(BUTTON,    INPUT_PULLUP);

  Serial.begin(115200);
  delay(50);

  // EncButton timing (mirror your demo defaults where relevant)
  eb.setBtnLevel(LOW);
  eb.setDebTimeout(50);
  eb.setClickTimeout(500);
  eb.setHoldTimeout(LONG_PRESS_MS);    // ← matches our long-hold

  if (I2C_SDA_PIN >= 0 && I2C_SCL_PIN >= 0) Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  else Wire.begin();
  Wire.setClock(100000);

  if (bmi160.softReset() == BMI160_OK && bmi160.I2cInit(I2C_ADDR) == BMI160_OK) {
    imuOK = true;
    Serial.println(F("BMI160 OK"));
  } else {
    Serial.println(F("BMI160 init failed"));
  }

  if (imuOK) {
    float sum = 0;
    for (int i = 0; i < 50; i++) {
      int16_t acc[3] = {0};
      if (bmi160.getAccelData(acc) == 0)
        sum += mag3(acc[0]*G_PER_LSB, acc[1]*G_PER_LSB, acc[2]*G_PER_LSB);
      delay(10);
    }
    baselineMagG = sum / 50.0f;
  }

  updateGreenLED();
  Serial.println(F("Fishing bite indicator (XIAO ESP32-S3) — default DISARMED"));

  // Small debounce after wake; if user already holding, arm after LONG_PRESS_MS
  delay(30);
  if (digitalRead(BUTTON) == LOW) {
    uint32_t t0 = millis();
    while (digitalRead(BUTTON) == LOW) {
      if (millis() - t0 >= LONG_PRESS_MS) { arm(); return; }
      delay(5);
    }
  }

  // Not armed → deep sleep until long hold
  disarmAndSleep();
}

void loop() {
  const uint32_t now = millis();
  eb.tick();

  // ===== Button logic (EncButton v3) =====
  if (eb.click()) {          // short click toggles
    if (armed) disarmAndSleep();
    else arm();
  }
  if (eb.hold()) {           // long hold always sleeps
    disarmAndSleep();
  }

  updateGreenLED();
  if (!armed) return;        // normally sleeping already

  // ===== Motion sampling =====
  if (now - lastSample >= SAMPLE_INTERVAL_MS) {
    lastSample = now;

    if (imuOK) {
      int16_t acc[3] = {0};
      if (bmi160.getAccelData(acc) == 0) {
        float gx = acc[0]*G_PER_LSB, gy = acc[1]*G_PER_LSB, gz = acc[2]*G_PER_LSB;
        float mag = mag3(gx, gy, gz);

        baselineMagG = baselineMagG * 0.999f + mag * 0.001f;
        float delta = fabsf(mag - baselineMagG);

        if (delta >= TRIGGER_DELTA_G) {
          if (consecutiveTriggers < 255) consecutiveTriggers++;
        } else if (consecutiveTriggers > 0) {
          consecutiveTriggers--;
        }

        bool motionNow = (consecutiveTriggers >= TRIGGER_SAMPLES);
        if (motionNow && !inMotion) {
          inMotion = true;
          motionStartMs = now;
          startPulse(now);             // immediate short pulse
        } else if (!motionNow && inMotion) {
          inMotion = false;
        }
      }
    }
  }

  // ===== Pulse engine =====
  if (pulseOn && (int32_t)(now - pulseOffAt) >= 0) stopPulse();

  if (inMotion) {
    uint32_t dur = now - motionStartMs;
    if (dur >= CONTINUOUS_THRESH_MS) {
      if (!pulseOn && (now - lastPulseStart) >= PULSE_PERIOD_MS) {
        startPulse(now);               // repeat short pulses while motion continues
      }
    }
  }
}
