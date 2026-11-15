# Fishing Bite Sensor — sensor bench release

A two-device ESP-NOW rod-motion prototype: an LSM6DS3 sensor and a separate SH1106 receiver hub. The current bench is Seeed XIAO ESP32-C3; the planned rod sensor is Seeed XIAO ESP32-S3. **Field bite-detection accuracy and battery endurance are not measured.**

[Русская версия](README_RU.md) · [Release scope, validation and local binaries](docs/bench-release.md) · [Detailed bench/flash/CSV instructions](docs/bench-testing.md)

## Current sensor behavior

- Checked LSM6DS3 I²C XYZ bursts, Data Ready and fault recovery; accelerometer ±2g/104Hz, unused gyro off.
- Dynamic XYZ residual versus an explicit Legacy magnitude comparison; threshold and measured noise/effective threshold shown in the web UI.
- Asynchronous 2-second quiet noise calibration, movement rejection; manual threshold/noise multiplier persist separately from the legacy `deltaG` key.
- Nonblocking output pulses; passive-buzzer maximum maps to 50% carrier duty, not DC.
- Temporary movement suppression, then automatic recovery at the stable **new** orientation. Manual disarm and IMU fault never auto-arm.
- Web Status / Sensor / Settings / Diagnostics, English/Russian labels; full-rate bounded Serial CSV and host replay using the firmware core.
- Original packed 22-byte BitePacket stays compatible with the existing hub; no application-level ACK is added in this release.

There is **no 10+10-second Smart Learn procedure**. Noise calibration measures quiet acceleration noise; it is not training a classifier for fish or rod poses.

## Boards, builds and pins

Platform `platformio/espressif32@8.0.0`, Arduino-ESP32 3.3.12 and direct library versions are pinned in `platformio.ini`.

```sh
pio run -e xiao_esp32c3
pio run -e seeed_xiao_esp32s3
python3 tools/run_host_tests.py
```

D-pins remain unchanged: green D7, red D8, buzzer D9, arm D6, power hold D0, power-off input D1, battery ADC D2, SDA D4, SCL D5. Exact C3/S3 GPIO mappings and upload commands are in [bench-testing.md](docs/bench-testing.md).

**D6 is not deep-wake-capable on the declared XIAO boards. Automatic deep sleep is safely blocked with a visible reason until hardware wiring is reviewed.** No replacement pin is silently chosen. Native USB CDC keeps Serial away from the D6/D7 UART/button/LED collision.

## Setup and measurements

Connect to the existing sensor AP (fresh default `BiteSensor`), open `http://192.168.4.1`. Leave the IMU still for calibration; ARM requires a further settling period. Lower threshold means higher sensitivity; noise protection can raise the effective threshold. Network fields are drafts until Validate/Save/Restart. Ordinary firmware update keeps `/config.db`; do not erase flash/upload filesystem just to update firmware.

```sh
python3 tools/replay_csv.py tests/fixtures/weak_transverse.csv > dynamic.csv
python3 tools/replay_csv.py tests/fixtures/weak_transverse.csv --mode legacy > legacy.csv
```

The synthetic comparison shows two transverse impulses detected by Dynamic and missed by Legacy at their default thresholds; it does not establish real bite recall. Mounting, wind, false alarms, low-battery behavior and complete-device current require bench/field tests.

## Hub

The receiver firmware remains in `ESP_HUB/`. This sensor release does not repair the hub's separate dependency, packet queue, display-failure or application-delivery issues identified in the audit. See its project README and the audit plan for that next stage.
