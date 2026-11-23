# Sensor bench release — software scope and validation

The sensor software at commit `1b8c6b6ca829fa9100899844ca68a552fe0ec190` is ready for user-operated bench measurements. This is a software bench release; real bite detection, range and battery endurance have not been measured. See [bench testing](bench-testing.md) for wiring, web controls, flash precautions and the physical acceptance procedure.

## Released behavior

- Dynamic XYZ measures the residual vector after gravity estimation; Legacy magnitude remains an explicit comparison mode. Both signals and the effective threshold are visible in the web UI.
- Asynchronous 2-second quiet-noise calibration rejects movement. The web threshold and noise multiplier persist separately from the old Legacy `deltaG` key; saved noise does not replace a fresh quiet check at startup. There is no 10+10-second Smart Learn classifier.
- Checked I²C transactions, complete XYZ bursts and Data Ready supervise the LSM6DS3 at ±2 g / 104 Hz with the unused gyro off. Missing first data reaches FAULT after 150 ms; reinitialization retries are spaced by at least 2 seconds. Recovery requires calibration and explicit arming.
- Arming settles before detection. Movement suppression resumes at a stable new orientation; manual disarm and IMU faults never automatically arm the sensor. Output pulses are nonblocking.
- Status / Sensor / Settings / Diagnostics support English and Russian. Full-rate Serial CSV uses a bounded 2048-byte RAM buffer and drops whole rows under backpressure, with a visible drop count. Host replay uses the same portable `SensorCore.h` as the firmware.
- Existing D-pins and the packed 22-byte hub packet remain unchanged. The hub firmware is outside this release; application-level delivery ACK is not added.

## Recorded software validation

These results describe the completed acceptance run, not physical tests:

| Check | Recorded result |
|---|---|
| Portable core regression suite | 29 tests, 0 failures |
| Actual pinned GyverDBFile storage integration | 0 failures after the C3 dependency build |
| Independent clean-clone builds | C3 and S3 passed |
| Additional working-project build | C6 passed; outside clean-clone acceptance |
| Software acceptance goals | 18 of 18 met, no blocking review findings |
| Synthetic transverse replay | 701 samples; 2 Dynamic events versus 0 Legacy events at default thresholds |
| Locally prepared C3/S3 images | esptool chip/checksum/validation-hash checks passed; SHA-256 matched independent acceptance |

The synthetic result demonstrates a Legacy feature blind spot; it does not measure real bite recall or false-alarm rates. The platform is pinned to `platformio/espressif32@8.0.0` / Arduino-ESP32 3.3.12, with direct library versions pinned in `platformio.ini`. There is no complete transitive lockfile.

## Reproduce checks and generate local binaries

Install PlatformIO Core CLI or activate its environment so `pio --version` works in your terminal. Python 3 and a C++17 compiler (`c++`, or set `CXX`) are required for host checks and replay; no additional Python packages are needed for those scripts. The first PlatformIO build needs registry access to download the pinned dependencies. Run from the repository root:

```sh
pio run -e xiao_esp32c3
pio run -e seeed_xiao_esp32s3
python3 tools/run_host_tests.py
# Requires GyverDBFile installed by the C3 build above:
python3 tests/test_storage_integration.py
python3 tools/replay_csv.py tests/fixtures/weak_transverse.csv > dynamic.csv
python3 tools/replay_csv.py tests/fixtures/weak_transverse.csv --mode legacy > legacy.csv
```

For the additional C6 compile check, run `pio run -e xiao_esp32c6`. It does not establish physical C6 operation.

The C3 application is generated at `.pio/build/xiao_esp32c3/firmware.bin`; the S3 application is at `.pio/build/seeed_xiao_esp32s3/firmware.bin`. Build intermediates and any locally prepared release bundle under `.pio/releases/` are ignored local outputs. The Git repository carries source, tests and documentation; these images are not published GitHub Release assets.

`firmware.bin` is an application image, **not an image to write at address 0**. Select the correct physical board and port, then use the user-operated PlatformIO upload commands in [bench testing](bench-testing.md#сборка-и-загрузка). Do not erase flash or run `uploadfs` for an ordinary firmware update; preserve the existing `/config.db`. No device was flashed automatically during software acceptance.

## Known limitations and next measurements

- **P3 web arm widget:** after a hardware-button action or FAULT, its position can lag until page refresh. Read the live **State** text for the actual detector state, then refresh to synchronize the widget.
- **Deep wake:** D6 is not a deep-wake-capable pin on the declared C3/C6/S3 boards. Automatic deep sleep is blocked with a visible reason until wiring is reviewed; firmware does not silently substitute a pin.
- **Hardware and field work remains:** runtime web behavior, physical reboot persistence, IMU disconnect/reconnect, radio reception, buzzer output, ADC accuracy, power latch, wake, current, range and actual bite performance were not physically verified by this software acceptance. Follow the bench procedure and record CSV, background false alarms and repeatable weak movements before drawing performance conclusions.
- **Hub work remains:** dependency/queue/display-failure handling and a future delivery protocol belong to the next hub stage. MAC transmission success does not guarantee display on the receiver. Battery percentage remains an approximate voltage-derived estimate.

An open AP and the existing OTA/file endpoints are a bench setup, without a completed production access policy. Quiet calibration and settling times are initial bench parameters, not demonstrated fishing optima.
