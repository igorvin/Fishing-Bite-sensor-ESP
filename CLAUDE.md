# CLAUDE.md

Guidance for coding agents working in this repository. Read `.autopilot/PROGRESS.md` for current decisions, remaining work, and known issues before making changes.

## Scope and status

This is a two-device ESP-NOW fishing-rod motion sensor system. The current software release is a **bench candidate**, not a field-validated bite detector. The existing sensor bench uses Seeed XIAO ESP32-C3 + LSM6DS3; the planned sensor board is Seeed XIAO ESP32-S3. The hub uses a SH1106 OLED.

The detector baseline is `1b8c6b6`; current HU02 two-role firmware is `9916cf0`. Independent clean-clone checks passed 29 sensor tests, 36 hub tests (ASan/UBSan), four C3/S3 builds and eight actual-renderer frames. Both identified physical XIAO C3 roles were flashed with verified data and no backups as authorized. Startup radio registration and healthy sensor IMU/DISARMED status were observed. Field bite accuracy, actual motor vibration, RF range, battery charging and 48-hour endurance remain unvalidated. Deep-sleep work of 2026-11-22 (not yet committed): 37 sensor host tests, C3/S3/C6/`bite_v4_s3` builds, bench C3 sensor flashed and button sleep/wake/idle-sleep checked over USB. 2026-11-23/24 (red heartbeat, ARM diagnostics and journal, arm-in-wind `SteadyWindow`, simplified web UI, impulse detector from field recordings, hub D3 button and MAC on the startup screen): 46 sensor and 39 hub host tests, C3/S3/C6/v4 sensor and C3/S3 hub builds. The user flashed both C3 boards up to the SETTLING-indication change and reported results from the journal; the wind change is not yet tried on hardware.

## Commands

Run sensor commands from the repository root. Install/activate PlatformIO CLI first; if `pio` is not on PATH, use the executable from the installed PlatformIO environment. Host tools require Python 3 and a C++17 compiler.

```sh
# Host behavior regressions: no Arduino dependencies required
python3 tools/run_host_tests.py

# Sensor builds (default environment: xiao_esp32c3)
pio run -e xiao_esp32c3
pio run -e seeed_xiao_esp32s3
pio run -e xiao_esp32c6
pio run -e bite_v4_s3      # PCB v4 pin profile
pio run -e s3_zero         # Waveshare ESP32-S3-Zero (BITE_BOARD_S3_ZERO, raw GPIO numbers)

# Storage integration needs the actual library installed by the C3 build
python3 tests/test_storage_integration.py

# Same-core replay, fixed calibration/arming schedule
python3 tools/replay_csv.py tests/fixtures/weak_transverse.csv > dynamic.csv
python3 tools/replay_csv.py tests/fixtures/weak_transverse.csv --mode legacy > legacy.csv
```

The fixture has 701 synthetic samples and yields 2 Dynamic events versus 0 Legacy events at default settings (still true with the impulse detector). This is a regression example, not a field detection rate. `test_storage_integration.py --baseline` intentionally exits nonzero with six failures to demonstrate the original unsafe writes.

User-operated upload/monitor procedures are in `docs/bench-testing.md`. Sensor C3 upload was performed separately; software checks alone do not establish hardware acceptance. Do **not** routinely run erase-flash or uploadfs after a firmware update: these can destroy settings. An application-only `firmware.bin` is not a complete image to write at address zero.

The hub is a separate PlatformIO project in `ESP_HUB/`. HU02 defaults to a compact summary (with the hub MAC) for 20 s (`ScreenOnMs`, user choice 2026-11-23; was 8 s), then OLED off; fresh alerts wake it. List/history/service are manual USB views. Startup registration, a hub-only restart recovering the running sensor, no-motion diagnostic registration and a quiet control-traffic window were verified on the C3 pair. Details and remaining acceptance: `docs/hub-testing.md`; wire contract: `docs/startup-registration.md`; battery diagram and unmeasured energy target: `docs/power-and-wiring.md`.

Hub checks from the repository root:

```sh
python3 ESP_HUB/tools/run_host_tests.py
python3 ESP_HUB/tools/render_preview.py
pio run -d ESP_HUB -e xiao_esp32c3 -e seeed_xiao_esp32s3 -e s3_zero
```

## Source pointers

- `ESP_HUB/src/main.cpp`, `ESP_HUB/include/HubCore.h`, `HubRender.h`, `DriverSession.h`: hub hardware adapter, portable model, shared one-bit renderer and OLED retry ownership.
- `docs/hub-testing.md`: limited HU02 hardware results and remaining physical acceptance.
- `kicad/kicad_hub/`: hub PCB rev A draft (XIAO C3, BQ24074, LiPo 1200 mAh, 68 x 40 mm) and "beeper" enclosure with belt clip / stand; regenerate with `_generator/run_hub.sh` and `enclosure/build.sh`. Not ordered or built; assumptions listed in its README.
- `shared/StartupRadio.h`: shared bounded startup control parser/state used by both firmware roles.
- `src/main.cpp`: firmware integration, Settings UI, configuration, outputs, ESP-NOW, USB CSV, and sleep policy.
- `include/SensorCore.h`: portable production detector, also used by host tests and replay.
- `include/ImuDriver.h`, `include/ImuRead.h`: LSM6DS3 initialization and checked, fresh XYZ burst reads.
- `include/ImuSession.h`: first-frame timeout and bounded reinitialization lifecycle.
- `include/ConfigDatabase.h`: guards automatic, explicit, and destructor writes after an existing database fails to load.
- `include/BoardProfile.h`: bench, PCB v4 (`BITE_BOARD_V4`) and Waveshare ESP32-S3-Zero (`BITE_BOARD_S3_ZERO`: SDA 4, SCL 5, button 6, VBAT 7, green 8, red 9, buzzer 10; raw GPIO because that variant's D-names differ from XIAO; GPIO0/3/19/20/21/43/44 avoided; no on-board charger) pin profiles and wake eligibility. The S3-Zero profile is build-verified only.
- `include/PowerButton.h`: single-button ARM/sleep gesture, wake confirmation, low-battery guard (host-tested).
- `include/Feedback.h`: armed red heartbeat, error cue, `ArmAssist` (button ARM deferred until release + 1 s when uncalibrated; bounded retry of a rejected boot calibration).
- `include/EventLog.h`: 32-entry RAM journal printed live on USB (not during CSV) and by the `log` command.
- `include/Sensitivity.h`: the five web sensitivity presets (impulse floor + ambient multiplier: 0.030/x10 … 0.012/x6 default … 0.009/x5) and the mapping of old saved Dynamic thresholds via the former 0.050…0.012 g table; every other detector setting is fixed to `Config` defaults.
- `tools/capture_csv.py`: field CSV capture with Enter marks; `tools/replay_csv.py --skip-ms --sensitivity` replays captures through the core.
- `include/CsvBuffer.h`: bounded nonblocking CSV buffering.
- `tests/` and `tools/`: production-core regressions, actual storage integration, and replay tools.
- `docs/bench-testing.md`: wiring, calibration, configuration, capture, upload instructions, and physical checks.

## Behavior and compatibility traps

- The IMU is LSM6DS3, configured for ±2 g and 104 Hz. The unused gyroscope is off. Actual I²C status, read length, and Data Ready are checked; Adafruit `getEvent()` alone does not establish a successful read.
- Boot requires a fresh two-second calibration and starts DISARMED. ARM settles for 1.5 seconds. There is **no Smart Learn 10+10-second sequence**.
- Since 2026-11-23 (user: arm in wind) calibration, SETTLING and the return from SUPPRESSED use one `SteadyWindow` rule instead of per-sample stillness: every |a| 0.85..1.15 g, per-axis peak-to-peak <= max(0.20 g, 6 x SD) capped at 0.80 g, half-window mean drift <= 0.035 g, SD <= 0.15 g (relaxed with the impulse detector so the recorded strong wind still arms). Windows are consecutive blocks, so arming can take up to ~3 s.
- Since 2026-11-24 the Dynamic detector is the **impulse** (sharpness) detector in `SensorCore.h` (`Impulse`), designed on `tests/fixtures/field` (`docs/field-analysis-2026-11-24.md`): 2nd-order 12 Hz high-pass (coefficients for the measured 111 Hz rate), envelope = max of the last 5 samples, running ambient = median envelope over ~10 s excluding the newest ~1 s, refreshed every 0.5 s in all states. Effective threshold = max(`thresholdG` floor, `noiseMultiplier` x ambient); `noiseG` (window SD) is information only. The field regression `field-2026-11-24-wind-and-bites` must stay green (wind: armed, zero alarms at every preset; calm: >= 30 alarms). Slow smooth pulls are not detected by design. The old `|a - gravity|` value is still computed as `dynamic_g` for CSV/diagnostics.
- A finger on the board (RESET, wake press, ARM press) rejects that calibration, and the core then silently drops a pending ARM. This was the 2026-11-23 "button cannot ARM, web can" report (reproduced in `button-arm-uncalibrated-reproduction`). Firmware now defers an uncalibrated button ARM until release + 1 s, retries a rejected boot calibration up to 3 times (never arms), and signals a cancelled ARM with 3 beeps/red flashes. Change steadiness limits only with field data. The red LED flashes 50 ms every 5 s in every awake state, armed or not; only deep sleep is dark (user decisions 2026-11-23: 1 s was too busy, and disarmed must flash too). Green blinks while an ARM request is pending/calibrating/SETTLING and is solid only in ARMED; the long 300 ms beep sounds when ARMED is actually reached after an explicit ARM (not at the request), so a second press during SETTLING (which disarms) is less likely. A SETTLING that keeps restarting is journaled every 3 s with restart count and cause (`settleResets`). The hub startup screen shows the hub MAC on its bottom line.
- Dynamic measures `norm(accel - gravityEstimate)`; Legacy measures the change in acceleration magnitude. New `dynamicThresholdG` and noise settings are distinct from legacy `deltaG`; do not reinterpret or erase old configuration silently.
- Motion suppression returns through SETTLING to ARMED in a stable **new** position. Manual DISARMED and an IMU FAULT never silently auto-arm. Recovery from an IMU fault requires calibration and an explicit ARM.
- First-frame wait is bounded to 150 ms; reinitialization retries are spaced by 2000 ms. Time calculations must remain safe across uint32 rollover.
- Keep output timeout servicing independent of detector early returns. A documented passive buzzer uses a 3 kHz carrier; maximum level maps to 50% duty, not DC.
- One active-LOW button on D1 does ARM (hold 1–3 s), deep sleep (hold ≥3 s, sleeps on release) and wake (hold ~1 s; shorter touches go straight back to sleep). The user physically moved the bench button from D6 to D1 on 2026-11-22; D0 latch and D6 are unused. Gesture, wake confirmation and the 3.30 V low-battery guard live in portable `include/PowerButton.h`. Idle sleep: DISARMED 3 min with no button/USB command/AP client/CSV. Timestamps passed to `disarmedClock.enter()` inside `loop()` must be the loop's `now`, never a later `millis()` (elapsed() wrap caused instant sleep once). PCB v4 uses env `bite_v4_s3` (`BITE_BOARD_V4` pin profile). Sleep/wake verified on the bench C3 over USB only; sleep current and battery operation are unmeasured.
- Web UI simplified on 2026-11-24 (user decision): one page (state, battery %, hub, Armed, Sensitivity preset, Volume, Test signal) plus a Settings menu (rod name = AP name, Wi-Fi password, hub MAC, language, Save and restart). A non-empty hub MAC enables ESP-NOW. Removed from the UI and fixed: algorithm (Dynamic only), legacy threshold, pulse/repeat/continuous timings, suppression switch and angles, LED brightness (100 %), manual calibration, Serial CSV (now USB `csv on|off`) and the Details block (diagnostics only via USB `status`/`log`). Old `/config.db` keys are left untouched and no longer read. Important messages pop up via `say()` -> `u.notice`.
- Native USB CDC avoids sharing UART D6/D7 with the button/LED. Preserve that routing when changing logging.
- Preserve `/config.db`. A failed load of an existing database must leave its bytes intact and disable all automatic/manual writes. Do not format a nonempty filesystem to hide an error; saved settings must not be claimed when only RAM changes succeeded.
- CSV records fresh samples through a 2048-byte buffer; overflow drops whole lines and is counted. Flash is not written per sample. Replay uses a fixed calibration/ARM schedule, not a full reconstruction of past UI/radio actions.
- Battery voltage uses calibrated millivolts and the existing divider. Voltage-derived percentage is approximate; no measured battery-life claim is available.
- The hub packet remains **packed, 22 bytes**: `rodName[16]`, `uint8_t eventType`, `uint8_t batteryPct`, `float deltaG`. Events 1/2/3 mean first bite, continuous motion, and low battery. There is no alarm ACK or guaranteed alarm delivery. Separate 36-byte DISCOVER/ANNOUNCE/ACK control packets perform bounded one-off startup registration; this ACK does not establish a live link. Legacy alarms have no battery-validity bit, so re-register after an actual battery/power change to refresh an unknown validity flag.

Sensor PlatformIO and direct libraries are pinned in `platformio.ini`: official Espressif32 8.0.0 / Arduino-ESP32 3.3.12. Do not restore floating `stable` URLs or mix platform providers in one cache. Direct versions are pinned; this is not a complete transitive lockfile.

Hub traps: preserve D4/D5 OLED, D1 LED, D2 motor, D3 button (active LOW, added 2026-11-23: hold 1.5 s = MAC screen 30 s, short = status/close) and A0 divider pins. Hub env `s3_zero` (`HUB_BOARD_S3_ZERO`, build-verified only) uses GPIO4/5 OLED, 8 LED, 9 motor, 6 button, 7 divider. Motor wiring and the hub PCB plan: `docs/hub-hardware.md`. Receiver channel 1 was verified for startup exchange on the physical C3 pair. Radio stays awake; no ongoing heartbeat, alarm ACK or remote ARM telemetry. The 8-sender registry and 12-episode history are RAM-only. Counts mean known senders since boot, not currently reachable sensors; numbering resets on reboot.

OLED retries must destroy the previous driver before initialization and destroy partially failed candidates; retain production `DriverSession` ownership. Do not reintroduce idle screen rotation, probes or ADC polling while a healthy OLED is off. USB status confirmed startup ON then OFF; physical motor and OLED disconnect/reconnect recovery still need acceptance.

## Ordinary work and history

Before a small follow-up, read `.autopilot/PROGRESS.md`, the relevant code, and the bench guide. Update progress with any fixed/open issue or new user decision. Preserve unrelated local edits and hardware reference files.

Use targeted history: `git log -n 10 -- <path>`, `git log --oneline --grep="Requirements:"`, or `git show --stat <commit>`. Finished Autopilot work is recorded under `.autopilot/2026-11-14-functional-audit/`; its audit and roadmap are historical documents, not proof that future work is implemented.
