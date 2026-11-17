# Minimal CLAUDE.md corrections for HU02

Ready-to-apply replacements; this proposal does not edit `CLAUDE.md`.

1. Replace the sensor status paragraph (beginning “The sensor source release”) with:

> Sensor core release `1b8c6b6` remains the detector baseline; current two-role HU02 firmware is `9916cf0`. Independent fresh-clone verification passed 29 sensor tests, 36 hub tests with ASan/UBSan, all four sensor/hub C3/S3 builds, and eight frames from the actual hub renderer. Both identified physical XIAO ESP32-C3 roles were flashed through esptool without backup as authorized; upload hashes were verified and hard resets completed. Sensor USB reports healthy IMU and DISARMED state. Earlier sensor USB XYZ and phone UI acceptance remain valid. Physical bite/rod accuracy, actual motor vibration, RF distance, battery/charging and 48-hour runtime remain unvalidated.

2. Replace the hub status paragraph (beginning “The hub is a separate”) with:

> The hub is a separate PlatformIO project in `ESP_HUB/`. Current HU02 firmware `9916cf0` retains the SH1106 renderer and F2 `DriverSession` lifetime fix. Default display is a compact summary for 8 s, then off; fresh events wake it, while list/history/service are manual USB views. Both physical C3 roles completed startup registration: sensor `registered=1 attempts=1 node=1`, hub `radio=1 oled=1 rods=1 history=0`, with startup screen ON then OFF. Manual registration created no extra rod/history or screen wake; invalid broadcast peer was rejected without mutation; control counters stayed unchanged during a 10 s quiet window. Restarting only the hub automatically re-registered the already running sensor via DISCOVER, with one rod and empty history. Existing valid peer settings were retained. This is limited hardware verification, not field acceptance.

3. Replace the source pointer `docs/hub-testing.md`: “pending physical hub acceptance” with “limited HU02 hardware results and remaining physical acceptance”.

4. Replace the packet compatibility bullet with:

> The legacy alarm packet remains packed, 22 bytes: `rodName[16]`, `uint8_t eventType`, `uint8_t batteryPct`, `float deltaG`; event types 1/2/3 are first bite, continuous motion and low battery. Separate 36-byte startup control packets implement bounded DISCOVER/ANNOUNCE/ACK; startup ACK is not an alarm delivery guarantee or a live-link status. Preserve sensor configuration, pins and CSV behavior. Legacy alarms carry no battery-validity bit: an unknown startup battery remains unknown until registration refreshes it; re-register/reboot after an actual power/battery change.

5. Replace “Hub traps” paragraph with:

> Hub traps: preserve D4/D5 OLED, D1 LED, D2 motor and A0 divider pins. Startup exchange was verified on the physical C3 pair; radio stays awake, with no ongoing heartbeat, alarm ACK or remote ARM telemetry. The 8-sender registry and 12-episode history are RAM-only; numbering resets after boot. Registration counts known senders since boot, not currently reachable sensors.

6. Replace the final OLED paragraph with:

> OLED retries must destroy the previous driver before initialization and destroy partially failed candidates; preserve the F2 production `DriverSession` ownership seam. Hub tests pass 36/0 with ASan/UBSan; eight preview frames use the exact renderer. Physical USB reports OLED-ready and the startup ON-to-OFF transition; disconnect/reconnect, fault recovery and motor acceptance remain open.

Keep the existing sensor configuration, pin, CSV, storage and detector constraints unchanged. Do not include hardware MACs, ports or private captures in memory.
