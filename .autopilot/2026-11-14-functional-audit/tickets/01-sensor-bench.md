# 01 — Датчик: исправления, измеримая чувствительность и стендовый выпуск

**Требования:** R01, R02, R03, R04, R05, R06, G01, G02, G03, G04, G05, G06, G07, G08, G09, G10, G11, G12
**Зависит от:** —
**Зона:** `src/`, `include/`, `tests/`, `tools/`, `platformio.ini`, `docs/bench-testing.md`, `README.md`, `README_RU.md`
**Волна:** 1
**Ревью:** нет
**Модель:** сильная

## Контекст и слова пользователя

«IMU: LSM6DS3»; первый C3 стенд с плохой чувствительностью: «YES»; будущий sensor «Seeed XIAO ESP32-S3»; «Временно подавлять и возвращаться после стабилизации»; «конфигурировался в настройках через веб-интерфейс»; старт «Lets start».

Прочитать interfaces.md первым, затем notes.md, spec.md §§7–9 и audit.md по sensor bugs. Старые4 таска в roadmap — будущий полный план, этот таск реализует только согласованный первый sensor-выпуск. Аудит/библиотечные документы уже подготовлены; не переписывать .autopilot, коммиты делает оркестратор.

## Приёмка

1. Sensor lifecycle не блокируется beep/delay; stale clock/grace и pulse early-return исправлены; invalid wake не усыпляет навсегда. Pins не меняются.
2. Реальное проверяемое I²C чтение LSM6DS3: ошибки/short-read/no-new-data различаются; Fault не выдаёт READY; bounded retry/recovery и no unexpected re-arm.
3. Portable tested detector dynamic XYZ с legacy comparison, понятным threshold/noise, dt/gap handling и без накопителя255. Слабые поперечные события видны в синтетическом сравнении; quiet не тревожит.
4. Noise calibration async, неподвижность проверяется, движение/fault отвергаются. UI показывает progress/outcome и persisted config; schema migration без стирания старой DB.
5. MotionSuppressed временно отключает bite outputs, возобновляет в стабильном новом положении после settle; manualDisarmed/Fault сами не включаются.
6. Web menu/conditional reload работает; настройки чувствительности применяются к реально используемому core и переживают reboot; XYZ/signal/noise/threshold/health/rate/gaps доступны, CSV documented и ограничен по частоте.
7. Pinned sensor deps/platform; C3 и S3 сборки зелёные; прежний22byte packet совместим с текущим хабом. ESP_HUB/.vscode/пользовательские optional/hardware files/CLAUDE.md не меняются.
8. Targeted meaningful host tests проверяют именно integrated public core и регрессии; runner не подменяет firmware раздельным demo detector. Документация стенда говорит что измерить и что не проверено на железе; исходный README не обещает отсутствующий SmartLearn.

## Разрешённые зависимости

Текущий набор: platformio/espressif32@8.0.0; GyverDB1.4.4 Settings1.4.3 EncButton3.7.6 AdafruitLSM6DS4.7.4 UnifiedSensor1.1.15 BusIO1.17.4 и их существующие транзитивные зависимости. Native C++17/Pythonstdlib tests. Новые frameworks/libraries не добавлять.

## Проверки

`python3 tools/run_host_tests.py` и pinned PIO C3/S3 из interfaces.md. Полевая эффективность/ток/аппаратные wake/ADC не являются гарантией этой software-проверки. Вернуть стандартный контракт≤25строк, REDPROOF до5 тестов; без diff/code. Не коммитить и не запускать subagents.
