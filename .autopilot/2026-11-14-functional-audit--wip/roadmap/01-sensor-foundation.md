# 01 — Исправление основы датчика и безопасное выделение модулей

**Требования:** R01, R02, R03, R04, R05, G01, G02, G07, G08
**Зависит от:** —
**Зона:** `src/main.cpp`, `src/board/`, `src/state/`, `src/imu/`, `src/output/`, `src/config/`, `platformio.ini`, `README.md`, `tests/`
**Волна:** 1
**Ревью:** да — фундамент
**Модель:** сильная

Статус: ПЛАН БУДУЩЕЙ РЕАЛИЗАЦИИ. В текущем запросе не выполнялся, не исправлен и не закоммичен. Зависимости заданы консервативно последовательно из-за общего существующего main.cpp; ранние диагностические наблюдения из таска03 допустимы после01, полноценный параллельный запуск только после пересмотра реальных зон.

Прочитать interfaces.md, notes.md, audit.md, libraries.md и validation.md. Критерии и работа:


Зона: sensor src, BoardProfile, State/Output/Imu/Config modules, platformio config, README/pinout и targeted host tests. Сначала исправить воспроизводимые ошибки, затем перенести код с сохранением исправленного поведения. Реестр: F01–F03,F07–F12,F14–F20,F21; L03,L09,L10,L12,L14. Hub deps/ADC относятся к таску02.

1. Зафиксировать поддержанную XIAO S3 конфигурацию, известные совместимые версии платформы/прямых и существенных транзитивных libs; минимальный reproduce build. Другие env не объявлять поддержанными без сборки и pin-test.
2. Явные Boot/Disarmed/Settling/Armed/MotionSuppressed/Fault/SleepPending. Один владелец времени и переходов; хранить timestamps входа, учитывать rollover.
3. Beep/pulse без delay; output tick независимо от armed/IMU. Исправить passive tone и validate PWM attach; проверить старые core/channel differences.
4. IMU проверяемое чтение/health/recovery; отказ не выдаётся за готовность. Модель/WHO_AM_I/адрес фиксируются по железу.
5. Wake-compatible pin и проверка API return, flush перед sleep, latch/USB/reset behaviour. Переподключение GPIO — физическая операция по проверенной схеме, не автоматическая правка наугад.
6. Настройки draft→validate→apply→persist; menu reload; строгий MAC; ограничения SSID/password; live radio reconfigure с контрактом таска02; last-known-good после отказа сохранения.
7. ADC калибровка sensor; отделить проценты от фактических mV и low-event pending.
8. Выделить модули из main.cpp, не размножая мелкие обёртки; чистое ядро детектора для replay. Зафиксировать реальное поведение и убрать ложные обещания Smart Learn.

Готово: регрессии старых дефектов проходят; на выбранной плате подтверждены wake/power/tone/IMU fault/reconnect/UI persist. Если платы недоступны, явно «host готов, hardware pending», а не полная готовность.
