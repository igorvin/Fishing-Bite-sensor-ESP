# Предложение краткой памяти проекта (T0)

## Текущее состояние
- Источник истины — код и проверенные команды ниже; `interfaces.md` содержит и будущий план, и исторические заметки. Не воспринимать предложенные модули/состояния/времена как реализованные.
- Текущий проект — стендовый датчик ESP-NOW. Полевая точность, ток/автономность, дальность, поведение на реальной плате и загрузка прошивки физически не проверены.
- Проверено на clean clone HEAD `1b8c6b6`: `python3 tools/run_host_tests.py` — 29/0; replay `tests/fixtures/weak_transverse.csv` в Dynamic и Legacy — 701 sample, 2 против 0 событий.
- Проверены сборки `xiao_esp32c3` и `seeed_xiao_esp32s3` с `PLATFORMIO_CORE_DIR=/private/tmp/fishing-bench-core /Users/macbook-air/.platformio/penv/bin/pio run -e <env>` — SUCCESS. После C3 build: `python3 tests/test_storage_integration.py` — 0 failures; `--baseline` намеренно показывает 6 failures.
- Для новых проверок host runner нужны Python 3 и C++17. `pio` отсутствует в shell PATH: используй указанный абсолютный путь либо найденный установленный PIO с активацией; не выдумывай пакет/инсталляцию. Upload не выполнялся.
- Секретов и `.env` нет. D6 не поддерживает deep wake на объявленных платах: deep sleep явно блокируется, оставаясь доступным. Не подбирать новый GPIO без схемы.
- ESP_HUB и его packed 22-byte `BitePacket` не менялись. ACK нет; успешный MAC send не означает доставку/показ.

## Краткие указатели
- `src/main.cpp` — firmware loop, UI, конфигурация, радио, CSV, сон и сохранение.
- `include/SensorCore.h` — то же переносимое ядро для firmware и replay: Dynamic XYZ и Legacy magnitude, состояния, калибровка, suppression.
- `include/ImuDriver.h`, `include/ImuRead.h`, `include/ImuSession.h` — проверяемый I²C/XYZ/Data Ready и жизненный цикл IMU; first-frame deadline 150 ms, retry 2000 ms.
- `include/ConfigDatabase.h` — предохранитель GyverDBFile при ошибке чтения: auto/manual/destructor writes закрыты.
- `include/BoardProfile.h` — pin aliases и проверка wake; D6 оставлен неизменным.
- `tools/run_host_tests.py`, `tests/test_core.cpp` — host C++17 suite; `tools/replay_csv.py`, `tools/replay.cpp`, `tests/fixtures/weak_transverse.csv` — replay тем же ядром.
- `tests/test_storage_integration.py` — фактический интеграционный тест зависимости, доступной после C3 PIO fetch/build.
- `docs/bench-testing.md` — режимы стендовой проверки, ограничения, подключения и инструкции; `README.md` — краткая сводка.

## Неочевидные ловушки и устаревшие заметки
- `CLAUDE.md` описывает старое поведение. Smart Learn 20 секунд (10+10) отсутствует: вместо него тихая noise calibration 2 секунды; при старте обязательна свежая калибровка, сохранённый шум не делает READY.
- Сохранённые arm intent/pins не дают автоматического ARM после загрузки; после IMU fault тоже нужна явная команда. Motion suppression возвращается после стабилизации нового положения; ручной DISARMED и FAULT не авто-включаются.
- Старый `deltaG` оставлен для Legacy; Dynamic threshold хранится отдельно в `dynamicThresholdG`. Не переинтерпретировать ключ или менять схему молча.
- Документы описывают pin aliases; точные GPIO зависят от платы. D6 — кнопка, а не допустимый wake pin; не обещать deep-sleep/экономию батареи.
- Гироскоп выключен: алгоритм использует акселерометр. Батарейный процент приблизителен. CSV-буфер ограничен и может терять целые строки.
- При ошибке чтения существующей БД настройки остаются в RAM: нельзя форматировать раздел или считать сохранение успешным. Интеграционный storage-тест требует сначала C3 PIO build.
- Replay использует portable core, но не побитовую запись MCU/UI/радио: fixed calibration/ARM timing. Два события на fixture — синтетическая демонстрация, не оценка обнаружения на рыбалке.

## Работа без автопилота
Сначала прочитать этот файл и соответствующие исходники/README; source — истина при расхождении. Не редактировать пользовательский `CLAUDE.md`. Сохранять узкий scope, не менять `ESP_HUB`, pins, пакет или формат конфигурации без явной задачи и проверки совместимости. Перед изменением уточнять существующие локальные инструкции, не переписывать чужие артефакты.

## История git
Рабочая ветка/HEAD могут отличаться от проверенного clean-clone `1b8c6b6`; сначала сверить `git status` и `git log`. Не коммитить без запроса. После изменений сообщать файлы, проверки и непроверенные аппаратные аспекты.

## Продолжение проекта
См. `.autopilot/PROGRESS.md` для статуса и актуального продолжения; не копировать сюда backlog и не принимать план из `interfaces.md` за готовую реализацию.
