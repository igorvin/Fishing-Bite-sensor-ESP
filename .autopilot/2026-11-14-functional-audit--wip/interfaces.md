# Границы предлагаемой переработки

Статус: предложение для будущей реализации, не существующие API и не разрешение начать её.

## Правила проекта

Сохранять PlatformIO/Arduino и существующий UI на первом этапе. Не менять железо и GPIO до подтверждения платы/схемы. Текущий протокол и конфиг мигрировать явно, не менять формат struct молча. Документы и комментарии не являются доказательством реализованного поведения. Пользовательский CLAUDE.md и аппаратные файлы не перезаписывать.

## Модули и владельцы

- BoardProfile: compile-time GPIO, wake-capability, ADC attenuation/divider, PWM channels/timers, board/core capabilities; для неизвестной схемы ошибка конфигурации, а не угаданные значения.
- SensorState: единый переход Boot/Disarmed/Settling/Armed/MotionSuppressed/Fault/SleepPending; причина перехода, timestamps входа; фактическая готовность отдельно от сохранённого желания охраны.
- ImuSampler: sample timestamp, ax/ay/az в g, validity, timeout, счётчики ошибок; отдельное восстановление I2C/датчика.
- BiteDetector: чистая логика update(sample, config), результат None/Short/Continuous + признаки; получает реальное dt, не управляет Wi-Fi, файлами или выходами.
- RodMotionGate: захват ориентации, нормализованная угловая разность, suppress/disarm; не прячет отказ IMU и не обучается на активной поклёвке без правила.
- AlertOutput: неблокирующие beep/pulse с deadline; tick вызывается во всех состояниях, stop гарантируется при fault/disarm.
- RadioLink: init/reconfigure/send/receive, ограниченная очередь, подтверждения/повторы/дедупликация, статус доставки; callback не обращается к экрану/БД и не блокируется.
- ConfigStore + WebUi: validate(candidate) → apply atomically → persist explicit; меню/зависимые поля перестраиваются; отдельные понятные ошибки.
- BatteryMonitor: калиброванные mV, профиль платы, фильтрация/гистерезис; процент приблизительный; события низкого заряда с политикой повторов.
- HubApp: события из очереди → состояния удилищ/экран/вибро; отказ OLED не отключает радиоприём; история/heartbeat только в согласованном объёме.

## Швы для проверки

Clock/time injection для переходов/таймеров и rollover; replay IMU samples для детектора; transport fake для потерь/повторов/порядка; storage fake для отклонённой конфигурации/сохранения; physical board для ADC/PWM/sleep/power и радиодальности.

Сначала извлечь поведение и закрепить регрессионными проверками, затем менять алгоритм. Не распараллеливать изменения одного main.cpp. При миграции протокола сначала поддержка нового на хабе, затем датчики, после подтверждения совместимости удаление старого.

## Подтверждённые решения пользователя

- Основной датчик — Seeed XIAO ESP32-S3.
- Установка на средней части бланка, компактный корпус и небольшой аккумулятор.
- «Временно подавлять и возвращаться после стабилизации» — включить MotionSuppressed как отдельное состояние. Armed → MotionSuppressed → Settling → Armed; Manual Disarmed и Fault не возобновлять по этому пути. Не придумывать длительности стабилизации до измерений.
- Чувствительность к небольшим движениям и веб-настройка — обязательная цель переработки.

Отдельный ImuDriverAdapter изолирует Adafruit/альтернативу от ImuSampler; обёртка обязана сообщать ошибку транспорта и timestamp свежего кадра. Выбор драйвера не должен менять единицы измерения и смысл сохранённого порога молча.

## Контракт первого стендового выпуска (приоритет над проектом будущих модулей)

Один исполнитель владеет `src/`, `include/`, `tests/`, `tools/`, `platformio.ini`, `docs/bench-testing.md` и sensor README. Пользовательские CLAUDE.md/.vscode/.gitignore.optional и аппаратные файлы не трогать. ESP_HUB source/config не трогать; original packed BitePacket22bytes сохранить. Конкретные имена portable types могут быть уточнены исполнителем единообразно, без расширения scope.

Проверки: `python3 tools/run_host_tests.py` — standalone C++17 compiler runner без нового Python/JS framework; тесты публичного ядра и fake IO. Сборки: `PLATFORMIO_CORE_DIR=/private/tmp/fishing-review-standard-pio /Users/macbook-air/.platformio/penv/bin/pio run -e xiao_esp32c3 -e seeed_xiao_esp32s3`. PIO/cache могут требовать штатной sandbox escalation; использовать её для разрешённых сборок, не менять системную среду глобально. Binaries лежат в .pio/build/<env>/; packaging выполняет оркестратор после финального code commit.

Внешние контракты: timestamp uint32ms с wrap-safe elapsed; accel XYZ в g; optional data freshness/error/valid status; отдельные output events и display telemetry. UART CSV documented header и bounded rate. Host-код не включает Arduino/WiFi headers; драйвер/loop используют то же tested core. Нельзя заменять integrated logic отдельным тестовым двойником.

Безопасное sleep-поведение по текущим pins: invalidwake → видимая причина и остаёмся доступными. Не назначать другой GPIO без схемы. Новое шумовое schema/key не перезаписывает старый смысл deltaG. Первичная cal перед READY и явная ручная cal имеют проверяемые outcomes. Подавление автоматически возобновляет только желаемую пользователем охрану, а не ручной Disarmed/Fault.
