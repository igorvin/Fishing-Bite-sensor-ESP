> Исторический снимок подготовки и аудита. После него реализован первый стендовый выпуск датчика; актуальный итог — [release-report.md](release-report.md), статус — `.autopilot/PROGRESS.md`.

# Результаты проверки сборок

Дата2026-11-14. Исходники00e8e81. Сборки во временной копии, не в пользовательском checkout; ничего не загружалось в устройства. PlatformIO Core6.2.0. Поддержка железа и работа алгоритма не следуют из успешной компиляции.

| Проект/env | Результат | Среда/причина |
|---|---|---|
| Sensor C3 | SUCCESS (предыдущий проход ревью) | pioarduino55.03.312, Arduino3.3.12; RAM41532/327680, flash1224013/1310720 |
| Sensor C6 | SUCCESS | pioarduino55.03.312, Arduino3.3.12; RAM48524/327680, flash1247162/1310720 |
| Sensor Seeed XIAO S3 — выбранный | SUCCESS в изолированной стандартной среде | platformio espressif32@8.0.0, framework4.30312.261113 (Arduino3.3.12), S3-libs3.3.12, GCC14.2.0+20260121; RAM49836/327680 (15.2%), flash1053005/3342336 (31.5%) |
| Hub C3 | SUCCESS в той же стандартной среде | platformio8.0.0/core3.3.12, GFX1.13.0/SH110X2.1.15/BusIO1.17.4; RAM36604/327680, flash960979/1310720 |
| Hub S3 | FAILED: дефект зависимости | fatal Adafruit_GFX.h: No such file or directory; у env нет lib_deps |
| Hub C6 | FAILED до компиляции | UnknownPackageError adafruit/Adafruit LSM6DS3; также статически отсутствуют OLED dependencies |

Первый sensor S3 прогон в общей provider/cache среде не прошёл: отсутствовали стандартные headers и clang не понимал Xtensa flags. После изолированной стандартной установки этот же исходник успешно собрался. Не включать первоначальную ошибку окружения в реестр дефектов прошивки.

Сравнивать проценты flash между платами напрямую нельзя: app partitions имеют разный размер. C3/C6 сейчас близки к app-partition limit, но выбранный S3 имеет иной запас. Это не измерение runtime heap или battery consumption.

Основные sensor libraries: GyverDB1.4.4, Settings1.4.3, EncButton3.7.6, AdafruitLSM6DS4.7.4, UnifiedSensor1.1.15, BusIO1.17.4. Прямые/транзитивные версии и upstream проверка в libraries.md.

## Host-проверки

- В предыдущем проходе source-derived loop с заглушками воспроизвёл stale-now при button ARM и test pulse, остающийся включённым в DISARMED.
- Минимальный packed packet + небезопасное C-string чтение воспроизвели stack-buffer-overflow под AddressSanitizer. Фактический Arduino String constructor использует strlen, поэтому механизм дефекта совпадает.
- Вычислены norm-delta для поперечного движения, wrap±180, накопитель attack/release и EMA step response. Это математические/host проверки, не измерение настоящей поклёвки или всех путей MCU.

Аппаратных и полевых испытаний не было. Не выпускать firmware как проверенную на воде по этим результатам.
