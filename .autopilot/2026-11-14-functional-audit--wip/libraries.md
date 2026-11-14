# Аудит библиотек и платформы

Проверено 2026-11-14 по фактически установленным пакетам чистых временных сборок и первичным репозиториям сопровождающих проектов. Основной датчик пользователя — Seeed XIAO ESP32-S3. Сведения о текущих версиях относятся к этой дате; для будущей реализации повторно проверять release и закреплять проверенный набор.

## Главный вывод

Большинство прямых библиотек уже актуальны. Обновление «всего на latest» не исправит плохую чувствительность: главные причины — magnitude-only признак, пороги/счётчик, качество/свежесть чтения, крепление и отсутствие калибровки шума. Изменения библиотечного слоя нужны адресно: надёжная I²C-транзакция, понятная совместимость core, уменьшение ненужной работы/потребления и правильная интеграция Settings.

Рекомендация: оставить текущий набор с закреплёнными версиями, исправить ошибки использования и выделить ImuDriverAdapter. Другой driver проверять отдельным A/B экспериментом, не одновременно с новым алгоритмом.

## Прямые зависимости

| Компонент | Установлено / сверено у автора | Решение | Что требуется |
|---|---|---|---|
| Adafruit LSM6DS | 4.7.4 /4.7.4 | Оставить+проверяемый backend/wrapper; альтернативу экспериментально | getEvent не сообщает реальный I²C failure; нужен actual read status и свежесть |
| Adafruit Unified Sensor | 1.1.15 /1.1.15 | Оставить | Нормализованный API нужен Adafruit; units m/s²→g проверять в adapter |
| Adafruit BusIO | 1.17.4 /1.17.4 | Оставить | Ошибку транзакции нужно передавать наверх, не просто иметь BusIO в deps |
| GyverDB | 1.4.4 /1.4.4 | Оставить в первой переработке | Draft/validation, last-good, schema, commit-before-sleep и проверка записи |
| Settings | 1.4.3 /1.4.3 | Оставить с исправленной интеграцией | b.reload, неблокирующие handlers, auth/OTA policy, bounded UI/DB validation |
| EncButton | 3.7.6 /3.7.6 | Оставить | Неблокирующий loop; библиотека не исправляет неправильный wake pin |
| Adafruit GFX | 1.13.0 /1.13.0; hub C3 build SUCCESS | Оставить | Добавить правильные deps во все поддержанные hub env |
| Adafruit SH110X | 2.1.15 /2.1.15; hub C3 build SUCCESS | Оставить для SH1106 | Отказ дисплея не должен отключать радио/вибро |
| WiFi/esp_now/Wire/LittleFS/sleep/watchdog/LEDC | Встроены в Arduino-ESP32 | Закрепить единый core/platform | Не устанавливать случайные сторонние копии системных headers/libs |

## Платформа: современная версия уже доступна

В чистой стандартной среде platform=espressif32 разрешился в официальный PlatformIO8.0.0; framework-arduinoespressif324.30312.261113 соответствует Arduino3.3.12. Выбранный sensor S3 и hub C3 успешно собраны. Поэтому утверждение «официальный PlatformIO обязательно только core2.x» сейчас неверно. Pioarduino latest стабильный release55.03.312-1 также основан на Arduino3.3.12/IDF5.5.5; первая успешная C3/C6 сборка использовала55.03.312.

Предпочтительный кандидат для основного S3 — platformio/espressif32@8.0.0, закреплённый после smoke/hardware проверки. Не использовать floating stable ZIP и не смешивать providers в одном core-cache при сравнении. Проверить ABI ESP-NOW callbacks, LEDC, watchdog, filesystem и wake API. IDF-major guard сам по себе не фиксирует изменение сигнатуры внутри5.x. Release candidate Arduino4.x не нужен ради новизны.

Ранний S3 прогон в общей среде дал ошибки отсутствующих системных заголовков/clang flags; изолированная стандартная повторная сборка прошла. Этот первый результат — дефект/конфликт окружения теста, не доказательство ошибки исходника. Подробности в build-results.md.

## IMU: самое важное место для адресного изменения

Текущий upstream и установленная4.7.4 имеют getEvent→_read→return true; в _read результат data_reg.read(buffer,14) не проверяется. Значит if(!getEvent()) в приложении не даёт обещанной защиты. Один probe адреса перед чтением тоже не доказывает успешность следующего actual read.

ImuDriverAdapter должен гарантировать: проверяемая транзакция с ожидаемым числом байт, корректное масштабирование, timestamp свежего sample, ошибки/timeout, контролируемое recovery. Если API библиотеки не позволяет получить эту гарантию, нужен небольшой проверенный backend либо поддерживаемый локальный patch, а не wrapper, который просто снова возвращает getEvent bool.

Базовая init включает gyro104Hz, а приложение использует только accel. Для небольшой батареи сравнить accel-only против accel+gyro с реальным током и качеством временного подавления. Неиспользуемые gyro/temp чтения и постоянно включённый гироскоп убрать, если они не нужны выбранной модели движения. Сам рост ODR не равен улучшению слабого сигнала и может повысить шум/ток.

## Кандидаты на замену и компромиссы

| Кандидат | Когда полезен | Почему не заменяем автоматически | Цена и проверка |
|---|---|---|---|
| STMicroelectronics lsm6ds3-pid | Контролируемый register-level backend, status callbacks, FIFO/power/filter control | README ST прямо говорит obsolete/no longer maintained; это не более свежая поддерживаемая замена | Средняя: Wire read/write/status, units/range/ODR/recovery; закрепить commit и принять поддержку |
| SparkFun LSM6DS3 1.0.3 | FIFO/interrupt/register API и status-коды низкого уровня | Float getters также не возвращают транспортную ошибку вызывающему коду; замена getter не решит надёжность | Средняя: использовать именно status API, проверить snapshot XYZ, свежесть, ток и replay |
| Универсальный IMU/fusion stack | Если нужны обоснованные gyro/orientation features и другой IMU | Добавляет сложность, units/frames/настройки; нет доказанного выигрыша здесь | Отложить до измерений, не выбирать по популярности |
| Preferences/NVS вместо GyverDBFile | Небольшое ESP32-only хранилище с более явной моделью хранения | Нужно отвязать UI от GyverDB и мигрировать конфиг; не исправляет ошибку validate-after-save само | Средняя: schema/migration/power-loss tests и возврат к old config |
| Async web server вместо SettingsGyver | Только если measured sampling gaps действительно вызваны сервером и их нельзя убрать иначе | Не исправляет blocking application handlers, добавляет concurrency/stack | Средняя/высокая; сравнить gaps/rate/heap до и после |
| Другая OLED library | Новый display/font/memory requirement | GFX/SH110X работают с текущим SH1106; нет доказанного повода миграции | Не делать сейчас; сначала исправить dependencies/degraded mode |

## Транзитивные зависимости и лишний объём

Наблюдавшиеся версии: GTL1.4.7, StringUtils1.6.1, StreamIO1.0.5, FOR_MACRO1.0.0, GyverHTTP1.0.34, BSON3.0.1, Stamp1.5.1, Table1.3.6, GyverIO1.4.2. Это inventory разрешённого дерева, не отдельное заявление об аудите каждой строки или текущей последней версии каждого пакета.

Settings тянет HTTP/serialization/file/OTA компоненты. Не удалять их вручную из .pio: определить используемые возможности, отключить ненужные опции через поддержанный API/config и измерить footprint. Хаб, который только принимает ESP-NOW и показывает OLED, не нуждается в IMU/Settings/GyverDB зависимостях из C6 env. Пакета adafruit/Adafruit LSM6DS3 не найдено registry: правильное семейство Adafruit LSM6DS, но хабу оно вообще не требуется.

Сохранять manifest разрешённого dependency graph и pin direct versions; транзитивные constraints закреплять совместимо с direct API, не придумывать поддерживаемый lockfile там, где его нет. CI должен воспроизводить clean build, а не только собирать на тёплом cache. Проверять licenses при распространении firmware/вендоринге кода; миграция на новый driver не отменяет его лицензии.

## Settings: доступ и хранение

SettingsT имеет /ota,/fetch,/upload. SettingsBase.authenticate допускает доступ при unset pass; проект не задаёт sett.setPass, default AP открыт. Это функциональность библиотеки в текущей конфигурации, не доказательство утечки. Задать policy config/pairing/auth, выключить ненужные endpoints; не выдавать password input AP за защиту всех путей, если AP открыт/пароль не задан.

GyverDBFile откладывает autocommit примерно10s; перед Save&Restart и powerOff flush уже есть, перед sleep нет. Обязательны явная validation до сохранения, commit/ошибка записи и last-good. Настройки после неудачного Save не должны портить рабочий конфиг. Обновление GyverDB само по себе этого не исправляет.

## Критерии выбора backend/обновления

На одной плате/IMU и одинаковых записях: достоверность actual I²C error, whole XYZ snapshot, свежесть/пропуски/период, шум в g при одинаковом ODR/range/filter, RAM/flash/heap, полный ток, detach/reconnect/fault. Только затем реальные слабые движения и ложные тревоги. Менять backend и detector одновременно нельзя: иначе источник улучшения/регрессии неотделим.

## Первичные ссылки

- [Adafruit LSM6DS manifest](https://raw.githubusercontent.com/adafruit/Adafruit_LSM6DS/master/library.properties), [драйвер](https://raw.githubusercontent.com/adafruit/Adafruit_LSM6DS/master/Adafruit_LSM6DS.cpp).
- [Unified Sensor](https://raw.githubusercontent.com/adafruit/Adafruit_Sensor/master/library.properties), [BusIO](https://raw.githubusercontent.com/adafruit/Adafruit_BusIO/master/library.properties).
- [Settings](https://raw.githubusercontent.com/GyverLibs/Settings/main/library.properties), [SettingsT](https://raw.githubusercontent.com/GyverLibs/Settings/main/src/SettingsT.h), [SettingsBase](https://raw.githubusercontent.com/GyverLibs/Settings/main/src/core/SettingsBase.h).
- [GyverDB](https://raw.githubusercontent.com/GyverLibs/GyverDB/main/library.properties), [EncButton](https://raw.githubusercontent.com/GyverLibs/EncButton/main/library.properties).
- [GFX](https://raw.githubusercontent.com/adafruit/Adafruit-GFX-Library/master/library.properties), [SH110X](https://raw.githubusercontent.com/adafruit/Adafruit_SH110x/master/library.properties).
- [PlatformIO8.0.0](https://github.com/platformio/platform-espressif32/releases/tag/v8.0.0), [manifest тега](https://raw.githubusercontent.com/platformio/platform-espressif32/v8.0.0/platform.json).
- [Arduino3.3.12](https://github.com/espressif/arduino-esp32/releases/tag/3.3.12), [pioarduino55.03.312-1](https://github.com/pioarduino/platform-espressif32/releases/tag/55.03.312-1).
- [ST lsm6ds3-pid](https://github.com/STMicroelectronics/lsm6ds3-pid/blob/master/README.md), [register source](https://github.com/STMicroelectronics/lsm6ds3-pid/blob/master/lsm6ds3_reg.c).
- [SparkFun manifest](https://raw.githubusercontent.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library/master/library.properties), [source](https://raw.githubusercontent.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library/master/src/SparkFunLSM6DS3.cpp).
- [Preferences](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/preferences.html).
