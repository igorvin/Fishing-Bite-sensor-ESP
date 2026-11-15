# Независимая приёмка первого стендового выпуска

Source commit1b8c6b6ca829fa9100899844ca68a552fe0ec190. Clean clone:/private/tmp/fishing-blind-acceptance-20261115-a7. Проверено18/18 целей software scope, расхождений0. Первоначальная отметка «эффективность частично» уточнена тем же независимым проверяющим без новых тестов: качественная оценка выполнена, физические измерения в этом выпуске пользователь не требовал. Предел доказательств остаётся явным, физическая эффективность не подтверждена.

Команды из чистого клона:
- python3 tools/run_host_tests.py —29tests0failed.
- python3 tools/replay_csv.py tests/fixtures/weak_transverse.csv > dynamic.csv и --mode legacy > legacy.csv —701samples,2vs0событий, calibration success.
- PLATFORMIO_CORE_DIR=/private/tmp/fishing-bench-core /Users/macbook-air/.platformio/penv/bin/pio run -e xiao_esp32c3 —SUCCESS, app1178800bytes.
- Та же команда -e seeed_xiao_esp32s3 —SUCCESS, app1062000bytes.
- После C3 build python3 tests/test_storage_integration.py —actualGyverDBFile0failed; --baseline намеренно воспроизводит6исходныхошибок.

Чистый клон до/после — gitclean. Незакоммиченных softwareзависимостей не найдено. Registry downloads потребовали штатного разрешённого сетевого исполнения. Прямые версии закреплены, полного транзитивного lockfile нет. C6 дополнительно успешно собран в рабочем проекте, не отдельный объект этой clean-cloneприёмки.

Повторное ревьюF1: сохранность corrupt/unreadableconfig приauto/manual/destructor write CLOSED; firstframe150ms/2000ms sparse retry/calibrate→Disarmed CLOSED; новых регрессий нет. Старый хаб не менялся,22bytepacketсохранён.

Не проверено физически: runtimewebUI/реальное сохранение послеreboot/I²Cdetach-reconnect/радиоприём/tone/ADC/powerlatch/wake/ток/дальность/поклёвки. Это программныйстендовыйкандидат, не полевойсертификат.
