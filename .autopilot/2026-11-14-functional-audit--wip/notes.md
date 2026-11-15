> Исторический снимок подготовки и аудита. После него реализован первый стендовый выпуск датчика; актуальный итог — [release-report.md](release-report.md), статус — `.autopilot/PROGRESS.md`.

# Карта существующего проекта

База: commit 00e8e81. Датчик: src/main.cpp, 1313 строк. Хаб: ESP_HUB/src/main.cpp, 624 строки. Два независимых PlatformIO проекта, три заявленные платы каждый. Главная инструкция проекта: CLAUDE.md (пользовательская, не изменена).

Датчик: setArmed, suspendUntilLongPressToArm, wifiEnterConfigMode, wifiAfterArmWindow, espNowBegin/End, sendBitePacket, recalibrateBaseline, autoDisarmCaptureBaseline, checkAutoDisarmMove, startPulse/stopPulse, build/update, handleButton/handlePowerButton, setup/loop. Состояния заданы набором глобальных bool; алгоритм, UI, I/O и питание связаны внутри одного файла.

Хаб: onEspNowRecv → g_lastPkt/g_hasAlert → loop; showIdle/showAlertScreen, startMotorPattern/handleMotor, maybeDimOrOff, readVbat. Одна ячейка для входящих пакетов, списка состояний удилищ нет.

Общий контракт BitePacket дублируется; оба фактически packed по 22 байта: rodName[16], eventType uint8, batteryPct uint8, deltaG float. Версии, номера события, session ID и ACK нет. Существующий формат необходимо учитывать при миграции.

Проверки из предыдущего ревью: C3 датчик собирался успешно (core3.3.12, GyverDB1.4.4, Settings1.4.3, EncButton3.7.6, LSM6DS4.7.4). Host-проверки в /private/tmp/fishing-review-checks.cpp воспроизводили ошибку stale now, зависший testPulse и разность углов; ASan-проверка /private/tmp/fishing-packet-check.cpp воспроизводила небезопасное чтение имени. Это тесты с заглушками и отдельные минимальные примеры, не hardware-in-loop.

Пользовательские изменения до аудита: .DS_Store и каталог ESP_HUB/low-power-micro-one-key-switch-yr-1006-1.snapshot.8/. Они не включаются в изменения аудита.
