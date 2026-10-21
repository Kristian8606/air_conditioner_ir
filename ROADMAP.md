# Пътна карта: Matter климатик с IR (ESP32-C6, Matter over Thread)

## Основа
- Ново хранилище, клонинг на термостата
  github.com/Kristian8606/thermostat_remote_sensor (v2.3.0).
- Пренася се без промяна: Matter структурата, binding към дистанционния
  сензор (температура, влажност, батерия), светодиодът, бутонът, OTA,
  menuconfig.
- esp-matter: последна версия.
- Всяка версия е тясно насочена към една задача.
- Принцип: Matter атрибутите са единственият източник на истина.
  Callback-ът реагира само на POST_UPDATE, връща винаги ESP_OK и не
  вика update. Логиката чете стойностите с attribute::get_val().

## Състояние
- v0.1.0: копие на термостата v2.3.0 със собствена идентичност:
  VID/PID 0xFFF1/0x8001 (термостатът е 0x8000, затова HA Matter Server
  не предлага OTA образа на единия на другия; тестов DAC има и за
  0x8001), име "Air Conditioner", сериен префикс "ACIR-",
  OTA образ build/air_conditioner_ir-ota.bin.
- v0.2.0: endpoint 1 е Room Air Conditioner: On/Off (лог [POWER]),
  Thermostat и Fan Control (Off/Low/Medium/High/Auto, 3 скорости,
  лог [FAN]). Fan Control е code-driven клъстер: промените идват през
  FanControl::Delegate::OnFanDriveStateChanged(), не през POST_UPDATE,
  и не се четат с attribute::get_val(). Изключение от принципа, по
  същия начин като влажността. Релетата още следват логиката на
  термостата.

## 1. Matter устройство
- Тип Room Air Conditioner (0x0072): On/Off + Thermostat (Cooling
  задължително) + по избор Fan Control.
- esp-matter: room_air_conditioner::create() създава Identify, On/Off и
  Thermostat с feature Cooling. Fan Control се добавя отделно.
- Проверено досега (за потвърждаване с реално устройство):
  - Apple Home: Matter 1.2 типове от iOS 18. Известен бъг: Auto при
    Room AC изпраща Cool (Apple forums, FB25164477); като Thermostat
    работи правилно.
  - Home Assistant: climate entity (режим, температура). Fan Control
    вероятно като отделен fan entity / fan mode, да се провери.
- Възможен резервен вариант: тип Thermostat + Fan Control (без On/Off),
  ако Apple не се държи добре с Room AC.

## 2. Пренасяне на IRremoteESP8266 като ESP-IDF компонент (без Arduino)
- github.com/crankyoldgit/IRremoteESP8266, над 100 протокола.
- Всичките 86 .cpp файла се компилират без Arduino с
  -DUNIT_TEST -I src -I test. Почти целият код е независим от хардуера.
- Пренаписва се само хардуерният слой:
  - IRsend: mark(), space(), enableIROut(), _delayMicroseconds() →
    буфер с импулси, изпращане през RMT TX с носеща честота (38 kHz).
  - IRtimer: micros() → esp_timer_get_time().
  - IRrecv: RMT RX (етап 5).
- Лиценз LGPL-2.1: пазим лиценза и отбелязваме промените.
- Първа проверка: компонентът + прост тест, който изпраща една команда.

## 3. Избор на климатик в menuconfig
- Марка (decode_type_t) и модел. Пин на IR светодиода и на приемника.

## 4. Управление от Matter
- IRac (stdAc::state_t): power, mode, degrees, fanspeed, swingv/swingh.
- При всяко POST_UPDATE събираме текущите стойности от Matter и
  изпращаме ЕДИН пакет с цялото състояние.

## 5. IR приемник (промени от оригиналното дистанционно)
- IRrecv + IRac декодират сигнала и обновяват Matter атрибутите.
- При промяна от дистанционното НЕ изпращаме обратно IR (без цикъл).

## 6. По избор: компенсация с дистанционния сензор
- Климатикът мери под тавана. ESP коригира стойността, която праща,
  за да се постигне зададената температура при сензора.
