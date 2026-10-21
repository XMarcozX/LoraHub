# LoraHub

Version: 1.108V

Прошивка хаба LoRa (Heltec WiFi LoRa 32 V3) с приёмом данных от датчиков по LoRa,
передачей в Integro (HTTPS + WebSocket) и резервной отправкой на локальный HTTP-сервер.

> ВАЖНО: строка `Version: 1.107V` в самом начале файла используется хабом
> для OTA. При выпуске новой прошивки поднимай версию и здесь, и в `#define VERSION`
> в `.ino`. Формат: `Version: X.YYYV` (парсер берёт первый токен с цифрой после слова `version`).

## Возможности

- Приём пакетов `DATA|id|temp|hum|light|pt1000|battery` от датчиков по LoRa.
- Отправка показаний в Integro по HTTPS (`POST /rest/LORASENSOR`).
- WebSocket-канал только для команд и обратной связи (`/ws/LoraHubWebSoket`).
- Автоматический сбор данных и выравнивание периода sleep датчиков.
- Резервная отправка на локальный HTTP-сервер, если Integro недоступен.
- OTA-обновление с GitHub по версии из `README.md`.
- Настройки (имя хаба, имена датчиков, диапазон номеров) хранятся в NVS и переживают обновление.

## Железо

- Плата: Heltec WiFi LoRa 32 V3.2.
- LoRa: 868.0 МГц, BW 125 кГц, SF7, мощность 22 дБм.
- Ethernet: W5500 по SPI.

| W5500 | GPIO |
|-------|------|
| SCK   | 47   |
| MISO  | 19   |
| MOSI  | 48   |
| CS    | 20   |
| IRQ   | —    |
| RST   | —    |

## Сборка и прошивка

1. Arduino IDE, плата Heltec WiFi LoRa 32 V3.
2. Установи библиотеку `heltec_unofficial`, а также `radiolib` (зависимость).
3. Открой `HubLora_IntegratedControl_HTTP_Fallback_v3.ino`, собери и прошей.
4. Для OTA-репозитория собери бинарник и опубликуй его по URL из `HUB_FIRMWARE_URL`.

## Файлы в репозитории (ветка `main`)

| Файл | Назначение |
|------|-----------|
| `HubLora_IntegratedControl_HTTP_Data_WS_Commands_v2.bin` | Образ прошивки для OTA |
| `README.md` | Этот файл; содержит актуальную версию |

Хаб скачивает:
- прошивку: `https://raw.githubusercontent.com/XMarcozX/LoraHub/main/HubLora_IntegratedControl_HTTP_Data_WS_Commands_v2.bin`
- версию: `https://raw.githubusercontent.com/XMarcozX/LoraHub/main/README.md`

## Режимы транспорта

Переключаются командами `transport|...`, сохраняются в NVS.

- `integro` — Integro, при неудаче — локальный сервер (если задан).
- `auto` — то же поведение с резервом.
- `local` — только локальный HTTP-сервер, без WebSocket и Integro.

Порядок отправки: **сначала всегда Integro**, затем, если он не ответил (TCP/ошибка/не 2xx),
отправка на локальный сервер. Локальный сервер используется, если задан адрес и порт.

## Команды Serial (115200)

Настройка транспорта:
```
transport
transport|integro
transport|auto
transport|local
local|192.168.1.100|8080|/api/sensors
local_test
local_test_demo
```

Имя хаба (хранится в NVS, переживает OTA):
```
hubid
hubid|MyHubName
```

Датчики и очередь:
```
sensors
sensors|1|12
list
queue
json
auto
schedule
```

Время:
```
time
time|2026-09-04|12:34:56
```

Команды датчикам:
```
get|sensor_01
config|sensor_01
set|sensor_01|sleep|300
set|sensor_01|rref|998
set|sensor_01|vref|3.24
set|sensor_01|cal_gain|1.000000
set|sensor_01|cal_offset|0.000000
rename|sensor_01|NewName
```

Обновление:
```
update
```

## Формат данных

Пример полезной нагрузки (`buildJSON`):
```json
{
  "hub_id": "hub_03",
  "received_at": 1792051214,
  "sensors": [
    {
      "id": "T1_3",
      "temperature": 10.79,
      "humidity": 82.18,
      "light": 0.8,
      "pt1000": 10.48,
      "battery": 3.997,
      "rssi": -106.0,
      "snr": -2.5,
      "received_at": 1792051125
    }
  ]
}
```

## OTA

- Хаб раз в час (плюс после загрузки при наличии интернета) скачивает `README.md`.
- Ищет версию после слова `version`, например `Version: 1.107V`.
- Сравнивает по числам с `VERSION` прошивки.
- Если в README версия выше — скачивает `.bin` и перезагружается.
- Вручную: команда `update`.

Релиз-чеклист:
1. Поднять `#define VERSION` в `.ino`.
2. Собрать и залить новый `.bin` в репозиторий.
3. Поднять `Version:` в этом `README.md`.

## Сохранение настроек

В NVS (не стирается при OTA):

- имена датчиков — namespace `names`;
- диапазон номеров датчиков — namespace `hubcfg` (`sensor_min`, `sensor_max`);
- имя хаба — namespace `hubcfg` (`hub_id`);
- настройки транспорта — namespace `hub_transport`.

## Очередь команд

Команда из WebSocket сначала ставится в очередь и отправляется при следующем `DATA`
целевого датчика. Если подтверждение не получено, хаб делает до 3 попыток или
сбрасывает команду по таймауту (15 минут), чтобы не «спамить» одинаковыми ответами
в WebSocket.
