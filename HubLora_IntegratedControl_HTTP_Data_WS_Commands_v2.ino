#include <heltec_unofficial.h>
#include <SPI.h>
#include <ETH.h>
#include <NetworkClientSecure.h>
#include <Preferences.h>
#include <time.h>
#include <HTTPClient.h>
#include <Update.h>
#include <esp_system.h>

// ============================================================
//                       LoRa
// ============================================================

#define FREQUENCY        868.0
#define BANDWIDTH        125.0
#define SPREADING_FACTOR 7
#define TRANSMIT_POWER   22

// ============================================================
//                    ТАЙМИНГИ
// ============================================================

#define COMMAND_RESPONSE_TIMEOUT_MS 1500
#define HUB_RX_TIMEOUT_MS            1000

// ============================================================
//                  КОЛИЧЕСТВО ДАТЧИКОВ
// ============================================================

#define MAX_SENSORS 20
#define MAX_COMMANDS 60
#define SENSOR_ONLINE_TIMEOUT_MS 420000UL

// ============================================================
//                АВТОСБОР ДАННЫХ / INTEGRO
// ============================================================

#define AUTO_COLLECTION_ENABLED true
#define COLLECTION_TIMEOUT_MS 185000UL
#define INTEGRO_ENABLED true

// Integro HTTPS endpoint.
#define INTEGRO_HOST "integra.darym.ru"
#define INTEGRO_PORT 443
#define INTEGRO_PATH "/rest/LoraSensor"
#define INTEGRO_WS_ENABLED true
#define INTEGRO_WS_HOST "integra.darym.ru"
#define INTEGRO_WS_PORT 443
#define INTEGRO_WS_PATH "/rest/LoraHubWebSoket"
#define HUB_ID "hub_01"
#define HUB_FIRMWARE_URL "https://github.com/XMarcozX/LoraHub"

// Проверка доступности интернета.
#define INTERNET_CHECK_INTERVAL_MS 60000UL

#define W5500_ENABLED true
#define W5500_SCK  47
#define W5500_MISO 19
#define W5500_MOSI 48
#define W5500_CS   20
#define W5500_IRQ  -1
#define W5500_RST  -1

byte ethernetMac[] = {0x02, 0x48, 0x55, 0x42, 0x01, 0x01};
bool ethernetReady = false;
bool internetAvailable = false;
unsigned long lastDisplayUpdate = 0;
unsigned long lastInternetCheck = 0;
NetworkClientSecure wsClient;
bool wsConnected = false;
unsigned long lastWsConnectAttempt = 0;
#define WS_RECONNECT_INTERVAL_MS 10000UL
bool otaUpdateRequested = false;

#define MANAGED_SENSOR_MIN_DEFAULT 1
#define MANAGED_SENSOR_MAX_DEFAULT 12

int managedSensorMin = MANAGED_SENSOR_MIN_DEFAULT;
int managedSensorMax = MANAGED_SENSOR_MAX_DEFAULT;
Preferences hubPreferences;

// Автоматическое выравнивание датчиков по времени.
#define AUTO_SCHEDULE_ENABLED true
#define SCHEDULE_PERIOD_SEC 300UL
#define SCHEDULE_TOLERANCE_SEC 15UL
#define INITIAL_ALIGNMENT_ENABLED true
#define INITIAL_ALIGNMENT_MIN_SLEEP_SEC 10UL

// Начальная расстановка: датчики sensor_01...sensor_N
// получают равномерные позиции внутри 5-минутного цикла.
bool initialAlignmentActive = false;
unsigned long initialAlignmentAnchorMs = 0;
unsigned long initialAlignmentDeadlineMs = 0;
bool alignmentRestorePending[MAX_SENSORS];
bool alignmentDone[MAX_SENSORS];

// ============================================================
//                    СТРУКТУРА ДАННЫХ
// ============================================================

struct SensorData
{
    String id;

    float temperature;
    float humidity;
    float light;
    float pt1000;
    float battery;

    float rssi;
    float snr;

    unsigned long lastSeen;
    String receivedAt;
    time_t receivedUnix;
    String name;
    bool valid;

    // Контроль фактического интервала между DATA.
    unsigned long previousDataAt;
    bool periodInitialized;
};

SensorData sensors[MAX_SENSORS];

// ------------------------------------------------------------
// Автоматический цикл сбора.
// cycleExpected[] = датчики, которые были доступны в начале
// цикла; cycleFresh[] = кто уже прислал свежие DATA в этом цикле.
// ------------------------------------------------------------
bool collectionActive = false;
bool cycleExpected[MAX_SENSORS];
bool cycleFresh[MAX_SENSORS];
unsigned long collectionStartedAt = 0;

// ============================================================
//                   ОЧЕРЕДЬ КОМАНД
// ============================================================

struct PendingCommand
{
    String sensorId;
    String command;

    bool pending;
};

PendingCommand commandQueue[MAX_COMMANDS];

// ============================================================
//                УПРАВЛЯЕМЫЙ ДИАПАЗОН
// ============================================================

int sensorNumber(const String &id)
{
    if (!id.startsWith("sensor_"))
        return -1;
    int n = id.substring(7).toInt();
    if (n < 1 || n > MAX_SENSORS)
        return -1;
    return n;
}

bool isManagedSensor(const String &id)
{
    int n = sensorNumber(id);
    return n >= managedSensorMin && n <= managedSensorMax;
}

void loadManagedSensorRange()
{
    hubPreferences.begin("hubcfg", true);
    managedSensorMin = hubPreferences.getInt("sensor_min", MANAGED_SENSOR_MIN_DEFAULT);
    managedSensorMax = hubPreferences.getInt("sensor_max", MANAGED_SENSOR_MAX_DEFAULT);
    hubPreferences.end();

    if (managedSensorMin < 1 || managedSensorMax > MAX_SENSORS ||
        managedSensorMin > managedSensorMax)
    {
        managedSensorMin = MANAGED_SENSOR_MIN_DEFAULT;
        managedSensorMax = MANAGED_SENSOR_MAX_DEFAULT;
    }
}

void setManagedSensorRange(int first, int last)
{
    if (first < 1 || last > MAX_SENSORS || first > last)
    {
        Serial.println("Invalid sensor range.");
        return;
    }

    managedSensorMin = first;
    managedSensorMax = last;

    hubPreferences.begin("hubcfg", false);
    hubPreferences.putInt("sensor_min", managedSensorMin);
    hubPreferences.putInt("sensor_max", managedSensorMax);
    hubPreferences.end();

    Serial.print("Managed sensors: ");
    Serial.print(managedSensorMin);
    Serial.print("-");
    Serial.println(managedSensorMax);
}

void printManagedSensorRange()
{
    Serial.print("Managed sensors: ");
    Serial.print(managedSensorMin);
    Serial.print("-");
    Serial.println(managedSensorMax);
}

int findSensor(String id);
time_t currentUnixTime();

// ============================================================
//                 ИМЕНА ДАТЧИКОВ
// ============================================================

String sensorNameKey(const String &id)
{
    String key = id;
    key.replace("sensor_", "n");
    return key;
}

void loadSensorName(int index)
{
    if (index < 0 || index >= MAX_SENSORS || !sensors[index].valid) return;

    hubPreferences.begin("names", true);
    String saved = hubPreferences.getString(sensorNameKey(sensors[index].id).c_str(), "");
    hubPreferences.end();

    sensors[index].name = (saved.length() > 0) ? saved : sensors[index].id;
}

bool renameSensor(const String &id, const String &newName)
{
    if (!isManagedSensor(id) || newName.length() == 0) return false;

    int index = findSensor(id);
    if (index >= 0)
        sensors[index].name = newName;

    hubPreferences.begin("names", false);
    hubPreferences.putString(sensorNameKey(id).c_str(), newName);
    hubPreferences.end();
    return true;
}

// ============================================================
//                  ПОИСК ДАТЧИКА
// ============================================================

int findSensor(
    String id
)
{
    for (
        int i = 0;
        i < MAX_SENSORS;
        i++
    )
    {
        if (
            sensors[i].valid &&
            sensors[i].id == id
        )
        {
            return i;
        }
    }

    return -1;
}

// ============================================================
//                СОЗДАНИЕ НОВОГО ДАТЧИКА
// ============================================================

int getOrCreateSensor(
    String id
)
{
    int index =
        findSensor(id);

    if (index >= 0)
    {
        return index;
    }

    for (
        int i = 0;
        i < MAX_SENSORS;
        i++
    )
    {
        if (!sensors[i].valid)
        {
            sensors[i].id =
                id;

            sensors[i].temperature =
                NAN;

            sensors[i].humidity =
                NAN;

            sensors[i].light =
                NAN;

            sensors[i].pt1000 =
                NAN;

            sensors[i].battery =
                NAN;

            sensors[i].rssi =
                0;

            sensors[i].snr =
                0;

            sensors[i].lastSeen =
                millis();
            sensors[i].receivedAt = "";
            sensors[i].receivedUnix = 0;
            sensors[i].name = id;

            sensors[i].previousDataAt = 0;
            sensors[i].periodInitialized = false;

            sensors[i].valid =
                true;

            loadSensorName(i);

            return i;
        }
    }

    return -1;
}

// ============================================================
//                  ПОИСК ОЧЕРЕДИ
// ============================================================

int findCommand(
    String sensorId
)
{
    for (
        int i = 0;
        i < MAX_COMMANDS;
        i++
    )
    {
        if (
            commandQueue[i].pending &&
            commandQueue[i].sensorId ==
                sensorId
        )
        {
            return i;
        }
    }

    return -1;
}

// ============================================================
//                    ДОБАВИТЬ КОМАНДУ
// ============================================================

void queueCommand(
    String sensorId,
    String command
)
{
    sensorId.trim();
    command.trim();

    // Не добавляем точный дубликат.
    for (
        int i = 0;
        i < MAX_COMMANDS;
        i++
    )
    {
        if (
            commandQueue[i].pending &&
            commandQueue[i].sensorId == sensorId &&
            commandQueue[i].command == command
        )
        {
            Serial.println();
            Serial.println("Command already in queue.");
            return;
        }
    }

    // Добавляем новую команду в свободную ячейку.
    for (
        int i = 0;
        i < MAX_COMMANDS;
        i++
    )
    {
        if (!commandQueue[i].pending)
        {
            commandQueue[i].sensorId = sensorId;
            commandQueue[i].command = command;
            commandQueue[i].pending = true;

            Serial.println();
            Serial.println("Command added to queue:");
            Serial.print("Sensor: ");
            Serial.println(sensorId);
            Serial.print("CMD:    ");
            Serial.println(command);
            return;
        }
    }

    Serial.println("COMMAND QUEUE FULL");
}

// ============================================================
//                  УДАЛИТЬ КОМАНДУ
// ============================================================

void removeCommand(
    int index
)
{
    if (
        index < 0 ||
        index >= MAX_COMMANDS
    )
    {
        return;
    }

    commandQueue[index].pending =
        false;

    commandQueue[index].sensorId =
        "";

    commandQueue[index].command =
        "";
}

// ============================================================
//              ПЕЧАТЬ СПИСКА ДАТЧИКОВ
// ============================================================

void printSensors()
{
    Serial.println();
    Serial.println(
        "================================"
    );

    Serial.println(
        "CONNECTED SENSORS"
    );

    Serial.println(
        "================================"
    );

    bool found = false;

    for (
        int i = 0;
        i < MAX_SENSORS;
        i++
    )
    {
        if (!sensors[i].valid)
        {
            continue;
        }

        found = true;

        Serial.print(
            sensors[i].id
        );

        Serial.print(" | Name=");
        Serial.print(sensors[i].name);

        Serial.print(
            " | T="
        );

        Serial.print(
            sensors[i].temperature,
            2
        );

        Serial.print(
            " | PT1000="
        );

        Serial.print(
            sensors[i].pt1000,
            2
        );

        Serial.print(
            " | H="
        );

        Serial.print(
            sensors[i].humidity,
            2
        );

        Serial.print(
            " | RSSI="
        );

        Serial.print(
            sensors[i].rssi,
            1
        );

        Serial.print(
            " | SNR="
        );

        Serial.println(
            sensors[i].snr,
            1
        );
    }

    if (!found)
    {
        Serial.println(
            "No sensors"
        );
    }

    Serial.println(
        "================================"
    );
}

// ============================================================
//                         W5500
// ============================================================

void initEthernet()
{
    if (!W5500_ENABLED)
        return;

    // W5500 работает на SPI2 (стандартный SPI Arduino).
    SPI.begin(W5500_SCK, W5500_MISO, W5500_MOSI, W5500_CS);

    int result = ETH.begin(
        ETH_PHY_W5500,
        1,
        W5500_CS,
        W5500_IRQ,
        W5500_RST,
        SPI
    );

    ethernetReady = (result != 0);
}

int countOnlineSensors();

void syncInternetTime()
{
    internetAvailable = false;

    if (!ethernetReady)
        return;

    // UTC в JSON: однозначное время независимо от часового пояса Hub.
    configTime(0, 0, "pool.ntp.org", "time.nist.gov", "time.google.com");

    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 10000))
        internetAvailable = true;
}

void updateDisplay()
{
#ifndef HELTEC_NO_DISPLAY_INSTANCE
    display.clear();
    display.setFont(ArialMT_Plain_16);
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.drawString(0, 8, "Sensors: " + String(countOnlineSensors()));
    display.drawString(0, 36, internetAvailable ? "Internet: YES" : "Internet: NO");
    display.display();
#endif
}

bool sendHTTPPost(const String &json)
{
    if (!ethernetReady || strlen(INTEGRO_HOST) == 0)
        return false;

    NetworkClientSecure client;
    client.setInsecure();
    if (!client.connect(INTEGRO_HOST, INTEGRO_PORT))
        return false;

    client.setTimeout(3000);

    client.print("POST ");
    client.print(INTEGRO_PATH);
    client.println(" HTTP/1.1");
    client.print("Host: ");
    client.println(INTEGRO_HOST);
    client.println("Content-Type: application/json");
    client.print("Content-Length: ");
    client.println(json.length());

    client.println("Connection: close");
    client.println();
    client.print(json);

    // Считаем пакет доставленным только при HTTP 2xx.
    // Сам факт TCP-соединения не означает, что Integro его принял.
    unsigned long waitStart = millis();
    while (!client.available() && client.connected() &&
           (millis() - waitStart) < 3000UL)
    {
        delay(1);
    }

    bool success = false;

    if (client.available())
    {
        String statusLine = client.readStringUntil('\n');
        statusLine.trim();

        int firstSpace = statusLine.indexOf(' ');
        if (firstSpace >= 0)
        {
            int secondSpace = statusLine.indexOf(' ', firstSpace + 1);
            String codeText;

            if (secondSpace > firstSpace)
                codeText = statusLine.substring(firstSpace + 1, secondSpace);
            else
                codeText = statusLine.substring(firstSpace + 1);

            int statusCode = codeText.toInt();
            success = (statusCode >= 200 && statusCode < 300);
        }
    }

    // Забираем остаток ответа и закрываем соединение.
    unsigned long drainStart = millis();
    while (client.connected() && (millis() - drainStart) < 3000UL)
    {
        while (client.available())
            client.read();
    }

    client.stop();
    return success;
}

// ============================================================
//                 JSON ДЛЯ МОНИТОРИНГА
// ============================================================

// ============================================================
//             КОНТРОЛЬ ПЕРИОДА SLEEP
// ============================================================

int managedSensorCount()
{
    return managedSensorMax - managedSensorMin + 1;
}

void resetInitialAlignment()
{
    initialAlignmentActive = false;
    initialAlignmentAnchorMs = 0;
    initialAlignmentDeadlineMs = 0;

    for (int i = 0; i < MAX_SENSORS; i++)
    {
        alignmentRestorePending[i] = false;
        alignmentDone[i] = false;
    }
}

void startInitialAlignment(unsigned long now)
{
    if (!INITIAL_ALIGNMENT_ENABLED || initialAlignmentActive)
        return;

    initialAlignmentActive = true;
    initialAlignmentAnchorMs = now + (SCHEDULE_PERIOD_SEC * 1000UL);
    initialAlignmentDeadlineMs = now + (SCHEDULE_PERIOD_SEC * 2000UL);

    for (int i = 0; i < MAX_SENSORS; i++)
    {
        alignmentRestorePending[i] = false;
        alignmentDone[i] = false;
    }

    Serial.println();
    Serial.println("=== INITIAL SENSOR ALIGNMENT START ===");
}

void serviceInitialAlignment(unsigned long now)
{
    if (!initialAlignmentActive)
        return;

    // Даём начальной расстановке два полных цикла.
    // После этого новые данные работают уже через обычный монитор.
    if ((long)(now - initialAlignmentDeadlineMs) < 0)
        return;

    bool pending = false;
    for (int i = 0; i < MAX_SENSORS; i++)
    {
        if (alignmentRestorePending[i])
        {
            pending = true;
            break;
        }
    }

    if (!pending)
    {
        initialAlignmentActive = false;
        Serial.println("=== INITIAL SENSOR ALIGNMENT FINISHED ===");
    }
}

bool handleInitialAlignment(int sensorIndex, unsigned long now)
{
    if (!INITIAL_ALIGNMENT_ENABLED || !initialAlignmentActive)
        return false;

    if (sensorIndex < 0 || sensorIndex >= MAX_SENSORS)
        return false;

    int number = sensorNumber(sensors[sensorIndex].id);
    if (number < managedSensorMin || number > managedSensorMax)
        return false;

    // Если после временного sleep датчик снова проснулся,
    // возвращаем ему нормальный период 300 секунд.
    if (alignmentRestorePending[sensorIndex])
    {
        String command =
            "SET|" + sensors[sensorIndex].id +
            "|sleep|" + String(SCHEDULE_PERIOD_SEC);

        bool confirmed = sendCommandAndWait(command, "ACK|");

        if (confirmed)
        {
            alignmentRestorePending[sensorIndex] = false;
            alignmentDone[sensorIndex] = true;
        }

        return true;
    }

    if (alignmentDone[sensorIndex])
        return true;

    // Равномерно распределяем датчики по всему 300-секундному циклу.
    int count = managedSensorCount();
    unsigned long slotSec = SCHEDULE_PERIOD_SEC / count;
    if (slotSec < 1)
        slotSec = 1;

    unsigned long offsetSec =
        (unsigned long)(number - managedSensorMin) * slotSec;

    unsigned long targetMs =
        initialAlignmentAnchorMs + offsetSec * 1000UL;

    // Если выбранная позиция уже прошла, переносим её на следующий цикл.
    while ((long)(targetMs - now) < (long)(INITIAL_ALIGNMENT_MIN_SLEEP_SEC * 1000UL))
        targetMs += SCHEDULE_PERIOD_SEC * 1000UL;

    unsigned long sleepSec =
        (targetMs - now + 999UL) / 1000UL;

    if (sleepSec < INITIAL_ALIGNMENT_MIN_SLEEP_SEC)
        sleepSec = INITIAL_ALIGNMENT_MIN_SLEEP_SEC;

    String command =
        "SET|" + sensors[sensorIndex].id +
        "|sleep|" + String(sleepSec);

    Serial.println();
    Serial.println("INITIAL ALIGNMENT");
    Serial.print("Sensor: ");
    Serial.println(sensors[sensorIndex].id);
    Serial.print("Temporary sleep: ");
    Serial.print(sleepSec);
    Serial.println(" sec");

    bool confirmed = sendCommandAndWait(command, "ACK|");

    if (confirmed)
        alignmentRestorePending[sensorIndex] = true;

    return true;
}

void printSchedule()
{
    Serial.println();
    Serial.println("=== SLEEP PERIOD MONITOR ===");
    Serial.print("Target period: ");
    Serial.print(SCHEDULE_PERIOD_SEC);
    Serial.println(" sec");
    Serial.print("Tolerance: +/- ");
    Serial.print(SCHEDULE_TOLERANCE_SEC);
    Serial.println(" sec");

    unsigned long now = millis();

    for (int i = 0; i < MAX_SENSORS; i++)
    {
        if (!sensors[i].valid)
            continue;
        if (!isManagedSensor(sensors[i].id))
            continue;

        Serial.print(sensors[i].id);
        Serial.print(" | online=");
        Serial.print((now - sensors[i].lastSeen) <= SENSOR_ONLINE_TIMEOUT_MS ? "YES" : "NO");
        Serial.print(" | baseline=");
        Serial.print(sensors[i].periodInitialized ? "YES" : "NO");

        if (sensors[i].periodInitialized)
        {
            unsigned long ageSec = (now - sensors[i].lastSeen) / 1000UL;
            Serial.print(" | age=");
            Serial.print(ageSec);
            Serial.print("s");
        }

        Serial.println();
    }
}

// Проверяет фактический интервал между двумя DATA одного датчика.
// Первый DATA только создаёт baseline и никогда не вызывает команду.
bool sleepPeriodIsOutOfRange(int sensorIndex, unsigned long now, unsigned long &intervalSec)
{
    intervalSec = 0;

    if (sensorIndex < 0 || sensorIndex >= MAX_SENSORS)
        return false;

    if (!sensors[sensorIndex].periodInitialized)
        return false;

    unsigned long deltaMs = now - sensors[sensorIndex].previousDataAt;
    intervalSec = (deltaMs + 500UL) / 1000UL;

    long errorSec = (long)intervalSec - (long)SCHEDULE_PERIOD_SEC;

    return labs(errorSec) > (long)SCHEDULE_TOLERANCE_SEC;
}

void updateSleepPeriodMonitor(int sensorIndex, bool /*wasKnownOnline*/, unsigned long now)
{
    if (!AUTO_SCHEDULE_ENABLED)
        return;

    if (sensorIndex < 0 || sensorIndex >= MAX_SENSORS)
        return;

    // Первый DATA только создаёт baseline.
    // Дальше проверяем каждый фактический интервал, даже если он
    // оказался больше обычного online timeout: это может быть
    // реальный уход датчика с периода sleep.
    if (!sensors[sensorIndex].periodInitialized)
    {
        sensors[sensorIndex].previousDataAt = now;
        sensors[sensorIndex].periodInitialized = true;

        Serial.println();
        Serial.println("SLEEP MONITOR: baseline established.");
        Serial.print("Sensor: ");
        Serial.println(sensors[sensorIndex].id);
        Serial.println("No correction command sent.");
        return;
    }

    unsigned long intervalSec = 0;

    if (sleepPeriodIsOutOfRange(sensorIndex, now, intervalSec))
    {
        Serial.println();
        Serial.println("!!! SLEEP PERIOD OUT OF RANGE !!!");
        Serial.print("Sensor: ");
        Serial.println(sensors[sensorIndex].id);
        Serial.print("Actual interval: ");
        Serial.print(intervalSec);
        Serial.println(" sec");
        Serial.print("Expected: ");
        Serial.print(SCHEDULE_PERIOD_SEC);
        Serial.print(" +/- ");
        Serial.print(SCHEDULE_TOLERANCE_SEC);
        Serial.println(" sec");

        // Если для этого датчика уже есть пользовательская команда,
        // она имеет приоритет. После неё следующий DATA даст новый
        // фактический интервал, и монитор снова его проверит.
        if (findCommand(sensors[sensorIndex].id) < 0)
        {
            String command =
                "SET|" + sensors[sensorIndex].id +
                "|sleep|" + String(SCHEDULE_PERIOD_SEC);

            Serial.println("AUTO SLEEP CORRECTION");
            Serial.print("TX: ");
            Serial.println(command);

            bool confirmed =
                sendCommandAndWait(command, "ACK|");

            if (confirmed)
                Serial.println("Sleep period correction confirmed.");
            else
                Serial.println("Sleep period correction NOT confirmed.");
        }
        else
        {
            Serial.println("User command is pending; auto correction skipped.");
        }
    }
    else
    {
        Serial.println();
        Serial.println("SLEEP MONITOR: period OK.");
        Serial.print("Sensor: ");
        Serial.println(sensors[sensorIndex].id);
        Serial.print("Actual interval: ");
        Serial.print(intervalSec);
        Serial.println(" sec");
    }

    // Текущий DATA становится новой точкой отсчёта независимо
    // от результата коррекции. Следующий интервал покажет,
    // применился ли новый sleep.
    sensors[sensorIndex].previousDataAt = now;
}

// ============================================================
//                 ДАТА/ВРЕМЯ ПОЛУЧЕНИЯ
// ============================================================
time_t currentUnixTime()
{
    time_t now = time(nullptr);
    if (now < 1577836800) return 0;
    return now;
}

String jsonEscape(const String &value)
{
    String out;
    for (unsigned int i = 0; i < value.length(); i++)
    {
        char c = value[i];
        if (c == '"' || c == '\\') { out += '\\'; out += c; }
        else if (c == '\n') out += "\\n";
        else if (c == '\r') out += "\\r";
        else if (c == '\t') out += "\\t";
        else out += c;
    }
    return out;
}

String formatReceivedAt()
{
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo, 0)) return "";
    if (timeinfo.tm_year < 120) return "";
    char buf[25];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &timeinfo);
    return String(buf);
}

void setHubTime(String datePart, String timePart)
{
    int y=0, mo=0, d=0, hh=0, mm=0, ss=0;
    if (sscanf(datePart.c_str(), "%d-%d-%d", &y,&mo,&d)!=3 ||
        sscanf(timePart.c_str(), "%d:%d:%d", &hh,&mm,&ss)!=3 ||
        y<2020 || mo<1 || mo>12 || d<1 || d>31 ||
        hh<0 || hh>23 || mm<0 || mm>59 || ss<0 || ss>59)
    {
        Serial.println("Invalid time format.");
        Serial.println("Use: time|2026-09-04|12:34:56");
        return;
    }
    struct tm tmValue = {};
    tmValue.tm_year=y-1900; tmValue.tm_mon=mo-1; tmValue.tm_mday=d;
    tmValue.tm_hour=hh; tmValue.tm_min=mm; tmValue.tm_sec=ss;
    // Используем UTC, чтобы не зависеть от локальной TZ среды.
    setenv("TZ", "UTC0", 1);
    tzset();
    time_t epoch=mktime(&tmValue);
    struct timeval tv={epoch,0};
    settimeofday(&tv,nullptr);
    Serial.print("Hub time set to: ");
    Serial.println(formatReceivedAt());
}

void printHubTime()
{
    String now=formatReceivedAt();
    Serial.print("Hub time: ");
    if (now.length()>0) Serial.println(now); else Serial.println("NOT SET");
}

String buildJSON(bool freshOnly = false)
{
    String json = "{\"hub_id\":\"" + jsonEscape(String(HUB_ID)) +
                  "\",\"received_at\":" + String((unsigned long)currentUnixTime()) +
                  ",\"sensors\":[";

    bool first = true;

    for (int i = 0; i < MAX_SENSORS; i++)
    {
        if (!sensors[i].valid || !isManagedSensor(sensors[i].id)) continue;
        if (freshOnly && !cycleFresh[i]) continue;

        if (!first) json += ",";
        first = false;

        time_t rxTime = sensors[i].receivedUnix;
        if (rxTime <= 0) rxTime = currentUnixTime();

        json += "{\"id\":\"" + jsonEscape(sensors[i].id);
        json += "\",\"name\":\"" + jsonEscape(sensors[i].name);
        json += "\",\"temperature\":" + String(sensors[i].temperature,2);
        json += ",\"humidity\":" + String(sensors[i].humidity,2);
        json += ",\"light\":" + String(sensors[i].light,1);
        json += ",\"pt1000\":" + String(sensors[i].pt1000,2);
        json += ",\"battery\":" + String(sensors[i].battery,3);
        json += ",\"rssi\":" + String(sensors[i].rssi,1);
        json += ",\"snr\":" + String(sensors[i].snr,1);
        json += ",\"received_at\":" + String((unsigned long)rxTime);
        json += "}";
    }

    json += "]}";
    return json;
}

// ------------------------------------------------------------
// Сколько датчиков сейчас доступно.
// ------------------------------------------------------------
int countOnlineSensors()
{
    unsigned long now = millis();
    int count = 0;

    for (int i = 0; i < MAX_SENSORS; i++)
    {
        if (!sensors[i].valid)
            continue;
        if (!isManagedSensor(sensors[i].id))
            continue;

        if ((now - sensors[i].lastSeen) <= SENSOR_ONLINE_TIMEOUT_MS)
            count++;
    }

    return count;
}

// ------------------------------------------------------------
// Начать новый автоматический цикл сбора.
// ------------------------------------------------------------
void startCollectionCycle()
{
    if (!AUTO_COLLECTION_ENABLED || collectionActive)
        return;

    collectionActive = true;
    collectionStartedAt = millis();

    int expected = 0;

    for (int i = 0; i < MAX_SENSORS; i++)
    {
        cycleFresh[i] = false;
        cycleExpected[i] = false;

        if (!sensors[i].valid)
            continue;
        if (!isManagedSensor(sensors[i].id))
            continue;

        unsigned long age = millis() - sensors[i].lastSeen;

        if (age <= SENSOR_ONLINE_TIMEOUT_MS)
        {
            cycleExpected[i] = true;
            expected++;
        }
    }

    Serial.println();
    Serial.println("=== AUTO COLLECTION START ===");
    Serial.print("Available sensors: ");
    Serial.println(expected);
}

// ------------------------------------------------------------
// Проверка: все ожидаемые датчики уже дали свежие DATA?
// ------------------------------------------------------------
bool collectionComplete()
{
    bool haveExpected = false;

    for (int i = 0; i < MAX_SENSORS; i++)
    {
        if (!cycleExpected[i])
            continue;

        haveExpected = true;

        if (!cycleFresh[i])
            return false;
    }

    return haveExpected;
}

// ------------------------------------------------------------
// Отправка JSON в Integro.
// Если интернета нет — пакет не сохраняем и не отправляем.
// ------------------------------------------------------------
bool sendJSONToIntegro(const String &json)
{
#if INTEGRO_ENABLED
    if (!internetAvailable || json.length() == 0)
        return false;

    return sendHTTPPost(json);
#else
    return true;
#endif
}

// ------------------------------------------------------------
// Завершить цикл: отправить только то, что реально собрали.
// ------------------------------------------------------------
void finishCollectionCycle(bool timeout)
{
    if (!collectionActive)
        return;

    int freshCount = 0;
    int expectedCount = 0;

    for (int i = 0; i < MAX_SENSORS; i++)
    {
        if (cycleExpected[i])
        {
            expectedCount++;
            if (cycleFresh[i])
                freshCount++;
        }
    }

    String json = buildJSON(true);

    Serial.println();
    Serial.println("=== AUTO COLLECTION FINISH ===");
    Serial.print("Collected: ");
    Serial.print(freshCount);
    Serial.print(" / ");
    Serial.println(expectedCount);

    if (timeout)
        Serial.println("Reason: collection timeout");
    else
        Serial.println("Reason: all available sensors reported");

    if (freshCount > 0)
        sendJSONToIntegro(json);
    else
        Serial.println("Nothing fresh to send.");

    collectionActive = false;

    for (int i = 0; i < MAX_SENSORS; i++)
    {
        cycleExpected[i] = false;
        cycleFresh[i] = false;
    }
}

void serviceCollection()
{
    if (!AUTO_COLLECTION_ENABLED || !collectionActive)
        return;

    if (collectionComplete())
    {
        finishCollectionCycle(false);
        return;
    }

    if ((millis() - collectionStartedAt) >= COLLECTION_TIMEOUT_MS)
        finishCollectionCycle(true);
}

void printJSON()
{
    Serial.println();
    Serial.println("JSON:");
    Serial.println(buildJSON(false));
}

// ============================================================
//              WEBSOCKET / INTEGRO COMMANDS
//
// WebSocket используется ТОЛЬКО для команд и обратной связи.
// Показания датчиков в WebSocket не отправляются.
// Показания передаются в Integro отдельным HTTP POST.
// ============================================================

String jsonGetString(const String &json, const String &key)
{
    String needle = "\"" + key + "\"";
    int p = json.indexOf(needle);
    if (p < 0) return "";
    p = json.indexOf(':', p + needle.length());
    if (p < 0) return "";
    p++;
    while (p < (int)json.length() && (json[p] == ' ' || json[p] == '\t')) p++;
    if (p >= (int)json.length() || json[p] != '"') return "";
    p++;
    String result;
    while (p < (int)json.length())
    {
        char c = json[p++];
        if (c == '"') break;
        if (c == '\\' && p < (int)json.length())
        {
            char e = json[p++];
            if (e == '"' || e == '\\' || e == '/') result += e;
            else if (e == 'n') result += '\n';
            else if (e == 'r') result += '\r';
            else if (e == 't') result += '\t';
            else result += e;
        }
        else result += c;
    }
    return result;
}

String jsonGetRaw(const String &json, const String &key)
{
    String needle = "\"" + key + "\"";
    int p = json.indexOf(needle);
    if (p < 0) return "";
    p = json.indexOf(':', p + needle.length());
    if (p < 0) return "";
    p++;
    while (p < (int)json.length() && (json[p] == ' ' || json[p] == '\t')) p++;

    if (p < (int)json.length() && json[p] == '"')
    {
        p++;
        int end = p;
        while (end < (int)json.length())
        {
            if (json[end] == '"' && (end == p || json[end - 1] != '\\')) break;
            end++;
        }
        return json.substring(p, end);
    }

    int end = p;
    while (end < (int)json.length() && json[end] != ',' &&
           json[end] != '}' && json[end] != '\n' && json[end] != '\r') end++;
    String result = json.substring(p, end);
    result.trim();
    return result;
}

void wsSendText(const String &payload)
{
    if (!wsConnected || !wsClient.connected()) return;

    size_t len = payload.length();
    uint8_t header[10];
    size_t hlen = 0;
    header[hlen++] = 0x81;

    if (len < 126) header[hlen++] = 0x80 | (uint8_t)len;
    else if (len <= 65535)
    {
        header[hlen++] = 0x80 | 126;
        header[hlen++] = (len >> 8) & 0xFF;
        header[hlen++] = len & 0xFF;
    }
    else
    {
        header[hlen++] = 0x80 | 127;
        for (int i = 7; i >= 0; i--) header[hlen++] = (len >> (i * 8)) & 0xFF;
    }

    uint32_t r = esp_random();
    uint8_t mask[4] = {
        (uint8_t)r, (uint8_t)(r >> 8), (uint8_t)(r >> 16), (uint8_t)(r >> 24)
    };

    wsClient.write(header, hlen);
    wsClient.write(mask, 4);

    for (size_t i = 0; i < len; i++)
    {
        uint8_t b = ((uint8_t)payload[i]) ^ mask[i & 3];
        wsClient.write(&b, 1);
    }
}

void wsSendPong()
{
    if (!wsConnected || !wsClient.connected()) return;
    uint8_t frame[2] = {0x8A, 0x00};
    wsClient.write(frame, 2);
}

String base64Encode16(const uint8_t *data)
{
    const char table[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

    char out[25];
    int o = 0;

    for (int i = 0; i < 15; i += 3)
    {
        uint32_t v = ((uint32_t)data[i] << 16) |
                     ((uint32_t)data[i + 1] << 8) |
                     data[i + 2];

        out[o++] = table[(v >> 18) & 63];
        out[o++] = table[(v >> 12) & 63];
        out[o++] = table[(v >> 6) & 63];
        out[o++] = table[v & 63];
    }

    uint32_t v = ((uint32_t)data[15] << 16);
    out[o++] = table[(v >> 18) & 63];
    out[o++] = table[(v >> 12) & 63];
    out[o++] = '=';
    out[o++] = '=';
    out[o] = 0;

    return String(out);
}

bool connectWebSocket()
{
    if (!INTEGRO_WS_ENABLED || !ethernetReady) return false;

    wsClient.stop();
    wsClient.setInsecure();
    wsClient.setTimeout(3000);

    if (!wsClient.connect(INTEGRO_WS_HOST, INTEGRO_WS_PORT)) return false;

    uint8_t rawKey[16];
    for (int i = 0; i < 16; i += 4)
    {
        uint32_t r = esp_random();
        rawKey[i] = r & 0xFF;
        rawKey[i + 1] = (r >> 8) & 0xFF;
        rawKey[i + 2] = (r >> 16) & 0xFF;
        rawKey[i + 3] = (r >> 24) & 0xFF;
    }

    String key = base64Encode16(rawKey);

    wsClient.print("GET ");
    wsClient.print(INTEGRO_WS_PATH);
    wsClient.println(" HTTP/1.1");
    wsClient.print("Host: ");
    wsClient.println(INTEGRO_WS_HOST);
    wsClient.println("Upgrade: websocket");
    wsClient.println("Connection: Upgrade");
    wsClient.println("Sec-WebSocket-Version: 13");
    wsClient.print("Sec-WebSocket-Key: ");
    wsClient.println(key);
    wsClient.println();

    unsigned long start = millis();
    String status;

    while (wsClient.connected() && millis() - start < 5000UL)
    {
        if (wsClient.available())
        {
            status = wsClient.readStringUntil('\n');
            status.trim();
            break;
        }
        delay(1);
    }

    if (!status.startsWith("HTTP/1.1 101"))
    {
        wsClient.stop();
        return false;
    }

    while (wsClient.connected() && millis() - start < 5000UL)
    {
        if (!wsClient.available()) { delay(1); continue; }
        String line = wsClient.readStringUntil('\n');
        line.trim();
        if (line.length() == 0) break;
    }

    wsConnected = true;
    return true;
}

void sendIntegroAck(const String &target, const String &command,
                    const String &status, const String &message = "")
{
    String json = "{\"type\":\"ack\",\"hub_id\":\"" + jsonEscape(String(HUB_ID)) +
                  "\",\"target\":\"" + jsonEscape(target) +
                  "\",\"command\":\"" + jsonEscape(command) +
                  "\",\"status\":\"" + jsonEscape(status) + "\"";

    if (message.length() > 0)
        json += ",\"message\":\"" + jsonEscape(message) + "\"";

    json += "}";
    wsSendText(json);
}

void queueIntegroSensorCommand(const String &target, const String &command,
                               const String &parameter, const String &value)
{
    String realId;

    for (int i = 0; i < MAX_SENSORS; i++)
    {
        if (!sensors[i].valid || !isManagedSensor(sensors[i].id)) continue;
        if (sensors[i].id == target || sensors[i].name == target)
        {
            realId = sensors[i].id;
            break;
        }
    }

    if (realId.length() == 0 && target.startsWith("sensor_") && isManagedSensor(target))
        realId = target;

    if (realId.length() == 0)
    {
        sendIntegroAck(target, command, "error", "sensor not found");
        return;
    }

    if (command == "get")
    {
        queueCommand(realId, "GET|" + realId);
        sendIntegroAck(target, command, "queued");
    }
    else if (command == "config")
    {
        queueCommand(realId, "CONFIG|" + realId);
        sendIntegroAck(target, command, "queued");
    }
    else if (command == "set" && parameter.length() > 0 && value.length() > 0)
    {
        queueCommand(realId, "SET|" + realId + "|" + parameter + "|" + value);
        sendIntegroAck(target, command, "queued");
    }
    else
        sendIntegroAck(target, command, "error", "invalid sensor command");
}

void handleIntegroCommand(const String &json)
{
    String hub = jsonGetString(json, "hub_id");
    if (hub.length() > 0 && hub != HUB_ID && hub != "*" && hub != "all") return;

    String target = jsonGetString(json, "target");
    if (target.length() == 0) target = jsonGetString(json, "sensor");

    String command = jsonGetString(json, "command");
    String parameter = jsonGetString(json, "parameter");
    String value = jsonGetRaw(json, "value");
    command.toLowerCase();

    if (target == "hub" || target == HUB_ID)
    {
        if (command == "update")
        {
            sendIntegroAck(target, command, "started");
            otaUpdateRequested = true;
        }
        else if (command == "restart")
        {
            sendIntegroAck(target, command, "ok");
            delay(50);
            ESP.restart();
        }
        else
            sendIntegroAck(target, command, "error", "unknown hub command");

        return;
    }

    if (command == "rename")
    {
        String realId;
        for (int i = 0; i < MAX_SENSORS; i++)
        {
            if (!sensors[i].valid || !isManagedSensor(sensors[i].id)) continue;
            if (sensors[i].id == target || sensors[i].name == target)
            {
                realId = sensors[i].id;
                break;
            }
        }

        if (realId.length() == 0 && target.startsWith("sensor_") && isManagedSensor(target))
            realId = target;

        String newName = jsonGetString(json, "name");
        if (newName.length() == 0) newName = value;

        if (realId.length() > 0 && renameSensor(realId, newName))
            sendIntegroAck(target, command, "ok");
        else
            sendIntegroAck(target, command, "error", "sensor not found");

        return;
    }

    queueIntegroSensorCommand(target, command, parameter, value);
}

void serviceWebSocket()
{
    if (!INTEGRO_WS_ENABLED || !ethernetReady) return;

    if (!wsConnected || !wsClient.connected())
    {
        wsConnected = false;
        if (millis() - lastWsConnectAttempt >= WS_RECONNECT_INTERVAL_MS)
        {
            lastWsConnectAttempt = millis();
            connectWebSocket();
        }
        return;
    }

    while (wsClient.available())
    {
        uint8_t h[2];
        if (wsClient.readBytes(h, 2) != 2)
        {
            wsConnected = false;
            wsClient.stop();
            return;
        }

        uint8_t opcode = h[0] & 0x0F;
        bool masked = h[1] & 0x80;
        uint64_t len = h[1] & 0x7F;

        if (len == 126)
        {
            uint8_t x[2];
            if (wsClient.readBytes(x, 2) != 2) return;
            len = ((uint16_t)x[0] << 8) | x[1];
        }
        else if (len == 127)
        {
            uint8_t x[8];
            if (wsClient.readBytes(x, 8) != 8) return;
            len = 0;
            for (int i = 0; i < 8; i++) len = (len << 8) | x[i];
        }

        if (len > 8192)
        {
            wsConnected = false;
            wsClient.stop();
            return;
        }

        uint8_t mask[4] = {0,0,0,0};
        if (masked && wsClient.readBytes(mask, 4) != 4) return;

        String payload;
        payload.reserve((size_t)len);

        for (uint64_t i = 0; i < len; i++)
        {
            int c = wsClient.read();
            if (c < 0) return;
            if (masked) c ^= mask[i & 3];
            payload += (char)c;
        }

        if (opcode == 0x8)
        {
            wsConnected = false;
            wsClient.stop();
            return;
        }

        if (opcode == 0x9)
        {
            wsSendPong();
            continue;
        }

        if (opcode == 0x1)
            handleIntegroCommand(payload);
    }
}

bool updateHubFromGitHub()
{
    if (strlen(HUB_FIRMWARE_URL) == 0) return false;

    NetworkClientSecure client;
    client.setInsecure();
    client.setTimeout(15000);

    HTTPClient https;
    if (!https.begin(client, HUB_FIRMWARE_URL)) return false;

    https.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS);
    int code = https.GET();

    if (code != HTTP_CODE_OK)
    {
        https.end();
        return false;
    }

    int contentLength = https.getSize();
    if (contentLength <= 0)
    {
        https.end();
        return false;
    }

    if (!Update.begin((size_t)contentLength))
    {
        https.end();
        return false;
    }

    NetworkClient *stream = https.getStreamPtr();
    size_t written = Update.writeStream(*stream);
    bool ok = written == (size_t)contentLength;

    if (ok) ok = Update.end(true);

    https.end();

    if (ok) ESP.restart();
    return ok;
}

// ============================================================
//              ПЕЧАТЬ ОЧЕРЕДИ КОМАНД
// ============================================================

void printQueue()
{
    Serial.println();
    Serial.println(
        "PENDING COMMANDS:"
    );

    bool found = false;

    for (
        int i = 0;
        i < MAX_COMMANDS;
        i++
    )
    {
        if (
            commandQueue[i].pending
        )
        {
            found = true;

            Serial.print(
                commandQueue[i].sensorId
            );

            Serial.print(
                " -> "
            );

            Serial.println(
                commandQueue[i].command
            );
        }
    }

    if (!found)
    {
        Serial.println(
            "Queue empty"
        );
    }
}

// ============================================================
//                ОТПРАВКА КОМАНДЫ
// ============================================================

bool sendCommandAndWait(
    String command,
    String expectedSensor,
    String *responseOut = nullptr
)
{
    //Serial.println();
    //Serial.println("================================");
    Serial.println("SENDING COMMAND");
    Serial.print("TX: ");
    Serial.println(command);
  
    int txState =
        radio.transmit(
            command.c_str(),
            command.length()
        );

    if (txState != RADIOLIB_ERR_NONE)
    {
        //Serial.print("TX ERROR: ");
        //Serial.println(txState);
        return false;
    }

    delay(5);

    String response;
    int rxState =
        radio.receive(
            response,
            COMMAND_RESPONSE_TIMEOUT_MS
        );

    if (rxState != RADIOLIB_ERR_NONE)
    {
        Serial.print("No valid response. RX state: ");
        Serial.println(rxState);
        return false;
    }

    response.trim();

    if (responseOut != nullptr)
        *responseOut = response;

    Serial.print("RAW RX: ");
    Serial.println(response);

    // Проверяем тип ответа.
    bool typeOK =
        response.startsWith(expectedSensor);

    if (!typeOK)
    {
        Serial.println("Unexpected response type.");
        return false;
    }

    // Проверяем ID датчика во втором поле.
    int p1 = response.indexOf('|');
    int p2 = response.indexOf('|', p1 + 1);

    if (p1 < 0)
    {
        Serial.println("Response has no sensor ID.");
        return false;
    }

    String responseSensor;

    if (p2 >= 0)
        responseSensor = response.substring(p1 + 1, p2);
    else
        responseSensor = response.substring(p1 + 1);

    responseSensor.trim();

    // expectedSensor здесь остаётся типом ответа (ACK| / CONFIG|).
    // Реальный ID проверяется вызывающей функцией через command.
    int cp1 = command.indexOf('|');
    int cp2 = command.indexOf('|', cp1 + 1);

    String commandSensor;

    if (cp1 >= 0)
    {
        if (cp2 >= 0)
            commandSensor = command.substring(cp1 + 1, cp2);
        else
            commandSensor = command.substring(cp1 + 1);
    }

    commandSensor.trim();

    if (responseSensor != commandSensor)
    {
        Serial.print("Wrong sensor response. Expected: ");
        Serial.print(commandSensor);
        Serial.print(" | Received: ");
        Serial.println(responseSensor);
        return false;
    }

    if (expectedSensor == "CONFIG|")
        Serial.println("CONFIG RECEIVED FROM CORRECT SENSOR");
    else
        Serial.println("ACK RECEIVED FROM CORRECT SENSOR");

    return true;
}

void processData(String packet)
{
    // --------------------------------------------------------
    // Ожидаемый формат:
    //
    // DATA|id|temp|hum|light|pt1000|battery
    // --------------------------------------------------------

    String parts[7];

    int start = 0;

    for (int i = 0; i < 6; i++)
    {
        int separator = packet.indexOf('|', start);

        if (separator < 0)
        {
            Serial.println("Invalid DATA packet: not enough fields");
            return;
        }

        parts[i] =
            packet.substring(
                start,
                separator
            );

        start = separator + 1;
    }

    // Последнее поле
    parts[6] =
        packet.substring(start);

    // --------------------------------------------------------
    // Проверяем тип пакета
    // --------------------------------------------------------

    if (parts[0] != "DATA")
    {
        Serial.println("Invalid packet type");
        return;
    }

    // --------------------------------------------------------
    // Защита battery от мусора
    //
    // Берём только нормальное числовое начало.
    // Например:
    //
    // 3.87
    // 3.8700?$...
    //
    // превратится в:
    //
    // 3.8700
    // --------------------------------------------------------

    String batteryClean = "";

    bool decimalFound = false;

    for (unsigned int i = 0; i < parts[6].length(); i++)
    {
        char c = parts[6][i];

        if (i == 0 && c == '-')
        {
            batteryClean += c;
            continue;
        }

        if (c >= '0' && c <= '9')
        {
            batteryClean += c;
            continue;
        }

        if (c == '.' && !decimalFound)
        {
            batteryClean += c;
            decimalFound = true;
            continue;
        }

        // Первый ненормальный символ — дальше не читаем
        break;
    }

    // --------------------------------------------------------
    // Данные
    // --------------------------------------------------------

    String id         = parts[1];

    if (!isManagedSensor(id))
        return;

    String temperature = parts[2];
    String humidity    = parts[3];
    String light       = parts[4];
    String pt1000      = parts[5];
    String battery     = batteryClean;

    // --------------------------------------------------------
    // Найти / создать датчик
    // --------------------------------------------------------

    unsigned long now = millis();

    int index =
        getOrCreateSensor(id);

    if (index < 0)
    {
        Serial.println(
            "Sensor table full"
        );

        return;
    }

    sensors[index].temperature =
        temperature.toFloat();

    sensors[index].humidity =
        humidity.toFloat();

    sensors[index].light =
        light.toFloat();

    sensors[index].pt1000 =
        pt1000.toFloat();

    sensors[index].battery =
        battery.toFloat();

    sensors[index].rssi =
        radio.getRSSI();

    sensors[index].snr =
        radio.getSNR();

    sensors[index].lastSeen = now;
    sensors[index].receivedUnix = currentUnixTime();
    sensors[index].receivedAt = formatReceivedAt();

    sensors[index].valid =
        true;

    updateDisplay();

    // Первый DATA запускает начальное выравнивание.
    if (INITIAL_ALIGNMENT_ENABLED && !initialAlignmentActive)
        startInitialAlignment(now);

    // Пока идёт первоначальная расстановка, монитор периода не вмешивается.
    bool alignmentHandled = handleInitialAlignment(index, now);

    if (!alignmentHandled)
    {
        // Контролируем только фактический интервал между DATA.
        // Первый DATA создаёт baseline и ничего не меняет.
        updateSleepPeriodMonitor(index, true, now);
    }

    // --------------------------------------------------------
    // Автоматический сбор.
    // Первый DATA запускает цикл; последующие отмечаются
    // как свежие данные текущего цикла.
    // --------------------------------------------------------
    if (AUTO_COLLECTION_ENABLED)
    {
        if (!collectionActive)
            startCollectionCycle();

        if (collectionActive && cycleExpected[index])
            cycleFresh[index] = true;
    }

    // --------------------------------------------------------
    // Вывод
    // --------------------------------------------------------

    Serial.println();
    Serial.println(
        "================================"
    );

    Serial.println(
        "DATA RECEIVED"
    );

    Serial.print(
        "ID:        "
    );

    Serial.println(id);

    Serial.print(
        "Temp:      "
    );

    Serial.print(
        sensors[index].temperature,
        2
    );

    Serial.println(
        " C"
    );

    Serial.print(
        "Humidity:  "
    );

    Serial.print(
        sensors[index].humidity,
        2
    );

    Serial.println(
        " %"
    );

    Serial.print(
        "Light:     "
    );

    Serial.print(
        sensors[index].light,
        1
    );

    Serial.println(
        " lux"
    );

    Serial.print(
        "PT1000:    "
    );

    Serial.print(
        sensors[index].pt1000,
        2
    );

    Serial.println(
        " C"
    );

    Serial.print(
        "Battery:   "
    );

    Serial.print(
        sensors[index].battery,
        3
    );

    Serial.println(
        " V"
    );

    Serial.print(
        "RSSI:      "
    );

    Serial.print(
        sensors[index].rssi,
        2
    );

    Serial.println(
        " dBm"
    );

    Serial.print(
        "SNR:       "
    );

    Serial.print(
        sensors[index].snr,
        2
    );

    Serial.println(
        " dB"
    );

    Serial.println(
        "================================"
    );

    // --------------------------------------------------------
    // Если есть команда для этого датчика — отправить
    // --------------------------------------------------------

    int commandIndex = findCommand(id);

    if (commandIndex >= 0)
    {
        String command =
            commandQueue[commandIndex].command;

        Serial.println();
        Serial.println(
            "SENDING QUEUED COMMAND"
        );

        Serial.print(
            "TX: "
        );

        Serial.println(command);

        String expected = "ACK|";

        if (command.startsWith("GET|") ||
            command.startsWith("CONFIG|"))
        {
            expected = "CONFIG|";
        }

        String response;
        bool confirmed =
            sendCommandAndWait(
                command,
                expected,
                &response
            );

        if (confirmed)
        {
            Serial.println(
                "Queued command confirmed."
            );

            String commandType = "set";
            if (command.startsWith("GET|")) commandType = "get";
            else if (command.startsWith("CONFIG|")) commandType = "config";

            wsSendText(
                "{\"type\":\"command_result\",\"hub_id\":\"" +
                jsonEscape(String(HUB_ID)) +
                "\",\"target\":\"" +
                jsonEscape(sensors[index].name) +
                "\",\"sensor_id\":\"" +
                jsonEscape(sensors[index].id) +
                "\",\"command\":\"" +
                jsonEscape(commandType) +
                "\",\"status\":\"ok\",\"response\":\"" +
                jsonEscape(response) +
                "\"}"
            );

            removeCommand(commandIndex);
        }
        else
        {
            Serial.println(
                "Queued command was NOT confirmed."
            );

            Serial.println(
                "It remains in the queue."
            );

            String commandType = "set";
            if (command.startsWith("GET|")) commandType = "get";
            else if (command.startsWith("CONFIG|")) commandType = "config";

            wsSendText(
                "{\"type\":\"command_result\",\"hub_id\":\"" +
                jsonEscape(String(HUB_ID)) +
                "\",\"target\":\"" +
                jsonEscape(sensors[index].name) +
                "\",\"sensor_id\":\"" +
                jsonEscape(sensors[index].id) +
                "\",\"command\":\"" +
                jsonEscape(commandType) +
                "\",\"status\":\"timeout\"}"
            );
        }
    }


}

// ============================================================
//                  ОБРАБОТКА SERIAL
// ============================================================

void handleSerialCommand(
    String input
)
{
    input.trim();

    if (input.length() == 0)
    {
        return;
    }

    // ========================================================
    // MANAGED SENSOR RANGE
    // sensors
    // sensors|1|12
    // ========================================================

    if (input == "sensors")
    {
        printManagedSensorRange();
        return;
    }

    if (input.startsWith("sensors|"))
    {
        int p1 = input.indexOf('|');
        int p2 = input.indexOf('|', p1 + 1);

        if (p1 < 0 || p2 < 0)
        {
            Serial.println("Invalid sensor range.");
            return;
        }

        int first = input.substring(p1 + 1, p2).toInt();
        int last = input.substring(p2 + 1).toInt();
        setManagedSensorRange(first, last);
        return;
    }

    // ========================================================
    // ETHERNET STATUS
    // ========================================================

    if (input == "eth")
    {
        if (!ethernetReady)
        {
            Serial.println("Ethernet: not connected");
            return;
        }

        Serial.print("Ethernet IP: ");
        Serial.println(ETH.localIP());
        return;
    }

    // ========================================================
    // LIST
    // ========================================================

    if (
        input == "list"
    )
    {
        printSensors();

        return;
    }

    // ========================================================
    // QUEUE
    // ========================================================

    if (
        input == "queue"
    )
    {
        printQueue();

        return;
    }

    // ========================================================
    // JSON
    // ========================================================

    if (
        input == "json"
    )
    {
        printJSON();

        return;
    }

    // ========================================================
    // HELP
    // ========================================================

    if (
        input == "help"
    )
    {
        Serial.println();
        Serial.println(
            "Commands:"
        );

        Serial.println(
            "sensors"
        );

        Serial.println(
            "sensors|1|12"
        );

        Serial.println(
            "eth"
        );

        Serial.println(
            "list"
        );

        Serial.println(
            "queue"
        );

        Serial.println(
            "json"
        );

        Serial.println(
            "time"
        );

        Serial.println(
            "time|2026-09-04|12:34:56"
        );

        Serial.println(
            "auto"
        );

        Serial.println(
            "schedule    (sleep period monitor status)"
        );

        Serial.println(
            "get|sensor_01"
        );

        Serial.println(
            "config|sensor_01"
        );

        Serial.println(
            "set|sensor_01|sleep|300"
        );

        Serial.println(
            "set|sensor_01|rref|998"
        );

        Serial.println(
            "set|sensor_01|vref|3.24"
        );

        Serial.println(
            "set|sensor_01|cal_gain|1.000000"
        );

        Serial.println(
            "set|sensor_01|cal_offset|0.000000"
        );

        Serial.println("rename|sensor_01|NewName");
        Serial.println("update");

        return;
    }

    // ========================================================
    // ========================================================
    // TIME
    // ========================================================

    if (input == "time")
    {
        printHubTime();
        return;
    }

    if (input.startsWith("time|"))
    {
        int p1=input.indexOf('|');
        int p2=input.indexOf('|',p1+1);
        if (p1<0 || p2<0)
        {
            Serial.println("Invalid time command.");
            Serial.println("Use: time|2026-09-04|12:34:56");
            return;
        }
        String datePart=input.substring(p1+1,p2);
        String timePart=input.substring(p2+1);
        datePart.trim(); timePart.trim();
        setHubTime(datePart,timePart);
        return;
    }

    // AUTO STATUS
    // ========================================================

    if (input == "auto")
    {
        Serial.println();
        Serial.print("Known sensors: ");
        int known = 0;
        for (int i = 0; i < MAX_SENSORS; i++)
            if (sensors[i].valid && isManagedSensor(sensors[i].id)) known++;
        Serial.println(known);

        Serial.print("Online sensors: ");
        Serial.println(countOnlineSensors());

        Serial.print("Collection active: ");
        Serial.println(collectionActive ? "YES" : "NO");

        if (collectionActive)
        {
            Serial.print("Collected this cycle: ");
            int n = 0;
            for (int i = 0; i < MAX_SENSORS; i++)
                if (cycleFresh[i]) n++;
            Serial.println(n);
        }
        return;
    }

    // ========================================================
    // SLEEP PERIOD MONITOR
    // ========================================================

    if (input == "schedule")
    {
        printSchedule();
        return;
    }

    // ========================================================
    // RENAME
    // rename|sensor_01|Улица
    // ========================================================

    if (input.startsWith("rename|"))
    {
        int p1 = input.indexOf('|');
        int p2 = input.indexOf('|', p1 + 1);

        if (p1 < 0 || p2 < 0)
        {
            Serial.println("Invalid rename command.");
            return;
        }

        String target = input.substring(p1 + 1, p2);
        String newName = input.substring(p2 + 1);
        target.trim();
        newName.trim();

        String realId;
        for (int i = 0; i < MAX_SENSORS; i++)
        {
            if (!sensors[i].valid || !isManagedSensor(sensors[i].id)) continue;
            if (sensors[i].id == target || sensors[i].name == target)
            {
                realId = sensors[i].id;
                break;
            }
        }

        if (realId.length() == 0 && target.startsWith("sensor_") && isManagedSensor(target))
            realId = target;

        if (realId.length() > 0 && renameSensor(realId, newName))
            Serial.println("Sensor renamed.");
        else
            Serial.println("Sensor not found.");

        return;
    }

    if (input == "update")
    {
        otaUpdateRequested = true;
        return;
    }

    // ========================================================
    // GET
    // ========================================================

    if (
        input.startsWith(
            "get|"
        )
    )
    {
        String id =
            input.substring(4);

        id.trim();

        String command =
            "GET|" + id;

        queueCommand(
            id,
            command
        );

        return;
    }

    // ========================================================
    // CONFIG
    // ========================================================

    if (
        input.startsWith(
            "config|"
        )
    )
    {
        String id =
            input.substring(7);

        id.trim();

        String command =
            "CONFIG|" + id;

        queueCommand(
            id,
            command
        );

        return;
    }

    // ========================================================
    // SET
    //
    // set|sensor_01|sleep|300
    // set|sensor_01|rref|998
    // set|sensor_01|vref|3.24
    // set|sensor_01|cal_gain|1.0
    // set|sensor_01|cal_offset|0.0
    // ========================================================

    if (
        input.startsWith(
            "set|"
        )
    )
    {
        int p1 =
            input.indexOf('|');

        int p2 =
            input.indexOf(
                '|',
                p1 + 1
            );

        int p3 =
            input.indexOf(
                '|',
                p2 + 1
            );

        if (
            p1 < 0 ||
            p2 < 0 ||
            p3 < 0
        )
        {
            Serial.println(
                "Invalid SET command"
            );

            return;
        }

        String id =
            input.substring(
                p1 + 1,
                p2
            );

        String parameter =
            input.substring(
                p2 + 1,
                p3
            );

        String value =
            input.substring(
                p3 + 1
            );

        id.trim();
        parameter.trim();
        value.trim();

        String command =
            "SET|" +
            id +
            "|" +
            parameter +
            "|" +
            value;

        queueCommand(
            id,
            command
        );

        return;
    }

    Serial.println(
        "Unknown command. Type 'help'."
    );
}

// ============================================================
//                         SETUP
// ============================================================

void setup()
{
    Serial.begin(
        115200
    );

    delay(100);

    Serial.println();
    Serial.println(
        "================================"
    );

    Serial.println(
        "          LORA HUB"
    );

    Serial.println(
        "================================"
    );

    loadManagedSensorRange();

    // --------------------------------------------------------
    // OLED / Vext для Heltec WiFi LoRa 32 V3.2
    // GPIO36 управляет питанием Vext/OLED.
    // Для V3.2 LOW = Vext ON.
    // Делаем это ДО heltec_setup(), чтобы OLED уже был запитан
    // в момент display.init().
    // --------------------------------------------------------
    pinMode(36, OUTPUT);
    digitalWrite(36, LOW);

    // --------------------------------------------------------
    // Heltec
    // --------------------------------------------------------

    heltec_setup();

    // --------------------------------------------------------
    // LoRa
    // --------------------------------------------------------

    RADIOLIB_OR_HALT(
        radio.begin()
    );

    RADIOLIB_OR_HALT(
        radio.setFrequency(
            FREQUENCY
        )
    );

    RADIOLIB_OR_HALT(
        radio.setBandwidth(
            BANDWIDTH
        )
    );

    RADIOLIB_OR_HALT(
        radio.setSpreadingFactor(
            SPREADING_FACTOR
        )
    );

    RADIOLIB_OR_HALT(
        radio.setOutputPower(
            TRANSMIT_POWER
        )
    );

    initEthernet();

    // --------------------------------------------------------
    // Очистка таблиц
    // --------------------------------------------------------

    for (
        int i = 0;
        i < MAX_SENSORS;
        i++
    )
    {
        sensors[i].valid = false;
    }

    for (
        int i = 0;
        i < MAX_COMMANDS;
        i++
    )
    {
        commandQueue[i].pending = false;
        commandQueue[i].sensorId = "";
        commandQueue[i].command = "";
    }

    resetInitialAlignment();

    syncInternetTime();
    lastInternetCheck = millis();
    updateDisplay();
    lastDisplayUpdate = millis();

    Serial.println(
        "LoRa hub ready."
    );

    Serial.println(
        "Type 'help' for commands."
    );
}

// ============================================================
//                          LOOP
// ============================================================

void loop()
{
    heltec_loop();

    if (W5500_ENABLED)
    {
        bool linkNow = ETH.linkUp();

        if (!linkNow)
        {
            ethernetReady = false;
            internetAvailable = false;
            wsConnected = false;
            wsClient.stop();
        }
        else
        {
            ethernetReady = true;

            // Периодически проверяем именно доступ в интернет через NTP,
            // а не только наличие физического Ethernet-link.
            if ((millis() - lastInternetCheck) >= INTERNET_CHECK_INTERVAL_MS)
            {
                lastInternetCheck = millis();
                syncInternetTime();
            }

        }
    }

    if ((millis() - lastDisplayUpdate) >= 1000UL)
    {
        updateDisplay();
        lastDisplayUpdate = millis();
    }

    serviceWebSocket();

    if (otaUpdateRequested)
    {
        otaUpdateRequested = false;
        updateHubFromGitHub();
    }

    // --------------------------------------------------------
    // SERIAL
    // --------------------------------------------------------

    if (Serial.available())
    {
        String input =
            Serial.readStringUntil(
                '\n'
            );

        handleSerialCommand(
            input
        );
    }

    // --------------------------------------------------------
    // RX DATA
    //
    // Полностью синхронный режим.
    // Никаких rxFlag / callback / startReceive().
    // --------------------------------------------------------

    String packet;

    int state =
        radio.receive(
            packet,
            HUB_RX_TIMEOUT_MS
        );

    if (
        state == RADIOLIB_ERR_NONE
    )
    {
        packet.trim();

        Serial.println();
        Serial.print(
            "RAW RX: "
        );

        Serial.println(
            packet
        );

        if (
            packet.startsWith(
                "DATA|"
            )
        )
        {
            processData(
                packet
            );
        }
        else
        {
            Serial.println(
                "Unknown packet"
            );
        }
    }

    serviceInitialAlignment(millis());

    // Автоматически закрываем цикл, когда все доступные
    // датчики ответили или вышло время ожидания.
    serviceCollection();
}
