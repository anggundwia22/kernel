#include <Arduino.h>
#include <DHT.h>
#include <RTClib.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>

// ========== PIN CONFIGURATION ==========
#define DHT_PIN_1               4
#define DHT_PIN_2               15
#define DHT_TYPE                DHT21

#define RELAY_OPEN_PLUS_PIN     27
#define RELAY_OPEN_MINUS_PIN    26
#define RELAY_CLOSE_PLUS_PIN    25
#define RELAY_CLOSE_MINUS_PIN   33
#define LED_OPEN_PIN            32

// ========== RELAY CONTROL ==========
#define RELAY_ACTIVE_LOW        true

// ========== SYSTEM CONFIGURATION ==========
#define CONFIG_FILE             "/config.json"
#define LOG_FILE                "/silo_log.json"
#define LOG_INTERVAL_DEFAULT_MS 3600000UL   // 1 jam
#define RELAY_CHECK_DEFAULT_MS  3600000UL   // 1 jam
#define WIFI_CHECK_INTERVAL     5000UL
#define WIFI_TIMEOUT            15

/** Stroke mekanik linear (batas atas aman untuk Y_max di UI) */
#define ACT_STROKE_PHYSICAL_CM  50.0f

// ========== GLOBAL OBJECTS ==========
DHT         dht1(DHT_PIN_1, DHT_TYPE);
DHT         dht2(DHT_PIN_2, DHT_TYPE);
RTC_DS3231  rtc;
WebServer   server(80);

// ========== WIFI / NTP CONFIGURATION ==========
struct WiFiConfig
{
    const char* ssid        = "KernelSilo";
    const char* password    = "12345678";
} wifiCfg;

// ========== SENSOR DATA ==========
struct SensorData
{
    float temp1     = 0.0f;
    float hum1      = 0.0f;
    float temp2     = 0.0f;
    float hum2      = 0.0f;
    float rawTemp1  = 0.0f;
    float rawHum1   = 0.0f;
    float rawTemp2  = 0.0f;
    float rawHum2   = 0.0f;
    String timestamp = "-";
} sensorData;

// ========== CONFIGURATION (persisted to CONFIG_FILE) ==========
// Semua setting yang disimpan ke LittleFS JSON digabung dalam satu struct
struct Config
{
    // Calibration
    float temp1Offset = 0.0f;
    float hum1Offset  = 0.0f;
    float temp2Offset = 0.0f;
    float hum2Offset  = 0.0f;
    // Actuator settings
    float hMin       = 0.0f;
    float hMax       = 10.0f;
    float yMin       = 0.0f;
    float yMax       = 30.0f;
    float speedCmS   = 1.724f;
    float maxDurS    = 11.6f;
    float ptTotalCm  = 50.0f;
    float dtTotalS   = 29.0f;
    float yCurrent   = 0.0f;   // posisi aktuator terakhir
    // System settings
    unsigned long logIntervalMs  = LOG_INTERVAL_DEFAULT_MS;
    unsigned long relayCheckMs   = RELAY_CHECK_DEFAULT_MS;
    /** true = kontrol stem dari RH otomatis; false = hanya manual dari web */
    bool          modeAuto       = true;
} config;

// ========== ACTUATOR STATE ==========
enum ActuatorState : uint8_t
{
    ACT_IDLE = 0,
    ACT_SETTLE,
    ACT_MOVE
};

struct ActuatorControl
{
    ActuatorState state      = ACT_IDLE;
    unsigned long phaseEnd   = 0;
    unsigned long moveDurMs  = 0;
    int8_t        dir        = 0;
    float         pendingY   = 0.0f;
    float         yCurrent   = 0.0f;
    float         yTarget    = 0.0f;
    float         yPrevious  = 0.0f;
    float         lastDurS   = 0.0f;    // durasi gerak terakhir (detik)
    bool          valveOpen  = false;
    String        valveStatus = "0.0 cm";
} actuator;

// ========== SYSTEM SETTINGS (runtime only, not persisted) ==========
struct SystemSettings
{
    unsigned long lastLogTime     = 0;
    /** Waktu terakhir pembacaan sensor + (mode auto) cek aktuator; pakai relayCheckMs. */
    unsigned long lastMeasurementMs = 0;
    unsigned long lastWifiCheck   = 0;
    bool          firstLogDone    = false;
    /** RH terklamp [hMin,hMax] dari siklus pemeriksaan sebelumnya (mode auto). */
    float         hTemporer       = NAN;
} sysSettings;

// ========== SYSTEM STATUS ==========
struct SystemStatus
{
    bool rtcOk        = false;
    bool littleFsOk   = false;
    bool wifiSta      = false;
    bool webServerOn  = false;
} sysStatus;

// ========== FUNCTION PROTOTYPES ==========

// --- Config (LittleFS JSON) ---
void createDefaultConfig();
bool loadConfig();
bool saveConfig();

// --- Relay / Actuator ---
void relayOn(uint8_t pin);
void relayOff(uint8_t pin);
void valveRelaysIdle();
void updateValveStatusFromPosition();
bool actuatorIsBusy();
void actuatorService();
bool startActuatorMove(int8_t dir, unsigned long durationMs, float newY);
float manualOpenCloseSpeedCmS();
bool manualValveOn(float pd);
bool openValve(float cmRequested);
bool closeValve(float cmRequested);
void updateActuatorFromHumidity();

// --- Sensor / Helper ---
float applyCalibration(float value, float offset);
void readDHT();
String getTimestamp();

// --- WiFi / NTP ---
bool connectWiFi();
bool setRtcFromYmdHms(int y, int mo, int d, int h, int mi, int s);
void checkWiFi();

// --- Setup helpers ---
void initRTC();
void setupWiFi();

// --- Log / LittleFS ---
bool initLittleFS();
bool writeLog(String event = "SCHEDULED", const char* dir = nullptr);
bool writeLogManual(float yBefore, float yInput, int8_t dir);
void clearLog();
int  countLogEntries();

// --- Web Server ---
void setupWebServer();


// ============================================================
//  CONFIG — LittleFS JSON
// ============================================================

void createDefaultConfig()
{
    File file = LittleFS.open(CONFIG_FILE, FILE_WRITE);
    if (!file) {
        Serial.println("[Config] ERROR: Gagal buat config default!");
        return;
    }
    JsonDocument doc;
    // Calibration
    doc["temp1Offset"] = config.temp1Offset;
    doc["hum1Offset"]  = config.hum1Offset;
    doc["temp2Offset"] = config.temp2Offset;
    doc["hum2Offset"]  = config.hum2Offset;
    // Actuator
    doc["hMin"]       = config.hMin;
    doc["hMax"]       = config.hMax;
    doc["yMin"]       = config.yMin;
    doc["yMax"]       = config.yMax;
    doc["speedCmS"]   = config.speedCmS;
    doc["maxDurS"]    = config.maxDurS;
    doc["ptTotalCm"]  = config.ptTotalCm;
    doc["dtTotalS"]   = config.dtTotalS;
    doc["yCurrent"]   = config.yCurrent;
    // System
    doc["logIntervalMs"]  = config.logIntervalMs;
    doc["relayCheckMs"]   = config.relayCheckMs;
    doc["modeAuto"]       = config.modeAuto;
    serializeJson(doc, file);
    file.close();
    Serial.println("[Config] Default config dibuat.");
}

bool loadConfig()
{
    File file = LittleFS.open(CONFIG_FILE, FILE_READ);
    if (!file) {
        Serial.println("[Config] File tidak ditemukan, gunakan default.");
        return false;
    }
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, file);
    file.close();
    if (err) {
        Serial.printf("[Config] Parse error: %s\n", err.c_str());
        return false;
    }
    // Calibration
    config.temp1Offset = doc["temp1Offset"] | config.temp1Offset;
    config.hum1Offset  = doc["hum1Offset"]  | config.hum1Offset;
    config.temp2Offset = doc["temp2Offset"] | config.temp2Offset;
    config.hum2Offset  = doc["hum2Offset"]  | config.hum2Offset;
    // Actuator
    config.hMin      = doc["hMin"]      | config.hMin;
    config.hMax      = doc["hMax"]      | config.hMax;
    config.yMin      = doc["yMin"]      | config.yMin;
    config.yMax      = doc["yMax"]      | config.yMax;
    config.speedCmS  = doc["speedCmS"]  | config.speedCmS;
    config.maxDurS   = doc["maxDurS"]   | config.maxDurS;
    config.ptTotalCm = doc["ptTotalCm"] | config.ptTotalCm;
    config.dtTotalS  = doc["dtTotalS"]  | config.dtTotalS;
    config.yCurrent  = doc["yCurrent"]  | config.yCurrent;
    // System
    config.logIntervalMs = doc["logIntervalMs"] | config.logIntervalMs;
    config.relayCheckMs  = doc["relayCheckMs"]  | config.relayCheckMs;
    if (doc["modeAuto"].is<bool>())
        config.modeAuto = doc["modeAuto"].as<bool>();

    // Validasi dan clamp actuator
    if (config.hMax <= config.hMin) config.hMax = config.hMin + 0.1f;
    if (config.yMax < config.yMin) { float t = config.yMin; config.yMin = config.yMax; config.yMax = t; }
    if (config.yMax > ACT_STROKE_PHYSICAL_CM) config.yMax = ACT_STROKE_PHYSICAL_CM;
    if (config.speedCmS  < 0.05f) config.speedCmS = 0.05f;
    if (config.maxDurS   < 0.5f)  config.maxDurS  = 0.5f;
    if (config.ptTotalCm < 1.0f)  config.ptTotalCm = 1.0f;
    if (config.ptTotalCm > ACT_STROKE_PHYSICAL_CM) config.ptTotalCm = ACT_STROKE_PHYSICAL_CM;
    if (config.dtTotalS  < 0.2f)  config.dtTotalS = 0.2f;
    if (config.dtTotalS  > 600.0f) config.dtTotalS = 600.0f;
    if (config.yCurrent  < config.yMin) config.yCurrent = config.yMin;
    if (config.yCurrent  > config.yMax) config.yCurrent = config.yMax;
    // Clamp system settings
    if (config.logIntervalMs < 1000UL)       config.logIntervalMs = 1000UL;
    if (config.logIntervalMs > 86400000UL)   config.logIntervalMs = 86400000UL;
    if (config.relayCheckMs  < 5000UL)       config.relayCheckMs  = 5000UL;
    if (config.relayCheckMs  > 86400000UL)   config.relayCheckMs  = 86400000UL;

    Serial.println("[Config] Konfigurasi dimuat dari LittleFS.");

    // Sync posisi aktuator runtime dari config
    actuator.yCurrent = config.yCurrent;
    actuator.yTarget  = config.yCurrent;
    return true;
}

bool saveConfig()
{
    if (!sysStatus.littleFsOk) return false;
    File file = LittleFS.open(CONFIG_FILE, FILE_WRITE);
    if (!file) {
        Serial.println("[Config] ERROR: Gagal buka file config untuk disimpan!");
        return false;
    }
    JsonDocument doc;
    doc["temp1Offset"] = config.temp1Offset;
    doc["hum1Offset"]  = config.hum1Offset;
    doc["temp2Offset"] = config.temp2Offset;
    doc["hum2Offset"]  = config.hum2Offset;
    doc["hMin"]        = config.hMin;
    doc["hMax"]        = config.hMax;
    doc["yMin"]        = config.yMin;
    doc["yMax"]        = config.yMax;
    doc["speedCmS"]    = config.speedCmS;
    doc["maxDurS"]     = config.maxDurS;
    doc["ptTotalCm"]   = config.ptTotalCm;
    doc["dtTotalS"]    = config.dtTotalS;
    doc["yCurrent"]    = config.yCurrent;
    doc["logIntervalMs"]  = config.logIntervalMs;
    doc["relayCheckMs"]   = config.relayCheckMs;
    doc["modeAuto"]       = config.modeAuto;
    serializeJson(doc, file);
    file.close();
    Serial.println("[Config] Konfigurasi disimpan ke LittleFS.");
    return true;
}

// ============================================================
//  RELAY
// ============================================================

void relayOn(uint8_t pin)
{
    digitalWrite(pin, RELAY_ACTIVE_LOW ? LOW : HIGH);
}

void relayOff(uint8_t pin)
{
    digitalWrite(pin, RELAY_ACTIVE_LOW ? HIGH : LOW);
}

/** Semua koil relay valve off (aman saat boot / antar siklus). */
void valveRelaysIdle()
{
    relayOff(RELAY_OPEN_PLUS_PIN);
    relayOff(RELAY_OPEN_MINUS_PIN);
    relayOff(RELAY_CLOSE_PLUS_PIN);
    relayOff(RELAY_CLOSE_MINUS_PIN);
}

void updateValveStatusFromPosition()
{
    char buf[24];
    snprintf(buf, sizeof(buf), "%.1f cm", actuator.yCurrent);
    actuator.valveStatus = String(buf);
    actuator.valveOpen   = (actuator.yCurrent > config.yMin + 0.05f);
    digitalWrite(LED_OPEN_PIN, (actuator.yCurrent > 0.05f) ? LOW : HIGH);
}

// ============================================================
//  ACTUATOR STATE MACHINE
// ============================================================

bool actuatorIsBusy()
{
    return actuator.state != ACT_IDLE;
}

/** Jalankan di setiap iterasi loop — non-blocking. */
void actuatorService()
{
    if (actuator.state == ACT_IDLE) return;

    unsigned long now = millis();
    if (now < actuator.phaseEnd) return;

    if (actuator.state == ACT_SETTLE)
    {
        if (actuator.dir > 0) {
            relayOn(RELAY_OPEN_PLUS_PIN);
            relayOn(RELAY_OPEN_MINUS_PIN);
        } else if (actuator.dir < 0) {
            relayOn(RELAY_CLOSE_PLUS_PIN);
            relayOn(RELAY_CLOSE_MINUS_PIN);
        }
        actuator.phaseEnd = now + actuator.moveDurMs;
        actuator.state    = ACT_MOVE;
        return;
    }

    if (actuator.state == ACT_MOVE)
    {
        valveRelaysIdle();
        actuator.yCurrent = actuator.pendingY;
        if (actuator.yCurrent < config.yMin) actuator.yCurrent = config.yMin;
        if (actuator.yCurrent > config.yMax) actuator.yCurrent = config.yMax;
        config.yCurrent = actuator.yCurrent;  // sync ke config sebelum disimpan
        saveConfig();
        updateValveStatusFromPosition();
        actuator.state = ACT_IDLE;
        Serial.printf("[ACT] Posisi → %.2f cm (target UI %.2f)\n",
                      actuator.yCurrent, actuator.yTarget);
    }
}

bool startActuatorMove(int8_t dir, unsigned long durationMs, float newY)
{
    if (actuator.state != ACT_IDLE) return false;
    if (durationMs < 50) durationMs = 50;

    actuator.yPrevious  = actuator.yCurrent;
    actuator.dir        = dir;
    actuator.moveDurMs  = durationMs;
    actuator.lastDurS   = durationMs / 1000.0f;
    actuator.pendingY   = newY;
    valveRelaysIdle();
    actuator.state      = ACT_SETTLE;
    actuator.phaseEnd   = millis() + 500;
    return true;
}

float manualOpenCloseSpeedCmS()
{
    if (config.dtTotalS < 0.05f) return 1.0f;
    return config.ptTotalCm / config.dtTotalS;
}

/**
 * Satu tombol ON: pd = posisi diinginkan (cm).
 * x = -((Y*k) - (k*pd)) dengan k = DT/PT → tanda x = tanda (pd - Y).
 * Durasi gerak = |pd - Y| * (DT/PT).
 */
bool manualValveOn(float pd)
{
    if (actuatorIsBusy()) return false;

    float y   = actuator.yCurrent;
    float yLogBefore = y;
    float PT  = config.ptTotalCm;
    float DT  = config.dtTotalS;
    if (PT < 1.0f) PT = 1.0f;
    if (DT < 0.05f) DT = 0.05f;

    float k   = DT / PT;
    float pdc = pd;
    if (pdc < config.yMin) pdc = config.yMin;
    if (pdc > config.yMax) pdc = config.yMax;

    float x = -1.0f * ((y * k) - (k * pdc));
    const float eps = 0.05f;
    if (fabsf(pdc - y) < eps) return false;

    int8_t dir = 0;
    if (x > 0)      dir = 1;
    else if (x < 0) dir = -1;
    else             return false;

    float deltaCm = pdc - y;
    float durS    = fabsf(deltaCm) * (DT / PT);
    unsigned long ms = (unsigned long)(durS * 1000.0f + 0.5f);
    if (ms < 80) ms = 80;

    actuator.yTarget = pdc;
    if (!startActuatorMove(dir, ms, pdc)) return false;

    Serial.printf("[ACT] ON  PD=%.2f cm  Y=%.2f  x=%.4f  dur=%.2f s  %s\n",
                  pdc, y, x, durS, dir > 0 ? "OPEN" : "CLOSE");
    if (sysStatus.littleFsOk) writeLogManual(yLogBefore, pdc, dir);
    return true;
}

bool openValve(float cmRequested)
{
    float maxSpan = config.yMax - actuator.yCurrent;
    if (maxSpan <= 0.01f) return false;

    float cm  = (cmRequested > 0.01f) ? fminf(cmRequested, maxSpan)
                                       : fminf(config.ptTotalCm, maxSpan);
    float spd = manualOpenCloseSpeedCmS();
    if (spd < 0.01f) spd = 0.01f;
    unsigned long ms = (unsigned long)(cm / spd * 1000.0f + 0.5f);
    if (ms < 80) ms = 80;
    if (!startActuatorMove(1, ms, actuator.yCurrent + cm)) return false;

    Serial.printf("[ACT] Manual BUKA %.2f cm → %lu ms (%.3f cm/s)\n", cm, ms, spd);
    if (sysStatus.littleFsOk) writeLog("VALVE_OPEN", "buka");
    return true;
}

bool closeValve(float cmRequested)
{
    float maxSpan = actuator.yCurrent - config.yMin;
    if (maxSpan <= 0.01f) return false;

    float cm  = (cmRequested > 0.01f) ? fminf(cmRequested, maxSpan)
                                       : fminf(config.ptTotalCm, maxSpan);
    float spd = manualOpenCloseSpeedCmS();
    if (spd < 0.01f) spd = 0.01f;
    unsigned long ms = (unsigned long)(cm / spd * 1000.0f + 0.5f);
    if (ms < 80) ms = 80;
    if (!startActuatorMove(-1, ms, actuator.yCurrent - cm)) return false;

    Serial.printf("[ACT] Manual TUTUP %.2f cm → %lu ms (%.3f cm/s)\n", cm, ms, spd);
    if (sysStatus.littleFsOk) writeLog("VALVE_CLOSE", "tutup");
    return true;
}

// --- MODE AUTO (RH → aktuator, analog dengan rumus manual) ---
// hTemporer = RH siklus lalu, diklamp ke [hMin, hMax] (nilai > hMax disimpan sebagai hMax).
// H untuk rumus = RH sekarang diklamp ke [hMin, hMax] (nilai > hMax diperlakukan sebagai hMax).
// k = maxDurS / hMax,  x = -((H*k) - (hTemporer*k)). Tanda x: x>0 → buka ke yMax, x<0 → tutup ke yMin.
// Durasi gerak = |x| (maks maxDurS). Manual & auto: stroke min = yMin (tutup), stroke maks = yMax (buka).
// void updateActuatorFromHumidity()
// {
//     if (actuatorIsBusy()) return;

//     float H = sensorData.hum1;
//     if (isnan(H)) {
//         Serial.println("[ACT-AUTO] RH tidak valid, lewati");
//         return;
//     }

//     const float hMin    = config.hMin;
//     const float hMax    = config.hMax;
//     const float yMax    = config.yMax;
//     const float yMin    = config.yMin;
//     const float maxDurS = config.maxDurS;

//     if (maxDurS <= 0.0f || hMax <= hMin || yMax <= yMin + 0.01f) {
//         Serial.println("[ACT-AUTO] Parameter auto tidak valid, lewati");
//         return;
//     }

//     float Hc = fminf(fmaxf(H, hMin), hMax);

//     if (isnan(sysSettings.hTemporer)) {
//         sysSettings.hTemporer = Hc;
//         Serial.printf("[ACT-AUTO] Init hTemporer=%.2f%%\n", Hc);
//         return;
//     }

//     float hPrev = sysSettings.hTemporer;
//     float k     = maxDurS / hMax;
//     // H dalam rumus = RH sekarang setelah klamp [hMin,hMax] (sama seperti nilai disimpan di hTemporer).
//     float x     = -1.0f * ((Hc * k) - (hPrev * k));

//     const float rhEps = 0.08f;
//     if (fabsf(Hc - hPrev) < rhEps) {
//         sysSettings.hTemporer = Hc;
//         return;
//     }

//     int8_t dir = 0;
//     if (x > 0.0f)      dir = 1;
//     else if (x < 0.0f) dir = -1;
//     else {
//         sysSettings.hTemporer = Hc;
//         return;
//     }

//     float durS = fabsf(x);
//     if (durS > maxDurS) durS = maxDurS;
//     unsigned long ms = (unsigned long)(durS * 1000.0f + 0.5f);
//     if (ms < 80) {
//         sysSettings.hTemporer = Hc;
//         return;
//     }

//     float yTgt = (dir > 0) ? yMax : yMin;
//     actuator.yTarget = yTgt;
//     Serial.printf("[ACT-AUTO] H=%.2f%% (pakai %.2f%%) hTemporer=%.2f%% k=%.4f x=%.4f dur=%.2fs -> %s target=%.2f cm\n",
//                   H, Hc, hPrev, k, x, durS, dir > 0 ? "OPEN" : "CLOSE", yTgt);

//     if (!startActuatorMove(dir, ms, yTgt)) {
//         Serial.println("[ACT-AUTO] Gerak ditolak (masih sibuk)");
//         return;
//     }
//     sysSettings.hTemporer = Hc;
//     if (sysStatus.littleFsOk) writeLog("AUTO_MOVE");
// }

void updateActuatorFromHumidity()
{
    if (actuatorIsBusy()) return;

    float H = sensorData.hum1;
    if (isnan(H)) {
        Serial.println("[ACT-AUTO] RH tidak valid, lewati");
        return;
    }

    const float hMin    = config.hMin;
    const float hMax    = config.hMax;
    const float yMax    = config.yMax;
    const float yMin    = config.yMin;
    const float maxDurS = config.maxDurS;

    if (maxDurS <= 0.0f || hMax <= hMin || yMax <= yMin + 0.01f) {
        Serial.println("[ACT-AUTO] Parameter auto tidak valid, lewati");
        return;
    }

    // Clamp RH
    float Hc = fminf(fmaxf(H, hMin), hMax);

    // Init pertama
    if (isnan(sysSettings.hTemporer)) {
        sysSettings.hTemporer = Hc;
        Serial.printf("[ACT-AUTO] Init hTemporer=%.2f%%\n", Hc);
        return;
    }

    float hPrev = sysSettings.hTemporer;

    // 🔥 PERBAIKAN DI SINI
    float k = maxDurS / hMax;
    float x = (hPrev - Hc) * k;

    // Deadband biar tidak jitter
    const float rhEps = 0.08f;
    if (fabsf(Hc - hPrev) < rhEps) {
        sysSettings.hTemporer = Hc;
        return;
    }

    int8_t dir = 0;
    if (x > 0.0f)      dir = 1;   // buka
    else if (x < 0.0f) dir = -1;  // tutup
    else {
        sysSettings.hTemporer = Hc;
        return;
    }

    float durS = fabsf(x);
    if (durS > maxDurS) durS = maxDurS;

    unsigned long ms = (unsigned long)(durS * 1000.0f + 0.5f);

    if (ms < 80) {
        sysSettings.hTemporer = Hc;
        return;
    }

    float yTgt = (dir > 0) ? yMax : yMin;
    actuator.yTarget = yTgt;

    Serial.printf(
        "[ACT-AUTO] H=%.2f -> %.2f | prev=%.2f | x=%.4f | dur=%.2fs | %s\n",
        H, Hc, hPrev, x, durS, dir > 0 ? "OPEN" : "CLOSE"
    );

    if (!startActuatorMove(dir, ms, yTgt)) {
        Serial.println("[ACT-AUTO] Gerak ditolak (masih sibuk)");
        return;
    }

    sysSettings.hTemporer = Hc;

    if (sysStatus.littleFsOk)
        writeLog("AUTO_MOVE", dir > 0 ? "buka" : "tutup");
}

// ============================================================
//  SENSOR & HELPER
// ============================================================

float applyCalibration(float value, float offset)
{
    if (isnan(value)) return value;
    return value + offset;
}

void readDHT()
{
    sensorData.rawTemp1 = dht1.readTemperature();
    sensorData.rawHum1  = dht1.readHumidity();
    sensorData.rawTemp2 = dht2.readTemperature();
    sensorData.rawHum2  = dht2.readHumidity();

    sensorData.temp1 = applyCalibration(sensorData.rawTemp1, config.temp1Offset);
    sensorData.hum1  = applyCalibration(sensorData.rawHum1,  config.hum1Offset);
    sensorData.temp2 = applyCalibration(sensorData.rawTemp2, config.temp2Offset);
    sensorData.hum2  = applyCalibration(sensorData.rawHum2,  config.hum2Offset);
}

String getTimestamp()
{
    if (!sysStatus.rtcOk)
        return String("RTC tidak tersedia");
    DateTime now = rtc.now();
    char buf[20];
    snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d",
             now.year(), now.month(), now.day(),
             now.hour(), now.minute(), now.second());
    return String(buf);
}

// ============================================================
//  WIFI & NTP
// ============================================================

bool connectWiFi()
{
    if (WiFi.softAP(wifiCfg.ssid, wifiCfg.password)) {
        Serial.printf("[WiFi AP] SSID=%s → http://%s\n",
                      wifiCfg.ssid, WiFi.softAPIP().toString().c_str());
        return true;
    }
    Serial.println("[WiFi AP] ERROR: gagal start hotspot.");
    return false;
}

bool setRtcFromYmdHms(int y, int mo, int d, int h, int mi, int s)
{
    if (!sysStatus.rtcOk) return false;
    if (y < 2020 || mo < 1 || mo > 12 || d < 1 || d > 31 ||
        h < 0 || h > 23 || mi < 0 || mi > 59 || s < 0 || s > 59)
        return false;
    rtc.adjust(DateTime(y, mo, d, h, mi, s));
    return true;
}

void checkWiFi()
{
    // AP mode: pastikan hotspot tetap hidup.
    if (WiFi.getMode() != WIFI_AP && WiFi.getMode() != WIFI_AP_STA) {
        Serial.println("[WiFi AP] mode berubah, restart hotspot...");
        WiFi.mode(WIFI_AP);
    }
    if (WiFi.softAPgetStationNum() >= 0) {
        if (!sysStatus.wifiSta) {
            sysStatus.wifiSta = true;
            Serial.printf("[WiFi AP] aktif → http://%s\n",
                          WiFi.softAPIP().toString().c_str());
        }
        return;
    }
    sysStatus.wifiSta = connectWiFi();
}

void initRTC()
{
    Wire.begin();
    if (!rtc.begin()) {
        sysStatus.rtcOk = false;
        Serial.println("[RTC] Modul tidak terpasang / tidak ditemukan — lanjut tanpa RTC.");
        return;
    }
    if (rtc.lostPower()) {
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }
    sysStatus.rtcOk = true;
    Serial.println("[RTC] Initialized");
}
void setupWiFi()
{
    WiFi.mode(WIFI_AP);
    WiFi.persistent(false);
    sysStatus.wifiSta = connectWiFi();
    if (sysStatus.wifiSta) {
        Serial.printf("[WiFi AP] hotspot aktif: %s (password: %s)\n",
                      wifiCfg.ssid, wifiCfg.password);
    } else {
        Serial.println("[WiFi AP] hotspot belum aktif.");
    }
}

// ============================================================
//  LITTLEFS & LOGGING
// ============================================================

bool initLittleFS()
{
    if (!LittleFS.begin(true)) {
        Serial.println("[LittleFS] ERROR: Gagal mount!");
        return false;
    }
    Serial.printf("[LittleFS] OK. Total: %d KB, Used: %d KB\n",
                  LittleFS.totalBytes() / 1024,
                  LittleFS.usedBytes() / 1024);
    return true;
}

/** Tulis log JSON ke LittleFS. `dir` = "buka" | "tutup" untuk gerak aktuator; nullptr jika tidak berlaku. */
bool writeLog(String event, const char* dir)
{
    File file = LittleFS.open(LOG_FILE, FILE_APPEND);
    if (!file) {
        Serial.println("[LittleFS] ERROR: Gagal buka file log!");
        return false;
    }
    JsonDocument doc;
    doc["Time"]   = sensorData.timestamp;
    doc["Event"]  = event;
    if (dir != nullptr)
        doc["dir"] = dir;
    doc["T1"]     = serialized(String(sensorData.temp1, 2));
    doc["H1"]     = serialized(String(sensorData.hum1, 2));
    doc["T2"]     = serialized(String(sensorData.temp2, 2));
    doc["H2"]     = serialized(String(sensorData.hum2, 2));
    doc["durasi"] = serialized(String(actuator.lastDurS, 3));

    String line;
    serializeJson(doc, line);
    file.println(line);
    file.close();

    Serial.println("[LittleFS] Log [" + event + "]: " + line);
    return true;
}

/** Log manual ON: posisi sebelum & input target (cm), plus arah buka/tutup. */
bool writeLogManual(float yBefore, float yInput, int8_t dir)
{
    File file = LittleFS.open(LOG_FILE, FILE_APPEND);
    if (!file) {
        Serial.println("[LittleFS] ERROR: Gagal buka file log!");
        return false;
    }
    JsonDocument doc;
    doc["Time"]      = sensorData.timestamp;
    doc["Event"]     = "MANUAL_ON";
    doc["dir"]       = (dir > 0) ? "buka" : "tutup";
    doc["T1"]        = serialized(String(sensorData.temp1, 2));
    doc["H1"]        = serialized(String(sensorData.hum1, 2));
    doc["T2"]        = serialized(String(sensorData.temp2, 2));
    doc["H2"]        = serialized(String(sensorData.hum2, 2));
    doc["durasi"]    = serialized(String(actuator.lastDurS, 3));
    doc["y_sebelum"] = serialized(String(yBefore, 2));
    doc["y_input"]   = serialized(String(yInput, 2));

    String line;
    serializeJson(doc, line);
    file.println(line);
    file.close();

    Serial.println("[LittleFS] Log [MANUAL_ON]: " + line);
    return true;
}

void clearLog()
{
    if (LittleFS.exists(LOG_FILE)) {
        LittleFS.remove(LOG_FILE);
        Serial.println("[LittleFS] Log dihapus!");
    }
}

int countLogEntries()
{
    File file = LittleFS.open(LOG_FILE, FILE_READ);
    if (!file) return 0;
    int count = 0;
    while (file.available()) {
        file.readStringUntil('\n');
        count++;
    }
    file.close();
    return count;
}


// ============================================================
//  WEB SERVER
// ============================================================

void setupWebServer()
{
    static bool s_inited = false;
    if (s_inited) return;
    s_inited = true;

    server.on("/", HTTP_GET, []() {
        File file = LittleFS.open("/index.html", FILE_READ);
        if (!file) {
            server.send(500, "text/plain", "index.html tidak ditemukan di LittleFS");
            return;
        }
        server.streamFile(file, "text/html");
        file.close();
    });

    server.on("/data", HTTP_GET, []() {
        JsonDocument doc;
        doc["timestamp"]          = sensorData.timestamp;
        doc["temp1_cal"]          = serialized(String(sensorData.temp1, 2));
        doc["hum1_cal"]           = serialized(String(sensorData.hum1, 2));
        doc["temp2_cal"]          = serialized(String(sensorData.temp2, 2));
        doc["hum2_cal"]           = serialized(String(sensorData.hum2, 2));
        doc["valve_status"]       = actuator.valveStatus;
        doc["valve_open"]         = actuator.valveOpen;
        doc["avg_hum"]            = serialized(String(sensorData.hum1, 2));
        doc["t1off"]              = config.temp1Offset;
        doc["h1off"]              = config.hum1Offset;
        doc["t2off"]              = config.temp2Offset;
        doc["h2off"]              = config.hum2Offset;
        doc["relay_interval_ms"]  = config.relayCheckMs;
        doc["relay_interval_min"] = serialized(String(config.relayCheckMs / 60000.0f, 1));
        doc["log_interval_ms"]    = config.logIntervalMs;
        doc["rtc_now"]            = getTimestamp();
        doc["act_h_min"]          = config.hMin;
        doc["act_h_max"]          = config.hMax;
        doc["act_y_min"]          = config.yMin;
        doc["act_y_max"]          = config.yMax;
        if (isnan(sysSettings.hTemporer))
            doc["act_h_temporer"] = "-";
        else
            doc["act_h_temporer"] = serialized(String(sysSettings.hTemporer, 2));
        doc["act_speed"]          = config.speedCmS;
        doc["act_max_dur"]        = config.maxDurS;
        doc["act_y_cur"]          = serialized(String(actuator.yCurrent,  2));
        doc["act_y_tgt"]          = serialized(String(actuator.yTarget,   2));
        doc["act_y_prev"]         = serialized(String(actuator.yPrevious, 2));
        doc["act_busy"]           = actuatorIsBusy();
        doc["act_stroke_cm"]      = ACT_STROKE_PHYSICAL_CM;
        doc["pt_cm"]              = config.ptTotalCm;
        doc["dt_s"]               = config.dtTotalS;
        doc["man_speed"]          = serialized(String(manualOpenCloseSpeedCmS(), 3));
        doc["mode_auto"]          = config.modeAuto;
        String json;
        serializeJson(doc, json);
        server.send(200, "application/json", json);
    });

    /** Waktu RTC saja — ringan untuk polling 1 Hz di UI */
    server.on("/time", HTTP_GET, []() {
        JsonDocument doc;
        doc["rtc_now"] = getTimestamp();
        String json;
        serializeJson(doc, json);
        server.send(200, "application/json", json);
    });

    server.on("/control/mode", HTTP_GET, []() {
        if (server.hasArg("auto")) {
            config.modeAuto = (server.arg("auto").toInt() != 0);
            if (config.modeAuto)
                sysSettings.lastMeasurementMs = millis();
            saveConfig();
            Serial.printf("[MODE] Kontrol %s\n", config.modeAuto ? "AUTO" : "MANUAL");
        }
        JsonDocument doc;
        doc["success"]   = true;
        doc["mode_auto"] = config.modeAuto;
        String json;
        serializeJson(doc, json);
        server.send(200, "application/json", json);
    });

    server.on("/actuator/save", HTTP_GET, []() {
        float nhmin = config.hMin, nhmax = config.hMax;
        float nymin = config.yMin, nymax = config.yMax;
        float nspd  = config.speedCmS, ndur = config.maxDurS;
        float npt   = config.ptTotalCm, ndt = config.dtTotalS;

        if (server.hasArg("hmin"))    nhmin = server.arg("hmin").toFloat();
        if (server.hasArg("hmax"))    nhmax = server.arg("hmax").toFloat();
        if (server.hasArg("ymin"))    nymin = server.arg("ymin").toFloat();
        if (server.hasArg("ymax"))    nymax = server.arg("ymax").toFloat();
        if (server.hasArg("speed"))   nspd  = server.arg("speed").toFloat();
        if (server.hasArg("maxdur"))  ndur  = server.arg("maxdur").toFloat();
        if (server.hasArg("pt"))      npt   = server.arg("pt").toFloat();
        if (server.hasArg("dt"))      ndt   = server.arg("dt").toFloat();

        if (nymin < 0) nymin = 0;
        if (nymax > ACT_STROKE_PHYSICAL_CM) nymax = ACT_STROKE_PHYSICAL_CM;

        JsonDocument doc;
        if (nhmax <= nhmin) {
            doc["success"] = false; doc["error"] = "RH max harus lebih besar dari RH min";
        } else if (nymax < nymin) {
            doc["success"] = false; doc["error"] = "Stroke max harus >= stroke min";
        } else if (npt < 1.0f || npt > ACT_STROKE_PHYSICAL_CM) {
            doc["success"] = false; doc["error"] = "PT (panjang total): 1–50 cm";
        } else if (ndt < 0.2f || ndt > 600.0f) {
            doc["success"] = false; doc["error"] = "DT (durasi total): 0.2–600 s";
        } else {
            if (nspd < 0.05f) nspd = 0.05f;
            if (nspd > 25.0f) nspd = 25.0f;
            if (ndur < 0.5f)  ndur = 0.5f;
            if (ndur > 120.0f) ndur = 120.0f;
            config.hMin      = nhmin;  config.hMax      = nhmax;
            config.yMin      = nymin;  config.yMax      = nymax;
            config.speedCmS  = nspd;   config.maxDurS   = ndur;
            config.ptTotalCm = npt;    config.dtTotalS  = ndt;
            if (actuator.yCurrent < config.yMin) actuator.yCurrent = config.yMin;
            if (actuator.yCurrent > config.yMax) actuator.yCurrent = config.yMax;
            config.yCurrent = actuator.yCurrent;
            saveConfig();
            updateValveStatusFromPosition();
            doc["success"] = true;
        }
        String json;
        serializeJson(doc, json);
        server.send(200, "application/json", json);
    });

    server.on("/calibrate", HTTP_GET, []() {
        if (server.hasArg("t1")) config.temp1Offset = server.arg("t1").toFloat();
        if (server.hasArg("h1")) config.hum1Offset  = server.arg("h1").toFloat();
        if (server.hasArg("t2")) config.temp2Offset = server.arg("t2").toFloat();
        if (server.hasArg("h2")) config.hum2Offset  = server.arg("h2").toFloat();
        bool saved = saveConfig();
        if (saved) {
            readDHT();
            sensorData.timestamp = getTimestamp();
        }
        Serial.printf("[CAL] Offset baru → T1:%.1f H1:%.1f T2:%.1f H2:%.1f\n",
                      config.temp1Offset, config.hum1Offset,
                      config.temp2Offset, config.hum2Offset);
        server.send(200, "application/json", saved ? "{\"success\":true}" : "{\"success\":false}");
    });

    server.on("/valve/on", HTTP_GET, []() {
        if (actuatorIsBusy()) {
            server.send(409, "application/json", "{\"success\":false,\"error\":\"Aktuator sedang bergerak\"}");
            return;
        }
        if (config.modeAuto) {
            server.send(409, "application/json", "{\"success\":false,\"error\":\"Pilih mode Manual untuk kontrol valve\"}");
            return;
        }
        if (!server.hasArg("pd")) {
            server.send(400, "application/json", "{\"success\":false,\"error\":\"Parameter pd (cm) wajib\"}");
            return;
        }
        float pd = server.arg("pd").toFloat();
        if (!manualValveOn(pd)) {
            server.send(400, "application/json", "{\"success\":false,\"error\":\"Sudah di target atau pd tidak valid\"}");
            return;
        }
        JsonDocument doc;
        doc["success"]   = true;
        doc["target_cm"] = actuator.yTarget;
        String json;
        serializeJson(doc, json);
        server.send(200, "application/json", json);
    });

    server.on("/valve/open", HTTP_GET, []() {
        if (actuatorIsBusy()) {
            server.send(409, "application/json", "{\"success\":false,\"error\":\"Aktuator sedang bergerak\"}");
            return;
        }
        if (config.modeAuto) {
            server.send(409, "application/json", "{\"success\":false,\"error\":\"Pilih mode Manual untuk kontrol valve\"}");
            return;
        }
        float cmReq = server.hasArg("cm") ? server.arg("cm").toFloat() : -1.0f;
        if (!openValve(cmReq)) {
            server.send(400, "application/json", "{\"success\":false,\"error\":\"Sudah di posisi maks\"}");
            return;
        }
        server.send(200, "application/json", "{\"success\":true,\"status\":\"OPEN\"}");
    });

    server.on("/valve/close", HTTP_GET, []() {
        if (actuatorIsBusy()) {
            server.send(409, "application/json", "{\"success\":false,\"error\":\"Aktuator sedang bergerak\"}");
            return;
        }
        if (config.modeAuto) {
            server.send(409, "application/json", "{\"success\":false,\"error\":\"Pilih mode Manual untuk kontrol valve\"}");
            return;
        }
        float cmReq = server.hasArg("cm") ? server.arg("cm").toFloat() : -1.0f;
        if (!closeValve(cmReq)) {
            server.send(400, "application/json", "{\"success\":false,\"error\":\"Sudah di posisi min\"}");
            return;
        }
        server.send(200, "application/json", "{\"success\":true,\"status\":\"CLOSED\"}");
    });

    server.on("/setinterval", HTTP_GET, []() {
        if (server.hasArg("ms")) {
            long v = server.arg("ms").toInt();
            if (v < 5000L)       v = 5000L;
            if (v > 86400000L)   v = 86400000L;
            config.relayCheckMs = (unsigned long)v;
            saveConfig();
            Serial.printf("[INTERVAL] Diubah → %lu ms\n", config.relayCheckMs);
        }
        JsonDocument doc;
        doc["success"]     = true;
        doc["interval_ms"] = config.relayCheckMs;
        String json;
        serializeJson(doc, json);
        server.send(200, "application/json", json);
    });

    server.on("/log/setinterval", HTTP_GET, []() {
        JsonDocument doc;
        if (!server.hasArg("ms")) {
            doc["success"] = false; doc["error"] = "Parameter ms wajib";
        } else {
            long v = server.arg("ms").toInt();
            if (v < 1000L)     v = 1000L;
            if (v > 86400000L) v = 86400000L;
            config.logIntervalMs = (unsigned long)v;
            saveConfig();
            doc["success"]        = true;
            doc["log_interval_ms"] = config.logIntervalMs;
            Serial.printf("[LOG] Interval log diubah → %lu ms\n", config.logIntervalMs);
        }
        String json;
        serializeJson(doc, json);
        server.send(200, "application/json", json);
    });

    server.on("/rtc/set", HTTP_GET, []() {
        JsonDocument doc;
        if (!server.hasArg("y") || !server.hasArg("mo") || !server.hasArg("d") ||
            !server.hasArg("h") || !server.hasArg("mi") || !server.hasArg("s")) {
            doc["success"] = false; doc["error"] = "Parameter waktu belum lengkap";
        } else {
            bool ok = setRtcFromYmdHms(
                server.arg("y").toInt(),  server.arg("mo").toInt(), server.arg("d").toInt(),
                server.arg("h").toInt(),  server.arg("mi").toInt(), server.arg("s").toInt()
            );
            doc["success"] = ok;
            if (!ok) doc["error"] = "Format waktu tidak valid";
            doc["rtc_now"] = getTimestamp();
            if (ok) Serial.printf("[RTC] Set manual → %s\n", getTimestamp().c_str());
        }
        String json;
        serializeJson(doc, json);
        server.send(200, "application/json", json);
    });

    server.on("/log/download", HTTP_GET, []() {
        File file = LittleFS.open(LOG_FILE, FILE_READ);
        if (!file) { server.send(404, "text/plain", "Log kosong."); return; }
        server.sendHeader("Content-Disposition", "attachment; filename=silo_log.json");
        server.streamFile(file, "application/json");
        file.close();
        Serial.println("[WEB] Log didownload!");
    });

    server.on("/log/view", HTTP_GET, []() {
        File file = LittleFS.open(LOG_FILE, FILE_READ);
        if (!file) { server.send(200, "text/plain", "Log kosong."); return; }
        server.streamFile(file, "text/plain");
        file.close();
    });

    server.on("/log/clear", HTTP_GET, []() {
        clearLog();
        server.send(200, "application/json", "{\"success\":true}");
        Serial.println("[WEB] Log dihapus via web!");
    });

    server.on("/log/info", HTTP_GET, []() {
        JsonDocument doc;
        doc["total_kb"] = LittleFS.totalBytes() / 1024;
        doc["used_kb"]  = LittleFS.usedBytes()  / 1024;
        doc["free_kb"]  = (LittleFS.totalBytes() - LittleFS.usedBytes()) / 1024;
        doc["entries"]  = countLogEntries();
        String json;
        serializeJson(doc, json);
        server.send(200, "application/json", json);
    });

    server.on("/system/restart", HTTP_GET, []() {
        if (actuatorIsBusy()) {
            server.send(409, "application/json",
                         "{\"success\":false,\"error\":\"Aktuator sedang bergerak\"}");
            return;
        }
        Serial.println("[WEB] Restart sistem diminta");
        server.send(200, "application/json", "{\"success\":true}");
        delay(200);
        ESP.restart();
    });

    server.on("/favicon.ico", HTTP_GET, []() {
        server.send(204, "text/plain", "");
    });

    server.onNotFound([]() {
        server.send(404, "text/plain", "Not found");
    });

    server.begin();
    sysStatus.webServerOn = true;
    Serial.println("[WEB] Server started!");
}

// ============================================================
//  SETUP
// ============================================================
void setup()
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("================================");
    Serial.println("  Kernel Silo Monitor + WebUI");
    Serial.println("================================");

    // --- Relay & LED (GPIO aman tanpa sensor eksternal) ---
    pinMode(RELAY_OPEN_PLUS_PIN,   OUTPUT);
    pinMode(RELAY_OPEN_MINUS_PIN,  OUTPUT);
    pinMode(RELAY_CLOSE_PLUS_PIN,  OUTPUT);
    pinMode(RELAY_CLOSE_MINUS_PIN, OUTPUT);
    valveRelaysIdle();
    pinMode(LED_OPEN_PIN, OUTPUT);
    digitalWrite(LED_OPEN_PIN, HIGH);
    Serial.println("[RELAY] Open 27/26, Close 25/33 (active-low)");
    Serial.println("[LED] Initialized GPIO 32");

    // --- LittleFS & Config (tanpa blokir WiFi jika gagal) ---
    sysStatus.littleFsOk = initLittleFS();
    if (!loadConfig()) {
        Serial.println("[Config] File tidak ada atau gagal baca.");
        if (sysStatus.littleFsOk) {
            Serial.println("[Config] Buat default di LittleFS.");
            createDefaultConfig();
            loadConfig();
        } else {
            Serial.println("[Config] Tanpa LittleFS — pakai nilai default di RAM.");
        }
    }

    // --- WiFi AP + Web (sebelum RTC/DHT agar hotspot tetap muncul tanpa modul opsional) ---
    setupWiFi();
    setupWebServer();

    // --- RTC (opsional — tidak memblokir boot) ---
    initRTC();

    // --- DHT (opsional; begin() tidak menunggu sensor fisik) ---
    dht1.begin();
    Serial.println("[DHT21-1] Initialized GPIO 4");
    dht2.begin();
    Serial.println("[DHT21-2] Initialized GPIO 15");

    updateValveStatusFromPosition();

    Serial.println("[SYSTEM] Setup selesai!\n");
    delay(1000);
}

// ============================================================
//  MAIN LOOP
// ============================================================
void loop()
{
    unsigned long nowMs = millis();

    // Layanan non-blocking: aktuator & web
    actuatorService();
    if (sysStatus.webServerOn) server.handleClient();

    // Cek koneksi WiFi secara periodik
    if (nowMs - sysSettings.lastWifiCheck >= WIFI_CHECK_INTERVAL) {
        sysSettings.lastWifiCheck = nowMs;
        checkWiFi();
    }

    // --- Pembacaan sensor + (auto) cek aktuator pada interval yang sama (relayCheckMs) ---
    if (sysSettings.lastMeasurementMs == 0ULL
        || nowMs - sysSettings.lastMeasurementMs >= config.relayCheckMs) {
        sysSettings.lastMeasurementMs = nowMs;
        readDHT();
        sensorData.timestamp = getTimestamp();
        if (config.modeAuto)
            updateActuatorFromHumidity();
    }

    // --- Log LittleFS ---
    if (sysStatus.littleFsOk && !isnan(sensorData.rawTemp1)) {
        if (!sysSettings.firstLogDone) {
            Serial.println("--------------------------------");
            Serial.printf("Waktu : %s\n", sensorData.timestamp.c_str());
            Serial.printf("WiFi AP: %s\n", sysStatus.wifiSta
                          ? WiFi.softAPIP().toString().c_str() : "off");
            Serial.printf("[S1] Temp: %.2f→%.2f°C  RH: %.2f→%.2f%%\n",
                          sensorData.rawTemp1, sensorData.temp1,
                          sensorData.rawHum1,  sensorData.hum1);
            Serial.printf("[S2] Temp: %.2f→%.2f°C  RH: %.2f→%.2f%%\n",
                          sensorData.rawTemp2, sensorData.temp2,
                          sensorData.rawHum2,  sensorData.hum2);
            Serial.println("--------------------------------\n");
            writeLog();
            sysSettings.lastLogTime  = nowMs;
            sysSettings.firstLogDone = true;
            Serial.println("[LittleFS] Log pertama saat boot tersimpan!");
        } else if (nowMs - sysSettings.lastLogTime >= config.logIntervalMs) {
            Serial.println("--------------------------------");
            Serial.printf("Waktu : %s\n", sensorData.timestamp.c_str());
            Serial.printf("WiFi AP: %s\n", sysStatus.wifiSta
                          ? WiFi.softAPIP().toString().c_str() : "off");
            Serial.printf("[S1] Temp: %.2f→%.2f°C  RH: %.2f→%.2f%%\n",
                          sensorData.rawTemp1, sensorData.temp1,
                          sensorData.rawHum1,  sensorData.hum1);
            Serial.printf("[S2] Temp: %.2f→%.2f°C  RH: %.2f→%.2f%%\n",
                          sensorData.rawTemp2, sensorData.temp2,
                          sensorData.rawHum2,  sensorData.hum2);
            Serial.println("--------------------------------\n");
            sysSettings.lastLogTime = nowMs;
            writeLog();
        }
    }

    delay(10);
}