/* * @Hardware: M5Atom S3 + Unit LoRaWAN CN470, EU868, US915, AS923 */

#include <M5Unified.h>
#include "rak3172_p2p.hpp"
#include "ATC_MiThermometer.h"
#include <vector>
#include <math.h>
#include <esp_task_wdt.h>

// LoRa settings
#define LORA_FREQ 868E6
#define LORA_CONFIG_PRLEN 8
#define LORA_CONFIG_PWR 22
#define LORA_RX_PIN 1
#define LORA_TX_PIN 2

int cr = 1;
int sf = 10;
int bw = 250;
int send_delay = 20000;

// Sensor IDs
#define SENSOR1_ID 1
#define SENSOR2_ID 2

// Sensor indexes
#define OUT_SENSOR 0
#define IN_SENSOR 1
#define SENSOR_COUNT 2

// Sensor state
#define SENSOR_TIMEOUT_MS 60000UL

// PWM for fan
#define FAN_PIN 5
#define FAN_PWM_CHANNEL 0
#define FAN_PWM_FREQ 5000
#define FAN_PWM_RESOLUTION 8
#define FAN_MAX_SPEED 125

// Hysteresis thresholds in %
#define HUM_DIFF_ON 5.0f
#define HUM_DIFF_OFF 4.0f

static bool fanOn = false;
static bool loraReady = false;

// Globals
RAK3172P2P lora;

std::vector<std::string> knownBLE = {
    "a4:c1:38:fd:0f:62", // OUT SENSOR (exterior)
    "a4:c1:38:52:8c:33"  // IN SENSOR (interior)
};

ATC_MiThermometer miTh(knownBLE);

MiThData_S miThData[SENSOR_COUNT];
SemaphoreHandle_t mutex[SENSOR_COUNT];

bool everSeen[SENSOR_COUNT] = {false, false};
uint32_t lastSeenMs[SENSOR_COUNT] = {0, 0};

// CRC16-CCITT calculation, poly 0x1021.
static uint16_t computeCRC16(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFF;

  while (len--)
  {
    crc ^= (uint16_t)(*data++) << 8;

    for (uint8_t i = 0; i < 8; i++)
    {
      crc = (crc & 0x8000)
                ? (uint16_t)((crc << 1) ^ 0x1021)
                : (uint16_t)(crc << 1);
    }
  }

  return crc;
}

// Copy all sensor-related fields under the same mutex.
// This avoids reading miThData, everSeen, and lastSeenMs inconsistently.
static bool copySensorState(
    int index,
    MiThData_S *data,
    bool *seen,
    uint32_t *lastMs)
{
  if (index < 0 || index >= SENSOR_COUNT)
    return false;
  if (data == NULL || seen == NULL || lastMs == NULL)
    return false;
  if (mutex[index] == NULL)
    return false;

  if (xSemaphoreTake(mutex[index], pdMS_TO_TICKS(10)) != pdTRUE)
  {
    return false;
  }

  *data = miThData[index];
  *seen = everSeen[index];
  *lastMs = lastSeenMs[index];

  xSemaphoreGive(mutex[index]);
  return true;
}

static bool sensorFreshFromSnapshot(bool seen, uint32_t lastMs, uint32_t now)
{
  return seen && ((now - lastMs) < SENSOR_TIMEOUT_MS);
}

static char getStatusByteFromSnapshots(
    bool inSeen,
    bool outSeen,
    bool inOk,
    bool outOk)
{
  if (!inSeen || !outSeen)
    return 'D';
  if (!inOk && !outOk)
    return 'D';
  if (!inOk)
    return 'I';
  if (!outOk)
    return 'O';
  return fanOn ? 'R' : 'S';
}

// Task prototype
void miThReadingTask(void *pvParameters);

void setup()
{
  delay(5000);

  esp_task_wdt_init(30, true);

  ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
  ledcAttachPin(FAN_PIN, FAN_PWM_CHANNEL);

  esp_task_wdt_add(NULL);

  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);

  Serial.println("[DEBUG] Setup started");

  for (int i = 0; i < SENSOR_COUNT; i++)
  {
    mutex[i] = xSemaphoreCreateMutex();

    if (mutex[i] == NULL)
    {
      Serial.printf("[ERROR] Failed to create mutex for sensor %d\n", i + 1);
      delay(2000);
      ESP.restart();
    }

    // Keep original behavior: send zeros until BLE data arrives.
    miThData[i].temperature = 0;
    miThData[i].humidity = 0;
    miThData[i].batt_level = 0;
    miThData[i].valid = true;

    everSeen[i] = false;
    lastSeenMs[i] = 0;
  }

  // LoRa init: keep repository behavior, retrying until the module answers.
  Serial.print("[DEBUG] Init LoRa...");
  while (!lora.init(&Serial2, LORA_RX_PIN, LORA_TX_PIN, RAK3172_BPS_115200))
  {
    Serial.print('.');
    delay(500);
    esp_task_wdt_reset();
  }

  loraReady = true;

  lora.setMode(P2P_RX_MODE, 0);
  lora.config(LORA_FREQ, sf, bw, cr, LORA_CONFIG_PRLEN, LORA_CONFIG_PWR);
  lora.setMode(P2P_TX_RX_MODE);
  Serial.println(" done");

  // Init BLE and start task.
  miTh.begin();
  Serial.println("[DEBUG] BLE init done");

  BaseType_t taskCreated = xTaskCreate(
      miThReadingTask,
      "miThTask",
      10000,
      NULL,
      1,
      NULL);

  if (taskCreated != pdPASS)
  {
    Serial.println("[ERROR] BLE task creation failed");
    delay(2000);
    ESP.restart();
  }

  Serial.println("[DEBUG] BLE task created");

  // LCD init
  M5.Display.setRotation(2);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE);
  M5.Display.setTextSize(2);

  Serial.println("[DEBUG] LCD initialized (M5Unified)");
  Serial.println("[DEBUG] Setup complete");
}

void loop()
{
  M5.update();

  if (loraReady)
  {
    lora.update();
  }

  esp_task_wdt_reset();

  uint32_t now = millis();

  MiThData_S d0;
  MiThData_S d1;

  bool outSeen = false;
  bool inSeen = false;
  uint32_t outLastSeenMs = 0;
  uint32_t inLastSeenMs = 0;

  bool outCopied = copySensorState(OUT_SENSOR, &d0, &outSeen, &outLastSeenMs);
  bool inCopied = copySensorState(IN_SENSOR, &d1, &inSeen, &inLastSeenMs);

  // Fallback keeps payload/display behavior safe if locking unexpectedly fails.
  if (!outCopied)
  {
    d0.temperature = 0;
    d0.humidity = 0;
    d0.batt_level = 0;
    d0.valid = true;
    outSeen = false;
    outLastSeenMs = 0;
    Serial.println("[ERROR] Failed to copy OUT sensor state");
  }

  if (!inCopied)
  {
    d1.temperature = 0;
    d1.humidity = 0;
    d1.batt_level = 0;
    d1.valid = true;
    inSeen = false;
    inLastSeenMs = 0;
    Serial.println("[ERROR] Failed to copy IN sensor state");
  }

  bool outOk = sensorFreshFromSnapshot(outSeen, outLastSeenMs, now);
  bool inOk = sensorFreshFromSnapshot(inSeen, inLastSeenMs, now);

  // Fan control only when both sensors are fresh.
  if (inOk && outOk)
  {
    float HumOut = d0.humidity * 0.01f;
    float HumIn = d1.humidity * 0.01f;
    float diff = HumIn - HumOut;

    if (!fanOn && diff >= HUM_DIFF_ON)
    {
      ledcWrite(FAN_PWM_CHANNEL, FAN_MAX_SPEED);
      fanOn = true;
      Serial.println("[DEBUG] Fan ON");
    }
    else if (fanOn && diff <= HUM_DIFF_OFF)
    {
      ledcWrite(FAN_PWM_CHANNEL, 0);
      fanOn = false;
      Serial.println("[DEBUG] Fan OFF");
    }
  }
  else
  {
    // Safe mode: do not run fan from stale/missing data.
    if (fanOn)
    {
      ledcWrite(FAN_PWM_CHANNEL, 0);
      fanOn = false;
      Serial.println("[DEBUG] Fan OFF - stale sensor data");
    }
  }

  // Important: calculate status after fan control,
  // so R/S reflects the updated fan state.
  char status = getStatusByteFromSnapshots(inSeen, outSeen, inOk, outOk);

  // LCD
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setCursor(0, 0);
  M5.Display.setTextSize(2);

  M5.Display.setTextColor(TFT_WHITE);
  M5.Display.printf("ST:%c F:%d\n", status, fanOn ? 1 : 0);

  M5.Display.setTextColor(outOk ? TFT_YELLOW : TFT_RED);
  M5.Display.printf("O:%4.1fC\n", d0.temperature * 0.01f);

  M5.Display.setTextColor(outOk ? TFT_CYAN : TFT_RED);
  M5.Display.printf("H:%4.1f%%\n", d0.humidity * 0.01f);

  M5.Display.setTextColor(d0.batt_level < 15 ? TFT_RED : TFT_WHITE);
  M5.Display.printf("B: %d%%\n", d0.batt_level);

  M5.Display.setTextColor(inOk ? TFT_YELLOW : TFT_RED);
  M5.Display.printf("I:%4.1fC\n", d1.temperature * 0.01f);

  M5.Display.setTextColor(inOk ? TFT_CYAN : TFT_RED);
  M5.Display.printf("H:%4.1f%%\n", d1.humidity * 0.01f);

  M5.Display.setTextColor(d1.batt_level < 15 ? TFT_RED : TFT_WHITE);
  M5.Display.printf("B: %d%%\n", d1.batt_level);

  // Payload:
  // S1 TTT HHH BB S2 TTT HHH BB F CRC
  //
  // F contains:
  // R = both sensors OK and fan running
  // S = both sensors OK and fan stopped
  // I = IN sensor failed
  // O = OUT sensor failed
  // D = default / missing readings / both failed
  char buf[25];

  int t1 = (int)round(d0.temperature * 0.1f);
  int h1 = (int)round(d0.humidity * 0.1f);
  int b1 = d0.batt_level == 100 ? 99 : d0.batt_level;

  int t2 = (int)round(d1.temperature * 0.1f);
  int h2 = (int)round(d1.humidity * 0.1f);
  int b2 = d1.batt_level == 100 ? 99 : d1.batt_level;

  char fanState = status;

  int len = snprintf(
      buf,
      sizeof(buf) - 4,
      "%1d%03d%03d%02d%1d%03d%03d%02d%c",
      SENSOR1_ID,
      t1,
      h1,
      b1,
      SENSOR2_ID,
      t2,
      h2,
      b2,
      fanState);

  // Need room for current payload + 4 CRC chars + '\0'.
  if (len < 0 || (size_t)len + 4 >= sizeof(buf))
  {
    Serial.println("[ERROR] Payload formatting failed or too long");
    vTaskDelay(send_delay / portTICK_PERIOD_MS);
    return;
  }

  uint16_t crc = computeCRC16((const uint8_t *)buf, (size_t)len);

  Serial.printf("[DEBUG] CRC16: 0x%04X\n", crc);

  char crcStr[5];
  snprintf(crcStr, sizeof(crcStr), "%04X", crc);

  memcpy(buf + len, crcStr, 4);
  buf[len + 4] = '\0';
  len += 4;

  Serial.print("[DEBUG] Sending payload: ");
  Serial.write((const uint8_t *)buf, len);
  Serial.println();

  if (!loraReady)
  {
    Serial.println("[ERROR] LoRa not ready; packet not sent");
  }
  else if (lora.print(String(buf, len).c_str()))
  {
    Serial.println("[DEBUG] Packet sent");
  }
  else
  {
    Serial.println("[ERROR] Send failed");
  }

  vTaskDelay(send_delay / portTICK_PERIOD_MS);
}

void miThReadingTask(void *pvParameters)
{
  esp_task_wdt_add(NULL);

  while (1)
  {
    esp_task_wdt_reset();

    miTh.resetData();
    miTh.getData(5);

    uint32_t now = millis();

    for (int i = 0; i < SENSOR_COUNT; i++)
    {
      // Only overwrite cached RAM data when the new BLE reading is valid.
      // If the sensor disappears, keep the last known value in RAM.
      if (!miTh.data[i].valid)
      {
        continue;
      }

      if (mutex[i] == NULL)
      {
        Serial.printf("[ERROR] Sensor %d mutex is NULL\n", i + 1);
        continue;
      }

      if (xSemaphoreTake(mutex[i], pdMS_TO_TICKS(10)) == pdTRUE)
      {
        miThData[i] = miTh.data[i];
        miThData[i].valid = true;
        everSeen[i] = true;
        lastSeenMs[i] = now;

        Serial.printf(
            "[DEBUG] Sensor %d: T=%.1fC H=%.1f%% B=%d%%\n",
            i + 1,
            miThData[i].temperature * 0.01f,
            miThData[i].humidity * 0.01f,
            miThData[i].batt_level);

        xSemaphoreGive(mutex[i]);
      }
      else
      {
        Serial.printf("[ERROR] Failed to lock sensor %d mutex\n", i + 1);
      }
    }

    miTh.clearScanResults();

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}