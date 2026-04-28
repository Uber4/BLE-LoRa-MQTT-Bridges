/*
 * @Hardware: M5Atom S3 + Unit LoRaWAN CN470, EU868, US915, AS923
 */

#include <M5Unified.h>
#include "rak3172_p2p.hpp"
#include "ATC_MiThermometer.h"

#include <stdint.h>
#include <math.h>
#include <esp_task_wdt.h>

// LoRa settings
#define LORA_FREQ 868E6
#define LORA_CONFIG_PRLEN 8
#define LORA_CONFIG_PWR 22

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

// Globals
RAK3172P2P lora;

std::vector<std::string> knownBLE = {
    "a4:c1:38:fd:0f:62",
    "a4:c1:38:52:8c:33"};

ATC_MiThermometer miTh(knownBLE);
MiThData_S miThData[2];

SemaphoreHandle_t mutex[2];

bool everSeen[2] = {false, false};
uint32_t lastSeenMs[2] = {0, 0};

// CRC16-CCITT calculation (poly 0x1021)
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

static bool sensorFresh(int i)
{
  return everSeen[i] && ((millis() - lastSeenMs[i]) < SENSOR_TIMEOUT_MS);
}

static char getStatusByte()
{
  bool inOk = sensorFresh(IN_SENSOR);
  bool outOk = sensorFresh(OUT_SENSOR);

  if (!everSeen[IN_SENSOR] || !everSeen[OUT_SENSOR])
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

  for (int i = 0; i < 2; i++)
  {
    mutex[i] = xSemaphoreCreateMutex();

    // Initial default values: send zeros until BLE data arrives.
    miThData[i].temperature = 0;
    miThData[i].humidity = 0;
    miThData[i].batt_level = 0;
    miThData[i].valid = true;

    everSeen[i] = false;
    lastSeenMs[i] = 0;
  }

  // LoRa init
  Serial.print("[DEBUG] Init LoRa...");

  while (!lora.init(&Serial2, 1, 2, RAK3172_BPS_115200))
  {
    Serial.print('.');
    delay(500);
  }

  lora.setMode(P2P_RX_MODE, 0);
  lora.config(LORA_FREQ, sf, bw, cr, LORA_CONFIG_PRLEN, LORA_CONFIG_PWR);
  lora.setMode(P2P_TX_RX_MODE);

  Serial.println(" done");

  // Init BLE and start task
  miTh.begin();
  Serial.println("[DEBUG] BLE init done");

  xTaskCreate(miThReadingTask, "miThTask", 10000, NULL, 1, NULL);
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
  lora.update();
  esp_task_wdt_reset();

  // Safely get data
  MiThData_S d0, d1;

  xSemaphoreTake(mutex[OUT_SENSOR], 10);
  d0 = miThData[OUT_SENSOR];
  xSemaphoreGive(mutex[OUT_SENSOR]);

  xSemaphoreTake(mutex[IN_SENSOR], 10);
  d1 = miThData[IN_SENSOR];
  xSemaphoreGive(mutex[IN_SENSOR]);

  bool inOk = sensorFresh(IN_SENSOR);
  bool outOk = sensorFresh(OUT_SENSOR);

  // Fan control only when both sensors are fresh
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
    // Safe mode: do not run fan from stale/missing data
    if (fanOn)
    {
      ledcWrite(FAN_PWM_CHANNEL, 0);
      fanOn = false;
      Serial.println("[DEBUG] Fan OFF - stale sensor data");
    }
  }

  // Important: calculate status after fan control,
  // so R/S reflects the updated fan state.
  char status = getStatusByte();

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
  // F now contains:
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

  int len = snprintf(buf, sizeof(buf) - 4,
                     "%1d%03d%03d%02d%1d%03d%03d%02d%c",
                     SENSOR1_ID, t1, h1, b1,
                     SENSOR2_ID, t2, h2, b2,
                     fanState);

  uint16_t crc = computeCRC16((const uint8_t *)buf, len);

  Serial.printf("[DEBUG] CRC16: 0x%04X\n", crc);

  char crcStr[5];
  snprintf(crcStr, sizeof(crcStr), "%04X", crc);

  memcpy(buf + len, crcStr, 4);
  buf[len + 4] = '\0';
  len += 4;

  Serial.print("[DEBUG] Sending payload: ");
  Serial.write((const uint8_t *)buf, len);
  Serial.println();

  if (lora.print(String(buf, len).c_str()))
    Serial.println("[DEBUG] Packet sent");
  else
    Serial.println("[ERROR] Send failed");

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

    for (int i = 0; i < 2; i++)
    {
      // Only overwrite cached RAM data when the new BLE reading is valid.
      // If the sensor disappears, keep the last known value in RAM.
      if (!miTh.data[i].valid)
        continue;

      if (xSemaphoreTake(mutex[i], 10) == pdTRUE)
      {
        miThData[i] = miTh.data[i];
        miThData[i].valid = true;

        everSeen[i] = true;
        lastSeenMs[i] = now;

        Serial.printf("[DEBUG] Sensor %d: T=%.1fC H=%.1f%% B=%d%%\n",
                      i + 1,
                      miThData[i].temperature * 0.01f,
                      miThData[i].humidity * 0.01f,
                      miThData[i].batt_level);

        xSemaphoreGive(mutex[i]);
      }
    }

    miTh.clearScanResults();

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}