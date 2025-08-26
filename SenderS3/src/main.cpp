/*
 * @Hardware: M5Atom S3 + Unit LoRaWAN CN470, EU868, US915, AS923
 * @Dependent Libraries:
 *   - M5Atom@^0.1.3: https://github.com/m5stack/M5Atom
 *   - M5-LoRaWAN-RAK: https://github.com/m5stack/M5-LoRaWAN-RAK
 *   - ATC_MiThermometer: https://github.com/matthias-bs/ATC_MiThermometer
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
int sf = 10;            // Spreading factor
int bw = 250;           // Frequency
int send_delay = 20000; // Delay for sending LoRa packet (20s aprox. for a ToA of 185,4 ms)

// Sensor IDs
#define SENSOR1_ID 1
#define SENSOR2_ID 2

// PWM for fan
#define FAN_PIN 5
#define FAN_PWM_CHANNEL 0
#define FAN_PWM_FREQ 5000
#define FAN_PWM_RESOLUTION 8
#define FAN_MAX_SPEED 125

// Hysteresis thresholds in %
#define HUM_DIFF_ON 5.0f  // turn on if ≥ 5%
#define HUM_DIFF_OFF 4.0f // turn off if ≤ 4%

// Fan state
static bool fanOn = false;

// Globals
RAK3172P2P lora;
std::vector<std::string> knownBLE = {"a4:c1:38:fd:0f:62", "a4:c1:38:52:8c:33"};
ATC_MiThermometer miTh(knownBLE);
MiThData_S miThData[2];
SemaphoreHandle_t mutex[2];
bool haveReadings = false;

// CRC16-CCITT calculation (poly 0x1021)
static uint16_t computeCRC16(const uint8_t *data, size_t len)
{
  uint16_t crc = 0xFFFF;
  while (len--)
  {
    crc ^= (uint16_t)(*data++) << 8;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 0x8000)
                ? (uint16_t)((crc << 1) ^ 0x1021)
                : (uint16_t)(crc << 1);
  }
  return crc;
}

// Task prototype
void miThReadingTask(void *pvParameters);

void setup()
{
  // Delay for power supply stability
  delay(5000);
  // Initialize ESP-IDF watchdog: 30s timeout, panic on trigger
  esp_task_wdt_init(30, true);
  // Start PWM for the fan
  ledcSetup(FAN_PWM_CHANNEL, FAN_PWM_FREQ, FAN_PWM_RESOLUTION);
  ledcAttachPin(FAN_PIN, FAN_PWM_CHANNEL);
  // Register current (loop) task to watchdog
  esp_task_wdt_add(NULL);
  auto cfg = M5.config();
  cfg.serial_baudrate = 115200;
  M5.begin(cfg);
  Serial.println("[DEBUG] Setup started");

  // LoRa init
  Serial.print("[DEBUG] Init LoRa...");
  while (!lora.init(&Serial2, 1, 2, RAK3172_BPS_115200)) // 6,5 on DTU unit. Rx, Tx.
  {
    Serial.print('.');
    delay(500);
  }
  // Use TX_RX mode
  lora.setMode(P2P_RX_MODE, 0);
  lora.config(LORA_FREQ, sf, bw, cr, LORA_CONFIG_PRLEN, LORA_CONFIG_PWR);
  lora.setMode(P2P_TX_RX_MODE);
  Serial.println(" done");

  // Create mutexes
  for (int i = 0; i < 2; i++)
    mutex[i] = xSemaphoreCreateMutex();

  // Init BLE and start task
  miTh.begin();
  Serial.println("[DEBUG] BLE init done");
  xTaskCreate(miThReadingTask, "miThTask", 10000, NULL, 1, NULL);
  Serial.println("[DEBUG] BLE task created");

  // ===== LCD init using M5Unified =====
  M5.Display.setRotation(2);
  M5.Display.fillScreen(TFT_BLACK);
  M5.Display.setTextColor(TFT_WHITE);
  M5.Display.setTextSize(2);
  Serial.println("[DEBUG] LCD initialized (M5Unified)");
  // ======================================

  Serial.println("[DEBUG] Setup complete");
}

void loop()
{
  M5.update();
  lora.update();
  esp_task_wdt_reset();
  // Display waiting status
  if (!haveReadings)
  {
    M5.Display.fillScreen(TFT_BLACK);
    M5.Display.setCursor(0, 0);
    M5.Display.println("Status:\n   Waiting");
    M5.Display.println("BLE scan...");
    vTaskDelay(500 / portTICK_PERIOD_MS);
    return;
  }

  // Safely get data
  MiThData_S d0, d1;
  xSemaphoreTake(mutex[0], 10);
  d0 = miThData[0];
  xSemaphoreGive(mutex[0]);
  xSemaphoreTake(mutex[1], 10);
  d1 = miThData[1];
  xSemaphoreGive(mutex[1]);

  // Fan control
  float HumOut = d0.humidity * 0.01f;
  float HumIn = d1.humidity * 0.01f;

  // Hysteresis control
  float diff = HumIn - HumOut;
  if (!fanOn && diff >= HUM_DIFF_ON)
  {
    // exceeded turn-on threshold → start fan
    ledcWrite(FAN_PWM_CHANNEL, FAN_MAX_SPEED);
    fanOn = true;
    Serial.println("[DEBUG] Fan ON");
  }
  else if (fanOn && diff <= HUM_DIFF_OFF)
  {
    // dropped below turn-off threshold → stop fan
    ledcWrite(FAN_PWM_CHANNEL, 0);
    fanOn = false;
    Serial.println("[DEBUG] Fan OFF");
  }

  // Display readings
  M5.Display.fillScreen(TFT_BLACK);
  delay(100);
  M5.Display.setCursor(0, 0);
  M5.Display.setTextColor(TFT_YELLOW);
  M5.Display.printf("OUT: %.1fC\n", d0.temperature * 0.01f);
  M5.Display.setTextColor(TFT_CYAN);
  M5.Display.printf("  H: %.1f%%\n", d0.humidity * 0.01f);
  M5.Display.setTextColor(TFT_WHITE);
  if (d0.batt_level < 15)
  {
    M5.Display.setTextColor(TFT_RED);
  }
  else
  {
    M5.Display.setTextColor(TFT_WHITE);
  }
  M5.Display.printf("  B: %d%%\n", d0.batt_level);
  M5.Display.setTextColor(TFT_YELLOW);
  M5.Display.printf("INT: %.1fC\n", d1.temperature * 0.01f);
  M5.Display.setTextColor(TFT_CYAN);
  M5.Display.printf("  H: %.1f%%\n", d1.humidity * 0.01f);
  if (d1.batt_level < 15)
  {
    M5.Display.setTextColor(TFT_RED);
  }
  else
  {
    M5.Display.setTextColor(TFT_WHITE);
  }
  M5.Display.printf("  B: %d%%\n", d1.batt_level);

  M5.Display.setTextColor(TFT_WHITE);
  M5.Display.printf("\n");
  if (fanOn)
  {
    M5.Display.printf("Fan: On\n");
  }
  else
  {
    M5.Display.printf("Fan: Off\n");
  }

  // Build ASCII payload
  char buf[25];
  int t1 = (int)round(d0.temperature * 0.1f);
  int h1 = (int)round(d0.humidity * 0.1f);
  int b1 = d0.batt_level == 100 ? 99 : d0.batt_level;
  int t2 = (int)round(d1.temperature * 0.1f);
  int h2 = (int)round(d1.humidity * 0.1f);
  int b2 = d1.batt_level == 100 ? 99 : d1.batt_level;
  int fanState = fanOn ? 1 : 0;
  int len = snprintf(buf, sizeof(buf) - 4, "%1d%03d%03d%02d%1d%03d%03d%02d%1d",
                     SENSOR1_ID, t1, h1, b1, SENSOR2_ID, t2, h2, b2, fanState);

  uint16_t crc = computeCRC16((const uint8_t *)buf, len);
  Serial.printf("[DEBUG] CRC16: 0x%04X\n", crc);
  char crcStr[5];
  snprintf(crcStr, sizeof(crcStr), "%04X", crc);
  memcpy(buf + len, crcStr, 4);
  buf[len + 4] = '\0';
  len += 4;

  // Send via LoRa
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
  // Register this FreeRTOS task with the watchdog
  esp_task_wdt_add(NULL);
  while (1)
  {
    // Feed watchdog from this task
    esp_task_wdt_reset();
    miTh.resetData();
    miTh.getData(5);
    for (int i = 0; i < 2; i++)
    {
      if (xSemaphoreTake(mutex[i], 10) == pdTRUE)
      {
        miThData[i] = miTh.data[i];
        Serial.printf("[DEBUG] Sensor %d: T=%.1fC H=%.1f%% B=%d%%\n", i + 1,
                      miThData[i].temperature * 0.01f,
                      miThData[i].humidity * 0.01f,
                      miThData[i].batt_level);
        xSemaphoreGive(mutex[i]);
      }
    }
    miTh.clearScanResults();
    if (miThData[0].valid && miThData[1].valid)
      haveReadings = true;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
