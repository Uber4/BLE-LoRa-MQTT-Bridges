/*
 * @Hardware: M5Atom + Unit LoRaWAN CN470, EU868, US915, AS923
 * @Dependent Libraries:
 *   - M5Atom@^0.1.3: https://github.com/m5stack/M5Atom
 *   - M5-LoRaWAN-RAK: https://github.com/m5stack/M5-LoRaWAN-RAK
 */

// MQTT branch

#include <M5Atom.h>
#include "rak3172_p2p.hpp"
#include <stdint.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include "secrets.h"

// LoRa radio settings
#define PIN_M0 19 // LoRa module M0 pin
#define PIN_M1 22 // LoRa module M1 pin

#define LORA_FREQ 868E6     // Frequency in Hz
#define LORA_CONFIG_PRLEN 8 // Preamble
#define LORA_CONFIG_PWR 5   // Tx power
int cr = 1;
int sf = 10;  // Spreading factor
int bw = 250; // Frequency

RAK3172P2P lora; // LoRa P2P interface

// Global MQTT objects
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// CRC16-CCITT calculation function (polynomial 0x1021)
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

// Ensure MQTT connection with credentials
void ensureMqttConnected()
{
  // Loop until we're connected
  while (!mqttClient.connected())
  {
    Serial.print("[MQTT] Attempting connection...");
    // Attempt to connect with client ID and credentials
    if (mqttClient.connect("LoRa-Client", MQTT_USER, MQTT_PASSWORD))
    {
      Serial.println(" connected!");
      // No discovery publish needed here
    }
    else
    {
      // Print the error code and retry after 2 seconds
      Serial.print("[MQTT] Connection failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" — retrying in 2s");
      delay(2000);
    }
  }
}

// Publish readings for a given sensor to separate topics
void publishReadings(int sensorId,
                     float temperature,
                     float humidity,
                     int batt,
                     int rssi,
                     int snr,
                     bool includeLoRaMetrics,
                     bool fan_is_on)
{
  char topicTemp[32], topicHum[32], topicBatt[32];
  char msgTemp[64], msgHum[64], msgBatt[64];

  // Build topics and payloads
  snprintf(topicTemp, sizeof(topicTemp), "blelorasensors/%d/temperature", sensorId);
  snprintf(msgTemp, sizeof(msgTemp), "{\"temp\":%.1f}", temperature);

  snprintf(topicHum, sizeof(topicHum), "blelorasensors/%d/humidity", sensorId);
  snprintf(msgHum, sizeof(msgHum), "{\"hum\":%.1f}", humidity);

  snprintf(topicBatt, sizeof(topicBatt), "blelorasensors/%d/battery", sensorId);
  snprintf(msgBatt, sizeof(msgBatt), "{\"batt\":%d}", batt);

  // Publish temperature
  if (mqttClient.publish(topicTemp, msgTemp, true))
    Serial.printf("[MQTT] %s → %s\n", topicTemp, msgTemp);
  else
    Serial.printf("[MQTT] ERROR %s\n", topicTemp);

  // Publish humidity
  if (mqttClient.publish(topicHum, msgHum, true))
    Serial.printf("[MQTT] %s → %s\n", topicHum, msgHum);
  else
    Serial.printf("[MQTT] ERROR %s\n", topicHum);

  // Publish battery
  if (mqttClient.publish(topicBatt, msgBatt, true))
    Serial.printf("[MQTT] %s → %s\n", topicBatt, msgBatt);
  else
    Serial.printf("[MQTT] ERROR %s\n", topicBatt);

  // Only if includeLoRaMetrics==true, publish RSSI, SNR and FanStatus
  if (includeLoRaMetrics)
  {
    char topicRSSI[32], topicSNR[32];
    char msgRSSI[64], msgSNR[64];
    char topicFanStatus[32], msgFanStatus[16];

    snprintf(topicRSSI, sizeof(topicRSSI), "blelorasensors/rssi");
    snprintf(msgRSSI, sizeof(msgRSSI), "{\"rssi\":%d}", rssi);
    if (mqttClient.publish(topicRSSI, msgRSSI, true))
      Serial.printf("[MQTT] %s → %s\n", topicRSSI, msgRSSI);

    snprintf(topicSNR, sizeof(topicSNR), "blelorasensors/snr");
    snprintf(msgSNR, sizeof(msgSNR), "{\"snr\":%d}", snr);
    if (mqttClient.publish(topicSNR, msgSNR, true))
      Serial.printf("[MQTT] %s → %s\n", topicSNR, msgSNR);

    snprintf(topicFanStatus, sizeof(topicFanStatus), "blelorasensors/fanstatus");
    snprintf(msgFanStatus, sizeof(msgFanStatus), "{\"fan\":\"%s\"}", fan_is_on ? "ON" : "OFF");
    if (mqttClient.publish(topicFanStatus, msgFanStatus, true))
      Serial.printf("[MQTT] %s → %s\n", topicFanStatus, msgFanStatus);
  }
}

// Blinking (colour(0x000000), interval(ms), times(N))
void blinkColor(uint32_t color, uint32_t interval, uint8_t times = 1)
{
  for (uint8_t i = 0; i < times; i++)
  {
    M5.dis.fillpix(color);
    vTaskDelay(pdMS_TO_TICKS(interval));
    M5.dis.fillpix(0x000000);
    vTaskDelay(pdMS_TO_TICKS(interval));
  }
}

void setup()
{
  // Initialize M5Atom and Serial for debug messages
  delay(5000);
  M5.begin(true, false, true);
  Serial.begin(115200);
  Serial.println("[DEBUG] RX Setup started");
  M5.dis.fillpix(0x0000ff); // Blue: initialization stage

  esp_task_wdt_init(20, true); // Initialize watchdog with 20-second timeout, panic on trigger
  esp_task_wdt_add(NULL);      // Register the current (loop) task with the watchdog

  // Connect to WiFi
  Serial.print("[WiFi] Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  // Wait until connected
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.print("[WIFI] Connected, IP: ");
  Serial.println(WiFi.localIP());

  // === MQTT SETUP ===
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  // initial connect for debug
  Serial.println("[MQTT] Attempting initial connection...");
  ensureMqttConnected();

  // Initialize LoRa module in P2P RX mode
  Serial.print("[DEBUG] Initializing LoRa...");
  while (!lora.init(&Serial2, PIN_M0, PIN_M1, RAK3172_BPS_115200))
  {
    Serial.print('.');
    delay(500);
  }
  // Use TX_RX mode to continuously receive
  lora.setMode(P2P_RX_MODE, 0);
  lora.config(LORA_FREQ, sf, bw, cr, LORA_CONFIG_PRLEN, LORA_CONFIG_PWR);
  lora.setMode(P2P_TX_RX_MODE);

  Serial.println(" done");
  M5.dis.drawpix(0, 0xffff00); // Yellow: waiting for packets
  Serial.println("[DEBUG] LoRa initialized, ready to receive");
}

void loop()
{
  esp_task_wdt_reset(); // Reset watchdog timer to prevent system reset

  // === MQTT KEEP-ALIVE ===
  ensureMqttConnected();
  mqttClient.loop();

  // Update M5Atom and process incoming AT events
  M5.update();
  lora.update();

  // Check if parsed frames are available
  int count = lora.available();
  if (count > 0)
  {
    Serial.printf("[DEBUG] %d frame(s) available, reading...\n", count);
    // Retrieve all frames
    auto frames = lora.read();
    for (auto &frame : frames)
    {
      // Convert payload to String
      String payload = String(frame.payload);
      // Delete "/r", "/n", blank spaces...
      payload.trim();
      int len = payload.length();

      // Verify payload lenght (19 chars) + CRC16 at end (4 hex chars)
      if (len != 23)
      {
        Serial.printf("[ERROR] Payload not valid (%d bytes, expected 23)\n", len);
        blinkColor(0xff0000, 200, 5); // Blink red 5 times in 200ms to indicate received not valid
        continue;
      }
      String dataPart = payload.substring(0, len - 4);
      String crcStr = payload.substring(len - 4);
      uint16_t recvCrc = (uint16_t)strtol(crcStr.c_str(), NULL, 16);
      uint16_t calcCrc = computeCRC16((const uint8_t *)dataPart.c_str(), dataPart.length());

      if (recvCrc != calcCrc)
      {
        Serial.printf("[ERROR] CRC mismatch: recv=0x%04X calc=0x%04X\n", recvCrc, calcCrc);
        blinkColor(0xff0000, 200, 3); // Blink red 3 times in 200ms to indicate CRC not valid
        continue;
      }
      Serial.println("[DEBUG] CRC valid, parsing sensor data");

      // Extract fan status to publish it once
      uint8_t fanState = dataPart.charAt(dataPart.length() - 1) - '0';
      bool fanOn = (fanState == 1);
      int idx = 0;
      bool firstSensor = true;
      // Parse two sensor blocks
      for (int i = 0; i < 2; i++)
      {
        uint8_t sensorId = dataPart.charAt(idx++) - '0';
        int t = dataPart.substring(idx, idx + 3).toInt();
        idx += 3;
        int h = dataPart.substring(idx, idx + 3).toInt();
        idx += 3;
        int batt = dataPart.substring(idx, idx + 2).toInt();
        idx += 2;

        float temperature = t / 10.0f;
        float humidity = h / 10.0f;

        Serial.printf("[DEBUG] Sensor %d -> T=%.1f°C, H=%.1f%%, B=%d%%, Fan=%s\n",
                      sensorId, temperature, humidity, batt, fanOn ? "ON" : "OFF");

        // publish sensorId, temperature, humidity, batt...
        publishReadings(sensorId,
                        temperature,
                        humidity,
                        batt,
                        frame.rssi,
                        frame.snr,
                        firstSensor, // Only first one will publish RSSI/SNR/fanOn
                        fanOn);
        firstSensor = false;
      }
      Serial.println("[DEBUG] Frame processed\n");
      blinkColor(0x00ff00, 200, 1); // Blink green 1 time in 200ms
    }
    // Clear processed frames
    lora.flush();
  }
}
