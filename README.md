# BLE-LoRa-MQTT Bridges

This project implements a wireless sensor bridge using M5Atom devices and LoRa communication. The SenderS3 sketch reads environmental data (temperature, humidity, battery level) from Xiaomi BLE thermometers and transmits it over LoRa P2P, while also controlling a PWM fan based on temperature. The Receiver sketch listens for LoRa frames and forwards the received sensor data to an MQTT broker over WiFi, enabling integration with home automation systems or dashboards. 

This repository provides two sketches for ESP32 based devices. M5Atom modules are chosen for their compact size, ease of installation, and integrated features.

1. **SenderS3**  
   Reads temperature, humidity and battery level from two Xiaomi Mi Thermometers via BLE and sends the data over LoRa P2P. Displays readings on screen (using M5stack AtomS3).
   Includes PWM fan control based on temperature thresholds.

3. **Receiver**  
   Receives LoRa P2P frames and republishes the sensor readings (and LoRa metrics) to an MQTT broker over WiFi.

---

## Hardware Requirements

- **SenderS3**

  - M5stack AtomS3 with LCD for conveninece: https://docs.m5stack.com/en/core/AtomS3
  - LoRaWAN Unit (CN470 / EU868 / US915 / AS923, based on your region): https://docs.m5stack.com/en/unit/Unit%20LoRaWAN-EU868   
  - Some Xiaomi LYWSD03MMC or other ATC_MiThermometer compatible thermometers: https://github.com/atc1441/ATC_MiThermometer
  - PWM-controlled fan, or any fan with PWM driver (connected to PWM GPIO)

- **Receiver**
  - M5stack Atom Lite (any ESP32/ESP8266 should work) https://docs.m5stack.com/en/core/ATOM%20Lite
  - LoRaWAN Unit (same as sender)
  - WiFi network with access to an MQTT broker

---
```text
                   (Xiaomi LYWSD03MMC BLE Thermometers)
                   ┌───────────────┐   ┌───────────────┐
 BLE adv/data) ──▶│ Thermometer 1 │    │ Thermometer 2 │◀─ BLE (adv/data)
                   └───────────────┘   └───────────────┘
                              \             /
                               \           /
                                v         v
                         ┌─────────────────────────────┐           ┌──────────────┐
                         │          SENDER S3          │── PWM ──▶│     FAN      │
                         │  M5Stack AtomS3 with LCD    │  GPIO     │  (PWM drive) │
                         │  + LoRaWAN Unit             │           └──────────────┘
                         │  - Reads T°, H%, Battery    │
                         │  - Displays on LCD          │
                         │  - Controls FAN via PWM     │
                         │  - LoRa sends data + CRC    │
                         └───────────┬─────────────────┘
                                     │
                            LoRa P2P │  (frames with sensor data + metrics)
                                     ▼
                         ┌─────────────────────────────┐
                         │           RECEIVER          │
                         │   M5Stack Atom + LoRaWAN    │
                         │-----------------------------│
                         │  - Listens LoRa P2P         │
                         │  - Checks CRC               │
                         │  - WiFi + MQTT publish      │
                         └───────────┬─────────────────┘
                                     │ WiFi
                                     ▼
                           ┌───────────────────────┐
                           │       MQTT BROKER     │
                           │    HOME AUTOMATION    │
                           │      DASHBOARDS...    │
                           └───────────────────────┘
```
---

## Software Dependencies

Install via Arduino Library Manager or PlatformIO:

- **M5Atom** (`M5Atom@^0.1.3`)  
  https://github.com/m5stack/M5Atom
- **M5-LoRaWAN-RAK**  
  https://github.com/m5stack/M5-LoRaWAN-RAK
- **ATC_MiThermometer** (only for sensor-ble-lora)  
  https://github.com/matthias-bs/ATC_MiThermometer
- **WiFi** and **PubSubClient** (only for gateway-lora-mqtt)  
  Included in Arduino Library Manager

---

## Installation & Usage

1. **Clone the repo on VS Code with PlatformIO installed**
2. **Flash ATC_MiThermometer firmware in BLE thermometers (check https://github.com/atc1441/ATC_MiThermometer)**
3. **Compile and upload to the devices**
