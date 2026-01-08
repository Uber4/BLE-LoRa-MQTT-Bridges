// secrets.h
#pragma once

// WiFi credentials
#define WIFI_SSID "myssid"
#define WIFI_PASSWORD "mypass"

// MQTT broker configuration (TLS)
#define MQTT_SERVER "192.168.1.100"
#define MQTT_SECURE_PORT 8883 // TLS port
#define MQTT_PORT 1883        // Default port
#define MQTT_USER "mqttuser"
#define MQTT_PASSWORD "mqttpass"

// Root CA certificate (PEM)
const char *MQTT_ROOT_CA = R"EOF(
-----BEGIN CERTIFICATE-----
MIID...........AB
-----END CERTIFICATE-----
)EOF";