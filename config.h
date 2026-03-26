#pragma once

// WiFi
#define WIFI_SSID "WIFI-NET"
#define WIFI_PASS "WIFI-PASS"

// MQTT
#define MQTT_HOST "192.168.xxx.xxx"
#define MQTT_PORT 1883
#define MQTT_USER "mqtt-user"
#define MQTT_PASS "mqtt-pass"

// Hood features
// Set to false if your hood has no retractable cover (lift function).
// When false, Position, Hochfahren, Herunterfahren, and Cover State
// entities will not be created in Home Assistant.
#define HOOD_HAS_COVER true
