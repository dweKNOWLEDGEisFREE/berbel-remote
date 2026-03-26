#pragma once

// WiFi
#define WIFI_SSID "HomeIoT"
#define WIFI_PASS "FdsaJkl121212"

// MQTT
#define MQTT_HOST "192.168.178.10"
#define MQTT_PORT 1883
#define MQTT_USER "svcmqtt"
#define MQTT_PASS "FdsaJkl#121212"

// Hood features
// Set to false if your hood has no retractable cover (lift function).
// When false, Position, Hochfahren, Herunterfahren, and Cover State
// entities will not be created in Home Assistant.
#define HOOD_HAS_COVER true
