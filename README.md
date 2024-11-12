# IoT Environmental Monitor (ESP32 / WeMos D1 R32)

Environmental monitoring system using ESP32 with multi-sensor integration and MQTT communication.

## Overview

- Real-time environmental monitoring
- Multi-sensor data collection (DHT11, MQ135, MQ7)
- MQTT communication with QoS and retry mechanism
- FreeRTOS task management
- Automatic reconnection handling

## Hardware Requirements

### Components

- ESP32 Development Board
- DHT11 Temperature & Humidity Sensor
- MQ135 Air Quality Sensor
- MQ7 Carbon Monoxide Sensor

### Pin Configuration

```cpp
DHT11: GPIO 5
MQ135: GPIO 36
MQ7:   GPIO 34
```

## Setup

1. Copy `credential.example.h` to `credential.h`
2. Update credentails
```
WIFI_SSID
WIFI_PASSWORD
MQTT_HOST
MQTT_PORT
DEVICE_ID
```

## MQTT Topics

### Publishing

```
home/living_room/env/temperature
home/living_room/env/humidity
home/living_room/air/quality
home/living_room/air/co
home/living_room/status
```

### Message Format

```json
{
  "d_id": "DEVICE_ID",
  "ts": "timestamp",
  "r": "room",
  "t": "reading_type",
  "v": "value",
  "u": "unit"
}
```
