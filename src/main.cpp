#include <Arduino.h>
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <ArduinoJson.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "credential.h"

// Pin Configuration
#define DHT_PIN 5
#define MQ135_PIN 36
#define MQ7_PIN 34

// Sensor Configuration
#define DHTTYPE DHT11
const float R0_MQ135 = 108.67;
const float R0_MQ7 = 14.91;

// Topic Structure
struct MqttTopics {
  // Environment
  const char* TEMP = "home/living_room/env/temperature";
  const char* HUMIDITY = "home/living_room/env/humidity";
  
  // Air Quality
  const char* AIR = "home/living_room/air/quality";
  const char* CO = "home/living_room/air/co";
  
  // Status
  const char* STATUS = "home/living_room/status";
  const char* LAST_SEEN = "home/living_room/lastseen";
};

// Reading Structure
struct SensorReading {
  const char* type;
  float value;
  const char* unit;
  uint8_t qos;
  bool retain;
};

// Global Objects
AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;
DHT_Unified dht(DHT_PIN, DHTTYPE);
MqttTopics topics;

// Task Handles
TaskHandle_t envTaskHandle = NULL;
TaskHandle_t airQualityTaskHandle = NULL;
TaskHandle_t statusTaskHandle = NULL;

// Function Declarations
void connectToWifi();
void connectToMqtt();
void publishSensorData(const char* topic, SensorReading reading);
void publishWithRetry(const char* topic, const char* payload, uint8_t qos, bool retain, int maxRetries = 3);

// Utility Functions
const char* getTimestamp() {
  static char timestamp[32];
  unsigned long ms = millis();
  snprintf(timestamp, sizeof(timestamp), "%lu", ms);
  return timestamp;
}

// MQTT Publishing Function
void publishSensorData(const char* topic, SensorReading reading) {
  JsonDocument doc;
  char buffer[256];
  
  doc.clear();
  
  doc["d_id"] = DEVICE_ID;
  doc["ts"] = getTimestamp();
  doc["r"] = "living_room";
  doc["t"] = reading.type;
  doc["v"] = reading.value;
  doc["u"] = reading.unit;
  
  serializeJson(doc, buffer);
  publishWithRetry(topic, buffer, reading.qos, reading.retain);
}

void publishWithRetry(const char* topic, const char* payload, uint8_t qos, bool retain, int maxRetries) {
    int retries = 0;
    bool published = false;
    
    while (!published && retries < maxRetries) {
        if (mqttClient.publish(topic, qos, retain, payload)) {
            published = true;
        } else {
            retries++;
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    if (!published) {
        // Log failure
        Serial.printf("Failed to publish to %s after %d retries\n", topic, maxRetries);
    }
}

// Task Implementations
void envTask(void *parameter) {
  for(;;) {
    sensors_event_t event;
    
    // Temperature Reading
    if (dht.temperature().getEvent(&event)) {
      SensorReading reading = {
        .type = "temp",
        .value = event.temperature,
        .unit = "c",
        .qos = 1,
        .retain = false
      };
      publishSensorData(topics.TEMP, reading);
    }
    
    // Humidity Reading
    if (dht.humidity().getEvent(&event)) {
      SensorReading reading = {
        .type = "hum",
        .value = event.relative_humidity,
        .unit = "%",
        .qos = 1,
        .retain = false
      };
      publishSensorData(topics.HUMIDITY, reading);
    }
    
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
}

void airQualityTask(void *parameter) {
  for(;;) {
    // MQ135 Reading
    float airQuality = analogRead(MQ135_PIN) / R0_MQ135;
    SensorReading reading = {
      .type = "air",
      .value = airQuality,
      .unit = "ratio",
      .qos = 1,
      .retain = false
    };
    publishSensorData(topics.AIR, reading);
    
    // MQ7 Reading
    float coLevel = analogRead(MQ7_PIN) / R0_MQ7;
    SensorReading coReading = {
      .type = "co",
      .value = coLevel,
      .unit = "ratio",
      .qos = 1,
      .retain = false
    };
    publishSensorData(topics.CO, coReading);
    
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void statusTask(void *parameter) {
  for(;;) {
    if (mqttClient.connected()) {
      JsonDocument doc;
      char buffer[256];

      doc.clear();
      
      doc["d_id"] = DEVICE_ID;
      doc["ts"] = getTimestamp();
      doc["status"] = "online";
      doc["rssi"] = WiFi.RSSI();
      doc["uptime"] = millis() / 1000;
      
      serializeJson(doc, buffer);
      publishWithRetry(topics.STATUS, buffer, 2, true);
    }
    vTaskDelay(pdMS_TO_TICKS(30000));
  }
}

// WiFi Event Handler
void WiFiEvent(WiFiEvent_t event) {
  switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0);
      xTimerStart(wifiReconnectTimer, 0);
      break;
    default:
      break;
  }
}

// MQTT Event Handlers
void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  
  // Set Last Will and Testament
  JsonDocument doc;
  char buffer[128];
  
  doc["d_id"] = DEVICE_ID;
  doc["ts"] = getTimestamp();
  doc["status"] = "offline";
  
  serializeJson(doc, buffer);
  mqttClient.setWill(topics.STATUS, 2, true, buffer);
  
  // Create tasks
  xTaskCreatePinnedToCore(envTask, "ENV", 4096, NULL, 1, &envTaskHandle, 1);
  xTaskCreatePinnedToCore(airQualityTask, "AIR", 4096, NULL, 1, &airQualityTaskHandle, 1);
  xTaskCreatePinnedToCore(statusTask, "STATUS", 4096, NULL, 1, &statusTaskHandle, 1);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");
  
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
    Serial.println("Connecting to MQTT...");
    mqttClient.connect();
}

void setup() {
    Serial.begin(115200);
    
    dht.begin();
    
    mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
    wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));
    
    WiFi.onEvent(WiFiEvent);
    
    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setKeepAlive(60);
    
    connectToWifi();
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));  // Keep the task alive
}