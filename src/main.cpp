#include <Arduino.h>
#include <ThingsBoard.h>
#include <WiFi.h>
#include <Arduino_MQTT_Client.h>
#include <OTA_Firmware_Update.h>
#include <Shared_Attribute_Update.h>
#include <Attribute_Request.h>
#include <Espressif_Updater.h>
#include <DHT.h>
#include <Wire.h>
#include <HCSR04.h>
#include <ESP32Servo.h>
#include "RPC_Callback.h"

#define CONFIG_THINGSBOARD_ENABLE_DEBUG   false
#define LED_PIN                           GPIO_NUM_48
#define RL                                1.0  
#define VCC                               3.3 
#define Ro                                10.0 

constexpr int16_t TELEMETRY_SEND_INTERVAL = 5000U;
constexpr char CURRENT_FIRMWARE_TITLE[] = "OTA_Lab03";
constexpr char CURRENT_FIRMWARE_VERSION[] = "1.0.2";
constexpr uint8_t FIRMWARE_FAILURE_RETRIES = 12U;
constexpr uint16_t FIRMWARE_PACKET_SIZE = 4096U;

constexpr char WIFI_SSID[] = "HTH";
constexpr char WIFI_PASSWORD[] = "14062004";
constexpr char TOKEN[] = "YiqDJ4j5B7VxESqcdra5";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr char TEMPERATURE_KEY[] = "Temperature";
constexpr char HUMIDITY_KEY[] = "Humidity";

constexpr uint16_t THINGSBOARD_PORT = 1883U;
constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 512U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 512U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;
constexpr uint64_t REQUEST_TIMEOUT_MICROSECONDS = 10000U * 1000U;
constexpr int16_t telemetrySendInterval = 3000U;
constexpr uint16_t BLINKING_INTERVAL_MS_MIN = 10U;
constexpr uint16_t BLINKING_INTERVAL_MS_MAX = 60000U;
volatile uint16_t blinkingInterval = 1000U;
constexpr char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
constexpr char LED_MODE_ATTR[] = "ledMode";
constexpr char LED_STATE_ATTR[] = "ledState";
constexpr uint8_t MAX_ATTRIBUTES = 2U;
constexpr std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST = {
  LED_STATE_ATTR,
  BLINKING_INTERVAL_ATTR
};

WiFiClient espClient;
Arduino_MQTT_Client mqttClient(espClient);

OTA_Firmware_Update<> ota;
Shared_Attribute_Update<1U, MAX_ATTRIBUTES> shared_update;
Attribute_Request<2U, MAX_ATTRIBUTES> attr_request;
const std::array<IAPI_Implementation*, 3U> apis = { &shared_update, &attr_request, &ota };
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);
Espressif_Updater<> updater;

volatile bool attributesChanged = false;
volatile bool ledState = false;
volatile bool fanState = false;
bool shared_update_subscribed = false;
bool currentFWSent = false;
bool updateRequestSent = false;
bool requestedShared = false;

void processSharedAttributes(const JsonObjectConst &data) {
  Serial.println("Process shared attributes");
  if (data.containsKey(BLINKING_INTERVAL_ATTR)) {
    const uint16_t new_interval = data[BLINKING_INTERVAL_ATTR].as<uint16_t>();
    if (new_interval >= BLINKING_INTERVAL_MS_MIN && new_interval <= BLINKING_INTERVAL_MS_MAX) {
      blinkingInterval = new_interval;
      Serial.print("Blinking interval: ");
      Serial.println(new_interval);
    }
  }

  if (data.containsKey(LED_STATE_ATTR)) {
    ledState = data[LED_STATE_ATTR].as<bool>();
    digitalWrite(LED_PIN, bool(ledState));
    Serial.print("LED state: ");
    Serial.println(ledState);
  }

  attributesChanged = true;
}

void requestTimedOut() {
  Serial.printf("Attribute request timed out after %llu microseconds.\n", REQUEST_TIMEOUT_MICROSECONDS);
}

void InitWiFi() {
  Serial.println("Connecting to AP ...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to AP");
}

void update_starting_callback() {}

void finished_callback(const bool & success) {
  if (success) {
    Serial.println("Done, Reboot now");
    esp_restart();
  } else {
    Serial.println("Downloading firmware failed");
  }
}

void progress_callback(const size_t & current, const size_t & total) {
  Serial.printf("Progress %.2f%%\n", static_cast<float>(current * 100U) / total);
}

void processSharedAttributeUpdate(const JsonObjectConst &data) {
  const size_t jsonSize = Helper::Measure_Json(data);
  char buffer[jsonSize];
  serializeJson(data, buffer, jsonSize);
  Serial.println(buffer);
}

void processSharedAttributeRequest(const JsonObjectConst &data) {
  const size_t jsonSize = Helper::Measure_Json(data);
  char buffer[jsonSize];
  serializeJson(data, buffer, jsonSize);
  Serial.println(buffer);
}

void WiFiTask(void *pvParameters) {
  InitWiFi();
  while (true) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi disconnected, reconnecting...");
      WiFi.reconnect();
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
  }
}

void ThingsBoardTask(void *pvParameters) {
  while (true) {
    if (!tb.connected()) {
      Serial.printf("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, TOKEN);
      if (tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {
        Serial.println("Connected to ThingsBoard");
        if (!requestedShared) {
          const Attribute_Request_Callback<MAX_ATTRIBUTES> sharedCallback(&processSharedAttributeRequest, REQUEST_TIMEOUT_MICROSECONDS, &requestTimedOut, SHARED_ATTRIBUTES_LIST);
          requestedShared = attr_request.Shared_Attributes_Request(sharedCallback);
        }
        if (!shared_update_subscribed) {
          const Shared_Attribute_Callback<MAX_ATTRIBUTES> callback(&processSharedAttributeUpdate, SHARED_ATTRIBUTES_LIST);
          shared_update_subscribed = shared_update.Shared_Attributes_Subscribe(callback);
        }
      } else {
        Serial.println("Failed to connect");
        vTaskDelay(pdMS_TO_TICKS(5000));
        continue;
      }
    }
    if (!currentFWSent) {
      currentFWSent = ota.Firmware_Send_Info(CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION);
    }
    if (!updateRequestSent) {
      const OTA_Update_Callback callback(
        CURRENT_FIRMWARE_TITLE, CURRENT_FIRMWARE_VERSION,
        &updater,
        &finished_callback,
        &progress_callback,
        &update_starting_callback,
        FIRMWARE_FAILURE_RETRIES,
        FIRMWARE_PACKET_SIZE
      );
      bool started = ota.Start_Firmware_Update(callback);
      bool subscribed = ota.Subscribe_Firmware_Update(callback);
      if (started && subscribed) {
        Serial.println("Firmware Update Started & Subscribed.");
        updateRequestSent = true;
      } else {
        Serial.println("Firmware Update FAILED to start or subscribe.");
      }
    }    
    tb.loop();
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(SERIAL_DEBUG_BAUD);
  delay(1000);
  xTaskCreatePinnedToCore(WiFiTask, "WiFiTask", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(ThingsBoardTask, "ThingsBoardTask", 8192, NULL, 1, NULL, 1);
}

void loop() {
  //
}
