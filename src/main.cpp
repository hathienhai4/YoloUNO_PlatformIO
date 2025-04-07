#include <WiFi.h>
#include <PubSubClient.h>
#include <Arduino_MQTT_Client.h>
#include <ThingsBoard.h>
#include "DHT.h"
#include <Wire.h>
#include <ArduinoOTA.h>
#include <HCSR04.h>
#include <ESP32Servo.h>

#define FIRMWARE_VERSION "1.0.2"

#define MQ_PIN      1
#define SERVO_PIN   5
#define DHT_PIN     6
#define TRIG_PIN    8
#define ECHO_PIN    9
#define FAN_PIN     10
#define SDA_PIN     11
#define SCL_PIN     12
#define LED_PIN     48

#define RL 1.0  // Điện trở tải (kΩ)
#define VCC 3.3 // Điện áp cấp cho cảm biến (ESP32 dùng 3.3V)
#define Ro 10.0 // Hiệu chuẩn trước đó (Ro đo trong không khí sạch)

constexpr char WIFI_SSID[] = "HTH";
constexpr char WIFI_PASSWORD[] = "14062004";
constexpr char TOKEN[] = "YiqDJ4j5B7VxESqcdra5";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";

constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 256U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 256U;

constexpr uint32_t MAX_MESSAGE_SIZE = 1024U;
constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;
constexpr int16_t telemetrySendInterval = 3000U;

constexpr char BLINKING_INTERVAL_ATTR[] = "blinkingInterval";
constexpr char LED_MODE_ATTR[] = "ledMode";
constexpr char LED_STATE_ATTR[] = "ledState";

volatile bool attributesChanged = false;
volatile bool ledState = false;
volatile int ledMode = 0;
volatile bool fanState = false;

constexpr uint16_t BLINKING_INTERVAL_MS_MIN = 10U;
constexpr uint16_t BLINKING_INTERVAL_MS_MAX = 60000U;
volatile uint16_t blinkingInterval = 1000U;

uint32_t previousStateChange;
uint32_t previousDataSend;

constexpr std::array<const char *, 2U> SHARED_ATTRIBUTES_LIST = {
  LED_STATE_ATTR,
  BLINKING_INTERVAL_ATTR
};

WiFiClient wifiClient;
Arduino_MQTT_Client mqttClient(wifiClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_SIZE);
DHT dht11(DHT_PIN, DHT11);
UltraSonicDistanceSensor distanceSensor(TRIG_PIN, ECHO_PIN);
Servo myServo;

TaskHandle_t wifiTaskHandle, mqttTaskHandle, sensorTaskHandle, ledTaskHandle, fanTaskHandle, distanceTaskHandle, servoTaskHandle;

RPC_Response setLedSwitchState(const RPC_Data &data) {
  Serial.println("Received Switch state");
  bool newState = data;
  Serial.print("Switch state change: ");
  Serial.println(newState);
  digitalWrite(LED_PIN, newState);
  attributesChanged = true;
  return RPC_Response("setLedSwitchValue", newState);
}

const std::array<RPC_Callback, 1U> callbacks = {
  RPC_Callback{ "setLedSwitchValue", setLedSwitchState }
};

void processSharedAttributes(const Shared_Attribute_Data &data) {
  for (auto it = data.begin(); it != data.end(); ++it) {
    if (strcmp(it->key().c_str(), BLINKING_INTERVAL_ATTR) == 0) {
      const uint16_t new_interval = it->value().as<uint16_t>();
      if (new_interval >= BLINKING_INTERVAL_MS_MIN && new_interval <= BLINKING_INTERVAL_MS_MAX) {
        blinkingInterval = new_interval;
        Serial.print("Blinking interval is set to: ");
        Serial.println(new_interval);
      }
    } else if (strcmp(it->key().c_str(), LED_STATE_ATTR) == 0) {
      ledState = it->value().as<bool>();
      digitalWrite(LED_PIN, ledState);
      Serial.print("LED state is set to: ");
      Serial.println(ledState);
    }
  }
  attributesChanged = true;
}

const Shared_Attribute_Callback attributes_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());
const Attribute_Request_Callback attribute_shared_request_callback(&processSharedAttributes, SHARED_ATTRIBUTES_LIST.cbegin(), SHARED_ATTRIBUTES_LIST.cend());

void SharedAttributesTask(void *pvParameters) {
  bool isSubcribed = false;
  while (true) {
    if (WiFi.status() == WL_CONNECTED && tb.connected()) {
      if(!isSubcribed){
        // Serial.println("Requesting shared attributes...");
        // tb.Shared_Attributes_Request(attribute_shared_request_callback);  // Yêu cầu dữ liệu ban đầu

        Serial.println("Subscribing to shared attributes updates...");
        tb.Shared_Attributes_Request(attribute_shared_request_callback);
        tb.Shared_Attributes_Subscribe(attributes_callback);  // Đăng ký nhận cập nhật từ server
        isSubcribed = true;
      }

      // vTaskDelete(NULL); // Xóa task sau khi hoàn thành
    }
    else {
      Serial.println("MQTT disconnected, reconnecting...");
      isSubcribed = false;
    }
    vTaskDelay(telemetrySendInterval / portTICK_PERIOD_MS);  // Kiểm tra mỗi 3 giây
  }
}

void taskWiFi(void *pvParameters)
{
  Serial.println("Connecting to AP ...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (true)
  {
    // Serial.printf("WiFi status: %d\n", WiFi.status());
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("WiFi disconnected, reconnecting...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void otaTask(void *parameter)
{
  Serial.println("[DEBUG] OTA Task Started...");

  ArduinoOTA.setPort(3232);            // Đặt cổng OTA
  ArduinoOTA.setHostname("MyESP32");   // Đặt hostname cho thiết bị
  ArduinoOTA.setRebootOnSuccess(true); // Tự động reboot sau khi cập nhật thành công

  ArduinoOTA.onStart([]()
                     { Serial.println("[DEBUG] OTA Update Started..."); });
  ArduinoOTA.onEnd([]()
                   { Serial.println("[DEBUG] OTA Update Completed! Restarting..."); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("[DEBUG] OTA Progress: %u%%\n", (progress * 100) / total); });
  ArduinoOTA.onError([](ota_error_t error)
                     { 
                       Serial.printf("[DEBUG] OTA Error[%u]: ", error);
                       if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
                       else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
                       else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
                       else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
                       else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

  ArduinoOTA.begin();
  Serial.println("[DEBUG] OTA Initialized!");

  while (true)
  {
    ArduinoOTA.handle();

    // Nếu không có OTA nào đang chạy, ta có thể in trạng thái để kiểm tra
    static uint32_t lastPrintTime = 0;
    if (millis() - lastPrintTime > 5000) // In mỗi 5 giây
    {
      Serial.println("[DEBUG] OTA Task Running... Waiting for update...");
      lastPrintTime = millis();
    }

    vTaskDelay(100 / portTICK_PERIOD_MS); // Giữ CPU mượt mà
  }
}

void TaskPrintTemp_Humi(void *pvParameters)
{
  Wire.begin(11, 12);
  
  while (1)
  {
    sensors_event_t event;
    dht.temperature().getEvent(&event);

    if (isnan(event.temperature)) {
        Serial.println(F("Error reading temperature!"));
    }
    else {
        Serial.print(F("Temperature: "));
        Serial.print(event.temperature);
        Serial.println(F("°C"));
    }
    // Get humidity event and print its value.
    dht.humidity().getEvent(&event);
    if (isnan(event.relative_humidity)) {
        Serial.println(F("Error reading humidity!"));
    }
    else {
        Serial.print(F("Humidity: "));
        Serial.print(event.relative_humidity);
        Serial.println(F("%"));
    }
    vTaskDelay(telemetrySendInterval / portTICK_PERIOD_MS);
  }
}

void taskDoorControl(void *pvParameters)
{
  while (true)
  {
    float distance = distanceSensor.measureDistanceCm();
    Serial.printf("Distance: %.2f cm\n", distance);
    if (distance < 15)
    {
      myServo.write(0);
      vTaskDelay(telemetrySendInterval / portTICK_PERIOD_MS);
    }
    else
    {
      myServo.write(90);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    tb.sendTelemetryData("distance", distance);
  }
}

void taskFanControl(void *pvParameters)
{
  while (true)
  {
    digitalWrite(FAN_PIN, fanState ? HIGH : LOW);
    // Serial.printf("Fan state: %d\n", fanState);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void TaskBlinkLed(void *pvParameters)
{
  while (true)
  {
    Serial.println("My MSSV: 2210882");
    tb.sendAttributeData("ledState", ledState);
    vTaskDelay(telemetrySendInterval / portTICK_PERIOD_MS);
  }
}

RPC_Response setLedState(const RPC_Data &data)
{
  Serial.println("Received Switch state");
  bool newState = data;
  Serial.print("Switch state change: ");
  Serial.println(newState);
  digitalWrite(LED_PIN, newState);
  ledState = newState;
  return RPC_Response("setLedValue", newState);
}

RPC_Response setFanState(const RPC_Data &data)
{
  bool newState = data;
  Serial.printf("Received fan state: %d\n", newState);
  fanState = newState;
  return RPC_Response("setFanValue", fanState);
}

RPC_Response getLedState(const RPC_Data &data)
{
  return RPC_Response("getLedValue", ledState);
}

RPC_Response getFanState(const RPC_Data &data)
{
  return RPC_Response("getFanValue", fanState);
}

const std::array<RPC_Callback, 4U> callbacks = {
    RPC_Callback{"setLedValue", setLedState},
    RPC_Callback{"getLedValue", getLedState},
    RPC_Callback{"setFanValue", setFanState},
    RPC_Callback{"getFanValue", getFanState}};

void taskMQTT(void *pvParameters)
{
  while (true)
  {
    if (!tb.connected())
    {
      Serial.println("Connecting to ThingsBoard...");
      if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT))
      {
        Serial.println("Failed to connect");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        continue;
      }
      Serial.println("Connected to ThingsBoard");
      tb.RPC_Subscribe(callbacks.cbegin(), callbacks.cend());
    }
    tb.loop();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void taskMQ(void *pvParameters)
{
  pinMode(MQ_PIN, INPUT);
  while (true)
  {
    int adcValue = analogRead(MQ_PIN) % 2000;
    float Vout = (adcValue * VCC) / 4095.0;
    float Rs = ((VCC - Vout) / Vout) * RL;
    float ratio = Rs / Ro; // Rs/Ro

    // Tính toán ppm theo công thức của LPG
    float COValue = pow(10, (-2.862 * log10(ratio) + 1.578));
    Serial.printf("MQ Sensor Value: %.2f\n", COValue);
    Serial.printf("ADC Value: %d\n", adcValue);
    tb.sendTelemetryData("mq_value", COValue);
    vTaskDelay(telemetrySendInterval / portTICK_PERIOD_MS);
  }
}

void taskServerControl(void *pvParameters)
{
  while (true)
  {
    tb.sendAttributeData("ledState", ledState);
    tb.sendAttributeData("fanState", fanState);
    tb.sendAttributeData("version", FIRMWARE_VERSION);
    vTaskDelay(telemetrySendInterval / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(115200);
  delay(2000);
  dht11.begin();
  pinMode(LED_PIN, OUTPUT);
  pinMode(FAN_PIN, OUTPUT);
  myServo.attach(SERVO_PIN);

  xTaskCreate(taskWiFi, "WiFiTask", 4096, NULL, 2, &wifiTaskHandle);
  xTaskCreate(taskMQTT, "MQTTTask", 4096, NULL, 2, &mqttTaskHandle);
  // xTaskCreate(otaTask, "OTA Task", 16384, NULL, 2, NULL);
  xTaskCreate(TaskPrintTemp_Humi, "DHTTask", 4096, NULL, 2, &sensorTaskHandle);
  xTaskCreate(SharedAttributesTask, "SharedAttributesTask", 4096, NULL, 2, NULL);
  // xTaskCreate(taskDoorControl, "DoorControlTask", 4096, NULL, 2, &distanceTaskHandle);
  // xTaskCreate(taskServoControl, "ServoTask", 4096, NULL, 1, &servoTaskHandle);
  // xTaskCreate(TaskBlinkLed, "LEDControlTask", 4096, NULL, 3, &ledTaskHandle);
  // xTaskCreate(taskFanControl, "FanControlTask", 4096, NULL, 3, &fanTaskHandle);
  // xTaskCreate(taskServerControl, "ServerControlTask", 4096, NULL, 3, NULL); // send attribute to server
  // xTaskCreate(taskMQ, "MQTask", 4096, NULL, 2, NULL);                       // air quality sensor
}

void loop()
{
  // 
}