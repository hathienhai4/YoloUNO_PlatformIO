#include <Arduino.h>
#include <DHT20.h>

#ifdef ESP32
#include <WiFi.h>
#include <WiFiClientSecure.h>
#endif // ESP32

#include <Arduino_MQTT_Client.h>
#include <Server_Side_RPC.h>
#include <ThingsBoard.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>

#define ENCRYPTED   false
#define DHTPIN      6 
#define DHTTYPE     DHT11

DHT_Unified dht(DHTPIN, DHTTYPE);

constexpr char WIFI_SSID[] = "ACLAB";
constexpr char WIFI_PASSWORD[] = "ACLAB2023";
constexpr char TOKEN[] = "YiqDJ4j5B7VxESqcdra5";
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";

#if ENCRYPTED
constexpr uint16_t THINGSBOARD_PORT = 8883U;
#else
constexpr uint16_t THINGSBOARD_PORT = 1883U;
#endif

constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 256U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 256U;

constexpr uint32_t SERIAL_DEBUG_BAUD = 115200U;
constexpr int16_t TELEMETRY_SEND_INTERVAL = 5000U;


uint32_t previousTelemetrySend;

constexpr char TEMPERATURE_KEY[] = "Temperature";
constexpr char HUMIDITY_KEY[] = "Humidity";

#if ENCRYPTED
constexpr char ROOT_CERT[] = R"(-----BEGIN CERTIFICATE-----
MIIFazCCA1OgAwIBAgIRAIIQz7DSQONZRGPgu2OCiwAwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMTUwNjA0MTEwNDM4
WhcNMzUwNjA0MTEwNDM4WjBPMQswCQYDVQQGEwJVUzEpMCcGA1UEChMgSW50ZXJu
ZXQgU2VjdXJpdHkgUmVzZWFyY2ggR3JvdXAxFTATBgNVBAMTDElTUkcgUm9vdCBY
MTCCAiIwDQYJKoZIhvcNAQEBBQADggIPADCCAgoCggIBAK3oJHP0FDfzm54rVygc
h77ct984kIxuPOZXoHj3dcKi/vVqbvYATyjb3miGbESTtrFj/RQSa78f0uoxmyF+
0TM8ukj13Xnfs7j/EvEhmkvBioZxaUpmZmyPfjxwv60pIgbz5MDmgK7iS4+3mX6U
A5/TR5d8mUgjU+g4rk8Kb4Mu0UlXjIB0ttov0DiNewNwIRt18jA8+o+u3dpjq+sW
T8KOEUt+zwvo/7V3LvSye0rgTBIlDHCNAymg4VMk7BPZ7hm/ELNKjD+Jo2FR3qyH
B5T0Y3HsLuJvW5iB4YlcNHlsdu87kGJ55tukmi8mxdAQ4Q7e2RCOFvu396j3x+UC
B5iPNgiV5+I3lg02dZ77DnKxHZu8A/lJBdiB3QW0KtZB6awBdpUKD9jf1b0SHzUv
KBds0pjBqAlkd25HN7rOrFleaJ1/ctaJxQZBKT5ZPt0m9STJEadao0xAH0ahmbWn
OlFuhjuefXKnEgV4We0+UXgVCwOPjdAvBbI+e0ocS3MFEvzG6uBQE3xDk3SzynTn
jh8BCNAw1FtxNrQHusEwMFxIt4I7mKZ9YIqioymCzLq9gwQbooMDQaHWBfEbwrbw
qHyGO0aoSCqI3Haadr8faqU9GY/rOPNk3sgrDQoo//fb4hVC1CLQJ13hef4Y53CI
rU7m2Ys6xt0nUW7/vGT1M0NPAgMBAAGjQjBAMA4GA1UdDwEB/wQEAwIBBjAPBgNV
HRMBAf8EBTADAQH/MB0GA1UdDgQWBBR5tFnme7bl5AFzgAiIyBpY9umbbjANBgkq
hkiG9w0BAQsFAAOCAgEAVR9YqbyyqFDQDLHYGmkgJykIrGF1XIpu+ILlaS/V9lZL
ubhzEFnTIZd+50xx+7LSYK05qAvqFyFWhfFQDlnrzuBZ6brJFe+GnY+EgPbk6ZGQ
3BebYhtF8GaV0nxvwuo77x/Py9auJ/GpsMiu/X1+mvoiBOv/2X/qkSsisRcOj/KK
NFtY2PwByVS5uCbMiogziUwthDyC3+6WVwW6LLv3xLfHTjuCvjHIInNzktHCgKQ5
ORAzI4JMPJ+GslWYHb4phowim57iaztXOoJwTdwJx4nLCgdNbOhdjsnvzqvHu7Ur
TkXWStAmzOVyyghqpZXjFaH3pO3JLF+l+/+sKAIuvtd7u+Nxe5AW0wdeRlN8NwdC
jNPElpzVmbUq4JUagEiuTDkHzsxHpFKVK7q4+63SM1N95R1NbdWhscdCb+ZAJzVc
oyi3B43njTOQ5yOf+1CceWxG1bQVs5ZufpsMljq4Ui0/1lvh+wjChP4kqKOJ2qxq
4RgqsahDYVvTH9w7jXbyLeiNdd8XM2w9U/t7y0Ff/9yi0GE44Za4rF2LN9d11TPA
mRGunUHBcnWEvgJBQl9nJEiU0Zsnvgc/ubhPgXRR4Xq37Z0j4r7g1SgEEzwxA57d
emyPxgcYxn/eR44/KJ4EBs+lVDR3veyJm+kXQ99b21/+jh5Xos1AnX5iItreGCc=
-----END CERTIFICATE-----
)";
#endif

constexpr const char RPC_JSON_METHOD[] = "example_json";
constexpr const char RPC_TEMPERATURE_METHOD[] = "example_set_temperature";
constexpr const char RPC_SWITCH_METHOD[] = "example_set_switch";
constexpr const char RPC_TEMPERATURE_KEY[] = "temp";
constexpr const char RPC_SWITCH_KEY[] = "switch";
constexpr uint8_t MAX_RPC_SUBSCRIPTIONS = 3U;
constexpr uint8_t MAX_RPC_RESPONSE = 5U;

#if ENCRYPTED
WiFiClientSecure espClient;
#else
WiFiClient espClient;
#endif

Arduino_MQTT_Client mqttClient(espClient);

Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_RESPONSE> rpc;
const std::array<IAPI_Implementation *, 1U> apis = {
    &rpc};

ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);

volatile bool ledState = false;
volatile bool ledStateChanged = false;
volatile uint16_t blinkingInterval = 1000U;
constexpr uint8_t MAX_ATTRIBUTES = 1U;

constexpr std::array<const char*, MAX_ATTRIBUTES> sharedAttributes = {
  "ledState"
};

bool subscribed = false;

void InitWiFi()
{
    Serial.println("Connecting to AP ...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        Serial.print(".");
    }
    Serial.println("Connected to AP");
#if ENCRYPTED
    espClient.setCACert(ROOT_CERT);
#endif
}


bool reconnect()
{
    const wl_status_t status = WiFi.status();
    if (status == WL_CONNECTED)
    {
        return true;
    }

    InitWiFi();
    return true;
}

void processGetJson(const JsonVariantConst &data, JsonDocument &response)
{
    Serial.println("Received the json RPC method");

    StaticJsonDocument<JSON_OBJECT_SIZE(4)> innerDoc;
    innerDoc["string"] = "exampleResponseString";
    innerDoc["int"] = 5;
    innerDoc["float"] = 5.0f;
    innerDoc["bool"] = true;
    response["json_data"] = innerDoc;
}

void processTemperatureChange(const JsonVariantConst &data, JsonDocument &response)
{
    Serial.println("Received the set temperature RPC method");

    const float example_temperature = data[TEMPERATURE_KEY];

    Serial.print("Example temperature: ");
    Serial.println(example_temperature);

    response["string"] = "exampleResponseString";
    response["int"] = 5;
    response["float"] = 5.0f;
    response["double"] = 10.0;
    response["bool"] = true;
}

void processSwitchChange(const JsonVariantConst &data, JsonDocument &response)
{
    Serial.println("Received the set switch method");


    const bool switch_state = data[HUMIDITY_KEY];

    Serial.print("Example switch state: ");
    Serial.println(switch_state);

    response.set(22.02);
}
float temperature, humidity;
void TaskConnectWifi(void *pvParameters)
{
    InitWiFi();
    while (1)
    {
        delay(1000);

        if (!reconnect())
        {
            return;
        }

        if (!tb.connected())
        {
            Serial.printf("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, TOKEN);
            if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT))
            {
                Serial.println("Failed to connect");
                return;
            }
        }

        if (!subscribed)
        {
            Serial.println("Subscribing for RPC...");
            const std::array<RPC_Callback, MAX_RPC_SUBSCRIPTIONS> callbacks = {
                RPC_Callback{RPC_JSON_METHOD, processGetJson},
                RPC_Callback{RPC_TEMPERATURE_METHOD, processTemperatureChange},
                RPC_Callback{RPC_SWITCH_METHOD, processSwitchChange}};
            if (!rpc.RPC_Subscribe(callbacks.cbegin(), callbacks.cend()))
            {
                Serial.println("Failed to subscribe for RPC");
                return;
            }

            Serial.println("Subscribe done");
            subscribed = true;
        }

        if (millis() - previousTelemetrySend > TELEMETRY_SEND_INTERVAL) {
          // Uncomment if using DHT20
          /*    
          dht20.read();    
          float temperature = dht20.getTemperature();
          float humidity = dht20.getHumidity();
          */
      
          // Uncomment if using DHT11/22
          sensors_event_t event;
          dht.temperature().getEvent(&event);
          if (!isnan(event.temperature)) {
              temperature = event.temperature;
          }

          dht.humidity().getEvent(&event);
          if (!isnan(event.relative_humidity)) {
              humidity = event.relative_humidity;
          }
      
      
          Serial.println("Sending telemetry. Temperature: " + String(temperature, 1) + " humidity: " + String(humidity, 1));
      
          tb.sendTelemetryData(TEMPERATURE_KEY, temperature);
          tb.sendTelemetryData(HUMIDITY_KEY, humidity);
          tb.sendAttributeData("rssi", WiFi.RSSI()); // also update wifi signal strength
          previousTelemetrySend = millis();
        }

        tb.loop();
    }
}
void TaskBlinkLed(void *pvParameters)
{
    pinMode(48, OUTPUT);
    int status = 0;
    while (1)
    {
        Serial.println("My MSSV: 2210882");
        digitalWrite(48, status);
        status = 1 - status;
        vTaskDelay(1000);
    }
}

void TaskPrintTemp_Humi(void *pvParameters)
{
  Wire.begin(11, 12);
  
  while (1)
  {
    sensors_event_t event;
    dht.temperature().getEvent(&event);
    float temperature = event.temperature;

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
    float humidity = event.relative_humidity;
    if (isnan(event.relative_humidity)) {
        Serial.println(F("Error reading humidity!"));
    }
    else {
        Serial.print(F("Humidity: "));
        Serial.print(event.relative_humidity);
        Serial.println(F("%"));
    }
    
    vTaskDelay(1000);
  }
}

void setup()
{
  dht.begin();
    // put your setup code here, to run once:
    Serial.begin(115200);
    xTaskCreate(&TaskConnectWifi, "Connect Wifi", 4096, NULL, 2, NULL);
    xTaskCreate(&TaskBlinkLed, "TaskBlinkLed", 2048, NULL, 2, NULL);
    xTaskCreate(&TaskPrintTemp_Humi, "TaskPrintTemp_Humi", 2048, NULL, 2, NULL);
}

void loop()
{
}
