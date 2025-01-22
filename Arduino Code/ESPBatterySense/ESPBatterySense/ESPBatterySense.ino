#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "Adafruit_Si7021.h"

//#define DEBUG

// Setup our debug printing
#ifdef DEBUG
  #define debug(x)     Serial.print(x)
  #define debugln(x)   Serial.println(x)
#else
  #define debug(x)     // define empty, so macro does nothing
  #define debugln(x)
#endif

#define sensorPowerPin 12

#define WIFI_SSID "xxxxxxxxxxxxxxx"
#define WIFI_PASS "xxxxxxxxxxxxxxx"
#define MQTT_PORT 1883

char fmversion[7] = "3.1";                   // Firmware version of this sensor
char mqtt_server[] = "xxxxxxxxxxx";          // MQTT broker IP address
char mqtt_username[] = "bmesensors";         // Username for MQTT broker
char mqtt_password[] = "xxxxxxxxxxxx";       // Password for MQTT broker
char mqtt_clientid[30];                       // Client ID for connections to MQTT broker

const unsigned int sleepTimeSeconds = 3600;   // Deep sleep for 3600 seconds (1 hour)
const unsigned int connectionAttempts = 12;   // Max attempts to connect to WiFi before sleeping

String baseTopic;
String tempTopic;
String humiTopic;
String vccTopic;
String fwTopic;
String batteryCapacityTopic;

char temperature[10];
char batteryCapacity[10];
char humidity[10];
char vcc[10];

WiFiClient WiFiClient;
PubSubClient mqttclient(WiFiClient);
Adafruit_Si7021 sensor = Adafruit_Si7021();

ADC_MODE(ADC_VCC);

void setup() {
  // Generate the unique baseTopic using the chip ID
  String chipID = String(ESP.getChipId(), HEX); // Convert chip ID to hexadecimal
  chipID.toUpperCase(); // Convert to uppercase for consistency
  baseTopic = "bmesensors/sensor-" + chipID;

  // Recompute the derived MQTT topics
  tempTopic = baseTopic + "/temperature";
  humiTopic = baseTopic + "/humidity";
  vccTopic  = baseTopic + "/vcc";
  fwTopic   = baseTopic + "/firmwarever";

  // Update the MQTT client ID
  snprintf(mqtt_clientid, sizeof(mqtt_clientid), "sensor-%s", chipID.c_str());

  // Configure the sensor power pin as output
  pinMode(sensorPowerPin, OUTPUT);

  #ifdef DEBUG
    Serial.begin(115200);
  #endif

  debugln("Waking up to send data to MQTT server...");
  debugln("Searching for sensors");

  // Power on the sensor and initialize
  digitalWrite(sensorPowerPin, HIGH);
  delay(125); // Allow sensor to start ~100ms plus 20ms buffer
  if (!sensor.begin()) {
    debugln("Could not find a valid SI7021 sensor, check wiring!");
    while (1);
  }
  delay(350); // Allow sensor and power to settle

  // Log the dynamic topics for debugging
  batteryCapacityTopic = baseTopic + "/batterycapacity";
  debugln("Base Topic: " + baseTopic);
  debugln("Temperature Topic: " + tempTopic);
  debugln("Humidity Topic: " + humiTopic);
  debugln("VCC Topic: " + vccTopic);
  debugln("Firmware Topic: " + fwTopic);
  debugln("Battery Capacity Topic: " + batteryCapacityTopic);

  // Set up WiFi and MQTT client
  WiFi.hostname(baseTopic);
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  mqttclient.setServer(mqtt_server, MQTT_PORT);
}

void loop() {
  debugln("Reading sensor data...");
  sensorRead();
  delay(75);
  debugln("Reading VCC from ESP...");
  vccRead();

  int mqttRetValue;
  WiFi.forceSleepWake();
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned int wifiTry = 0;
  while ((WiFi.status() != WL_CONNECTED) && (wifiTry < connectionAttempts)) {
    delay(500);
    debugln("WiFi not connected, attempting reconnect...");
    wifiTry++;
  }

  // If WiFi connection failed, go to deep sleep
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.forceSleepBegin();
    debugln("Going to sleep now, we couldn't connect to WiFi!");
    ESP.deepSleep(sleepTimeSeconds * 1000000);
    while (true) {
      delay(50);
    }
  }

  debugln("WiFi connected");
  debugln("IP address: ");
  debugln(WiFi.localIP());

  debugln("Calling MQTT Connect");
  MQTT_connect();
  debugln("Pushing data to MQTT server");

  debug("Topic: ");
  debugln(tempTopic);
  debug("Sending temp value: ");
  debugln(temperature);
  mqttRetValue = mqttclient.publish(tempTopic.c_str(), temperature);
  debug("Temp return value: ");
  debugln(mqttRetValue);

  debug("Topic: ");
  debugln(humiTopic);
  debug("Sending humidity value: ");
  debugln(humidity);
  mqttRetValue = mqttclient.publish(humiTopic.c_str(), humidity);
  debug("Humidity return value: ");
  debugln(mqttRetValue);

  debug("Topic: ");
  debugln(vccTopic);
  debug("Sending vcc value: ");
  debugln(vcc);
  mqttRetValue = mqttclient.publish(vccTopic.c_str(), vcc);
  debug("VCC return value: ");
  debugln(mqttRetValue);

  debug("Topic: ");
  debugln(fwTopic);
  debug("Sending firmware version: ");
  debugln(fmversion);
  mqttRetValue = mqttclient.publish(fwTopic.c_str(), fmversion);
  debug("Firmware ver return value: ");
  debugln(mqttRetValue);

  debug("Topic: ");
  debugln(batteryCapacityTopic);
  debug("Sending battery capacity: ");
  debugln(batteryCapacity);
  mqttRetValue = mqttclient.publish(batteryCapacityTopic.c_str(), batteryCapacity);
  debug("Battery capacity return value: ");
  debugln(mqttRetValue);

  yield();
  delay(1500); // Allow MQTT message to be processed before sleeping
  WiFi.forceSleepBegin();

  debugln("Going to sleep now!");
  ESP.deepSleep(sleepTimeSeconds * 1000000);
  while (true) {
    delay(50);
  }
}

void MQTT_connect() {
  // Stop if already connected
  if (mqttclient.connected()) {
    return;
  }

  debug("Connecting to MQTT... ");
  while (!mqttclient.connected()) {
    debug("Attempting MQTT connection...");
    if (mqttclient.connect(mqtt_clientid, mqtt_username, mqtt_password)) {
      debugln("connected");
    } else {
      debug("failed, rc=");
      debug(mqttclient.state());
      debugln(" try again in 1 second");
      delay(1000);
    }
  }
  debugln("MQTT Connected!");
}

void vccRead() {
  float v = ESP.getVcc() / 1000.0; // Convert to volts
  dtostrf(v, 5, 2, vcc);          // Format voltage as string
  CalculateBatteryPercentage(v);
}

void CalculateBatteryPercentage(float voltage) {
  int percentage;

  if (voltage <= 2.5) {
    percentage = 0;  // Below or equal to 2.5V is 0%
  } else if (voltage >= 3.0) {
    percentage = 100;  // At or above 3.0V is 100%
  } else {
    // Linear interpolation between 2.5V and 3.0V
    percentage = ((voltage - 2.5) / 0.5) * 100;
  }

  snprintf(batteryCapacity, sizeof(batteryCapacity), "%d", percentage);
}

void sensorRead() {
  float h = sensor.readHumidity();
  float t = (sensor.readTemperature() * 1.8) + 32; // Convert to Fahrenheit
  dtostrf(t, 5, 2, temperature);  // Format temperature as string
  dtostrf(h, 5, 2, humidity);     // Format humidity as string
}
