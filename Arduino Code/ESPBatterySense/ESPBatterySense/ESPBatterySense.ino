#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "Adafruit_Si7021.h"


#define sensorPowerPin 12

#define WIFI_SSID "xxxxxxxx"
#define WIFI_PASS "xxxxxxxxxxx"
#define MQTT_PORT 1883

char  fmversion[7] = "v2.1";                  // firmware version of this sensor
char  mqtt_server[] = "192.168.0.1";          // MQTT broker IP address
char  mqtt_username[] = "xxxxxxxxxxxxxxxxx";    // username for MQTT broker (USE ONE)
char  mqtt_password[] = "xxxxxxxxxxxxxxxxx";    // password for MQTT broker
char  mqtt_clientid[] = "xxxxxxxxxxxxxxxxx";    // client id for connections to MQTT broker

const unsigned int sleepTimeSeconds = 3600;   // deep sleep for 3600 seconds, 1 hour

const String baseTopic = "filamentsensor2";
const String tempTopic = baseTopic + "/" + "temperature";
const String humiTopic = baseTopic + "/" + "humidity";
const String vccTopic  = baseTopic + "/" + "vcc";
const String fwTopic   = baseTopic + "/" + "firmwarever";


char temperature[10];
char humidity[10];
char vcc[10];

IPAddress ip;

WiFiClient WiFiClient;
PubSubClient mqttclient(WiFiClient);
Adafruit_Si7021 sensor = Adafruit_Si7021();


ADC_MODE(ADC_VCC);

void setup() {

  pinMode(sensorPowerPin, OUTPUT);
  Serial.begin(115200);

  Serial.println("Waking up to send data to MQTT server...");
  Serial.println("Searching for sensors");

  digitalWrite(sensorPowerPin, HIGH);
  delay(120); // allow sensor to start ~100ms plus 20ms buffer
  if (!sensor.begin()) {
    Serial.println("Could not find a valid SI7021 sensor, check wiring!");
    while (1);
  }
  delay(250); // allow sensor and power to settle
  
  WiFi.hostname(baseTopic);
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  mqttclient.setServer(mqtt_server, MQTT_PORT);
}

void loop() {

  Serial.println("Reading sensor data...");
  sensorRead();
  sensorRead();
  sensorRead();
  Serial.println("Reading VCC from ESP...");
  vccRead();

  int mqttRetValue;
  WiFi.forceSleepWake();
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  Serial.println("Calling MQTT Connect");
  MQTT_connect(); // connect to wifi/mqtt as needed
  Serial.println("Pushing data to MQTT server");

  Serial.print("Topic: ");
  Serial.println(tempTopic);
  Serial.print("Sending temp value: ");
  Serial.println(temperature);
  mqttRetValue = mqttclient.publish(tempTopic.c_str(), temperature);
  Serial.print("Temp return value: ");
  Serial.println(mqttRetValue);

  Serial.print("Topic: ");
  Serial.println(humiTopic);
  Serial.print("Sending humidity value: ");
  Serial.println(humidity);
  mqttRetValue = mqttclient.publish(humiTopic.c_str(), humidity);
  Serial.print("Humidity return value: ");
  Serial.println(mqttRetValue);

  Serial.print("Topic: ");
  Serial.println(vccTopic);
  Serial.print("Sending vcc value: ");
  Serial.println(vcc);
  mqttRetValue = mqttclient.publish(vccTopic.c_str(), vcc);
  Serial.print("VCC return value: ");
  Serial.println(mqttRetValue);

  Serial.print("Topic: ");
  Serial.println(fwTopic);
  Serial.print("Sending fw version value: ");
  Serial.println(fmversion);
  mqttRetValue = mqttclient.publish(fwTopic.c_str(), fmversion);
  Serial.print("firmware ver return value: ");
  Serial.println(mqttRetValue);

  yield();
  delay(1500); // yield and delay, if we go to sleep too quickly the mqtt message never gets processed
  WiFi.forceSleepBegin();

  Serial.println("Going to sleep now!");
  ESP.deepSleep(sleepTimeSeconds * 1000000); // put the esp into deep sleep mode for 1 hour

  // infinite loop to run for approx ~100ms while the deepsleep command completes
  // we don't want any code running after the deep sleep or for the loop to iterate again
  while (true) {
    delay(50);
  }
}



// #############################################################################
//  mqtt Connect function
//  This function connects and reconnects as necessary to the MQTT server and
//  WiFi.
//  Should be called in the loop to ensure connectivity
// #############################################################################
void MQTT_connect() {
  int8_t ret;

  // Stop if already connected.
  if (mqttclient.connected()) {
    return;
  }

  Serial.print("Connecting to MQTT... ");

  // Loop until we're reconnected
  while (!mqttclient.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttclient.connect(mqtt_clientid, mqtt_username, mqtt_password)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttclient.state());
      Serial.println(" try again in 2 seconds");
      // Wait 2 seconds before retrying
      delay(2000);
    }
  }
  Serial.println("MQTT Connected!");
}

void vccRead() {
  float v  = ESP.getVcc() / 1000.0;
  dtostrf(v, 5, 2, vcc);
}

void sensorRead() {
  float h = sensor.readHumidity();
  float t = ( (sensor.readTemperature() * 1.8) + 32); // converted to F from C
  
  dtostrf(t, 5, 2, temperature);  // 5 chars total, 2 decimals
  dtostrf(h, 5, 2, humidity);     // 5 chars total, 2 decimals
}


