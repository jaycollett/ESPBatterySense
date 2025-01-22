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

char  fmversion[7] = "2.9";                   // firmware version of this sensor
char  mqtt_server[] = "xxxxxxxxxxx";          // MQTT broker IP address
char  mqtt_username[] = "bmesensors";         // username for MQTT broker (USE ONE)
char  mqtt_password[] = "xxxxxxxxxxxx";       // password for MQTT broker
char  mqtt_clientid[] = "tempHumSensor1";     // client id for connections to MQTT broker

const unsigned int sleepTimeSeconds = 3600;   // deep sleep for 3600 seconds, 1 hour
const unsigned int connectionAttempts = 12;   // number of attempts to connect to wifi before sleeping

const String baseTopic = "bmesensors/tempHumSensor1";
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

  // only need serial if we are in debug mode...
  #ifdef DEBUG
    Serial.begin(115200);
  #endif
  
  debugln("Waking up to send data to MQTT server...");
  debugln("Searching for sensors");

  digitalWrite(sensorPowerPin, HIGH);
  delay(125); // allow sensor to start ~100ms plus 20ms buffer
  if (!sensor.begin()) {
    debugln("Could not find a valid SI7021 sensor, check wiring!");
    while (1);
  }
  delay(350); // allow sensor and power to settle
  
  WiFi.hostname(baseTopic);
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  mqttclient.setServer(mqtt_server, MQTT_PORT);
}

void loop() {

  debugln("Reading sensor data...");
  sensorRead();
  delay(75);
  sensorRead();
  delay(75);
  sensorRead();
  debugln("Reading VCC from ESP...");
  vccRead();

  int mqttRetValue;
  WiFi.forceSleepWake();
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned int wifiTry = 0;
  while ( (WiFi.status() != WL_CONNECTED) && (wifiTry < connectionAttempts) ) {
    delay(500);
    debugln("WiFi not connected, attempting reconnect...");
    wifiTry++;
  }

  // tried to connect to wifi, if it worked, run the code, else go to sleep and try again later
  if(WiFi.status() != WL_CONNECTED){
    WiFi.forceSleepBegin();

    debugln("Going to sleep now, we couldn't connect to WiFi!");
    ESP.deepSleep(sleepTimeSeconds * 1000000); // put the esp into deep sleep mode for 1 hour

    // infinite loop to run for approx ~100ms while the deepsleep command completes
    // we don't want any code running after the deep sleep or for the loop to iterate again
    while (true) {
      delay(50);
    }
  }

  debugln("");
  debugln("WiFi connected");
  debugln("IP address: ");
  debugln(WiFi.localIP());

  debugln("Calling MQTT Connect");
  MQTT_connect(); // connect to wifi/mqtt as needed
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
  debug("Sending fw version value: ");
  debugln(fmversion);
  mqttRetValue = mqttclient.publish(fwTopic.c_str(), fmversion);
  debug("firmware ver return value: ");
  debugln(mqttRetValue);

  yield();
  delay(1500); // yield and delay, if we go to sleep too quickly the mqtt message never gets processed
  WiFi.forceSleepBegin();

  debugln("Going to sleep now!");
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

  debug("Connecting to MQTT... ");

  // Loop until we're reconnected
  while (!mqttclient.connected()) {
    debug("Attempting MQTT connection...");
    // Attempt to connect
    if (mqttclient.connect(mqtt_clientid, mqtt_username, mqtt_password)) {
      debugln("connected");
    } else {
      debug("failed, rc=");
      debug(mqttclient.state());
      debugln(" try again in 1 seconds");
      // Wait 1 second before retrying
      delay(1000);
    }
  }
  debugln("MQTT Connected!");
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
