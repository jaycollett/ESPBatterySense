#include <DHT.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define DHTTYPE DHT22
#define DHTPIN 2

#define WIFI_SSID "xxxxxxx"
#define WIFI_PASS "xxxxxxxxxx!"
#define MQTT_PORT 1883

char  mqtt_server[] = "192.168.0.0";
char  mqtt_username[] = "filamentSensors";
char  mqtt_password[] = "!filsensors01A!";

ADC_MODE(ADC_VCC);

const int sleepTimeSeconds = 3600; // deep sleep for 1 hour

const String baseTopic = "fillamentsensor";
const String tempTopic = baseTopic + "/" + "temperature";
const String humiTopic = baseTopic + "/" + "humidity";
const String vccTopic  = baseTopic + "/" + "vcc";

char temperature[6];
char humidity[6];
char vcc[10];

IPAddress ip;

WiFiClient WiFiClient;
PubSubClient mqttclient(WiFiClient);
DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(115200);
  delay(150);
  Serial.println("Waking up to send data to MQTT server...");
  dht.begin();
  delay(2000); // give sensor time to wake up and process a temp reading, it's a very slow sensor
}

void loop() {
  Serial.println("Reading temp/hum from DHT22");
  dhtRead();
  Serial.println("Reading VCC from ESP");
  vccRead();

  Serial.println("Calling MQTT Connect");
  MQTT_connect(); // connect to wifi/mqtt as needed
  mqttclient.publish(tempTopic.c_str(), temperature);
  mqttclient.publish(humiTopic.c_str(), humidity);
  mqttclient.publish(vccTopic.c_str(), vcc);

  Serial.println("Going to sleep now!");
  ESP.deepSleep(sleepTimeSeconds * 1000000);
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
    if (mqttclient.connect(mqtt_server, mqtt_username, mqtt_password)) {
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

void dhtRead() {
  float t = dht.readTemperature(true); // true param tells the library to return temp in F, not C
  float h = dht.readHumidity();

  // Check if any reads failed and if so, set values to zero (obvious invalid readings)
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    t = 0;
    h = 0;
  }

  dtostrf(t, 5, 2, temperature);
  dtostrf(h, 6, 2, humidity);
}


