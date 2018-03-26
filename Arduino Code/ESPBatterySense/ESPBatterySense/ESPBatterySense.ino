#include <DHT.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

#define DHTTYPE DHT22
#define DHTPIN 2

#define WIFI_SSID "xxxxxxxxx"
#define WIFI_PASS "xxxxxxxxxxxxx"
#define MQTT_PORT 1883

char  mqtt_server[] = "192.168.0.5";
char  mqtt_username[] = "xxxxxxxxxxxxxxxxx";
char  mqtt_password[] = "xxxxxxxxxxxxxxx";
char  mqtt_clientid[] = "filamentsensor1";

ADC_MODE(ADC_VCC);

const int sleepTimeSeconds = 30; // deep sleep for 3600 seconds, 1 hour

const String baseTopic = "filamentsensor1";
const String tempTopic = baseTopic + "/" + "temperature";
const String humiTopic = baseTopic + "/" + "humidity";
const String vccTopic  = baseTopic + "/" + "vcc";
const String firmTopic = baseTopic + "/" + "firmwarever";

char temperature[6];
char humidity[6];
char vcc[10];
char firmwarever[7] = "v0.6a";

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

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  delay(3000); // give time for network to stabalize
  
  mqttclient.setServer(mqtt_server, MQTT_PORT);
}

void loop() {

  yield();
  
  Serial.println("Reading temp/hum from DHT22");
  dhtRead();
  Serial.println("Reading VCC from ESP");
  vccRead();

  int mqttRetValue;
  
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
  Serial.println(firmTopic);
  Serial.print("Sending fw version value: ");
  Serial.println(firmwarever);
  mqttRetValue = mqttclient.publish(firmTopic.c_str(), firmwarever);
  Serial.print("firmware ver return value: ");
  Serial.println(mqttRetValue);

  yield();
  delay(2500); // yield and delay, if we go to sleep too quickly the mqtt message never gets processed
  
  
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
  dtostrf(v, 5, 3, vcc);
}

void dhtRead() {
  float t;
  t = dht.readTemperature(true); // true param tells the library to return temp in F, not C
  float h;
  h = dht.readHumidity();

  // Check if any reads failed and if so, set values to zero (obvious invalid readings)
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    t = 99.99;
    h = 99.99;
  }

  dtostrf(t, 5, 2, temperature);
  dtostrf(h, 6, 2, humidity);
}


