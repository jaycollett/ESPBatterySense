#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define BMEPWRPIN 5

#define WIFI_SSID "GoofWiFi"
#define WIFI_PASS "Gabriel01!"
#define MQTT_PORT 1883

char  fmversion[7] = "v1.0";              // firmware version of this sensor
char  mqtt_server[] = "192.168.0.5";      // MQTT broker IP address
char  mqtt_username[] = "filamentsensors";     // username for MQTT broker (USE ONE)
char  mqtt_password[] = "!filsensors01a!";   // password for MQTT broker
char  mqtt_clientid[] = "filamentsensor1";     // client id for connections to MQTT broker


ADC_MODE(ADC_VCC);

const int sleepTimeSeconds = 30; // deep sleep for 3600 seconds, 1 hour

const String baseTopic = "filamentsensor1";
const String tempTopic = baseTopic + "/" + "temperature";
const String humiTopic = baseTopic + "/" + "humidity";
const String presTopic = baseTopic + "/" + "pressure";
const String vccTopic  = baseTopic + "/" + "vcc";
const String fwTopic   = baseTopic + "/" + "firmwarever";


char temperature[6];
char humidity[6];
char pressure[7];
char vcc[10];

IPAddress ip;

WiFiClient WiFiClient;
PubSubClient mqttclient(WiFiClient);
Adafruit_BME280 bme; // I2C init

void setup() {

  pinMode(BMEPWRPIN, OUTPUT);
  Serial.begin(115200);

  Serial.println("Waking up to send data to MQTT server...");
  Wire.begin(15, 4); 
  Wire.setClock(100000);
  Serial.println("Searching for sensors");

  digitalWrite(BMEPWRPIN, HIGH);
  delay(100);
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  mqttclient.setServer(mqtt_server, MQTT_PORT);
}

void loop() {

  yield();

  Serial.println("Reading BME data...");
  digitalWrite(BMEPWRPIN, HIGH);
  delay(100);
  bmeRead();
  digitalWrite(BMEPWRPIN, LOW); // shut down power to the DHT now, trying to save as much power as possible

  Serial.println("Reading VCC from ESP...");
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
  Serial.println(presTopic);
  Serial.print("Sending pressure value: ");
  Serial.println(pressure);
  mqttRetValue = mqttclient.publish(presTopic.c_str(), pressure);
  Serial.print("Pressure return value: ");
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
  dtostrf(v, 5, 2, vcc);
}

void bmeRead() {
  float t = (bme.readTemperature()*9/5+32-13);  // converted to F from C
  float h = bme.readHumidity();
  float p = bme.readPressure()/3389.39; // get pressure in inHg

  dtostrf(t, 5, 2, temperature); // 5 chars total, 2 decimals (BME's are really accurate)
  dtostrf(h, 5, 2, humidity);   // 5 chars total, 2 decimals (BME's are really accurate)
  dtostrf(p, 5, 2, pressure);  // 5 chars total, 2 decimals (BME's are really accurate)
}


