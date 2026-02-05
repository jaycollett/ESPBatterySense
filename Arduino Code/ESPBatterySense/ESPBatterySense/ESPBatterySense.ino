/*
 * ESPBatterySense - Battery-powered temperature/humidity sensor
 * Firmware Version: 3.4
 *
 * Publishes sensor data via MQTT, then deep sleeps to conserve battery.
 * Uses Si7021 sensor and stores WiFi BSSID/channel in RTC memory for fast reconnect.
 */

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "Adafruit_Si7021.h"

// ============================================================================
// USER CONFIGURATION - Change these values for your setup
// ============================================================================

// WiFi credentials
#define WIFI_SSID          "xxxxxxxx"       // Your WiFi network name
#define WIFI_PASS          "xxxxxxxxxx"     // Your WiFi password

// MQTT broker settings
#define MQTT_SERVER        "192.168.0.x"           // MQTT broker IP address
#define MQTT_PORT          1883                    // MQTT broker port (default: 1883)
#define MQTT_USERNAME      "xxxxxxxxxx"            // MQTT username
#define MQTT_PASSWORD      "xxxxxxxxxxxx"          // MQTT password

// Timing settings
#define SLEEP_TIME_SECONDS 3600                   // Deep sleep duration (3600 = 1 hour)

// Hardware settings
#define SENSOR_POWER_PIN   12                     // GPIO pin that powers the sensor

// Debug mode - uncomment to enable serial debug output at 115200 baud
//#define DEBUG

// ===================================================================================
// END OF USER CONFIGURATION - Do not modify below unless you know what you are doing
// ===================================================================================

char fmversion[7] = "3.4";

// Debug macros
#ifdef DEBUG
  #define debug(x)     Serial.print(x)
  #define debugln(x)   Serial.println(x)
#else
  #define debug(x)
  #define debugln(x)
#endif

// RTC memory structure to store WiFi connection info across deep sleep
// This allows skipping WiFi scanning on wake, saving 2-4 seconds
struct {
  uint32_t crc32;       // Checksum to validate data integrity
  uint8_t channel;      // WiFi channel
  uint8_t bssid[6];     // Access point MAC address
  uint8_t padding;      // Align to 4 bytes
} rtcData;

// Sensor reading configuration
const int SENSOR_READ_COUNT = 3;        // Number of readings to average
const int SENSOR_READ_DELAY_MS = 50;    // Delay between readings (ms)
const float TEMP_MIN_C = -40.0;         // Si7021 min temp
const float TEMP_MAX_C = 125.0;         // Si7021 max temp
const float HUMIDITY_MIN = 0.0;
const float HUMIDITY_MAX = 100.0;

const unsigned int connectionAttempts = 10;   // Max WiFi connection attempts

// MQTT topics (generated dynamically from chip ID)
String baseTopic;
String tempTopic;
String humiTopic;
String vccTopic;
String fwTopic;
char mqtt_clientid[30];

// Sensor data buffers
char temperature[10];
char humidity[10];
char vcc[10];
bool sensorReadValid = false;

// Objects
WiFiClient wifiClient;
PubSubClient mqttclient(wifiClient);
Adafruit_Si7021 sensor = Adafruit_Si7021();

ADC_MODE(ADC_VCC);

// Calculate CRC32 for RTC data validation
uint32_t calculateCRC32(const uint8_t *data, size_t length) {
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

void setup() {
  // Configure sensor power pin but keep sensor OFF during WiFi activity
  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, LOW);

  #ifdef DEBUG
    Serial.begin(115200);
  #endif

  debugln("Waking up to send data to MQTT server...");

  // Generate the unique baseTopic using the chip ID
  String chipID = String(ESP.getChipId(), HEX);
  chipID.toUpperCase();
  baseTopic = "bmesensors/sensor-" + chipID;

  // Recompute the derived MQTT topics
  tempTopic = baseTopic + "/temperature";
  humiTopic = baseTopic + "/humidity";
  vccTopic  = baseTopic + "/vcc";
  fwTopic   = baseTopic + "/firmwarever";

  // Update the MQTT client ID
  snprintf(mqtt_clientid, sizeof(mqtt_clientid), "sensor-%s", chipID.c_str());

  // Log the dynamic topics for debugging
  debugln("Base Topic: " + baseTopic);
  debugln("Temperature Topic: " + tempTopic);
  debugln("Humidity Topic: " + humiTopic);
  debugln("VCC Topic: " + vccTopic);
  debugln("Firmware Topic: " + fwTopic);

  // Set up WiFi and MQTT client
  WiFi.hostname(baseTopic);
  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  mqttclient.setServer(MQTT_SERVER, MQTT_PORT);
}

void loop() {
  // ===== PHASE 1: WiFi Connection =====
  // Keep sensor OFF during WiFi to avoid EMI interference and reduce current draw
  WiFi.forceSleepWake();
  delay(1);

  // Try to read WiFi config from RTC memory for fast reconnect
  bool rtcValid = false;
  if (ESP.rtcUserMemoryRead(0, (uint32_t*)&rtcData, sizeof(rtcData))) {
    uint32_t crc = calculateCRC32((uint8_t*)&rtcData.channel, sizeof(rtcData) - sizeof(rtcData.crc32));
    if (crc == rtcData.crc32) {
      rtcValid = true;
      debugln("RTC data valid, attempting fast connect...");
    }
  }

  // Attempt connection - use stored BSSID/channel if available
  if (rtcValid) {
    // Fast connect using stored channel and BSSID (skips scanning)
    WiFi.begin(WIFI_SSID, WIFI_PASS, rtcData.channel, rtcData.bssid, true);
  } else {
    // Normal connect with scanning
    debugln("No valid RTC data, performing full WiFi scan...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }

  unsigned int wifiTry = 0;
  while ((WiFi.status() != WL_CONNECTED) && (wifiTry < connectionAttempts)) {
    delay(500); // Check every 500ms for faster response
    wifiTry++;

    // If fast connect fails after a few tries, fall back to full scan
    if (rtcValid && wifiTry == 4 && WiFi.status() != WL_CONNECTED) {
      debugln("Fast connect failed, falling back to full scan...");
      WiFi.disconnect();
      delay(10);
      WiFi.begin(WIFI_SSID, WIFI_PASS);
      rtcValid = false; // Mark as invalid so we save new data
    }

    debug("WiFi not connected (try ");
    debug(wifiTry);
    debugln(")...");
  }

  // If WiFi connection failed, go to deep sleep
  if (WiFi.status() != WL_CONNECTED) {
    debugln("Going to sleep now, we couldn't connect to WiFi!");
    goToSleep();
  }

  debugln("WiFi connected");
  debug("IP address: ");
  debugln(WiFi.localIP());

  // Save WiFi connection info to RTC memory for next wake
  rtcData.channel = WiFi.channel();
  memcpy(rtcData.bssid, WiFi.BSSID(), 6);
  rtcData.crc32 = calculateCRC32((uint8_t*)&rtcData.channel, sizeof(rtcData) - sizeof(rtcData.crc32));
  ESP.rtcUserMemoryWrite(0, (uint32_t*)&rtcData, sizeof(rtcData));
  debugln("WiFi config saved to RTC memory");

  // ===== PHASE 2: Sensor Reading =====
  // Now that WiFi is connected and stable, power on and read the sensor
  debugln("Powering on sensor...");
  digitalWrite(SENSOR_POWER_PIN, HIGH);
  delay(100); // Allow sensor to power up

  // Initialize sensor
  debugln("Initializing sensor...");
  int sensorRetry = 0;
  while (!sensor.begin() && sensorRetry < 3) {
    debugln("Sensor not detected. Retrying...");
    delay(100);
    sensorRetry++;
  }

  if (sensorRetry >= 3) {
    debugln("Sensor initialization failed after 3 attempts.");
    sensorReadValid = false;
  } else {
    delay(250); // Allow sensor to settle before readings
    debugln("Reading sensor data...");
    sensorRead();
  }

  // Power off sensor immediately after reading
  digitalWrite(SENSOR_POWER_PIN, LOW);
  debugln("Sensor powered off");

  // ===== PHASE 3: VCC Reading =====
  // Read VCC after WiFi connected and sensor off for stable reading
  debugln("Reading VCC from ESP...");
  vccRead();

  // ===== PHASE 4: MQTT Publish =====
  debugln("Calling MQTT Connect");
  MQTT_connect();
  debugln("Pushing data to MQTT server");

  // Build MQTT payload - use null for invalid sensor readings
  char mqttPayload[256];
  if (sensorReadValid) {
    snprintf(mqttPayload, sizeof(mqttPayload),
            "{\"temperature\": \"%s\", \"humidity\": \"%s\", \"vcc\": \"%s\", \"firmware\": \"%s\"}",
            temperature, humidity, vcc, fmversion);
  } else {
    snprintf(mqttPayload, sizeof(mqttPayload),
            "{\"temperature\": null, \"humidity\": null, \"vcc\": \"%s\", \"firmware\": \"%s\"}",
            vcc, fmversion);
  }

  debug("Publishing JSON Payload: ");
  debugln(mqttPayload);

  // Publish with retain flag so HA keeps last value across restarts
  int mqttRetValue = mqttclient.publish(baseTopic.c_str(), mqttPayload, true);
  debug("Publish return value: ");
  debugln(mqttRetValue);

  // Retry once if publish failed
  if (mqttRetValue != 1) {
    debugln("MQTT publish failed, retrying...");
    delay(100);
    mqttclient.loop();
    mqttRetValue = mqttclient.publish(baseTopic.c_str(), mqttPayload, true);
    debug("Retry publish return value: ");
    debugln(mqttRetValue);
  }

  // ===== PHASE 5: Clean Shutdown =====
  // Ensure MQTT message is sent before disconnecting
  mqttclient.loop();
  yield();
  delay(100);
  mqttclient.disconnect();
  wifiClient.flush();
  wifiClient.stop();

  debugln("Going to sleep now!");
  goToSleep();
}

void MQTT_connect() {
  if (mqttclient.connected()) {
    return;
  }

  debugln("Connecting to MQTT broker...");
  int retryCount = 0;

  while (!mqttclient.connected() && retryCount < 10) {
    if (mqttclient.connect(mqtt_clientid, MQTT_USERNAME, MQTT_PASSWORD)) {
      debugln("MQTT connected.");
    } else {
      retryCount++;
      debug("Failed to connect, state: ");
      debugln(mqttclient.state());
      delay(1000);
    }
  }

  if (!mqttclient.connected()) {
    debugln("MQTT connection failed. Entering deep sleep.");
    goToSleep();
  }
}

void vccRead() {
  float v = ESP.getVcc() / 1000.0; // Convert to volts
  snprintf(vcc, sizeof(vcc), "%.2f", v); // Format voltage as string
}

void goToSleep() {
  // Ensure sensor is powered off
  digitalWrite(SENSOR_POWER_PIN, LOW);

  // Properly shut down WiFi to minimize power draw
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1); // Required for modem sleep to take effect

  debugln("Entering deep sleep...");
  ESP.deepSleep(SLEEP_TIME_SECONDS * 1000000UL);

  // Should never reach here, but just in case
  while (true) {
    delay(50);
  }
}

void sensorRead() {
  float tempSum = 0;
  float humiditySum = 0;
  int validReadings = 0;

  // Take multiple readings and average them for accuracy
  for (int i = 0; i < SENSOR_READ_COUNT; i++) {
    float t = sensor.readTemperature();
    float h = sensor.readHumidity();

    // Validate reading is not NaN and within sensor specs
    if (!isnan(t) && !isnan(h) &&
        t >= TEMP_MIN_C && t <= TEMP_MAX_C &&
        h >= HUMIDITY_MIN && h <= HUMIDITY_MAX) {
      tempSum += t;
      humiditySum += h;
      validReadings++;
      debug("Reading ");
      debug(i + 1);
      debug(": T=");
      debug(t);
      debug("C, H=");
      debug(h);
      debugln("%");
    } else {
      debug("Reading ");
      debug(i + 1);
      debugln(": Invalid or out of range");
    }

    if (i < SENSOR_READ_COUNT - 1) {
      delay(SENSOR_READ_DELAY_MS);
    }
  }

  // Check if we got enough valid readings
  if (validReadings == 0) {
    debugln("No valid sensor readings obtained.");
    sensorReadValid = false;
    snprintf(temperature, sizeof(temperature), "0.00");
    snprintf(humidity, sizeof(humidity), "0.00");
    return;
  }

  // Calculate averages
  float avgTempC = tempSum / validReadings;
  float avgHumidity = humiditySum / validReadings;

  // Convert to Fahrenheit
  float avgTempF = (avgTempC * 1.8) + 32;

  debug("Averaged ");
  debug(validReadings);
  debug(" readings: T=");
  debug(avgTempF);
  debug("F, H=");
  debug(avgHumidity);
  debugln("%");

  sensorReadValid = true;
  snprintf(temperature, sizeof(temperature), "%.2f", avgTempF);
  snprintf(humidity, sizeof(humidity), "%.2f", avgHumidity);
}
