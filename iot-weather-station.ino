/**
 * MQTT Weather Station (ESP8266)
 * 
 * Monitors air temperature, humidity and pressure (BME280). In addition to this it tracks lition battery voltage and
 * send health status. The status might be check periodically and could be used as a source of online or offline states.
 * It sends JSON data to MQTT server via single Request. In case of any of the measurements fails then the whole Request
 * is cancelled. The JSON payload looks like:
 * <code>
 * {
 *  "temperature": 22.0,
 *  "humidity": 51.0,
 *  "pressure" 985.0,
 *  "voltage": 4.2,
 *  "health": "ok"
 * }
 * </code>
 * 
 * ESP8266 chip enters into a Deep Sleep Mode and uses static network configuration to save extra energy
 * for a battery powered device. WiFi persistence is deliberately switched off to save EEPROM write cycles and
 * to avoid stucks. 
 * 
 * The Deep Sleep Mode could be switched off / on via an MQTT Topic. OTA is supported when 
 * a Deep Sleep Mode is switched off. When Deep Sleep Mode is on then there are no measurements done in the loop().
 * 
 * Battery level is measured via ADC. A voltage divider is used to scale 4.2V to up to 1V.
 * Voltage divider is switched on/off by an NPN transitor during the time of a measurement.
 * 
 * Author: Evgeny Doros
 * Email: eugene.dorosh@gmail.com
 * Github: https://github.com/edorosh
 * License: MIT 
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <PubSubClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include "DeepSleep.h"
#include "Serial.h"

#include "Config.h"
#include "DebugMacro.h"
#include "Version.h"

WiFiClient espClient;
PubSubClient client(espClient);
Adafruit_BME280 bme;

// static IP address of device
IPAddress IPADDRESS_CONFIG;  
IPAddress GATEWAY_CONFIG;
IPAddress SUBNET_CONFIG; 

// Global variable keeping deep sleep setting. Might change during the programm flow
bool DEEP_SLEEP_enabled = false;

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;    // will store last time DHT was updated

// Timeout before publishing and going into sleep mode in deep sleep mode. This timout is required to be able
// to get MQTT message to switch sleep mode off
unsigned long previousDSMillis = 0;   
const long intervalDS = 1e3; 

// Sensors values
float t, h, pressure = .0;
float volt = .0;

// JSON Object settings
const size_t capacity = JSON_OBJECT_SIZE(5);

/**
 * Closes all network clients and sends the chip into Deep Sleep Mode with WAKE_RF_DEFAULT.
 * The function does nothing in case global variable DEEP_SLEEP_enabled is true.
 */
void enterDeepSleepMode()
{
  if (client.connected())
  {
    DPRINTLN(F("Disconnecting MQTT Client"));
    client.disconnect();
  }

  DPRINTLN(F("Entering a deep sleep mode"));
  deepSleep(DEEP_SLEEP_MICRO_SECONDS);
}

/** Open Serial port for debugging purposes. */
inline void beginSerial()
{
  initSerial(SERIAL_SPEED_BAUD, SERIAL_TIMEOUT_MICRO_SECONDS);
  Serial.println(F("Booting..."));
}

/**
 * Coonects to WiFi in STA mode. Wakes Wifi by force. WiFi persistence is switched off.
 */ 
inline void connectToWiFi()
{
  WiFi.forceSleepWake();
  yield();

  DPRINTLN(F("Enabling STA mode"));

  // Disable the WiFi persistence.  The ESP8266 will not load and save WiFi settings in the flash memory.
  WiFi.persistent(false);

  WiFi.mode(WIFI_STA);
  WiFi.begin(STASSID, STAPSK);
  WiFi.config(ip, gateway, subnet);

  DPRINT(F("Connecting to WiFi network "));
  DPRINTLN(STASSID);

  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    DPRINTLNF("Connection Failed! Restarting in 500ms...");
    delay(500);
    ESP.restart();
  }

  DPRINT(F("Connected! IP address is "));
  DPRINTLN(WiFi.localIP().toString().c_str());
}

inline void connectToMQTT()
{
  client.setServer(MQTTSERVER, MQTTPORT);

  // Loop until we're reconnected
  while (!client.connected())
  {
    DPRINT("Attempting MQTT connection...");

    // Attempt to connect
    if (client.connect(MQTT_CLIENT_NAME))
    {
      DPRINTLN(F(" connected"));
    }
    else
    {
      DPRINT(F(" failed, rc="));
      DPRINT(client.state());
      DPRINTLN(F(" try again in 5 seconds"));

      // Wait some time before retrying
      delay(CONNECT_MQTT_TIMEOUT_MICRO_SECONDS);
    }
  }
}

void publishSensorsData()
{
  StaticJsonDocument<capacity> doc;
  char JSONmessageBuffer[100];
  char buffer[256];

  doc["temperature"] = !isnan(t) ? t : .0;
  doc["humidity"] = !isnan(h) ? h : .0;
  doc["pressure"] = !isnan(pressure) ? pressure : .0;
  doc["voltage"] = volt;
  doc["health"] = "ok";

  size_t n = serializeJson(doc, buffer);

  DPRINTLN(F("Pretty JSON message: "));
  #ifdef DEBUG
    serializeJsonPretty(doc, Serial);
  #endif
  DPRINTLN("");

  if (!client.publish("weatherStation/jsonData", (const uint8_t*) buffer, n, true)) {
    DPRINTLNF("Sending message to MQTT failed");
  }
}

float getVoltage() {
  int times = 5;
  int measurements[times];
  float raw = .0;
  float accum = .0;

  for (int i=0; i < times; i++) {
    // read sensor raw value
    measurements[i] = analogRead(A0);
    delay(50);
  }
  
  for (int i=0; i < times; i++) {
    accum += measurements[i];
  }

  raw = accum / times;

  DPRINT(F("RAW Voltage: "));
  DPRINTLN(raw);

  return raw;
}

bool readSensorsData()
{
   // Only needed in forced mode! In normal mode, you can remove the next line.
  bme.takeForcedMeasurement(); // has no effect in normal mode

  h = bme.readHumidity();
  t = bme.readTemperature();
  pressure = bme.readPressure() / 100.0F;

  float raw = getVoltage();
  volt=raw/1023.0;
  volt=volt*5.62;

  DPRINT(F("Humidity: "));
  DPRINT(h);
  DPRINT(F("%  Temperature: "));
  DPRINT(t);
  DPRINT(F("°C Pressure: "));
  DPRINT(pressure);
  DPRINT(F("hPa Voltage: "));
  DPRINT(volt);
  DPRINT(F("V"));
  DPRINTLN("");

  return true;
}

void callback(char* topic, byte* payload, unsigned int length) {
  DPRINT(F("Message arrived ["));
  DPRINT(topic);
  DPRINT(F("] "));
  for (int i = 0; i < length; i++) {
    DPRINT((char)payload[i]);
  }
  DPRINTLN();

  // Backup old value
  bool last_DEEP_SLEEP_enabled = DEEP_SLEEP_enabled;

  // Switch on Deep Sleep if an 1 was received as first character
  DEEP_SLEEP_enabled = ((char)payload[0] == '1');
  
  // DEBUG
  if (DEEP_SLEEP_enabled) {
    DPRINTLN(F("Deep Sleep enabled"));
  } else {
    DPRINTLN(F("Deep Sleep disabled"));
  }

  // save the DEEP_SLEEP state in flash memory
  if (last_DEEP_SLEEP_enabled != DEEP_SLEEP_enabled) {
    DPRINTLN(F("Writing to EEPROM deep sleep mode"));
    EEPROM.write(EEPROM_DEEP_SLEEP_ADDRESS, DEEP_SLEEP_enabled);
    if (!EEPROM.commit()) {
      DPRINTLN(F("Error has occured while saving Deep Sleep state into EEPROM."));
    }
    
    DPRINTLN(F("Restarting ESP ..."));
    ESP.restart();
  }
}

inline void setupOTA()
{
  DPRINT(F("Enabling OTA with hostname "));
  DPRINTLN(OTAHOSTNAME);

  // Hostname defaults to esp8266-[ChipID]
  ArduinoOTA.setHostname(OTAHOSTNAME);

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
    {
      type = "sketch";
    }
    else
    { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    DPRINTLN("Start updating " + type);
  });

  ArduinoOTA.onEnd([]() {
    DPRINTLN(F("\nEnd"));
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    DPRINTF("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    DPRINTF("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
    {
      DPRINTLN(F("Auth Failed"));
    }
    else if (error == OTA_BEGIN_ERROR)
    {
      DPRINTLN(F("Begin Failed"));
    }
    else if (error == OTA_CONNECT_ERROR)
    {
      DPRINTLN(F("Connect Failed"));
    }
    else if (error == OTA_RECEIVE_ERROR)
    {
      DPRINTLN(F("Receive Failed"));
    }
    else if (error == OTA_END_ERROR)
    {
      DPRINTLN(F("End Failed"));
    }
  });

  ArduinoOTA.begin();
}

void setup()
{
#ifdef DEBUG
  beginSerial();
  DPRINTLN("");
  DPRINT(F("Sketch starting: iot-weather-station "));
  DPRINTLN(FW_VERSION);
  DPRINT(F("Reset reason: "));
  DPRINTLN(ESP.getResetReason());
  DPRINT(F("Core Version: "));
  DPRINTLN(ESP.getCoreVersion());
  DPRINT(F("SDK Version: "));
  DPRINTLN(ESP.getSdkVersion());
  DPRINTLN("");
#endif

  // https://www.bakke.online/index.php/2017/05/21/reducing-wifi-power-consumption-on-esp8266-part-2/
  // Turn Wifi off until we have something to send
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  yield();

  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  DEEP_SLEEP_enabled = EEPROM.read(EEPROM_DEEP_SLEEP_ADDRESS);
  DPRINT(F("Read from EEPROM: deep sleep is "));
  DPRINTLN(DEEP_SLEEP_enabled);

  if (!bme.begin(BME_ADDRESS)) {
      DPRINTLN(F("Could not find a valid BME280 sensor, check wiring!"));
      while (1);
  }

  bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1, // temperature
                    Adafruit_BME280::SAMPLING_X1, // pressure
                    Adafruit_BME280::SAMPLING_X1, // humidity
                    Adafruit_BME280::FILTER_OFF   );

  readSensorsData();
  
  connectToWiFi();
  connectToMQTT();

  client.subscribe(DEEP_SLEEP_TOPIC);
  client.setCallback(callback);

  setupOTA();

  previousDSMillis = millis();
}

void loop()
{
  client.loop();
  
  if (DEEP_SLEEP_enabled)
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousDSMillis < START_STATION_TIMEOUT_IN_SLEEP_MODE)
    {
      //DPRINTLN(F("SKIP Deep Sleep Loop"));
      return;
    }

    publishSensorsData();
    enterDeepSleepMode();
  }

  ArduinoOTA.handle();
  
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= DEEP_SLEEP_MICRO_SECONDS)
  {
    // save the last time you updated the DHT values
    previousMillis = currentMillis;

    readSensorsData();
    publishSensorsData();
  }
}
