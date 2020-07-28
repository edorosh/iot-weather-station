/**
 * MQTT Weather Station
 * 
 * Monitors periodically temperature, humidity and light intersivity and sends JSON data to MQTT server.
 * ESP8266 chip enters sleep mode and uses static network configuration to save extra eneregy for a battery powered devices.
 * OTA is not implemented on purpose since it doesn't work out of the box in a deep sleep mode.
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
#include <DHT.h>

#include "DeepSleep.h"
#include "Serial.h"

#include "Config.h"
#include "DebugMacro.h"

// Initialize DHT sensor.
// Note that older versions of this library took an optional third parameter to
// tweak the timings for faster processors.  This parameter is no longer needed
// as the current DHT reading algorithm adjusts itself to work on faster procs.
DHT dht(DHTPIN, DHTTYPE);
WiFiClient espClient;
PubSubClient client(espClient);

// static IP address of device
IPAddress IPADDRESS_CONFIG;  
IPAddress GATEWAY_CONFIG;
IPAddress SUBNET_CONFIG; 

bool OTA_enabled = true;
bool DEEP_SLEEP_enabled = false;

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;    // will store last time DHT was updated
// Updates DHT readings every 10 seconds
const long interval = 25e3; 

// Sensors values
float t, h, l;

/**
 * Closes all network clients and sends the chip into Deep Sleep Mode with WAKE_RF_DEFAULT.
 * The function does nothing in case global variable DEEP_SLEEP_enabled is true.
 */
void enterDeepSleepMode()
{
  if (!DEEP_SLEEP_enabled)
  {
    DPRINTLNF("Deep Sleep disabled. Skipping current deep sleep request");
    return;
  }

  if (client.connected())
  {
    DPRINTLNF("Disconnecting MQTT Client");
    client.disconnect();
  }

  DPRINTLNF("Entering a deep sleep mode");
  deepSleep(DEEP_SLEEP_MICRO_SECONDS);
}

/** Open Serial port for debugging purposes. */
inline void beginSerial()
{
  initSerial(SERIAL_SPEED_BAUD, SERIAL_TIMEOUT_MICRO_SECONDS);
  Serial.println(F("Booting..."));
}

inline void connectToWiFi()
{
  WiFi.forceSleepWake();
  yield();

  DPRINTLNF("Enabling STA mode");

  // Disable the WiFi persistence.  The ESP8266 will not load and save WiFi settings in the flash memory.
  WiFi.persistent(false);

  WiFi.mode(WIFI_STA);
  WiFi.begin(STASSID, STAPSK);
  WiFi.config(ip, gateway, subnet);

  DPRINTF("Connecting to WiFi network ");
  DPRINTLN(STASSID);

  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    DPRINTLNF("Connection Failed! Rebooting via deep sleep...");
    enterDeepSleepMode();
  }

  DPRINTF("Connected! IP address is ");
  DPRINTLN(WiFi.localIP().toString().c_str());
}

void powerBusOn()
{
  pinMode(DHT_KEY_PIN, OUTPUT);

  DPRINTLNF("Set Power Bus pin to LOW");

  // By default NodeMCU keeps +3V. Let's make it LOW
  digitalWrite(DHT_KEY_PIN, LOW);
  yield();
}

void powerBusOff()
{
  digitalWrite(DHT_KEY_PIN, HIGH);
  DPRINTLNF("Set DHT pin to HIGH");
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
      DPRINTLNF(" connected");
    }
    else
    {
      DPRINTF(" failed, rc=");
      DPRINT(client.state());
      DPRINTLNF(" try again in 5 seconds");

      // Wait some time before retrying
      delay(CONNECT_MQTT_TIMEOUT_MICRO_SECONDS);
    }
  }

  client.loop();
}

void publishSensorsData()
{
  // TODO: return code check
  client.publish("weatherStation/temperature", String(t).c_str(), true);
  yield();

  client.publish("weatherStation/humidity", String(h).c_str(), true);
  yield();

  client.publish("weatherStation/light", String(l).c_str(), true);
  yield();
}

void setSensorsDataDefault() {
  t = h = l = .0;
}

void readSensorsData()
{
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  t = dht.readTemperature();
  // Read Photoresistor value
  l = analogRead(A0) / 1023.0f;

  // Check if any reads failed and exit early (to try again).
  // TODO: refactor with return code
  if (isnan(h) || isnan(t))
  {
    DPRINTLNF("Failed to read from DHT sensor!");

    if (DEEP_SLEEP_enabled)
    {
      powerBusOff();
    }

    enterDeepSleepMode();
  }

  DPRINTF("Humidity: ");
  DPRINT(h);
  DPRINTF("%  Temperature: ");
  DPRINT(t);
  DPRINTF("°C Light: ");
  DPRINTLN(l);
}

void callback(char* topic, byte* payload, unsigned int length) {
  DPRINTF("Message arrived [");
  DPRINT(topic);
  DPRINTF("] ");
  for (int i = 0; i < length; i++) {
    DPRINT((char)payload[i]);
  }
  DPRINTLN();

  // Switch on OTA if an 1 was received as first character
  if ((char)payload[0] == '1') {
    OTA_enabled = true;
    DPRINTLNF("OTA enabled. Deep Sleep disabled");
  } else {
    OTA_enabled = false;
    DPRINTLNF("OTA disabled. Deep Sleep enabled");
  }

  DEEP_SLEEP_enabled = !OTA_enabled;
}

inline void setupOTA()
{
  DPRINT("Emabling OTA with hostname ");
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
    DPRINTLN("\nEnd");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
    {
      DPRINTLNF("Auth Failed");
    }
    else if (error == OTA_BEGIN_ERROR)
    {
      DPRINTLNF("Begin Failed");
    }
    else if (error == OTA_CONNECT_ERROR)
    {
      DPRINTLNF("Connect Failed");
    }
    else if (error == OTA_RECEIVE_ERROR)
    {
      DPRINTLNF("Receive Failed");
    }
    else if (error == OTA_END_ERROR)
    {
      DPRINTLNF("End Failed");
    }
  });

  ArduinoOTA.begin();
}

void setup()
{
  // https://www.bakke.online/index.php/2017/05/21/reducing-wifi-power-consumption-on-esp8266-part-2/
  // Turn Wifi off until we have something to send
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  yield();

#ifdef DEBUG
  beginSerial();
#endif

  powerBusOn();

  dht.begin();

  if (DEEP_SLEEP_enabled)
  {
    // Let the sensor to initialize itself
    delay(DHT_INITIALIZE_TIMEOUT_MICRO_SECONDS);
    setSensorsDataDefault();
    readSensorsData();
  }

  connectToWiFi();
  connectToMQTT();

  client.subscribe("device/mac/ba1ec43d2e62/ota_enable");
  client.setCallback(callback);

  if (OTA_enabled)
  {
    setupOTA();
  }
}

void loop()
{
  client.loop();

  if (OTA_enabled)
  {
    ArduinoOTA.handle();
  }

  if (DEEP_SLEEP_enabled)
  {
    publishSensorsData();
    powerBusOff();
    enterDeepSleepMode();
  }

  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    // save the last time you updated the DHT values
    previousMillis = currentMillis;

    readSensorsData();
    publishSensorsData();
    setSensorsDataDefault();
  }
}
