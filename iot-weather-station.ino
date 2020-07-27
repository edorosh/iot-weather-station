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

void enterDeepSleepMode()
{
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
  DPRINTLNF("Enabling STA mode");

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

inline void powerDHTOn()
{
  pinMode(DHT_KEY_PIN, OUTPUT);

  DPRINTLNF("Set DHT pin to LOW");

  // By default NodeMCU keeps +3V. Let's make it LOW
  digitalWrite(DHT_KEY_PIN, LOW);

  // Let the sensor to initialize itself
  delay(DHT_INITIALIZE_TIMEOUT_MICRO_SECONDS);
}

void powerDHTOff()
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

inline void readSensorsDataAndPublish()
{
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  // Read Photoresistor value
  float l = analogRead(A0) / 1023.0f;

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t))
  {
    DPRINTLNF("Failed to read from DHT sensor!");
    powerDHTOff();
    enterDeepSleepMode();
  }

  DPRINTF("Humidity: ");
  DPRINT(h);
  DPRINTF("%  Temperature: ");
  DPRINT(t);
  DPRINTF("°C Light: ");
  DPRINTLN(l);

  client.publish("weatherStation/temperature", String(t).c_str(), true);
  client.publish("weatherStation/humidity", String(h).c_str(), true);
  client.publish("weatherStation/light", String(l).c_str(), true);

  // Allow client sending all the data
  delay(200);
}

void setup()
{
#ifdef DEBUG
  beginSerial();
#endif

  connectToWiFi();
  connectToMQTT();
  powerDHTOn();

  dht.begin();

  readSensorsDataAndPublish();
  powerDHTOff();
  enterDeepSleepMode();
}

void loop() {}
