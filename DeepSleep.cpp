/*
  DeepSleep.h - Sends esp8266 into a deep sleep mode.
*/

#include <ESP8266WiFi.h>

void deepSleep(int microSeconds)
{
  //https://github.com/esp8266/Arduino/issues/644
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  delay(1);
  ESP.deepSleep(microSeconds * 1000);
}