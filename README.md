# MQTT Weather Station
Monitors air temperature, humidity and pressure (BME280). In addition to this it tracks lition battery voltage and
sends health status. The status might be check periodically and could be used as a source of online or offline states.
It sends JSON data to MQTT server via single Request. In case of any of the measurements fails then the whole Request
is cancelled. The JSON payload looks like:
```
{
    "temperature": 22.0,
    "humidity": 51.0,
    "pressure" 985.0,
    "voltage": 4.2,
    "health": "ok"
}
```
ESP8266 chip enters into a Deep Sleep Mode and uses static network configuration to save extra energy
for a battery powered device. WiFi persistence is deliberately switched off to save EEPROM write cycles and
to avoid stucks. 


The Deep Sleep Mode could be switched off / on via an MQTT Topic. OTA is supported when 
a Deep Sleep Mode is switched off. When Deep Sleep Mode is on then there are no measurements done in the loop().

Battery level is measured via ADC. A voltage divider is used to scale 4.2V to up to 1V.

# Circuit
TODO

# Configuration
Please copy file `Config.h.local` into `Config.h` and define your MQTT and Network credentials there before building the firmware. The firmware expectsusing static IP address to save a battery energy. OTA is not implemented on purpose since it doesn't work out of the box in a deep sleep mode.

> Make sure not to commit your credentials to Git. `Config.h` is ignored in file .gitignore.

# TODO
* Enable local AP mode to change WiFi creds without firmwaring it