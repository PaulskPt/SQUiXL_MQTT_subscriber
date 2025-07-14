# Receiving and displaying MQTT messages on an Unexpected Maker SQUiXL device

by Paulus Schulinck (Github handle: @PaulskPt)

If you do not know what is the MQTT communication protocol see: [MQTT](https://en.wikipedia.org/wiki/MQTT).

For a successful MQTT communication you need: 
- a MQTT Publisher device. In my case: an Adafruit Feather ESP32-S3 TFT board;
- a MQTT Broker device. This can be an online broker or a Broker device in your Local Area Network (LAN). I prefered the latter. In my case: a Raspberry Pi Compute Module 5.
- one or more MQTT Subscriber device(s). This repo is intended to use an Unexpected Maker SQUiXL as Subscriber device.

How to install?

Download the latest version of Unexpected Maker [SQUiXL DevOS](https://github.com/UnexpectedMaker/SQUiXL-DevOS). Install that repo on your PC in a folder of your preference. Copy the files of this repo from the subfolders of: /src/Subscriber/SQUiXL-DevOS_A07/ to the same subfolders where you installed the Unexpected Maker SQUiXL DevOS firmware source files. You need to have installed on your PC: Microsoft Visual Studio Code (VSCode) and the PlatformIO extension. The first time I downloaded the sources of the UM SQUiXL DevOS firmware, it built and uploaded, inside VSCode + PlatformIO, to my SQUiXL without problem. Yes, during the build phase there were various warnings of duplicate defined variables, however these warnings did not stall the build process.

What files do my changes to the SQUiXL DevOS firmware sources contain?

The files to change the firmware for the MQTT Subscriber device are [here](https://github.com/PaulskPt/SQUiXL_MQTT_subscriber/tree/main/src/Subscriber/SQUiXL-DevOS_A07) This folder and its subfolders only contain files that have been changed by me, @PaulskPt, to make the Unexpected Maker SQUiXL Alpha version of the SQUiXL-DevOS firmware (minimum version: Alpha v0.6), handle and display received MQTT messages in it's ```MQTT Messages``` screen. The folder ```/platformio/pio/libdeps/squixl``` contains a photo of the libraries that are needed to be installed. These libraries are referenced in the file ```platformio.ini```. Under normal conditions Platformio will download and install these libraries for you. Only when it cannot find a library, you have to search for it online, download the library, copy it to the folder ```/platformio/pio/libdeps/squixl``` and, when the link to this library not exists in the file ```platform.ini```, copy the link to that library into the file ```platformio.ini```, under ```lib_deps = ```.

Use of the SQUiXL Web Portal:

After successful building and uploading the (modified) SQUiXL-DevOS, using ```Microsoft VSCode + PlatformIO extension```, you can use the ```SQUiXL Webportal``` in your webbrowser, for example at address ```http://192.168._.__/wifi``` (the "._.___" must contain the values of your SQUiXL LAN IP-address. In the SQUiXL Web Portal, subpage ```WiFi & WEB```, in the section ```MQTT Settings```, you have to set the following:

```
Enabled: YES 
Broker IP : <The LAN or PUBLIC IP-address of your broker device>.
In my case I use a Raspberry Pi Compute Module 5 running the mosquitto app.
This RPiCM5 has LAN IP-address: 192.168._.___.
Broker Port: 1883
Device Name: <The name of your Publisher device>. In my case an Adafruit Feather ESP32-S3 TFT. Name: "Feath".

In the section "Topics" you fill-in: 
Name: "Feath". (or the name of your publisher device)
Listen: "sensors/Feath/ambient". (or the topic text of your choice)
```

# MQTT message content

In the firmware for the Unexpected Maker SQUiXL device, in the file ```src/mqtt/mqtt.h``` is defined a structure of which most of the items are used in the MQTT messages that my MQTT Publisher sends.

```
struct MQTT_Payload
{
		std::string owner = "";
		std::string device_class = "";
		std::string state_class = "";
		std::string term = ""; // added by @PaulskPt. Like meterorological term: "Pressure"
		std::string unit_of_measurement = "";
		std::string sensor_value = "";
		std::string value_type = "";
		std::string min_value = "";
		std::string max_value = "";
		std::string description = "";  // added by @PaulskPt
		std::string timestampStr = ""; // added by @PaulskPt, human readable ISO format
		std::string msgID = ""; // added by @PaulskPt. In fact contains a Unix timestamp
		int timestamp = 0;  // This is not sent in the MQTT messages. It is used in the firmware

  ```
As you can see, I added some items to the MQTT_Payload structure

The mqtt messages are defined in a Json format.
To keep the length of the payload of the MQTT messages under 256 bytes, I have chosen to abbreviate the names of this struct.

Why keep the payload length under 256 bytes? 

I had a problem when using a Pimoroni Presto device as Subscriber device, which uses Micropython.
I discovered that MQTT messages received were cutoff. Initially my MQTT Publisher device sent messages with full names of the structure shown above, which made the payload longer than 256 bytes. That is why I decided to abbreviate the names. I managed to reduce the payload length to less than 256 bytes. Since then the MQTT messages sent by the MQTT Publisher device were received complete.

```
In the "doc" section:
owner        -> ow
description  -> de
device_class -> dc
state_class  -> sc     and its class "measurement" -> "meas"
value_type   -> vt     and the value_type "float"  -> "f"
timestamp    -> ts

In the "reads" subsection:
another four sub-sub-sections for each (term) of the BME280 sensor: "temperature", "pressure", "altitude" and "humidity":
sub-sub-section (term) "temperature" -> "t"
sub-sub-section (term) "pressure"    -> "p"
sub-sub-section (term) "altitude"    -> "a"
sub-sub-section (term) "humidity"    -> "h"

each sub-sub-section with the same definitions:
"sensor_value"        -> "v"
"unit_of_measurement" -> "u"
"minimum_value"       -> "mn"
"maximum_value"       -> "mx"
```

Here is an example of the contents of a MQTT message my MQTT Publisher device sends every minute:

```
{"ow":"Feath","de":"PC-Lab","dc":"BME280","sc":"meas","vt":"f","ts":1752189817,"reads":{"t":{"v":29,"u":"C","mn":-10,"mx":50},"p":{"v":1005.6,"u":"mB","mn":800,"mx":1200},"a":{"v":63.9,"u":"m","mn":0,"mx":3000},"h":{"v":41.7,"u":"%","mn":0,"mx":100}}}
```

# MQTT Publisher (other functionalities)

To build and upload the Arduino sketch for the MQTT Publisher device I used the Arduino IDE v2.3.5. In the Arduino sketch for the MQTT Publisher (Adafruit ESP32-S3 TFT board) I added functionality to set the display to "sleep" at a time defined in the file ```secrets.h```. In this moment 23h. And a "wakeup" time. In this moment 8h. See in the file secrets.h: 
```
#define SECRET_DISPLAY_SLEEPTIME "23"  // Feather display going to sleep (black) time
#define SECRET_DISPLAY_AWAKETIME "8"   // Feather display wakeup time
```
During the "sleep" hours, the MQTT Publisher device continues to send MQTT messages at the programmed interval time (in this moment once per minute). It also continues, at intervals of 15 minutes, to synchronize the via I2C connected external M5Stack Unit RTC from a NTP datetime stamp.

In case the Arduino sketch of the Publisher device encounters that it cannot read the values from the BME280 sensor, the sketch will issue a software reset by calling the function ```reset()``` which calls the function ```ESP.restart()```.

The source of the Arduino sketch for the MQTT Publisher device is [here](https://github.com/PaulskPt/SQUiXL_MQTT_subscriber/tree/main/src/Publisher)

# File secrets.h (for the MQTT Publisher device)

To have the Publisher device be able to connect to the internet, to get, at intervals, a Unixtime datetime stamp from an NTP server, you have to fill-in the WiFi SSID and PASSWORD. Further you can change the following settings in the file secrets.h:

```
#define SECRET_TIMEZONE_OFFSET "1" // Europe/Lisbon (UTC offset in hours)
// #define TIMEZONE_OFFSET "-4" // America/New_York
#define SECRET_NTP_SERVER1 "1.pt.pool.ntp.org"
#define SECRET_MQTT_BROKER "5.196.78.28" // test.mosquitto.org"
#define SECRET_MQTT_BROKER_LOCAL1 "192.168._.___"  // Your Local mosquitto broker app on PC ____
#define SECRET_MQTT_BROKER_LOCAL2 "192.168._.___"  // Your Local mosquitto broker app on Raspberry Pi ___ (in my case a RPi CM5)
#define SECRET_MQTT_PORT "1883" 
#define SECRET_MQTT_TOPIC "sensors/Feath/ambient"  // Sent by the Publisher device (in my case an Adafruit Feather ESP32-S3 TFT)
#define SECRET_DISPLAY_SLEEPTIME "23"  // Feather display going to sleep (black) time
#define SECRET_DISPLAY_AWAKETIME "8"   // Feather display wakeup time
```

# MQTT broker

If you, like me, also use a Raspberry Pi model to host a Mosquitto broker application, see the files in ```/etc```
- ```/etc/hosts.allow``` : insert in this file the ip-addres of your mosquitto broker. In my case: ```mosquitto: 127.0.0.1```
- ```/etc/mosquitto/mosquitto.conf```. See the contents of the mosquitto.conf file that I use in the folder: ```SQUiXL_MQTT_subscriber/src/Broker/etc/mosquitto```.

See also photos of sites where to download the mosquitto broker app for Raspberry Pi or for a MS Windows PC in the folder. ```SQUiXL_MQTT_subscriber/src/Broker```.

# MQTT Subscriber 

Note that in the file ```/SQUiXL_MQTT_subscriber/src/Subscriber/SQUiXL-DevOS_A07/platformio/src/mqtt/mqtt.h``` I added the following C++ preprocessor directive:
```
#ifndef USE_PAULSKPT_PARTS
#define USE_PAULSKPT_PARTS   (1)  // use
#endif
```
This directive results in selected parts of my code changes being used instead of similar UM SQUiXL DevOS firmware parts.


# Hardware used:

For the MQTT Publisher device: Adafruit Feather ESP32-S3 TFT [info](https://www.adafruit.com/product/5483);

Accessories for the MQTT Publisher device:
Equipment connected to the Publisher device:
- Pimoroni multi-sensor-stick (PIM 745) [info](https://shop.pimoroni.com/products/multi-sensor-stick?variant=42169525633107);
- M5Stack Unit-RTC [info](https://docs.m5stack.com/en/unit/UNIT%20RTC);
- M5Stack Grove Hub [info](https://shop.m5stack.com/products/mini-hub-module)

For the MQTT Broker device:
- a Raspberry Pi Compute Module 5 [info](https://www.raspberrypi.com/products/compute-module-5/?variant=cm5-104032);
- a Raspberry Pi Compute Module IO Board [info](https://thepihut.com/products/raspberry-pi-compute-module-5-io-board)
- a case for the Raspberry Pi Compute Module 5 [info](https://thepihut.com/products/raspberry-pi-compute-module-5-io-case)

For the MQTT Subscriber device:
- Unexpected Maker SQUiXL device: [info](https://unexpectedmaker.com/shop.html#!/SQUiXL/p/743870537).

# Known problems:

Be aware that the DevOS firmware for the Unexpected Maker SQUiXL is still in an Alpha state. This means that this firmware is subject to changes.

My advise for the Publisher device: the Adafruit Feather ESP32-S3 TFT (and probably any other device used as MQTT Publisher device) and also the attached BME280 sensor, it is really necessary to use a 5,1 Volt DC power source of good quality. My experience is at this hardware / this sensor needs at least 5,1 Volt DC. For example: the USB port of my desktop PC delivered 5,132 Volt DC. That was OK. I also used an original Raspberry Pi 5,1 Volt DC power apdapter. That was also OK. When I used a power source that delivered 5,058 Volt DC, that was not insufficient. At times the BME280 was not recognized and at times the MQTT Publisher device sent messages containing a wrong NTP Unixtime value as MsgID. When using a good quality 5,1 Volt DC power supply, the MQTT Publisher device runs many hours without problem, resulting in the MQTT Broker receiving MQTT message correctly and the MQTT Subscriber device(s) do the same.

