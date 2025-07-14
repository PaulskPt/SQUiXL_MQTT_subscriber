/*
  Sunday 2025-06-16 20h07 utc +1
  Adafruit Feather ESP32-S3 TFT MQTT test
  This sketch is a port from a sketch for an Uno R4 WiFi to send BME280 sensor data to a MQTT broker
  and to receive and display these MQTT messages onto the display of:
  a) an Unexpected Maker SQUiXL device;
  b) a Pimoroni Presto device.
  Update Saturday 2025-06-28 13h27 utc +1
  Successfully added:
  c) in this sketch, neccesary components to comply with the UM SQUiXL system;
  d) in the SQUiXL-DevOS firmware (Alpha 0.7) changes in files: 
  - platformio/src/mqtt/squixl.h;
  - platformio/src/mqtt/mqtt.h;
  - platformio/src/mqtt/mqtt.cpp;
  - platformio/src/ui/ui_scrollarea.cpp.
  The MQTT messages are now displayed in the "MQTT Messages" screen of the UM SQUiXL.

  Notes about power consumption (2025-07-13):
  When connected to USB (long cable) of a MS Windows 11 desktop PC, the average voltage is 5,124 V.
  The current draw is average 0.069 A however with incidently increased draw up to 0,143 A.
  When connected to my multi-port 5Volt power supply, the voltage is: 5,058 V  (0,066 Volt less than the PC).
  I saw that when the Feather is connected to the multi-port 5Volt power supply, the sketch executes with failures,
  like wrong unixTime, consequently wrong derived hh which has strang effects to the checks for gotoSleep or awake of the display.
  When I connect a good Raspberry Pi 5V adapter to the Feather, the Feather works without problem. With a VOM I measured,
  between GND and VBUS pins of the Feather, 5,29 V (+ 0,232V more than on the multi-port 5Volt power supply)
*/
#include <Arduino.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Adafruit_BME280.h>
//#include <Wire.h>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <string>
#include <ctime>
#include <cstdio>  // for snprintf
#include <time.h>
#include <Unit_RTC.h>
#include <stdlib.h>  // for setenv
#include "secrets.h"

#include "Adafruit_MAX1704X.h"
#include "Adafruit_LC709203F.h"
#include <Adafruit_NeoPixel.h>
#include "Adafruit_TestBed.h"
#include <Adafruit_BME280.h>
#include <Adafruit_ST7789.h> 
//#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeMono12pt7b.h>

Adafruit_LC709203F lc_bat;
Adafruit_MAX17048 max_bat;
extern Adafruit_TestBed TB;

#define DEFAULT_I2C_PORT &Wire

// Some boards have TWO I2C ports, how nifty. We should scan both
#if defined(ARDUINO_ARCH_RP2040) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S2) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_NOPSRAM) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32S3_N4R2) \
    || defined(ARDUINO_ADAFRUIT_QTPY_ESP32_PICO) \
    || defined(ARDUINO_SAM_DUE) \
    || defined(ARDUINO_ARCH_RENESAS_UNO)
  #define SECONDARY_I2C_PORT &Wire1
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

#ifndef NEOPIXEL_POWER
#define NEOPIXEL_POWER 34
#endif
// NEOPIXEL_POWER_ON

// Which pin on the Arduino is connected to the NeoPixels?
#ifndef PIN_NEOPIXEL
#define PIN_NEOPIXEL   33 // On Trinket or Gemma, suggest changing this to 1
#endif

#ifndef NUMPIXELS
#define NUMPIXELS         1     // Only one onboard NeoPixel
#endif

#define BLACK (0, 0, 0)  // Not defined for the Neopixel in Adafruit_testbed.h

Adafruit_NeoPixel pixel(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

GFXcanvas16 canvas(240, 135);

int canvas_width = canvas.width();
int canvas_height = canvas.height();

bool maxfound = false;
bool lcfound = false;
bool squixl_mode = true; // compose mqtt messages conform the SQUiXL system (send 4 messages, each for: temp, pres, alti and humi)

bool my_debug = false;
bool timeSynced = false;
bool do_test_reset = false;
bool isItBedtime = false;
bool display_can_be_used = true; // true if the current hour is between sleepTime and awakeTime

unsigned long msgGrpID = 0L;
unsigned long msgGrpID_old = 0L;
unsigned long msgGrpID_max = 999L;
unsigned long unixUTC = 0L; // = unixtime UTC
unsigned long unixLOC = 0L; // = unixtime Local (including (+-) tz_offset in seconds)
unsigned long mqttMsgID = 0L;

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

// unsigned int localPort = 2390;  // local port to listen for UDP packets

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

// this IP did not work: "85.119.83.194" // for test.mosquitto.org
// 5.196.0.0 - 5.196.255.255 = FR-OVH-20120823, Country code: FR (info from https://lookup.icann.org/en/lookup)
#define USE_BROKER_LOCAL   (1)

#ifdef USE_BROKER_LOCAL
const char broker[]  = SECRET_MQTT_BROKER_LOCAL2; // "192.168.1.114";
#else
const char broker[]  = SECRET_MQTT_BROKER; // test.mosquitto.org";
#endif

int        port      = atoi(SECRET_MQTT_PORT);   // 1883;

WiFiUDP ntpUDP; // A UDP instance to let us send and receive packets over UDP
int tzOffset = atoi(SECRET_TIMEZONE_OFFSET); // can be negative or positive (hours)
signed long utc_offset = tzOffset * 3600;    // utc_offset in seconds. Attention: signed long! Can be negative or positive
unsigned long uxTimeUTC = 0L;
unsigned long uxTimeLocal = 0L;
uint8_t HoursLocal = 255;  // Initialized to an invalid value
unsigned long ntp_interval_t = (15 * 60 * 1000L) - 5; // 15 minutes - 5 secs
//NTPClient timeClient(ntpUDP, SECRET_NTP_SERVER1, utc_offset, ntp_interval_t);
NTPClient timeClient(ntpUDP, SECRET_NTP_SERVER1, 0, ntp_interval_t); // no utc_offset
bool rtc_is_synced = false;
char isoBufferUTC[26];  // 19 characters + null terminator representing UTC time
char isoBufferLocal[26];  // same, representing Local time
char timestamp[24] = "not_synced";  // e.g., "2025-07-02T13:32:02"
std::string dispTimeStr; // the local time to be shown on the TFT display see func getUnixTimeFromRTC()
float temperature, humidity, pressure, altitude;
const size_t CAPACITY = 1024;

Unit_RTC RTC;

//                        from Unit_RTC.h    uint8_t buf[3] = {0};
rtc_time_type RTCtime; // uint8_t buf[0] Seconds, uint8_t buf[1] Minutes, buf[2] uint8_t Hours
//                        from Unit_RTC.h    uint8_t buf[4] = {0};
// Note RTC_DateStruct->Month MSB (0x80) if "1" then Year = 1900 + buf[3]. If "0" then Year = 2000 + buf[3]
rtc_date_type RTCdate; // uint_t buf[0] date = 0, uint8_t buf[1] weekDay = 0, uint8_t buf[2] month = 0, uint16_t buf[3] year = +1900 or +2000, 
// uint8_t DateString[9]; // from Unit_RTC.h
// uint8_t TimeString[9]; // also

uint8_t displaySleepTime;
uint8_t displayAwakeTime;

char str_buffer[64];

int led =  LED_BUILTIN;

bool led_is_on = false;

#define led_sw_cnt 20  // Defines limit of time count led stays on

int status = WL_IDLE_STATUS;

int count = 0;

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C
bool bmefound = false;
uint8_t bme_bad_read_cnt = 0;
uint8_t bme_bad_read_cnt_max = 3;

unsigned long delayTime;

void setupDisplayTimes() {
  // displaySleepTime and displayAwakeTime are global variables
  displaySleepTime = static_cast<uint8_t>(atoi(SECRET_DISPLAY_SLEEPTIME)); // in hours
  displayAwakeTime = static_cast<uint8_t>(atoi(SECRET_DISPLAY_AWAKETIME)); // in hours

  if (displaySleepTime > 23)
      displaySleepTime = 23;

  if (displayAwakeTime > 23)
      displayAwakeTime = 8;
  if (!my_debug) {
    Serial.print(F("displaySleepTime = "));
    Serial.println(displaySleepTime);
    Serial.print(F("displayAwakeTime = "));
    Serial.println(displayAwakeTime);
  }
}

bool isItDisplayBedtime() {
  // HoursLocal, displaySleepTime and displayAwakeTime are global variables
  // Check if the current hour is between displaySleepTime and displayAwakeTime

  // update the global variable HoursLocal
  // getUnixTimeFromRTC();  // This function is called in setup() and next called every minutes.
  
  bool is_it_disp_bedtime = true;
  if (HoursLocal >= displayAwakeTime && HoursLocal < displaySleepTime) {
    is_it_disp_bedtime = false;
  }
  if (my_debug) {
    Serial.print(F("isItDisplayBedtime(): HoursLocal = "));
    Serial.println(HoursLocal);
    Serial.print(F("is_it_disp_bedtime = ")); 
    Serial.println(is_it_disp_bedtime ? "true" : "false");
  }
  return is_it_disp_bedtime;
}

// Function to turn the built-in LED on
void led_on() {
  //blinks the built-in LED every second
  digitalWrite(led, HIGH);
  led_is_on = true;
  //delay(1000);
}

// Function to turn the built-in LED off
void led_off()
{
  digitalWrite(led, LOW);
  led_is_on = false;
  //delay(1000);
}

void neopixel_on()
{
  led_is_on = true;
  digitalWrite(NEOPIXEL_POWER, HIGH); // Switch on the Neopixel LED
  //Serial.println("Color GREEN");
  pixel.setPixelColor(0, pixel.Color(0, 150, 0)); // Green color
  pixel.show();

}

void neopixel_off()
{
  led_is_on = false;
  //Serial.println("Color BLACK (Off)");
  pixel.setPixelColor(0, pixel.Color(0, 0, 0)); // color black (Turn off)
  pixel.show();
  digitalWrite(NEOPIXEL_POWER, LOW); // Switch off the Neopixel LED
}

void neopixel_test()
{
  Serial.println("neopixel test");
  for (int j = 0; j < 4; j++)
  {
    if (j == 0)
    {
      //Serial.println("Color RED");
      pixel.setPixelColor(0, pixel.Color(255, 0, 0));
    }
    else if (j == 1)
    {
      //Serial.println("Color GREEN");
      pixel.setPixelColor(0, pixel.Color(0, 255, 0));
    }
    else if (j == 2)
    {
      //Serial.println("Color BLUE");
      pixel.setPixelColor(0, pixel.Color(0, 0, 255));
    }
    else if (j == 3)
    {
      //Serial.println("Color BLACK (Off)");
      pixel.setPixelColor(0, pixel.Color(0, 0, 0));
    }
    pixel.show();  // Send the updated pixel colors to the hardware.
    delay(500);
  }
}
/* Function to perform a software reset on an Arduino board,
   specifically using the ArduinoCore-renesas.
   Arduino has a built-in function named as resetFunc()
   which we need to declare at address 0 and when we 
   execute this function Arduino gets reset automatically.
   Using this function resulted in a "Fault on interrupt 
   or bare metal (no OS) environment crash!
*/
void reset() {
  ESP.restart();
}

// Default version for most use cases (160-byte buffer)
void serialPrintf(const char* format, ...) {
  char buffer[160];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);
  Serial.print(buffer);
}

// Extended version where you can specify buffer size
// (= Function Overloading!)
void serialPrintf(size_t bufferLen, const char* format, ...) {
  char buffer[bufferLen];
  va_list args;
  va_start(args, format);
  vsnprintf(buffer, bufferLen, format, args);
  va_end(args);
  Serial.print(buffer);
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  serialPrintf(PSTR("SSID: %s\n"), WiFi.SSID().c_str());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  serialPrintf(PSTR("IP Address: %s\n"), ip.toString().c_str());

/*
  // print the received signal strength:
  long rssi = WiFi.RSSI();
  serialPrintf(PSTR("signal strength (RSSI): %ld dBm\n"), rssi);
*/
}
bool ConnectToWiFi()
{
  bool ret = false;
  int connect_tries = 0;
  // attempt to connect to WiFi network:


  while (WiFi.status() != WL_CONNECTED) 
  {
    //serialPrintf(PSTR("Attempting to connect to SSID: %s\n"), ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    WiFi.begin(ssid, pass);
    //serialPrintf(PSTR("WiFi connection tries: %d.\n"), connect_tries);
    connect_tries++;
    if (connect_tries >= 5)
    {
      serialPrintf(PSTR("❌ WiFi connection failed %d times.\n"), connect_tries);
      break;
    }

    // wait 10 seconds for connection:
    delay(10000);
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print(F("✅ Connected to "));
    printWifiStatus();
    ret = true;
  }
  return ret;
}

void do_line(uint8_t le = 4)
{
  // Default
  if (le == 4)
  {
    for (uint8_t i= 0; i < le; i++)
    {
      Serial.print(F("----------")); // length 10
    }
  }
  // Variable
  else
  {
    for (uint8_t i= 0; i < le; i++)
    {
      Serial.print('-'); // length 1
    }
  }
  Serial.println();
}

void disp_msg(const char* arr[], size_t size, bool disp_on_serial = false) {
  uint8_t hPos = 0;
  uint8_t vPos = 25;

  if (size == 0)
    return;

  canvas.fillScreen(ST77XX_BLACK);

  for (uint8_t i = 0; i < size; i++) {
    switch (i) {
      case 0: canvas.setTextColor(ST77XX_GREEN);   vPos = 25;  break;
      case 1: canvas.setTextColor(ST77XX_YELLOW);  vPos = 50;  break;
      case 2: canvas.setTextColor(ST77XX_CYAN);    vPos = 75;  break;
      case 3: canvas.setTextColor(ST77XX_MAGENTA); vPos = 100; break;
      case 4: canvas.setTextColor(ST77XX_BLUE);    vPos = 125; break;
      default: break;
    }

    canvas.setCursor(hPos, vPos);
    canvas.println(arr[i]);

    if (disp_on_serial) {
      Serial.println(arr[i]);
    }
  }

  display.drawRGBBitmap(0, 0, canvas.getBuffer(), canvas_width, canvas_height);
}

void disp_intro(bool disp_on_serial = false) {
  const char *msg[] = {"Adafruit Feather",
                       "ESP32-S3 TFT as",
                       "MQTT publisher",
                      "device"};

  disp_msg(msg, sizeof(msg) / sizeof(msg[0]), disp_on_serial);
}

// msgGrpID, mqttMsgID and dispTimeStr are global variables
void disp_msg_info(bool disp_on_serial = false) {
  std::string msgGrpIDstr1 = "MQTT msg group";
  std::string msgGrpIDstr2 = std::to_string(msgGrpID) + " sent";
  std::string dispMsgID = "msgID: " + std::to_string(mqttMsgID);
  const char *msg[] = {dispTimeStr.c_str(), msgGrpIDstr1.c_str(), msgGrpIDstr2.c_str(), dispMsgID.c_str()};
  disp_msg(msg, sizeof(msg) / sizeof(msg[0]), disp_on_serial);
}

void disp_sensor_data(bool disp_on_serial = false) {
  static std::string tempStr, presStr, altitr, humiStr;

  std::ostringstream tempSS, presSS, altiSS, humiSS;
  canvas.print((char)0xF8); // ISO-8859-1 code for °
  tempSS << "Temp: " << std::right << std::setw(6) << std::fixed << std::setprecision(1) << temperature << " ºC";
  presSS << "Pres: " << std::right << std::setw(6) << std::fixed << std::setprecision(1) << pressure    << " hPa";
  altiSS << "Alt:  " << std::right << std::setw(6) << std::fixed << std::setprecision(1) << altitude    << " m";
  humiSS << "Humi: " << std::right << std::setw(6) << std::fixed << std::setprecision(1) << humidity    << " %";

  tempStr = tempSS.str();
  presStr = presSS.str();
  altitr  = altiSS.str();
  humiStr = humiSS.str();

  const char* msg[] = {tempStr.c_str(), presStr.c_str(), altitr.c_str(), humiStr.c_str()};
  disp_msg(msg, sizeof(msg) / sizeof(msg[0]), disp_on_serial);
}

void disp_goodnight() {
  const char *msg[] = {"Good night! 🌙", "Display go off", "Send msg", "continues!", "See you tomorrow!"};
  disp_msg(msg, sizeof(msg) / sizeof(msg[0]), true);
  delay(5000); // Show the text for 5 seconds
  canvas.fillScreen(ST77XX_BLACK);
}

void disp_goodmorning() {
  const char *msg[] = {"Good morning! ☀️", "Display goes on", "Have a nice day!"};
  disp_msg(msg, sizeof(msg) / sizeof(msg[0]), true);
  delay(5000); // Show the text for 5 seconds
  canvas.fillScreen(ST77XX_BLACK);
}

/*
  From Copilot:
  your custom portable_timegm() workaround safely converts a 
  UTC-based std::tm into a 
  time_t epoch 
  without relying on system-level timezone logic.
*/
time_t portable_timegm(std::tm* utc_tm) 
{
  char* old_tz = getenv("TZ");
  setenv("TZ", "", 1);  // Set to UTC
  tzset();

  time_t utc_epoch = std::mktime(utc_tm);

  // Restore previous timezone
  if (old_tz)
    setenv("TZ", old_tz, 1);
  else
    unsetenv("TZ");
  tzset();

  return utc_epoch;
}

// return an unsigned long unixUTC
/*
 Note:
 Epoch timestamp: 32503679999
 Timestamp in milliseconds: 32503679999
 Date and time (GMT): Tuesday, December 31, 2999 11:59:59 PM
 This function is also called when needed to update the global variable HoursLocal
 */
unsigned long getUnixTimeFromRTC() 
{
  static constexpr const char txt0[] PROGMEM = "getUnixTimeFromRTC(): ";
  //rtc_time_type RTCtime;  // is are global variable
  //rtc_date_type RTCdate;  // same

  // Get current time and date from RTC
  RTC.getTime(&RTCtime);
  RTC.getDate(&RTCdate);

  // Prepare a tm structure (interpreted as UTC time)
  std::tm timeinfoUTC{};

  timeinfoUTC.tm_year = RTCdate.Year - 1900;  // tm_year is years since 1900
  timeinfoUTC.tm_mon  = RTCdate.Month - 1;    // tm_mon is 0-based
  timeinfoUTC.tm_mday = RTCdate.Date;
  timeinfoUTC.tm_hour = RTCtime.Hours;
  timeinfoUTC.tm_min  = RTCtime.Minutes;
  timeinfoUTC.tm_sec  = RTCtime.Seconds;

  if (my_debug)
  {
    Serial.print(txt0);
    Serial.print(F("timeinfoUTC.tm_year etc..."));
    serialPrintf("%4d-%02d-%02dT%02d:%02d:%02d\n",
                  timeinfoUTC.tm_year, 
                  timeinfoUTC.tm_mon, 
                  timeinfoUTC.tm_mday, 
                  timeinfoUTC.tm_hour, 
                  timeinfoUTC.tm_min, 
                  timeinfoUTC.tm_sec);
  }

  // convert to an ISO8601 string (utc time) in buffer: isoBufferUTC
  toIso8601String(timeinfoUTC, isoBufferUTC, sizeof(isoBufferUTC), 0);

  // Convert to time_t (local time)
  // time_t utcEpoch = std::mktime(&timeinfoUTC);
  time_t utcEpoch = portable_timegm(&timeinfoUTC);
  
  // Add 1 hour to get Local epoch (since RTC is in UTC)
  // Adjust to UTC using global utc_offset
  if (utcEpoch > 32503679999) // 32503679999 is the epoch timestamp for 31 December 2999
  {
    Serial.print(txt0);
    Serial.println(F("❌ Epoch time is too large, returning 0"));
    return 0; // Return 0 if the epoch time is too large
  }
  time_t localEpoch = utcEpoch + utc_offset;

  // Prepare a tm structure (interpreted as local time)
  std::tm timeinfoLocal{};

  // Convert time_t to tm struct for local time 
  //std::tm* tmp = localtime(&localEpoch);
  std::tm* tmp = std::gmtime(&localEpoch);

  // Convert time_t LocalEpoch to std::tm struct timeinfoLocal
  //std::tm* tmp = std::localtime(&localEpoch);isobufferLocal
  if (tmp != nullptr) 
  {
    timeinfoLocal = *tmp;
    // ------------------------------------------
    // Set the global variable to the hours value 
    // (needed for display sleep or not)
    HoursLocal = timeinfoLocal.tm_hour;
    if (!my_debug)
    {
      Serial.print(txt0);
      Serial.print(F("HoursLocal = "));
      Serial.println(HoursLocal);
    }
    //-------------------------------------------
  }
  else
  {
    Serial.print(txt0);
    Serial.println(F("❌ Failed to create timeinfoLocal"));
    HoursLocal = 255; // Set to an invalid value
  }

  // convert to an ISO8601 string (local time) in buffer: isoBufferLocal
  toIso8601String(timeinfoLocal, isoBufferLocal, sizeof(isoBufferLocal), utc_offset);

  char timeStr[9] = {};  // HH:MM:SS\0
  std::snprintf(timeStr, 9, "%02d:%02d:%02d",
                timeinfoLocal.tm_hour,
                timeinfoLocal.tm_min,
                timeinfoLocal.tm_sec);
  
  dispTimeStr.assign(timeStr, 9); // "hh:mm:ss" is 8 characters long
  if (my_debug)
  {
    Serial.print(txt0);
    serialPrintf(PSTR("hh:mm:ss = %s LOCAL\n"), dispTimeStr.c_str());
  }

  unixUTC = static_cast<unsigned long>(utcEpoch);
  unixLOC = static_cast<unsigned long>(localEpoch);

  if (my_debug)
  {
    Serial.print(txt0);
    Serial.print(F("unixUTC = "));
    Serial.print(unixUTC);
    Serial.print(F(" = "));
    Serial.println(isoBufferUTC);
  }
  if (my_debug)
  {
    Serial.print(txt0);
    Serial.print(F("unixLOC = "));
    Serial.print(unixLOC);
    Serial.print(F(" = "));
    Serial.println(isoBufferLocal);
  }

  return unixLOC; // was: unixUTC
}
/*
uint8_t getLocalTimeFromRTC() {
  rtc_date_type myRTCdate;
  rtc_time_type myRTCtime;

  RTC.getDate(&myRTCdate);
  RTC.getTime(&myRTCtime);

  // Convert RTC time to time_t
  tm rtc_tm = {};
  rtc_tm.tm_year = myRTCdate.Year - 1900;
  rtc_tm.tm_mon  = myRTCdate.Month - 1;
  rtc_tm.tm_mday = myRTCdate.Date;
  rtc_tm.tm_hour = myRTCtime.Hours;
  rtc_tm.tm_min  = myRTCtime.Minutes;
  rtc_tm.tm_sec  = myRTCtime.Seconds;

  time_t utcEpoch = mktime(&rtc_tm);

  // Apply timezone offset to get local time
  time_t localEpoch = utcEpoch + utc_offset;

  // Convert back to tm structure
  tm* local_tm = localtime(&localEpoch);

  if (local_tm != nullptr) {
    return local_tm->tm_hour;
  } else {
    Serial.println(F("❌ Failed to convert to local hours"));
    return 255; // Sentinel value for error
  }
}
*/

void printISO8601FromRTC() 
{
  static constexpr const char txt0[] PROGMEM = "printISO8601FromRTC(): ";
  if (!rtc_is_synced)
  {
    Serial.print(txt0);
    Serial.println(F("RTC has not been set. Exiting this function"));
    isoBufferUTC[0] = '\0'; // empty the isoBufferUTC
  }
  // rtc_time_type RTCtime; // is a global variable
  // rtc_date_type RTCdate; // same

  RTC.getTime(&RTCtime); // UTC
  RTC.getDate(&RTCdate);

  std::tm timeinfoUTC = {};
  timeinfoUTC.tm_year = RTCdate.Year - 1900;
  timeinfoUTC.tm_mon  = RTCdate.Month - 1;
  timeinfoUTC.tm_mday = RTCdate.Date;
  timeinfoUTC.tm_hour = RTCtime.Hours;
  timeinfoUTC.tm_min  = RTCtime.Minutes;
  timeinfoUTC.tm_sec  = RTCtime.Seconds;

  // Convert RTC UTC time to epoch
  //time_t utcEpoch = portable_timegm(&timeinfoUTC);  // instead of mktime()
  time_t utcEpoch = portable_timegm(&timeinfoUTC);
  time_t localEpoch = utcEpoch + utc_offset;

  // Prepare a tm structure (interpreted as UTC time)
  // Use gmtime to avoid double-applying system timezone
  std::tm utcTime{};
  std::tm* tmp = std::gmtime(&utcEpoch);
  if (tmp) 
  {
    utcTime = *tmp;
  }
  // Use localtime
  // Prepare a tm structure (interpreted as Local time)
  std::tm localTime{};
  tmp = std::gmtime(&localEpoch);
  if (tmp) 
  {
    localTime = *tmp;
  }

  // Call the helper for both buffers
  toIso8601String(utcTime, isoBufferUTC, sizeof(isoBufferUTC), 0);
  toIso8601String(localTime, isoBufferLocal, sizeof(isoBufferLocal), utc_offset);
  if (my_debug)
  {
    Serial.print(txt0);
    Serial.print(F("isoBufferUTC   = "));
    Serial.println(isoBufferUTC);
    Serial.print(txt0);
    Serial.print(F("isoBufferLocal = "));
    Serial.println(isoBufferLocal);
  }
}

void toIso8601String(const std::tm& t, char* buffer, size_t bufferSize, int utc_offset_seconds) 
{
  char tz_suffix[7];  // Enough for "+HH:MM" or "-HH:MM" + null terminator

  int total_minutes = utc_offset_seconds / 60;
  int hours = total_minutes / 60;
  int minutes = (total_minutes >= 0) ? (total_minutes % 60) : -(total_minutes % 60);
  char sign = (utc_offset_seconds >= 0) ? '+' : '-';

  // Ensure hours is positive for formatting
  int abs_hours = (hours >= 0) ? hours : -hours;

  std::snprintf(tz_suffix, sizeof(tz_suffix), "%c%02d:%02d", sign, abs_hours, minutes);

  std::snprintf(buffer, bufferSize, "%04d-%02d-%02dT%02d:%02d:%02d%s",
                t.tm_year + 1900,
                t.tm_mon + 1,
                t.tm_mday,
                t.tm_hour,
                t.tm_min,
                t.tm_sec,
                tz_suffix);
}


void setRTCFromEpoch(unsigned long uxTime) 
{
  static constexpr const char txt0[] PROGMEM = "setRTCFromEpoch(): ";
  static constexpr const char *weekdays[] PROGMEM = {"Sun", "Mon", "Tues", "Wednes", "Thurs", "Fri", "Satur" };

  Serial.print(txt0);
  Serial.print(F("Epoch time received: "));
  Serial.println(uxTime);

  //std::tm* timeinfo = std::gmtime(reinterpret_cast<time_t*>(&uxTime));
  time_t rawTime = static_cast<time_t>(uxTime);
  std::tm* timeinfo = std::gmtime(&rawTime);

  //std::tm* timeinfo = std::localtime(reinterpret_cast<time_t*>(&uxTime));  // PaulskPt changed to use std::localtime instead std::gmtime
                                                                             // because timeClient is set for localtime
  if (timeinfo == nullptr) 
  {
    Serial.println(F("❌ Failed to convert epoch time."));
    return;
  }

  rtc_time_type newTime; // new local variable

  newTime.Hours   = static_cast<uint8_t>(timeinfo->tm_hour);
  newTime.Minutes = static_cast<uint8_t>(timeinfo->tm_min);
  newTime.Seconds = static_cast<uint8_t>(timeinfo->tm_sec);
  
  if (!my_debug)
  {
    Serial.print(txt0);
    serialPrintf(PSTR("%02d:%02d:%02d UTC\n"), newTime.Hours, newTime.Minutes, newTime.Seconds);
  }
  
  RTC.setTime(&newTime);  // UTC

  rtc_date_type newDate; // new local variable

  if (!my_debug)
  {
    Serial.print(txt0);
    serialPrintf(PSTR("timeinfo->tm_year = %d\n"), timeinfo->tm_year);
  }
  uint16_t fullYear = static_cast<uint16_t>(timeinfo->tm_year + 1900);
  uint8_t     month = static_cast<uint8_t>(timeinfo->tm_mon + 1);
  uint8_t      date = static_cast<uint8_t>(timeinfo->tm_mday);
  uint8_t   weekday = static_cast<uint8_t>(timeinfo->tm_wday);

  newDate.Year = fullYear;  // ✅ Full year (e.g. 2025)
  newDate.Month = month;
  newDate.Date = date;
  newDate.WeekDay = weekday;

  if (!my_debug)
  {
    Serial.print(txt0);
    serialPrintf(PSTR("%sday, %04u-%02d-%02d\n"), weekdays[weekday], fullYear, month, date);
  }
  
  RTC.setDate(&newDate); // UTC
  rtc_is_synced = true;
}


void rtc_sync() 
{
  //do_line(49);
  Serial.print("🕒 ===>>> ");
  timeClient.update();  // This prints also the text "Update from NTP Server"
  // Get the current date and time from an NTP server and convert
  // it to UTC +1 by passing the time zone offset in hours.
  // You may change the time zone offset to your local one.

  //if (my_debug)
  //  serialPrintf(PSTR("RTC before set: %s\n"), String(currentTime).c_str());

  uxTimeUTC = timeClient.getEpochTime();  // timezone offset already given at init of timeClient;
  uxTimeLocal = uxTimeUTC + utc_offset;

  serialPrintf(PSTR("✅ NTP Unix time (UTC) = %lu\n"), uxTimeUTC);
  // Set RTC with received NTP datetime stamp
  setRTCFromEpoch(uxTimeUTC);
  
  // Retrieve the date and time from the RTC and print them
  // RTCTime currentTime;
  //RTC.getTime(&RTCtime); 
  printISO8601FromRTC(); // this function writes result to global variable isoBufferUTC

  if (my_debug)
  {
    Serial.printf("RTC raw: %04d-%02d-%02d %02d:%02d:%02d\n",
      RTCdate.Year,  RTCdate.Month,   RTCdate.Date,
      RTCtime.Hours, RTCtime.Minutes, RTCtime.Seconds);

    Serial.print(F("RTC datetime = "));
    Serial.println(isoBufferUTC);
    //serialPrintf(PSTR("✅ RTC was set to: %s\n"), String(RTCtime).c_str());
  }
  do_line(55);
  // mqttMsgID = getUnixTimeFromRTC();
  // Get a compilation timestamp of the format: Wed May 10 08:54:31 2023
  // __TIMESTAMP__ is a GNU C extension macro
  // We can't use the standard macros __DATE__ and __TIME__ because they don't provide the day of the week
  // String timeStamp = __TIMESTAMP__;
}


void clr_payloadBuffer(char* buffer, size_t size) {
    memset(buffer, 0, size); // Clear the buffer
}

char payloadBuffer[768]; // was: 512

StaticJsonDocument<CAPACITY> doc;

int composePayload(char* outBuffer, size_t outSize,
                    float temperature, float pressure, float altitude, float humidity,
                    const char* timestamp) {

  clr_payloadBuffer(payloadBuffer, sizeof(payloadBuffer)); // Clear the payload buffer
  
  mqttMsgID = getUnixTimeFromRTC(); // Important! Used in composePayload()
  if (mqttMsgID < 170000000) // 17 million is the minimum value for a valid unix timestamp
  {
    Serial.println(F("❌ Invalid mqttMsgID, exiting composePayload()"));
    return -1; // Invalid timestamp
  }

  // Root fields
  doc["ow"] = "Feath";    // owner
  doc["de"] = "PC-Lab";   // description (room, office, etc.)
  doc["dc"] = "BME280";   // device_class
  doc["sc"] = "meas";     // state_class
  doc["vt"] = "f";        //  f = value type (for all values) float
  doc["ts"] = mqttMsgID;  //  global var mqttMsgID is an unsigned long (takes 10 bytes while a human-readable full datetime = 19bytes)

  // Readings
  JsonObject readings = doc.createNestedObject("reads");

  // Temperature
  JsonObject temp = readings.createNestedObject("t");
 
  temp["v"] = roundf(temperature * 10) / 10.0; // 1 decimal place  v = value
  temp["u"] = "C";  //  u = unit_of_measurement
  temp["mn"] = -10.0;  // mn = minimum_value
  temp["mx"] = 50.0;   // mx = maximum_value

  // Pressure
  JsonObject pres = readings.createNestedObject("p");
  pres["v"] = roundf(pressure * 10) / 10.0;
  pres["u"] = "mB";
  pres["mn"] = 800.0;
  pres["mx"] = 1200.0;

  // Altitude
  JsonObject alti = readings.createNestedObject("a");
  alti["v"] = roundf(altitude * 10) / 10.0;
  alti["u"] = "m";
  alti["mn"] = 0.0;
  alti["mx"] = 3000.0;

  // Humidity
  JsonObject humi = readings.createNestedObject("h");
  humi["v"] = roundf(humidity * 10) / 10.0;
  humi["u"] = "%";
  humi["mn"] = 0.0;
  humi["mx"] = 100.0;
  
  // Serialize JSON to the output buffer
  //doc["readings"]["temperature"]["value"] = serialized(String(temperature, 1));  // 1 decimal

  int written = serializeJson(doc, outBuffer, outSize);  // Return the int value written
  return written;
}

int ck_payloadBuffer(int wrt, bool pr_chrs = false)
{
  static constexpr const char txt0[] PROGMEM = "ck_payloadBuffer(): ";
  int ret = -1; // if >= 0, this search found a null-terminator inside the buffer up to the written value
  if (my_debug)
  {
    Serial.print(txt0);
    Serial.print(F("param wrt = "));
    Serial.println(wrt);
  }
  if (wrt <= 0)
    return ret;

  for (int i = 0; i <= wrt; ++i) {
    char c = payloadBuffer[i];
    if (pr_chrs) {
      Serial.print(F(" Char["));
      Serial.print(i);
      Serial.print(F("]: '"));
    }
    // Print visible character or escape for control chars if param pr_chrs is true
    if (pr_chrs) {
      if (isprint(c)) {
          Serial.print(c);
      } else if (c == '\0') {
        Serial.print(F("\\0"));
        Serial.print((uint8_t)c, HEX);  // Print hex for non-printables
      } else {
        Serial.print(F("\\x"));
        Serial.print((uint8_t)c, HEX);  // Print hex for non-printables
      }
      Serial.print(F("' (0x"));
      Serial.print((uint8_t)c, HEX);    // Hex value of the character
      Serial.println(F(")"));
    }
    
    if (c == '\0' && ret == -1) {
      ret = i;
      if (my_debug) {
        Serial.print(txt0);
        Serial.print(F("found a null-terminator at pos: "));
        Serial.println(i);
      }
    }
  }
  return ret;
}

// This function devides the mqtt_msg to be printed in half,
// then checks for the next comma character.
// If found a comma character, it inserts a new line print command
// This function is called from the function send_msg()
void prettyPrintPayload(const char* buffer, int length) {
    int halfway = length / 2;
    bool splitInserted = false;
    for (int i = 0; i < length && buffer[i] != '\0'; ++i) {
        Serial.print(buffer[i]);

        if (!splitInserted && i >= halfway && buffer[i] == ',') {
            Serial.println();      // Break line
            splitInserted = true; // Ensure only one line break
        }
    }
    Serial.println(); // Final newline
}


bool send_msg()
{
  static constexpr const char txt0[] PROGMEM = "send_msg(): ";
  const char *txts[] PROGMEM = { "reading",       // 0
                                 "temperature",   // 1
                                 "pressure",      // 2
                                 "altitude",      // 3
                                 "humidity",      // 4
                                 "is extreme",    // 5
                                 "resetting" };   // 6
  bool ret = false;
  bool do_reset = false;

  if (!isItBedtime)
    neopixel_on();

  read_bme280_data(); // Read BME280 data and store in global variables
  
  do_test_reset = false;
  /*
  if (do_test_reset)
  {
    temperature = 0.0;
    pressure = 1010.0;
    Altitude = 0.0;
    Humidity = 100.0;
  }
  */

  if (do_test_reset)
    temperature = -20.0; // For testing purposes only

  if (temperature < -10.0 || temperature > 50.0)
  {
    // Temperature:
    serialPrintf(PSTR("%s%s %s %s\n"), txt0, txts[0], txts[1], txts[5]);
    temperature = 0.0;
    serialPrintf(PSTR("%s%s %s to %3.1f\n"), txt0, txts[6], txts[1], temperature);
  }

  if (do_test_reset)
    pressure = 1200.0;  // For test purposes only

  if (pressure < 800.0 || pressure > 1100.0)
  {
    // Pressure:
    serialPrintf(PSTR("%s%s %s %s: %7.2f\n"), txt0, txts[0], txts[2], txts[5], pressure);
    pressure = 1010.0;
    serialPrintf(PSTR("%s%s %s to %6.1f\n"), txt0, txts[6], txts[2], pressure);
    
    // return ret; // don't accept unrealistic extremes
  }

  if (do_test_reset)
    altitude = NAN;

  if (isnan(altitude)) {
    // Altitude:
    serialPrintf(PSTR("%s%s %s %s %F\n"), txt0, txts[0], txts[3], "resulted in", altitude);
    altitude = 0.0; // or some sentinel value
    serialPrintf(PSTR("%s%s %s to %3.1f\n"), txt0, txts[6], txts[3], altitude);
    // Log or handle safely  
  }

  if (do_test_reset)
    humidity = 200.0;

  if (humidity > 100.0)
  {
    // Humidity:
    serialPrintf(PSTR("%s%s %s %s\n"), txt0, txts[0], txts[4], txts[5]);
    humidity = 100.0;
    serialPrintf(PSTR("%s%s %s to %4.1f\n"), txt0, txts[6], txts[4], humidity);
  }
  
  if ( isnan(temperature) && isnan(pressure) && isnan(humidity) ) // do not check Altitude (Altitude) because it will be 0.00 m
    do_reset = true;
  else if (temperature == 0.0 && pressure == 1010.0 && altitude == 0.0 && humidity == 100.0)
    do_reset = true;

  if (do_reset)
  {
    bool dummy = handle_bme280();
  }

  msgGrpID++;
  if (msgGrpID > msgGrpID_max)
    msgGrpID = 1;
 
  int written = composePayload(payloadBuffer, sizeof(payloadBuffer), temperature, pressure, altitude, humidity, timestamp);
  if (written > 0)
  {
    serialPrintf(PSTR("Bytes written by composePayload(): %d\n"), written);
    char topic[23] = "sensors/Feath/ambient";
    if (!my_debug)
    {
      Serial.print(F("Topic: "));
      serialPrintf(PSTR("\"%s\"\n"), topic);
      if (my_debug)
        Serial.println(F("contents payloadBuffer: "));
      int null_found = ck_payloadBuffer(written, false); 
      if (null_found == written)  // a null-terminator has been found at the end of the payloadBuffer
      {
        // No null-terminator char found inside the written part of the payloadBuffer
        //Serial.println(payloadBuffer);
        // Try to split the (long) payloadBuffer text into two parts
        if (my_debug)
          prettyPrintPayload(payloadBuffer, written); // function print-split nicely the payloadBuffer

      } else if (null_found < written) {
        // A null-terminator char found inside the written part of the payloadBuffer
        // so, don't split print the payloadBuffer!
        Serial.println(payloadBuffer);
      }
    }
    size_t topicLength = strlen(topic);
    Serial.print(F("Topic length: "));
    Serial.println(topicLength);
    size_t payloadLength = strlen(payloadBuffer);
    Serial.print(F("Payload length: "));
    Serial.println(payloadLength);
    Serial.print(F("MQTT message ID: "));
    Serial.print(mqttMsgID);
    Serial.print(F(" = "));
    Serial.println(isoBufferLocal);
    
    mqttClient.beginMessage(topic);
    mqttClient.print(payloadBuffer);
    mqttClient.endMessage();
  } else {
    Serial.println("⚠️ Failed to compose JSON payload");
  }
  // delay(1000);  // spread the four messages 1 second
  
  Serial.print(F("MQTT message group: "));
  serialPrintf(PSTR("%3d sent\n"), msgGrpID);
  // Prepare and show text on the TFT display
  // disp_msg_info();
  if (!isItBedtime)
    neopixel_off();

  do_line(55);
  
  ret = true;

  return ret;
}

bool handle_bme280()
{
  bool ret = false;
  // default settings
   
  // You can also pass in a Wire library object like &Wire2
  // status = bme.begin(0x76, &Wire2)
  /*
      Extract from the Bosch BME280 datasheet

      5.4.2 Register 0xE0 "reset"
      The "reset" register contains the soft reset word reset(7:0). 
      If the value 0xB6 is written to the register,
      the device is reset using the complete power-on-reset procedure.
      Writing other values than 0xB6 has no effect.
      The readout value is always 0x00.

      Calling the self-test procedure starts with a soft reset of the sensor
      
  */
  uint8_t bme_check_cnt = 0;
  uint8_t bme_check_cnt_max = 3;
  byte addr = 0x76;
  if (TB.scanI2CBus(addr)) {  // For the Pimoroni multi-sensor-stick
    Serial.print("✅ BME280 address: 0x");
    Serial.println(addr, HEX);
  }
  else
  {
    Serial.print("❌ BME280 not found at address: ");
    Serial.println(addr, HEX);
  }
  status = bme.begin(addr, TB.theWire); 
  if (status)
    ret = true;
  else
  {
    while (!status)
    {
      Serial.println(F("❌ Could not find a valid BME280 sensor, check wiring, address, sensor ID!"));
      Serial.print(F("SensorID was: 0x")); 
      Serial.println(bme.sensorID(),16);
      Serial.print(F("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n"));
      Serial.print(F("   ID of 0x56-0x58 represents a BMP 280,\n"));
      Serial.print(F("        ID of 0x60 represents a BME 280.\n"));
      Serial.print(F("        ID of 0x61 represents a BME 680.\n"));
      Serial.print(F("doing a BME280 reset in 5 seconds..."));
      delay(5000);
      status = bme.begin(0x76, &Wire1); 
      delay(100);
      if (status)
      {
        ret = true;
        break;
      }
      bme_check_cnt++;
      if (bme_check_cnt >= bme_check_cnt_max)
      {
        reset();
        //break;
      }
    }
  }
  if (ret)
    Serial.println(F("✅ BME280 successfully (re-)initiated."));
  return ret;
}

void read_bme280_data()
{
  static constexpr const char txt0[] PROGMEM = "read_bme280_data(): ";
  // Read temperature, pressure, altitude and humidity
  temperature = bme.readTemperature();
  pressure = bme.readPressure() / 100.0F; // Convert Pa to mBar
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA); // Altitude in meters
  humidity = bme.readHumidity();

  if (my_debug)
  {
    Serial.print(txt0);
    Serial.print(F("Temp: "));
    Serial.print(temperature);
    Serial.print(F(" °C, Pressure: "));
    Serial.print(pressure);
    Serial.print(F(" mBar, Altitude: "));
    Serial.print(altitude);
    Serial.print(F(" m, Humidity: "));
    Serial.print(humidity);
    Serial.println(F(" %"));
  }
}

void setup() 
{
  //Initialize serial and wait for port to open:
  Serial.begin(115200);  
  //while (!Serial) { // Do not use this wait loop. It blocks mqtt transmissions when only on 5Volt power source!
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}
  Serial2.begin(115200);  // WiFi/BT AT command processor on ESP32-S3

  delay(1000);

 // turn on the TFT / I2C power supply
#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3_TFT)
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
#endif

  pinMode(led, OUTPUT); // for the builtin single color (red) led

  pinMode(NEOPIXEL_POWER, OUTPUT);
  digitalWrite(NEOPIXEL_POWER, HIGH); // Switch off the Neopixel LED
  pixel.begin();
  pixel.setBrightness(50);
  neopixel_test();

  delay(10);

  display.init(135, 240);           // Init ST7789 240x135
  display.setRotation(3);
  //canvas.setFont(&FreeSans12pt7b);
  canvas.setFont(&FreeMono12pt7b);
  canvas.setTextColor(ST77XX_WHITE);
  if (lc_bat.begin()) 
  {
    Serial.println("✅ Found LC709203F");
    Serial.print("Version: 0x"); Serial.println(lc_bat.getICversion(), HEX);
    lc_bat.setPackSize(LC709203F_APA_500MAH);
    lcfound = true;
  }
  else 
  {
    Serial.println(F("❌ Couldn\'t find Adafruit LC709203F?\nChecking for Adafruit MAX1704X.."));
    delay(200);
    if (!max_bat.begin()) 
    {
      Serial.println(F("❌ Couldn\'t find Adafruit MAX1704X?\nMake sure a battery is plugged in!"));
      while (1) delay(10);
    }
    Serial.print(F("✅ Found MAX17048"));
    Serial.print(F(" with Chip ID: 0x")); 
    Serial.println(max_bat.getChipID(), HEX);
    maxfound = true;
    
  }
  
  setupDisplayTimes();

  pinMode(TFT_BACKLITE, OUTPUT);
  digitalWrite(TFT_BACKLITE, HIGH);

  //display.drawRGBBitmap(0, 0, canvas.getBuffer(), canvas_width, canvas_height);

  // ESP32 is kinda odd in that secondary ports must be manually
  // assigned their pins with setPins()!

  // Result output: Default port (Wire) I2C scan: 0x23, 0x36, 0x51, 0x6A, 0x76,
  Serial.print("Default port (Wire) ");
  TB.theWire = DEFAULT_I2C_PORT;
  TB.printI2CBusScan();

/*
#if defined(SECONDARY_I2C_PORT)
  Serial.print("Secondary port (Wire1) ");
  TB.theWire = SECONDARY_I2C_PORT;
  TB.printI2CBusScan();
#endif
*/
  byte rtc_address = 0x51;
  if (TB.scanI2CBus(rtc_address))
  { 
    Serial.print(F("✅ RTC found at address: 0x"));
    Serial.print(rtc_address, HEX);
    Serial.println(F(". Starting it."));
    RTC.begin();
  }
  else
  {
    Serial.println(F("❌ RTC not found."));
  }

  // We call this function now to have the global variable HoursLocal updated
  unsigned long dummy = getUnixTimeFromRTC();
  // HousLocal is now set. It will be used in isItDisplayBedtime()
  if (!isItDisplayBedtime())
    disp_intro();
  
  if (!handle_bme280())
  {
    Serial.println("❌ Initiating BME280 failed");
    Serial.println("Check wiring. Going into an endless loop...");
    while (true)
      delay(5000);
  }

  unsigned status;
  // attempt to connect to WiFi network:
  if (ConnectToWiFi())
  {
    if (WiFi.status() == WL_CONNECTED)
    {
      timeClient.begin();
      Serial.print(F("\nTimezone offset = "));
      if (tzOffset < 0)
        Serial.print("-");
      Serial.print(abs(utc_offset)/3600);
      Serial.println(F(" hour(s)"));
      Serial.println(F("Starting connection to NTP server..."));
      /*
      WiFiClient probe;
      if (probe.connect("192.168.1.114", 1883)) {  // 192.168.1.96 = PC Paul5, 192.168.1.114 = RaspberryPi CM5
        Serial.println(F("✅ TCP probe connect to broker successful"));
        probe.stop();
      } else {
        Serial.println(F("❌ TCP probe connect to broker failed"));
      }
      */
      //Serial.print(F("\nMQTT Attempting to connect to broker: "));
      //serialPrintf(PSTR("%s:%s\n"), broker, String(port).c_str());

      bool mqtt_connected = false;
      for (uint8_t i=0; i < 10; i++)
      {
        if (!mqttClient.connect(broker, port))
        {
          if (mqttClient.connectError() == 5) // = Connection refused. Client is not authorized
          {
            Serial.println(F("❌ MQTT connection refused. Client not authorized."));
            break;
          }
          else
          {
            serialPrintf(PSTR("❌ MQTT connection to broker failed! Error code = %s\n"), String(mqttClient.connectError()).c_str() );        
            delay(1000);
          }
        }
        else
        {
          mqtt_connected = true;
        }
      }
      if (!mqtt_connected)
      {
        Serial.print(F("❌ MQTT Unable to connect to broker in LAN. Going into infinite loop..."));
        while (true)
          delay(5000);
      }

      Serial.print(F("✅ MQTT You're connected to broker: "));
      serialPrintf(PSTR("%s:%s\n"), broker, String(port).c_str());
    }
  }
}

void loop() 
{
  static constexpr const char txt0[] PROGMEM = "loop(): ";
  char sID[] = "Feather";  // length 7 + 1
  //set interval for sending messages (milliseconds)
  unsigned long mqtt_start_t = millis();
  unsigned long mqtt_curr_t = 0L;
  unsigned long mqtt_elapsed_t = 0L;
  unsigned long mqtt_interval_t = 1 * 60 * 1000; // 1 minute

  unsigned long ntp_start_t = mqtt_start_t;
  unsigned long ntp_curr_t = 0L;
  unsigned long ntp_elapsed_t = 0L;
  unsigned long ntp_interval_t = 15 * 60 * 1000; // 15 minutes
  bool start = true;

  serialPrintf(PSTR("board ID = \"%s\"\n"), sID);

  uint8_t interval_in_mins = mqtt_interval_t / (60 * 1000);
  serialPrintf(PSTR("%sMQTT message send interval = %d minute%s\n"), txt0, interval_in_mins, interval_in_mins <= 1 ? "" : "s");

 
  bool displayIsAsleep = false;
  
  while (true)
  {
    // call poll() regularly to allow the library to send MQTT keep alive which
    // avoids being disconnected by the broker
    mqttClient.poll();

    unsigned long ntp_curr_t = millis();
    ntp_elapsed_t = ntp_curr_t - ntp_start_t;
    if (start || ntp_curr_t - ntp_start_t >= ntp_interval_t)
    {
      ntp_start_t = ntp_curr_t;
      rtc_sync();
    }

    mqtt_curr_t = millis();
    mqtt_elapsed_t = mqtt_curr_t - mqtt_start_t;
    if (start || mqtt_elapsed_t >= mqtt_interval_t) 
    {
      start = false;
      // save the last time a message was sent
      mqtt_start_t = mqtt_curr_t;

      uint8_t try_cnt = 0;
      uint8_t try_cnt_max = 10;

      while (!send_msg())
      {
        try_cnt++;
        if (try_cnt >= try_cnt_max)
          break;
        delay(50);
      } 
    }
    
    // Only display the messages if not in bedtime mode
    isItBedtime = isItDisplayBedtime();
    
    if (my_debug)
      serialPrintf(PSTR("isItBedtime = %s\n"), (isItBedtime) ? "true" : "false");
    if (isItBedtime)
    {
      // If it is bedtime, clear the display
      if (!displayIsAsleep)
      {
        // If the display is not asleep, show a goodnight message
        disp_goodnight();
        delay(5000); // Show the text for 5 seconds
        // Clear the display
        canvas.fillScreen(ST77XX_BLACK);
        display.drawRGBBitmap(0, 0, canvas.getBuffer(), canvas_width, canvas_height);
        //digitalWrite(TFT_I2C_POWER, LOW); // This probably also cuts off the I2C connection with the BME280
        displayIsAsleep = true;
      }
    }
    else
    {
      if (displayIsAsleep)
      {
        // Only display the messages if not in bedtime mode

        isItBedtime = isItDisplayBedtime();
        if (!isItBedtime) {
          Serial.print(F("isItBedTime = "));
          Serial.println((isItBedtime) ? "true" : "false");
        }
        digitalWrite(TFT_I2C_POWER, HIGH);
        disp_goodmorning();
        delay(5000); // Show the text for 5 seconds
        // Clear the display
        canvas.fillScreen(ST77XX_BLACK);
        display.drawRGBBitmap(0, 0, canvas.getBuffer(), canvas_width, canvas_height);
        displayIsAsleep = false; // reset the displayIsAsleep flag
      }
      // Alternatively show displays
      disp_msg_info(false);
      delay(3000); // wait for 3 seconds
      disp_sensor_data(false);
      delay(3000); // wait for 3 seconds}
    }
  }
}

