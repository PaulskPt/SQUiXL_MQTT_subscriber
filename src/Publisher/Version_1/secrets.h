#define SECRET_SSID "<Your WiFi SSID"
#define SECRET_PASS "<Your WiFi PASSWORD"
#define SECRET_TIMEZONE_OFFSET "1" // Europe/Lisbon (UTC offset in hours)
// #define TIMEZONE_OFFSET "-4" // America/New_York
#define SECRET_NTP_SERVER1 "1.pt.pool.ntp.org"
#define SECRET_MQTT_BROKER "5.196.78.28" // test.mosquitto.org"
#define SECRET_MQTT_BROKER_LOCAL1 "192.168.1.96"  // Your Local mosquitto broker app on PC ____
#define SECRET_MQTT_BROKER_LOCAL2 "192.168.1.114"  // Your Local mosquitto broker app on Raspberry Pi ___ (in my case a RPi CM5)
#define SECRET_MQTT_PORT "1883" 
#define SECRET_MQTT_TOPIC "sensors/Feath/ambient"  // Sent by the Publisher device (in my case an Adafruit Feather ESP32-S3 TFT)
#define SECRET_DISPLAY_SLEEPTIME "23"  // Feather display going to sleep (black) time
#define SECRET_DISPLAY_AWAKETIME "8"   // Feather display wakeup time


