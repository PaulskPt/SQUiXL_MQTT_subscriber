#pragma once
#include "squixl.h"

#ifndef USE_PAULSKPT_PARTS
#define USE_PAULSKPT_PARTS   (1)  // for now: do not use 
#endif

#ifdef USE_PAULSKPT_PARTS
#include <vector>        // same
#include <string>        // same
#endif

class ui_gauge;

/*
{
	"owner": "xxxxxxx",
	"device_class": "temperature",
	"state_class": "measurement",
	"sensor_value": 23.90,
	"value_type": "float",
	"unit_of_measurement": "°C",
	"min_value": "-1.0",
	"max_value": "60.0",
	"description": "Office BME280",
	"timestamp": 999999999
}

*/

struct MQTT_Payload
{
    psram_string owner;
    psram_string device_class;
    psram_string state_class;
    psram_string term; // Like meteorological term: "Pressure"
    psram_string unit_of_measurement;
    psram_string sensor_value;
    psram_string value_type;
    psram_string min_value;
    psram_string max_value;
    psram_string description;
    psram_string timestampStr; // human readable ISO format
    psram_string msgID;
    int timestamp = 0;

		// Non serialised data
		bool is_dirty = true;
		ui_gauge *dash_item = nullptr;

		void upate_from(MQTT_Payload new_data)
		{
			if (sensor_value != new_data.sensor_value)
			{
				is_dirty = true;
			}
			sensor_value = new_data.sensor_value;
			timestamp = new_data.timestamp;
			is_dirty = true;
		}

		void set_dash_item(ui_gauge *new_dash_item)
		{
			if (dash_item != nullptr)
			{
				Serial.printf("%s %s already has a dash item!!!\n", owner, device_class);
				return;
			}
			dash_item = new_dash_item;
		}

		psram_string get_sensor_value()
		{
			//return (sensor_value + unit_of_measurement);
			return psram_string(sensor_value + unit_of_measurement);
		}
};

using psram_string = std::basic_string<char, std::char_traits<char>, PsramAllocator<char>>;
using MQTTVector = std::vector<MQTT_Payload, PsramAllocator<MQTT_Payload>>;
using MQTTMap = std::map<
psram_string,
MQTTVector,
std::less<psram_string>,
PsramAllocator<std::pair<const psram_string, MQTTVector>>
>;

class MQTT_Stuff
{
	public:
		std::map<
				psram_string,
				std::vector<MQTT_Payload, PsramAllocator<MQTT_Payload>>,
				std::less<psram_string>,
				PsramAllocator<
						std::pair<
								const psram_string,
								std::vector<MQTT_Payload, PsramAllocator<MQTT_Payload>>
						>
				>
		> mqtt_topic_payloads;

    template<typename MapType>
    void mqtt_clean_map_if_needed(MapType& mqtt_topic_payloads);

		//std::map<std::string, std::vector<MQTT_Payload>> mqtt_topic_payloads;
    std::string mqtt_split_and_join(std::string msg);
		void mqtt_reconnect();
		void mqtt_callback(char *topic, byte *message, unsigned int length);
		void process_mqtt();

		// Static callback that wraps the instance method
		static void static_mqtt_callback(char *topic, byte *message, unsigned int length);

		bool mqtt_dirty = false;

	protected:

#ifdef USE_PAULSKPT_PARTS
		std::string convertIntToStr(int tmStamp);  // Added by @PaulskPt
#endif

		unsigned long next_mqtt_reconnect = 0;

		bool is_mqtt_connecting = false;
		bool mqtt_server_setup = false;

		int8_t retry_attemps = 3;
		uint16_t retry_time = 5000;
};

extern MQTT_Stuff mqtt_stuff;
