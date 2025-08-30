#include "mqtt/mqtt.h"
#include <PubSubClient.h>
#ifdef USE_PAULSKPT_PARTS
#include "squixl.h"
#include "RTC.h"
#include <ArduinoJson.h> // added by @PaulskPt (suggested by Copilot)
#include <vector>        // same
#include <string>        // same
#include <iostream>      // same
#include <sstream>
#include <iomanip>
#include <ctime>
#include "utils/RtcFormatter.h"
#endif

using json = nlohmann::json;

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(MQTT_Payload, owner, device_class, state_class, unit_of_measurement, sensor_value, value_type, min_value, max_value, description, timestamp);

WiFiClient espClientMQTT;
PubSubClient mqtt_client(espClientMQTT);

// Defined in settingsOption.h
#ifdef USE_PAULSKPT_PARTS

//METAR metar_data;

const size_t MAX_TOTAL_PAYLOADS = 1;

template<typename MapType>
void MQTT_Stuff::mqtt_clean_map_if_needed(MapType& mqtt_topic_payloads) {
    for (auto& [topic, payloads] : mqtt_topic_payloads) {
        if (payloads.size() > MAX_TOTAL_PAYLOADS) {
            MQTT_Payload latest = payloads.back();
            payloads.clear();
            payloads.push_back(latest);
        }
    }
}

METAR MQTT_Stuff::mqtt_split_metar(const std::string& msg) {
    std::istringstream iss(msg);
    std::vector<std::string> tokens;
    std::string token;

    // Split the METAR into individual tokens
    while (iss >> token) {
        tokens.push_back(token);
    }

    METAR metar;

    // Group tokens into three sections
    for (size_t i = 0; i < tokens.size(); ++i) {
        if (i < 3)
            metar.section1 += tokens[i] + " ";
        else if (i < 7)
            metar.section2 += tokens[i] + " ";
        else
            metar.section3 += tokens[i] + " ";
    }

    // Trim trailing spaces
    if (!metar.section1.empty()) metar.section1.pop_back();
    if (!metar.section2.empty()) metar.section2.pop_back();
    if (!metar.section3.empty()) metar.section3.pop_back();

#ifdef MY_DEBUG
		Serial.println(F("mqtt_split_metar(): "));
		Serial.print(F("parameter: msg = "));
		Serial.println(msg.c_str());
		Serial.print(F("metar.section1 = \""));
		Serial.print(metar.section1.c_str());
		Serial.println("\"");
		Serial.print(F("metar.section2 = \""));
		Serial.print(metar.section2.c_str());
		Serial.println("\"");
		Serial.print(F("metar.section3 = \""));
		Serial.print(metar.section3.c_str());
		Serial.println("\"");
#endif

    return metar;
}

#endif

// Static callback that wraps the instance method
void MQTT_Stuff::static_mqtt_callback(char *topic, byte *message, unsigned int length)
{
	mqtt_stuff.mqtt_callback(topic, message, length);
}

void MQTT_Stuff::mqtt_callback(char *topic, byte *message, unsigned int length)
{
	String messageTemp;
	for (int i = 0; i < length; i++)
		messageTemp += (char)message[i];

#ifndef USE_PAULSKPT_PARTS
	Serial.printf("New message on topic: %s\n", topic);
	Serial.printf("message: %s\n", messageTemp.c_str());
#endif

#ifdef USE_PAULSKPT_PARTS

	char payloadBuffer[length + 1]; 
	memcpy(payloadBuffer, message, length); 
	payloadBuffer[length] = '\0';

	Serial.printf("New message on topic: %s\n", topic); 
	Serial.printf("message: %s\n", payloadBuffer);

	if (strstr(topic, "sensor") != NULL || strstr(topic, "weather") != NULL)  
	{
		// Next line added on advise of MS Copilot
		mqtt_topic_payloads.clear(); // ✅ Clear old data before parsing new one
		// Static is depricated! (source: Copilot), 
		// however creates another error: :JsonDocument' is not a template
		StaticJsonDocument<512> doc;
		DeserializationError error = deserializeJson(doc, payloadBuffer);

		if (error) { 
			Serial.print("❌ JSON parse error: ");
			Serial.println(error.c_str()); 
			return;
		}
	
		/*
		  Example sensor tpah mqtt message type:
			{"hd":
				{
					"ow":"Feath","de":"Lab","dc":"BME280","sc":"meas","vt":"f","t":1755686296
				},
				"reads":
				{
					"t":{"v":30.1,"u":"C","mn":-10,"mx":50},
					"p":{"v":1005,"u":"mB","mn":800,"mx":1200},
					"a":{"v":93.5,"u":"m","mn":0,"mx":3000},
					"h":{"v":53.6,"u":"%","mn":0,"mx":100}
				}
			}
			Example metar mqtt message type:
			{"hd": 
				{
					"de": "Ext", "sc": "meas", "vt": "s", "t": 1756245600, "dc": "wx", "ow": "PL2XLW"
			  },
		    "metar": 
				{
					"raw": "METAR LPPT 262200Z 34014KT 310V010 CAVOK 19/15 Q1014"}
				}
	    }
		*/

		MQTT_Payload payload;
		
		JsonObject header = doc["hd"];
   
		psram_string owner        = header["ow"] | "";
		psram_string description  = header["de"] | "";
		psram_string device_class = header["dc"] | "";
		psram_string state_class  = header["sc"] | "";
		psram_string value_type   = header["vt"] | "";
		psram_string tsStr        = header["t"]  | "";
		unsigned long ts;
#ifdef MY_DEBUG
		Serial.print(F("mqtt_callback(): owner = "));
		Serial.println(owner.c_str());
#endif

#ifndef USE_LOCAL_TIME_PAULSKPT
#define USE_LOCAL_TIME_PAULSKPT
#endif

#ifdef USE_LOCAL_TIME_PAULSKPT
							// ------------ Addition 2025-08-30 by @PaulskPt
							// to convert the GMT related mqtt msg value of key header["t"]
							// to local time
							int utc_offset_seconds = settings.config.location.utc_offset * 3600;
							//int localTimestamp = payload.timestamp + utc_offset_seconds;
							int timestampLocalOriginal = header["t"].as<long>();
							int timestampLocalNew = timestampLocalOriginal + utc_offset_seconds;
							psram_string timestampStrLocalNew = rtc.unix_timestamp_to_time_str(timestampLocalNew);
							psram_string timestampStr = RtcFormatter::format_datetime(timestampLocalNew, true);
#ifdef MY_DEBUG
							Serial.print(F("mqtt_callback(): "));
							Serial.print(F("utc_offset_seconds = "));
							Serial.println(utc_offset_seconds);
							Serial.print(F("timeStampLocalOriginal (from mqtt msg header[\"t\"]) = "));
							Serial.println(timestampLocalOriginal);
							Serial.print(F("timestampLocalNew = "));
							Serial.println(timestampLocalNew);
							Serial.print(F("timestampStrLocalNew = "));
							Serial.println(timestampStrLocalNew.c_str());
							Serial.print(F("timestampStr = "));
							Serial.println(timestampStr.c_str());
#endif
							// ------- end of addition ---------------------
#endif

		payload.owner         = (owner == "Feath") ? "Feather" : owner;
		payload.description   = description;
		payload.device_class  = device_class;
		payload.state_class   = state_class;
		payload.value_type    = value_type;
		payload.timestamp     = header["t"].as<long>();
#ifdef USE_LOCAL_TIME_PAULSKPT
		payload.timestampStr = timestampStr;
#else
		payload.timestampStr  = convertIntToStr(payload.timestamp);
#endif
		payload.msgID = psram_string(std::to_string(header["t"].as<unsigned long>()));
		
#ifdef USE_METAR
#undefine USE_METAR
#endif 

//#ifdef USE_METAR
		if (owner == "Feath")
		{
		  JsonObject readings = doc["reads"];

			for (JsonPair kv : readings) 
			{ 
				const char* key = kv.key().c_str();  // "t", "p", "a", "h"
				JsonObject obj = kv.value().as<JsonObject>();

				// MQTT_Payload payload; 
				
				// Format values with 1 decimal place
				std::ostringstream valStream, minStream, maxStream;
				valStream << std::fixed << std::setprecision(1) << obj["v"].as<float>();
				minStream << std::fixed << std::setprecision(1) << obj["mn"].as<float>();
				maxStream << std::fixed << std::setprecision(1) << obj["mx"].as<float>();

				payload.sensor_value = psram_string(valStream.str());
				payload.min_value = psram_string(minStream.str());
				payload.max_value = psram_string(maxStream.str());

				psram_string unit = obj["u"] | "";
				if (unit == "C") unit = "°C";
				payload.unit_of_measurement = unit;


				// Assign human-readable description
				std::vector<std::string> terms = {"Temperature:", "Pressure:", "Altitude:", "Humidity:", "?"};
				
				if      (strcmp(key, "t") == 0) payload.term = psram_string(terms[0]); 
				else if (strcmp(key, "p") == 0) payload.term = psram_string(terms[1]); 
				else if (strcmp(key, "a") == 0) payload.term = psram_string(terms[2]); 
				else if (strcmp(key, "h") == 0) payload.term = psram_string(terms[3]); 
				else                            payload.term = psram_string(terms[4]);
				std::string longestTerm = *std::max_element(terms.begin(), terms.end(),
					[](const std::string& a, const std::string& b) {
							return a.length() < b.length();
					});
					// std::cout << "Longest term: " << longestTerm << std::endl;

				psram_string longestTermStr = psram_string(longestTerm); // ← Only if needed

				size_t longestTermLength = longestTerm.length();

				mqtt_dirty = false;

				// Insert or update the payload in the map
				if (mqtt_topic_payloads.find(payload.owner) == mqtt_topic_payloads.end()) 
				{ 
						mqtt_topic_payloads[payload.owner].push_back(payload); 
						Serial.print(F("\nMQTT: New Sensor owner: "));
						Serial.printf("%s\n",	payload.owner.c_str());
						Serial.print(F("MQTT: Added new sensor from "));
						Serial.printf("%s, loc: %s, dev: %s (%-*s) at: %s\n",
								payload.owner.c_str(),
								payload.description.c_str(),
								payload.device_class.c_str(), 
								(int)longestTermLength, payload.term.c_str(),
								payload.timestampStr.c_str());
								payload.msgID.c_str();
						mqtt_dirty = true; 
				} 
				else 
				{ 
					bool updated = false; 
					for (auto& existing : mqtt_topic_payloads[payload.owner]) 
					{ 
						if (existing.device_class == payload.device_class && 
								existing.term == payload.term && 
								existing.timestamp < payload.timestamp) 
						{

							bool use_human_readable_format = true;
							std::string timestampStr;

							existing.upate_from(payload); 


#ifndef USE_LOCAL_TIME_PAULSKPT
							if (use_human_readable_format)
							{
								// Original code line:
								//timestampStr = convertIntToStr(payload.timestamp);
							}
							else	// Use format: 1720099140
							{
								timestampStr = std::to_string(payload.timestamp).c_str();
								
							}
#endif
							payload.timestampStr = timestampStr;
							// "from" instead UM's "for"
							/*
							Serial.printf("MQTT: Updated sensor %s: %s (%s) from %s at %s\n",
														payload.description.c_str(),
														payload.device_class.c_str(), 
														payload.term.c_str(), 
														payload.owner.c_str(),
														payload.timestampStr.c_str()); 
							*/
						  Serial.print(F("MQTT: Updated sensor, from "));
							Serial.printf("%s, loc: %s, dev: %s (%-*s) at: %s\n",
								payload.owner.c_str(),
								payload.description.c_str(),
								payload.device_class.c_str(), 
								(int)longestTermLength, payload.term.c_str(), 
								payload.timestampStr.c_str());

							mqtt_dirty = true; 
							updated = true; 
							break;
						} 
					}

					if (!updated) 
					{
						mqtt_topic_payloads[payload.owner].push_back(payload); 
						Serial.print(F("\nMQTT: Added new sensor, msgID "));
						Serial.printf("%s from %s, loc: %s, dev: %s (%-*s) \nNow has %d sensors\n", 
													payload.msgID.c_str(),
													payload.owner.c_str(), 
													payload.description.c_str(),
													payload.device_class.c_str(),
													(int)longestTermLength, payload.term.c_str(), 
													mqtt_topic_payloads[payload.owner].size()); 
						delay(100); // try to get "Temperature" Serial.printf complete
						mqtt_dirty = true; 
					} 
				} 
			}
		}
//#else
		if (owner == "PL2XLW")
		{
			// Handle weather/PL2XLW/LPPT
			JsonObject metar = doc["metar"];
#ifdef MY_DEBUG
			Serial.print(F("metar = "));  // for test do a print to show what is received
#endif
			std::string metarDataStr = metar["raw"] | "";
#ifdef MY_DEBUG
			Serial.println(metarDataStr.c_str());  // for test do a print to show what is received
#endif
			// MQTT_Payload payload; 
			// msg = b'{"metar": {"raw": "METAR LPPT 262200Z 34014KT 310V010 CAVOK 19/15 Q1014"}, \
			// "hd": {"de": "Ext", "sc": "meas", "vt": "s", "t": 1756245600, "dc": "wx", "ow": "PL2XLW"}}'

			metar_data = mqtt_split_metar(metarDataStr);

			// In ui_scrollarea.cpp we are going to access metar_data directly through the struct
			
			payload.sensor_value = "";  // just for the sake of the program flow: give it an empty value

			mqtt_dirty = false;

			// Insert or update the payload in the map
			//if (mqtt_topic_payloads.find(payload.owner) == mqtt_topic_payloads.end()) 
			auto ownerKey = payload.owner;
			if (mqtt_topic_payloads.find(ownerKey) == mqtt_topic_payloads.end()) {
					mqtt_topic_payloads[ownerKey].push_back(payload);

					Serial.print(F("\nMQTT: New metar data owner: "));
					Serial.printf("%s\n", payload.owner.c_str());

					Serial.print(F("MQTT: Added metar data from "));
					Serial.printf("%s, loc: %s, dev: %s at: %s\n",
							payload.owner.c_str(),
							payload.description.c_str(),
							payload.device_class.c_str(),
							payload.timestampStr.c_str());
							//payload.sensor_value.c_str());
					Serial.printf("msg: %s\n%s\n%s\n",
						metar_data.section1.c_str(),
						metar_data.section2.c_str(),
						metar_data.section3.c_str());

					mqtt_dirty = true;
			}
			else 
			{ 
				bool updated = false; 
				for (auto& existing : mqtt_topic_payloads[psram_string(payload.owner.c_str())]) 
				{ 
					if (existing.device_class == payload.device_class && 
							existing.term == payload.term && 
							existing.timestamp < payload.timestamp) 
					{
						bool use_human_readable_format = true;
						std::string timestampStr;

						existing.upate_from(payload);

						timestampStr = use_human_readable_format
								? convertIntToStr(payload.timestamp)
								: std::to_string(payload.timestamp);

						payload.timestampStr = psram_string(timestampStr); // if needed

						Serial.print(F("MQTT: Updated metar data, from "));
						Serial.printf("%s, loc: %s, dev: %s at: %s\n",
								payload.owner.c_str(),
								payload.description.c_str(),
								payload.device_class.c_str(),
								payload.timestampStr.c_str());
								//payload.sensor_value.c_str());
						Serial.printf("msg: %s\n%s\n%s\n",
							metar_data.section1.c_str(),
							metar_data.section2.c_str(),
							metar_data.section3.c_str());
						
						mqtt_dirty = true;
						updated = true;
						break;
					} 
				}
			  if (!updated) {
					psram_string ownerKey = payload.owner;
					mqtt_topic_payloads[ownerKey].push_back(payload);

					mqtt_clean_map_if_needed(mqtt_topic_payloads);

#ifndef MY_DEBUG
    			std::cout << "Remaining topic_payloads (after call to \"mqtt_clean_map_if_needed()\"): "
              << mqtt_topic_payloads.size() << "\n";
#endif
					Serial.print(F("\nMQTT: Added new metar data, msgID "));
					//Serial.printf("%s from %s, loc: %s, dev: %s at: %s\nmsg: %s\nNow has %d metar data sets\n",
					Serial.printf("%s from %s, loc: %s, dev: %s at: %s\n",
						payload.msgID.c_str(),
						payload.owner.c_str(),
						payload.description.c_str(),
						payload.device_class.c_str(),
						payload.timestampStr.c_str());
						//payload.sensor_value.c_str(),
					Serial.printf("msg: %s\n%s\n%s\n",
						metar_data.section1.c_str(),
						metar_data.section2.c_str(),
						metar_data.section3.c_str());
					Serial.printf("Now has %d metar data sets\n",
						mqtt_topic_payloads[ownerKey].size());

					delay(100);
					mqtt_dirty = true;
				}

			} 
		}
//#endif
	}
#else
	// If the string "sensor" is in the topic, we can assume it's sensor data coming in
	if (strstr(topic, "sensor") != NULL)
	{
		// Parse the string into a Json structure
		psram_json json_data = json::parse(messageTemp);
		// Convert json to struct
		MQTT_Payload payload = json_data.get<MQTT_Payload>();

		mqtt_dirty = false;

		if (mqtt_topic_payloads.find(payload.owner.c_str()) == mqtt_topic_payloads.end())
		{
			mqtt_topic_payloads[payload.owner].push_back(payload);
			Serial.printf("\nMQTT: New Sensor owner: %s\nAdded new %s sensor at %s\n", payload.owner.c_str(), payload.device_class.c_str(), payload.description.c_str());
			mqtt_dirty = true;
		}
		else
		{
			for (int s = 0; s < mqtt_topic_payloads[payload.owner].size(); s++)
			{
				if (mqtt_topic_payloads[payload.owner][s].device_class == payload.device_class && mqtt_topic_payloads[payload.owner][s].timestamp < payload.timestamp)
				{
					mqtt_topic_payloads[payload.owner][s].upate_from(payload);
					Serial.printf("\nMQTT: Updated sensor %s for %s\n", payload.device_class.c_str(), payload.owner.c_str());
					mqtt_dirty = true;
					break;
				}
			}
		}

		if (!mqtt_dirty)
		{
			mqtt_topic_payloads[payload.owner].push_back(payload);
			Serial.printf("\nMQTT: Added new sensor %s for %s at %s\nNow has %d sensors\n", payload.device_class.c_str(), payload.owner.c_str(), payload.description.c_str(), mqtt_topic_payloads[payload.owner].size());

			mqtt_dirty = true;
		}
	}
#endif
}

void MQTT_Stuff::mqtt_reconnect()
{
	is_mqtt_connecting = true;

	if (!mqtt_server_setup)
	{
		Serial.println("\nAttempting MQTT Server Setup..");
		mqtt_server_setup = true;
		mqtt_client.setServer(settings.config.mqtt.broker_ip.c_str(), 1883);
		// mqtt_client.setServer(mqtt_server, 1883);
		mqtt_client.setCallback(MQTT_Stuff::static_mqtt_callback);
		mqtt_client.setBufferSize(512);
		is_mqtt_connecting = false;
		next_mqtt_reconnect = millis() + 5000;
		return;
	}

	Serial.print("\nMQTT: Attempting connection to ");
	Serial.println(settings.config.mqtt.broker_ip);
	// Serial.print("My IP address is ");
	// Serial.println(WiFi.localIP());
	// Serial.println();
	// Attempt to connect
	if (mqtt_client.connect(settings.config.mqtt.device_name.c_str(), settings.config.mqtt.username.c_str(), settings.config.mqtt.password.c_str()))
	{
		Serial.println("MQTT: connected");
		// Subscribe
		for (int i = 0; i < settings.config.mqtt.topics.size(); i++)
		{
			if (!settings.config.mqtt.topics[i].topic_listen.isEmpty())
			{
				Serial.printf("MQTT: Subscribing to topic: %s\n", settings.config.mqtt.topics[i].topic_listen.c_str());
				mqtt_client.subscribe(settings.config.mqtt.topics[i].topic_listen.c_str(), 1);
			}
		}
		is_mqtt_connecting = false;
	}
	else
	{
		retry_attemps--;
		retry_time += 5000;

		Serial.print(F("MQTT: connection failed, rc="));
		Serial.print(mqtt_client.state());
		if (retry_attemps >= 0)
			Serial.printf(" try again (attempt %d/3) in %d seconds\n", (4 - retry_attemps), (retry_time / 1000));
		else
			Serial.println(F("\nMQTT: No more attempts to cpnect to MQTT!"));

		// Wait 5 seconds before retrying

		is_mqtt_connecting = false;
	}
}

void MQTT_Stuff::process_mqtt()
{
	if (millis() - next_mqtt_reconnect > retry_time && !mqtt_client.connected())
	{
		next_mqtt_reconnect = millis();

		if (!is_mqtt_connecting && retry_attemps > 0)
			mqtt_reconnect();
	}
	else
	{
		mqtt_client.loop();
	}
}

// Function added by @PaulskPt
// This function was created at the time that the MQTT_Payload struct item timestamp was defined as an int.
std::string MQTT_Stuff::convertIntToStr(int tmStamp)
{
	std::string timeStmpStr;
		// Convert to tm struct (local time)
		
		// ✅ Cast to time_t
		std::time_t ts = static_cast<std::time_t>(tmStamp);

		// If tmStamp is already adjusted for UTC+1, then use gmtime
		std::tm* tm_ptr = std::gmtime(&ts);

		// Format to string
		std::ostringstream oss;
		oss << std::put_time(tm_ptr, "%Y-%m-%d %H:%M:%S");

		timeStmpStr = oss.str();
		return timeStmpStr;
}
MQTT_Stuff mqtt_stuff;