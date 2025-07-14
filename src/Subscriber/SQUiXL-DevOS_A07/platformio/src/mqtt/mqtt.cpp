#include "mqtt.h"
#include <PubSubClient.h>
#ifdef USE_PAULSKPT_PARTS
#include <ArduinoJson.h> // added by @PaulskPt (suggested by Copilot)
#include <vector>        // same
#include <string>        // same
#include <iostream>      // same
#include <sstream>
#include <iomanip>
#include <ctime>

#endif

using json = nlohmann::json;

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(MQTT_Payload, owner, device_class, state_class, unit_of_measurement, sensor_value, value_type, min_value, max_value, description, timestamp);

WiFiClient espClientMQTT;
PubSubClient mqtt_client(espClientMQTT);

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

	if (strstr(topic, "sensor") != NULL) 
	{ 
		// Static is depricated! (source: Copilot), 
		// however creates another error: :JsonDocument' is not a template
		StaticJsonDocument<512> doc;
		DeserializationError error = deserializeJson(doc, payloadBuffer);

		if (error) { 
			Serial.print("❌ JSON parse error: "); 
			Serial.println(error.c_str()); 
			return;
		}
		
		std::string owner        = doc["ow"] | ""; 
		std::string description  = doc["de"] | "";
		std::string device_class = doc["dc"] | ""; 
		std::string state_class  = doc["sc"] | ""; 
		std::string value_type   = doc["vt"] | ""; 
		std::string tsStr = doc["ts"] | "";  // "ts" is the timestamp in seconds, as string
		unsigned long ts;

	  JsonObject readings = doc["reads"];
    for (JsonPair kv : readings) 
    { 
			const char* key = kv.key().c_str();  // "t", "p", "a", "h"
			JsonObject obj = kv.value().as<JsonObject>();

			MQTT_Payload payload; 
			payload.owner         = (owner == "Feath") ? "Feather" : owner; // example "sensors/UnoR4W/ambient" or "sensors/Feath/ambient"
			payload.description   = description; // example: "PC-Lab" // Added by @PaulskPt
			payload.device_class  = device_class; 
			payload.state_class   = state_class; 
			payload.value_type    = value_type; 
			payload.timestamp     = doc["ts"].as<long>();  // ✅ assign as integer
			payload.timestampStr  = convertIntToStr(payload.timestamp);
			ts = doc["ts"].as<unsigned long>();  // or long, depending on your JSON
			tsStr = std::to_string(ts);
			payload.msgID = tsStr;

			// Format values with 1 decimal place
			std::ostringstream valStream, minStream, maxStream;
			valStream << std::fixed << std::setprecision(1) << obj["v"].as<float>();
			minStream << std::fixed << std::setprecision(1) << obj["mn"].as<float>();
			maxStream << std::fixed << std::setprecision(1) << obj["mx"].as<float>();

			payload.sensor_value = valStream.str(); 
			payload.min_value    = minStream.str(); 
			payload.max_value    = maxStream.str();

			std::string unit = obj["u"] | ""; 
			if (unit == "C") unit = "°C"; 
			payload.unit_of_measurement = unit;

			// Assign human-readable description
			std::vector<std::string> terms = {"Temperature", "Pressure", "Altitude", "Humidity", "?"};

			if      (strcmp(key, "t") == 0) payload.term = terms[0]; // "Temperature"; 
			else if (strcmp(key, "p") == 0) payload.term = terms[1]; // "Pressure"; 
			else if (strcmp(key, "a") == 0) payload.term = terms[2]; //"Altitude"; 
			else if (strcmp(key, "h") == 0) payload.term = terms[3]; //"Humidity"; 
			else                            payload.term = terms[4]; // "?";
			
		
      std::string longestTerm = *std::max_element(terms.begin(), terms.end(),
        [](const std::string& a, const std::string& b) {
            return a.length() < b.length();
        });

     		// std::cout << "Longest term: " << longestTerm << std::endl;

			size_t longestTermLength = longestTerm.length();

			mqtt_dirty = false;

			// Insert or update the payload in the map
			if (mqtt_topic_payloads.find(payload.owner) == mqtt_topic_payloads.end()) 
			{ 
					mqtt_topic_payloads[payload.owner].push_back(payload); 
					Serial.printf("\nMQTT: New Sensor owner: %s\n",
												payload.owner.c_str()); 
					Serial.printf("MQTT: Added new sensor from %s, loc: %s, dev: %s (%-*s) at: %s\n",
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

						if (use_human_readable_format)
						{
							timestampStr = convertIntToStr(payload.timestamp);
						}
						else	// Use format: 1720099140
						{
							timestampStr = std::to_string(payload.timestamp).c_str();
							
						}
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
						Serial.printf("MQTT: Updated sensor, from %s, loc: %s, dev: %s (%-*s) at: %s\n",
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
					Serial.printf("\nMQTT: Added new sensor, msgID %s from %s, loc: %s, dev: %s (%-*s) \nNow has %d sensors\n", 
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
#else

	// If the string "sensor" is in the topic, we can assume it's sensor data coming in
	if (strstr(topic, "sensor") != NULL)
	{
		// Parse the string into a Json structure
		json json_data = json::parse(messageTemp);
		// Convert json to struct
		MQTT_Payload payload = json_data.get<MQTT_Payload>();

		mqtt_dirty = false;

		if (mqtt_topic_payloads.find(payload.owner) == mqtt_topic_payloads.end())
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

		Serial.print("MQTT: connection failed, rc=");
		Serial.print(mqtt_client.state());
		if (retry_attemps >= 0)
			Serial.printf(" try again (attempt %d/3) in %d seconds\n", (4 - retry_attemps), (retry_time / 1000));
		else
			Serial.println("\nMQTT: No more attempts to cpnect to MQTT!");

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
