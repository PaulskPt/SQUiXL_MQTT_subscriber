#include "isDst.h"
#include "settings/settingsOption.h"
#include <Arduino.h>
#ifdef USE_DST
#include "squixl.h"
#include "utils/json_psram.h"
#include "utils/json_conversions.h"
#include "utils/json.h"    // in stead of ArduinoJson (because there was already a Json module in this project!)
#include <FS.h>
#include <LittleFS.h>
#include <time.h>

//#include <nlohmann/json.hpp>


using json = nlohmann::json;

DSTInfo getDSTInfo(time_t unixTime) {

static constexpr const char txt0[] PROGMEM = "getDSTInfo(): ";

#ifdef MY_DEBUG
  Serial.print(F("getDSTInfo(): parameter unixTime = "));
  Serial.println(unixTime);
#endif

  DSTInfo info = {false, 0, 0, 0, ""};

  if (unixTime == 0) {
    Serial.println(F("Invalid time — skipping DST check"));
    return info;
  }

  struct tm* timeinfo = gmtime(&unixTime);
  if (!timeinfo) {
    Serial.println(F("Failed to convert time"));
    return info;
  }

#ifdef SHOW_FILES_OF_FILESYSTEM
  // Open and read the JSON file
  list_files(); // Show the files present in the filesystem
  upload_dst_file();
  printDSTFileContents();
  delay(2000);
#endif

#ifdef REGION_EUROPE
  psram_string fn = "/EU_LISBON_dst_table.json";
#endif

#ifdef REGION_USA
  psram_string fn = "/US_NEW_YORK_dst_table.json";
#endif

  File file = LittleFS.open(fn.c_str(), "r");
  if (!file) {
    Serial.print(txt0);
    Serial.println(F("Failed to open DST file"));
    return info;
  }

  int year = timeinfo->tm_year + 1900;

#ifdef MY_DEBUG
  Serial.print(F("year (from timeinfo->tm_year + 1900) = "));
  Serial.println(year);
#endif

  std::string yearKey = std::to_string(year);
  std::string jsonContent;
  while (file.available()) {
    jsonContent += static_cast<char>(file.read());
  }
  file.close();

  // Parse JSON
  json dstData;
  try {
    dstData = json::parse(jsonContent);
  } catch (const json::parse_error& e) {
    Serial.print(txt0);
    Serial.print(F("JSON parse error: "));
    Serial.println(e.what());
    return info;
  }

  // Navigate to Europe/Lisbon region
  const std::string regionRootKey = "region";
  const std::string regionKey = "Europe/Lisbon";

  if (!dstData.contains(regionRootKey) || !dstData[regionRootKey].contains(regionKey)) {
    Serial.print(txt0);
    Serial.print(F("Region "));
    Serial.print(regionKey.c_str());
    Serial.println(F(" not found"));
    return info;
  }

  const json& lisbon = dstData[regionRootKey][regionKey];

  // Get DST start/end for the year
  bool stop = false;
  Serial.print(txt0);
  Serial.print(F("year "));
  Serial.print(yearKey.c_str());
  if (!lisbon["dst_start_end"].contains(yearKey)) {
    stop = true;
    Serial.print(F(" not"));
  }
  Serial.print(F(" found in DST start/end table for country/city: "));
  Serial.printf("%s/%s\n", settings.config.location.country.c_str(), settings.config.location.city.c_str());
  if (stop)
    return info;

  const auto& dstRange = lisbon["dst_start_end"][yearKey];
  time_t dstStart = dstRange[0];
  time_t dstEnd = dstRange[1];

  // Determine if current time is within DST range
  bool isDST = unixTime >= dstStart && unixTime < dstEnd;

  // Get timezone info
  const json& tzInfo = isDST ? lisbon["dst"] : lisbon["std"];
  int offset = tzInfo.value("utc_offset_dst", tzInfo.value("utc_offset_std", 0));
  String abbr = String(tzInfo.value("tz_abbr_dst", tzInfo.value("tz_abbr_std", "")).c_str());

  // Populate result
  info.is_dst = isDST;
  info.dst_start = dstStart;
  info.dst_end = dstEnd;
  info.utc_offset = offset;
  info.tz_abbr = abbr;

  return info;
}

#ifdef  SHOW_FILES_OF_FILESYSTEM

void list_files() {
  Serial.println("Listing LittleFS files:");
  File root = LittleFS.open("/");
  if (root && root.isDirectory()) {
    File file = root.openNextFile();
    while (file) {
      Serial.print("  ");
      Serial.println(file.name());
      file = root.openNextFile();
    }
  }
}

void upload_dst_file() {
  if (!LittleFS.exists("/EU_LISBON_dst_table.json")) {
    Serial.println(F("DST file missing — creating default..."));
    File file = LittleFS.open("/EU_LISBON_dst_table.json", FILE_WRITE);
    if (file) {
      file.print("{\"region\" : {\"Europe/Lisbon\": {\"dst_start_end\" : {\"2025\": [1741503600, 1762063200],\"2026\": [1772953200, 1793512800],\"2027\": [1805007600, 1825567200],\"2028\": [1836457200, 1857016800],\"2029\": [1867906800, 1888466400]},\"dst\": {\"utc_offset_dst\": 1,\"tz_abbr_dst\": \"WEST\"},\"std\": {\"utc_offset_std\": 0,\"tz_abbr_std\": \"WET\"}}}}");
      file.close();
      Serial.println(F("DST file created successfully."));
    } else {
      Serial.println(F("Failed to create DST file."));
    }
  }
}

#include <LittleFS.h>

void printDSTFileContents() {
  const char* filename = "/EU_LISBON_dst_table.json";

  if (!LittleFS.exists(filename)) {
    Serial.println(F("DST file not found."));
    return;
  }

  File file = LittleFS.open(filename, "r");
  if (!file) {
    Serial.println(F("Failed to open DST file."));
    return;
  }

  Serial.println(F("DST file contents:"));
  while (file.available()) {
    Serial.write(file.read());  // Print raw content byte-by-byte
  }
  file.close();
  Serial.println();  // Add newline after file output
}

#endif //  SHOW_FILES_OF_FILESYSTEM

#endif