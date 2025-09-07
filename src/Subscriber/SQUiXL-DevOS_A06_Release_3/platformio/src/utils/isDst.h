// isDst.h
/* 
   Note @PaulskPt 2025-09-06: the file EU_LISBON_dst_table.json contains
   dst_star and dst_end time_t (unixTime) values for years 2025 - 2029 (5 years)
*/
#ifndef ISDST_H
#define ISDST_H

#include <Arduino.h>
#include "settings/settingsOption.h"

#ifdef USE_DST

#define REGION_EUROPE
// #define REGION_USA

struct DSTInfo {
  bool is_dst;
  int utc_offset;
  time_t dst_start;
  time_t dst_end;
  String tz_abbr;
};

DSTInfo getDSTInfo(time_t unixTime);

#ifdef SHOW_FILES_OF_FILESYSTEM
void list_files();
void upload_dst_file();
void printDSTFileContents();
#endif

#endif

#endif