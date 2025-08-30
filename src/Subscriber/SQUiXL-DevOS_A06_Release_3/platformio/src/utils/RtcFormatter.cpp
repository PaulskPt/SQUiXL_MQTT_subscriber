/* 
    Added 2025-08-30 by PaulskPt, with help of MS Copilot 
    Needed in mqtt.cpp

*/
#include "RtcFormatter.h"
#include <cstdio>
#include <time.h>
#include "squixl.h" // for the utc_offset_seconds

namespace RtcFormatter {
  psram_string format_datetime(int timestamp, bool useIso8601) {

    time_t rawTime = static_cast<time_t>(timestamp);
    struct tm *timeInfo = localtime(&rawTime);
    char buffer[40];

    if (useIso8601) {
        strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%S", timeInfo);

        int utc_offset_seconds = settings.config.location.utc_offset * 3600;
        //extern int utc_offset_seconds;
        int offset = utc_offset_seconds;
        int hours = offset / 3600;
        int minutes = abs(offset % 3600) / 60;

        char tzOffset[7];
        snprintf(tzOffset, sizeof(tzOffset), "%+03d:%02d", hours, minutes);

        return psram_string(buffer) + psram_string(tzOffset);
    } else {
        strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M", timeInfo);
        return psram_string(buffer);
    }
  }
}



