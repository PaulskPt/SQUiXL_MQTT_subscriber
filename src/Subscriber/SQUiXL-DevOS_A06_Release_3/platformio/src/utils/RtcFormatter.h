/* 
    Added 2025-08-30 by PaulskPt, with help of MS Copilot 
    Needed in mqtt.cpp

*/
#ifndef RTC_FORMATTER_H
#define RTC_FORMATTER_H

#include "squixl.h"   // for psram_string
#include <time.h>     // for struct tm

namespace RtcFormatter {
  psram_string format_datetime(int timestamp, bool useIso8601);

}

#endif  // RTC_FORMATTER_H

