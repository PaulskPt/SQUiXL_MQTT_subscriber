#pragma once
#include <string>

struct METAR {
    std::string section1;
    std::string section2;
    std::string section3;
};

extern METAR metar_data;
