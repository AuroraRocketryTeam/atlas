#pragma once

#include "esp_timer.h"
#include <Logger.hpp>

enum class TimeSource {
    REAL,
    SIMULATION
};
constexpr char* timeSourceToString(TimeSource src)
{
    switch (src)
    {
        case TimeSource::REAL:       return "REAL";
        case TimeSource::SIMULATION: return "SIMULATION";
        default:                     return "UNKNOWN";
    }
}

class Utils {
public:
    static uint32_t millis();
    static void setSimMillis(uint32_t t);
    static void setTimeSource(TimeSource src);
    static int readLine(char* buf, int maxLen);

private:
    static volatile uint32_t _simMillis;
    static volatile TimeSource _source;
};

