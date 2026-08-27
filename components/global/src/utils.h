#pragma once

#include "config.h"

#include <cstdint>
#include <string>

class Utils {
public:
    static uint32_t millis();
    static uint32_t realMillis();
    static void setSimMillis(uint32_t t);
    static int readLine(char* buf, int maxLen);

    // string helper functions
    static void toUpperString(std::string& s);
    static void trimString(std::string& s);

private:
    static volatile uint32_t _simMillis;
};
