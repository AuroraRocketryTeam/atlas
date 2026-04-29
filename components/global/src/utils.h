#pragma once

#include <string>

class Utils {
public:
    static uint32_t millis();

    static int readLine(char* buf, int maxLen);

    // string helper functions
    static void toUpperString(std::string& s);
    static void trimString(std::string& s);
};
