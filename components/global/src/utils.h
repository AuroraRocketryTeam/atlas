#pragma once

#include "esp_timer.h"

class Utils {
public:
    static uint32_t millis();
    static uint32_t realMillis();
    static void setSimMillis(uint32_t t);
    static int readLine(char* buf, int maxLen);

private:
    static volatile uint32_t _simMillis;
};