#pragma once

#include "esp_timer.h"

class Utils {
public:
    static uint32_t millis() {
        return esp_timer_get_time() / 1000;
    }

    static int readLine(char* buf, int maxLen);
};
