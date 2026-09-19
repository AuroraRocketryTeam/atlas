#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

// One processed barometer sample as seen by the altitude/apogee pipeline.
struct AltitudeData {
    float filtered_pressure_pa = 0.0f;
    float altitude_m = 0.0f;
    float ols_slope_mps = 0.0f;
    bool ols_ready = false;
    bool rising = true;
    uint32_t timestamp = 0;

    size_t serializeJson(char* buffer, size_t maxLength) const {
        return snprintf(buffer, maxLength,
            "{\"source\":\"ALTITUDE\",\"sensorData\":{"
            "\"filtered_pressure_pa\":%f,\"altitude_m\":%f,\"ols_slope_mps\":%f,"
            "\"ols_ready\":%s,\"rising\":%s,\"timestamp\":%lu}}\n",
            filtered_pressure_pa, altitude_m, ols_slope_mps,
            ols_ready ? "true" : "false", rising ? "true" : "false",
            static_cast<unsigned long>(timestamp));
    }
};
