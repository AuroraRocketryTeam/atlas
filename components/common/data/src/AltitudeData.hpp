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
        return std::snprintf(buffer, maxLength,
            "{\"src\":\"ALTITUDE\",\"t\":%lu,\"pf_pa\":%.8g,\"alt_m\":%.6g,\"vz_mps\":%.6g,"
            "\"ready\":%s,\"rising\":%s}\n",
            static_cast<unsigned long>(timestamp), filtered_pressure_pa, altitude_m, ols_slope_mps,
            ols_ready ? "true" : "false", rising ? "true" : "false");
    }
};
