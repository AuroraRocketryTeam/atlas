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
    // Accelerometer apogee detector at the same instant. `rising` above stays
    // the barometer OLS verdict; the FSM sees apogee when `rising` is false or
    // when accel_valid is true and accel_rising is false.
    float accel_velocity_mps = 0.0f;
    bool accel_valid = false;
    bool accel_rising = true;

    size_t serializeJson(char* buffer, size_t maxLength) const {
        return snprintf(buffer, maxLength,
            "{\"source\":\"ALTITUDE\",\"sensorData\":{"
            "\"filtered_pressure_pa\":%f,\"altitude_m\":%f,\"ols_slope_mps\":%f,"
            "\"ols_ready\":%s,\"rising\":%s,\"timestamp\":%lu,"
            "\"accel_velocity_mps\":%f,\"accel_valid\":%s,\"accel_rising\":%s}}\n",
            filtered_pressure_pa, altitude_m, ols_slope_mps,
            ols_ready ? "true" : "false", rising ? "true" : "false",
            static_cast<unsigned long>(timestamp),
            accel_velocity_mps, accel_valid ? "true" : "false",
            accel_rising ? "true" : "false");
    }
};
