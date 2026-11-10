#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>

struct AirbrakesData {
    // controller input
    float altitude_m = 0.0f;            // m AGL
    float vertical_velocity_mps = 0.0f;
    float mach = 0.0f;
  
    // controller output
    float deployment = 0.0f;            // command sent to RocketModel, 0..1
    float cd_cmd = 0.0f;                // Cd of the commanded opening at the current Mach
    float predicted_apogee_m = 0.0f;    // m AGL
    float time_to_apogee_s = 0.0f;
    uint32_t compute_us = 0;            // control-law execution time
    uint8_t candidates = 0;             // apogee predictions run this cycle
    bool armed = false;                 // control law allowed to open the brakes
    bool velocity_fresh = true;         // vertical velocity updated recently
    uint32_t timestamp = 0;

    size_t serializeJson(char* buffer, size_t maxLength) const {
        return std::snprintf(buffer, maxLength,
            "{\"src\":\"AIRBRAKES\",\"t\":%lu,\"alt_m\":%.6g,\"vz_mps\":%.6g,\"mach\":%.4g,"
            "\"cmd\":%.4g,\"cd\":%.4g,\"apo_m\":%.6g,\"tta_s\":%.4g,"
            "\"cpu_us\":%lu,\"cand\":%u,\"armed\":%s,\"vz_fresh\":%s}\n",
            static_cast<unsigned long>(timestamp), altitude_m, vertical_velocity_mps, mach,
            deployment, cd_cmd, predicted_apogee_m, time_to_apogee_s,
            static_cast<unsigned long>(compute_us), static_cast<unsigned>(candidates),
            armed ? "true" : "false", velocity_fresh ? "true" : "false");
    }
};
