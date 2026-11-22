#pragma once

#include <cmath>
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
    float apogee_closed_m = NAN;        // m AGL predicted with brakes held closed
    float time_to_apogee_s = 0.0f;
    uint32_t compute_us = 0;            // control-law execution time
    uint8_t candidates = 0;             // apogee predictions run this cycle
    bool armed = false;                 // control law allowed to open the brakes
    bool velocity_fresh = true;         // vertical velocity updated recently
    uint32_t timestamp = 0;

    size_t serializeJson(char* buffer, size_t maxLength) const {
        char apogeeClosed[24];
        if (std::isfinite(apogee_closed_m)) {
            std::snprintf(apogeeClosed, sizeof(apogeeClosed), "%.6g", apogee_closed_m);
        } else {
            std::snprintf(apogeeClosed, sizeof(apogeeClosed), "null");
        }
        return std::snprintf(buffer, maxLength,
            "{\"src\":\"AIRBRAKES\",\"t\":%lu,\"alt_m\":%.6g,\"vz_mps\":%.6g,\"mach\":%.4g,"
            "\"cmd\":%.4g,\"cd\":%.4g,\"apo_m\":%.6g,\"apo0_m\":%s,\"tta_s\":%.4g,"
            "\"cpu_us\":%lu,\"cand\":%u,\"armed\":%s,\"vz_fresh\":%s}\n",
            static_cast<unsigned long>(timestamp), altitude_m, vertical_velocity_mps, mach,
            deployment, cd_cmd, predicted_apogee_m, apogeeClosed, time_to_apogee_s,
            static_cast<unsigned long>(compute_us), static_cast<unsigned>(candidates),
            armed ? "true" : "false", velocity_fresh ? "true" : "false");
    }
};

/**
 * @brief Latest airbrake control cycle plus the parameters it ran with.
 *
 * AirbrakesTask publishes one every cycle through RocketModel so the HIL task
 * can report the controller to the simulator for comparison against the
 * simulated truth (not logged to flash).
 */
struct AirbrakesTelemetry {
    /** Incremented by every published cycle (0 = the control law has not run yet) */
    uint32_t cycle = 0;
    uint32_t velocity_timestamp_ms = 0;
    AirbrakesData record;

    // Controller parameters
    float target_apogee_m = 0.0f;
    float opening_altitude_m = 0.0f;
    float dry_mass_kg = 0.0f;
    float reference_area_m2 = 0.0f;
    float launch_site_altitude_m = 0.0f;
    float prediction_dt_s = 0.0f;
    uint32_t prediction_max_steps = 0;
};
