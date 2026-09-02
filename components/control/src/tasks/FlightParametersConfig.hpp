#pragma once

#include <cstdint>
#include "config.h"

// FSM timing and recovery policy are owned by the flight-state implementation.
struct FlightParametersConfig {
    float liftoff_accel_threshold_mps2;
    uint32_t liftoff_timeout_ms;
    uint32_t apogee_lockout_ms;
    uint32_t drogue_apogee_timeout_ms;
    float main_altitude_threshold_m;
    uint32_t launch_to_ballistic_threshold_ms;
    uint32_t launch_to_apogee_threshold_ms;
    float touchdown_altitude_threshold_m;
};

struct CalibrationConfig {
    uint32_t timeout_ms;
    uint32_t barometer_samples;
    uint32_t temperature_samples;
};

struct RecoveryConfig {
    RecoveryMode mode;
};
