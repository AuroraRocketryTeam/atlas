#pragma once

#include <cstdint>
#include "FlightState.hpp"

/**
 * @brief Packed wire format (TCP payload)
 */
struct __attribute__((packed)) fc_command_packet_t {
    float sim_time;
    uint8_t open_main;
    uint8_t open_drogue;
    float airbrakes_deployment;
    uint8_t fsm_state;

    // Realtime data used to plot a comparison against its noise-free truth.
    float est_filtered_pressure_pa;
    float est_altitude_m;
    float est_vertical_velocity_mps;
    uint32_t est_timestamp_ms;

    // Airbrake controller cycle (AirbrakesTelemetry)
    uint32_t ab_cycle;
    uint32_t ab_timestamp_ms;
    uint32_t ab_velocity_timestamp_ms;
    uint8_t ab_flags;                     // bit0 armed, bit1 velocity fresh
    float ab_altitude_m;
    float ab_vertical_velocity_mps;
    float ab_deployment;                  // 0..1
    float ab_predicted_apogee_m;
    float ab_apogee_closed_m;             // apogee predicted with the brakes held closed
    uint32_t ab_compute_us;
    // Per-flight controller parameters
    float ab_target_apogee_m;
    float ab_opening_altitude_m;
    float ab_dry_mass_kg;
    float ab_reference_area_m2;
    float ab_launch_site_altitude_m;
    float ab_prediction_dt_s;
    uint32_t ab_prediction_max_steps;
};

// Keep in sync with COMMAND_FMT in main/hil/hil_communication.py and mock_manny_fc_server.py.
static_assert(sizeof(fc_command_packet_t) == 92, "fc_command_packet_t size mismatch");


/**
 * @brief High-level command used inside firmware
 */
class Command {
public:

    /* ===================== SETTERS ===================== */

    void setMain(bool open) {
        _open_main = open;
    }

    void setDrogue(bool open) {
        _open_drogue = open;
    }

    void setAirbrakes(float deployment) {
        _airbrakes = deployment;
    }

    void reset() {
        _open_main = false;
        _open_drogue = false;
        _airbrakes = 0.0f;
    }

    /* ===================== GETTERS ===================== */

    bool getMain() const { return _open_main; }
    bool getDrogue() const { return _open_drogue; }
    float getAirbrakes() const { return _airbrakes; }

    /* ===================== SERIALIZATION ===================== */

    fc_command_packet_t serialize(float sim_time, RocketState fsm_state) const {
        fc_command_packet_t w{};
        w.sim_time = sim_time;
        w.open_main = _open_main ? 1u : 0u;
        w.open_drogue = _open_drogue ? 1u : 0u;
        w.airbrakes_deployment = _airbrakes;
        w.fsm_state = static_cast<uint8_t>(fsm_state); // from enum to uint8_t
        return w;
    }

private:

    bool _open_main = false;
    bool _open_drogue = false;
    float _airbrakes = 0.0f;
};