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
};

static_assert(sizeof(fc_command_packet_t) == 11, "fc_command_packet_t size mismatch");


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