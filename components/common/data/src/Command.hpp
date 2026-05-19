#pragma once

#include <cstdint>

/**
 * @brief Packed wire format (TCP payload)
 */
struct __attribute__((packed)) FcCommandWire {
    float sim_time;
    uint8_t open_main;
    uint8_t open_drogue;
    float airbrakes_deployment;
};

static_assert(sizeof(FcCommandWire) == 10, "FcCommandWire size mismatch");


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

    FcCommandWire serialize(float sim_time) const {
        FcCommandWire w{};
        w.sim_time = sim_time;
        w.open_main = _open_main ? 1u : 0u;
        w.open_drogue = _open_drogue ? 1u : 0u;
        w.airbrakes_deployment = _airbrakes;
        return w;
    }

private:

    bool _open_main = false;
    bool _open_drogue = false;
    float _airbrakes = 0.0f;
};