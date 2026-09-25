#pragma once

#include "AirbrakesAero.hpp"
#include "ApogeePredictor.hpp"

#include <cstdint>

namespace airbrakes
{
    /**
     * @brief Which control law drives the airbrakes.
     *
     * Both predictive laws come from GNC and share the same apogee predictor;
     * they differ only in how they turn a prediction into a drag request. The
     * mode is a RuntimeConfig field so the two can be compared in HIL without
     * reflashing.
     */
    enum class ControlMode : uint8_t
    {
        /**
         * Altitude-triggered open/close, with no apogee prediction. This is the
         * pre-existing firmware behaviour, kept as a fallback for bench tests
         * and for flights where the predictive inputs are not trusted.
         */
        AltitudeBangBang = 0,

        /**
         * GNC controller v1. Sweeps a discrete grid of drag coefficients and
         * commands the first one whose predicted apogee falls below target.
         * Open loop in the drag axis: no integrator, no memory.
         */
        PredictiveCdSearch = 1,

        /**
         * GNC controller v2. A PI law on predicted apogee error drives a
         * normalized brake demand, which is mapped onto the drag range. The
         * predictor is fed the drag the vehicle currently has, closing the loop.
         */
        PredictivePid = 2,
    };

    /**
     * @brief Number of drag steps swept by ControlMode::PredictiveCdSearch.
     *
     * The GNC reference sweeps `Cd_clean + c*0.025` for c in 0..12, which is
     * exactly 12 steps across the 0.30 to 0.60 drag range. Deriving the step
     * from the configured range instead of hard-coding 0.025 keeps the sweep
     * spanning the whole envelope if the range is ever retuned.
     */
    inline constexpr int kCdSearchSteps = 12;

    /**
     * @brief Tuning and vehicle constants, copied from the locked flight snapshot.
     */
    struct ControllerParams
    {
        ControlMode mode;
        /** Apogee the controller aims for, m AGL. */
        float target_apogee_m;
        /** Altitude below which the brakes stay shut, m AGL ("openingAlt"). */
        float opening_altitude_m;
        /** Altitude above which the brakes are shut again, m AGL. */
        float close_altitude_m;
        /** Aerodynamic reference area, m^2. */
        float reference_area_m2;
        /** Burnout mass, kg. */
        float dry_mass_kg;
        /** Drag coefficient with the brakes fully retracted. */
        float cd_clean;
        /** Drag coefficient with the brakes fully extended. */
        float cd_max;
        /** Proportional gain on apogee error, 1/m. */
        float kp;
        /** Integral gain on apogee error, 1/(m*s). */
        float ki;
        /** Servo extension rate, in normalized travel per second. */
        float open_rate_per_s;
        /** Servo retraction rate, in normalized travel per second. */
        float close_rate_per_s;
    };

    /**
     * @brief One cycle of flight state.
     */
    struct ControllerInputs
    {
        float altitude_m;
        float vertical_velocity_mps;
        float air_density_kgm3;
        float mach;
        /** Elapsed time since the previous update, seconds. */
        float dt_s;
    };

    /**
     * @brief One cycle of controller output, including its internals.
     *
     * The prediction and error terms are returned, not just the servo command,
     * because they are what makes an airbrake flight log reviewable: a command
     * alone cannot be told apart from a command based on a bad prediction.
     */
    struct ControllerOutputs
    {
        /** Normalized servo position reached this cycle, 0 shut to 1 fully open. */
        float commanded_deployment;
        /** The same position as a percentage of full travel. */
        float opening_pct;
        /** Servo detent the brakes are travelling toward, in percent. */
        float target_opening_pct;
        /** Drag coefficient the control law asked for. */
        float requested_cd;
        /** Drag coefficient the position reached this cycle actually produces. */
        float achieved_cd;
        float predicted_apogee_m;
        float time_to_apogee_s;
        /** Predicted apogee minus target; positive means overshooting. */
        float apogee_error_m;
        /** Integrator state, in m*s. */
        float integrator;
        /** True while the control law is allowed to open the brakes. */
        bool armed;
    };

    /**
     * @brief Predictive airbrake control law.
     *
     * The loop each cycle is:
     *
     *  1. Read back the drag the vehicle currently has, from the achieved servo
     *     position and the current Mach.
     *  2. Predict apogee at that drag, held constant to apogee.
     *  3. Turn the prediction into a requested drag coefficient (PI law, or a
     *     sweep over the drag grid).
     *  4. Invert the aero table to get an opening, and snap it to a servo detent.
     *  5. Slew-limit toward that detent at the servo's real rate.
     *
     * Step 1 uses the achieved position rather than the previous request, so the
     * prediction stays honest while the servo is still travelling. Step 5's
     * output is the position the vehicle actually reaches, and is what feeds
     * step 1 on the next cycle.
     *
     * The class holds no hardware or RTOS state, so it can be exercised
     * standalone against recorded or simulated flights.
     */
    class AirbrakesController
    {
    public:
        /**
         * @brief Install tuning constants and clear all controller state.
         *
         * Call once before the first update(). Parameters are not re-read
         * afterwards, matching the locked-flight-snapshot rule.
         */
        void configure(const ControllerParams &params);

        /**
         * @brief Clear integrator and servo state without touching the tuning.
         */
        void reset();

        /**
         * @brief Advance the control law by one cycle.
         */
        ControllerOutputs update(const ControllerInputs &inputs);

        /** @brief Last commanded normalized deployment. */
        float deployment() const { return _deployment; }

    private:
        /** @brief Drag request from the PI law on predicted apogee error. */
        float updatePid(float predictedApogeeM, float dtSeconds);

        /** @brief Drag request from the discrete sweep of controller v1. */
        float searchCd(const ControllerInputs &inputs) const;

        ControllerParams _params{};
        float _integrator = 0.0f;
        float _deployment = 0.0f;
        /** Detent currently commanded, held across cycles for the hysteresis. */
        float _targetOpeningPct = 0.0f;
    };
}
