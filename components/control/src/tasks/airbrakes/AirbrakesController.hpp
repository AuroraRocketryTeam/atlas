#pragma once

#include <cstddef>
#include <cstdint>

/**
 * @brief Predictive airbrake control law, ported from the GNC "Airbrakes.m" code.
 *
 * Each cycle the law:
 *
 *  1. Stays closed below the opening altitude or when not climbing.
 *  2. For every opening breakpoint of the aero table, from closed upward,
 *     integrates the unpowered vertical coast to apogee with that opening
 *     held, using Cd(Mach) from that table row and ISA density and speed of
 *     sound re-evaluated at each step's altitude.
 *  3. Picks the first opening whose predicted apogee is at or below target and
 *     interpolates linearly against the previous one so the predicted apogee
 *     lands on the target. If even the closed brakes undershoot it stays
 *     closed; if even full opening overshoots it opens fully.
 *
 * Holds the per-flight constants; each call to computeCommand() is otherwise
 * stateless.
 */
class AirbrakesController
{
public:
    static constexpr size_t kMaxOpeningBreakpoints = 8;
    static constexpr size_t kMaxMachBreakpoints = 8;

    /**
     * @brief Airbrake drag map Cd(opening, Mach), sized at runtime.
     *
     * Loaded from airbrakes_cd_table.csv (only the first "opening_count" x "mach_count"
     * entries are used.
     */
    struct AeroTable
    {
        uint8_t opening_count;
        uint8_t mach_count;
        /** Opening breakpoints, percent of full travel, strictly increasing. */
        float opening_bp_pct[kMaxOpeningBreakpoints];
        /** Mach breakpoints, strictly increasing. */
        float mach_bp[kMaxMachBreakpoints];
        /** Drag coefficient indexed [opening][Mach] (rows = opening). */
        float cd[kMaxOpeningBreakpoints][kMaxMachBreakpoints];
    };

    /**
     * @brief Constants for one flight, copied from the locked flight snapshot.
     */
    struct Params
    {
        /** Altitude below which the brakes stay closed, m AGL ("openingAlt") */
        float opening_altitude_m;
        /** Apogee the controller aims for, m AGL ("targetAlt") */
        float target_apogee_m;
        /** Launch-site elevation, m MSL. Added to AGL altitude for the ISA model. */
        float launch_site_altitude_m;
        /** Burnout mass, kg ("dry_mass") */
        float dry_mass_kg;
        /** Aerodynamic reference area, m^2 ("Aref") */
        float reference_area_m2;
        /** Prediction integration step, s ("dt") */
        float prediction_dt_s;
        /** Maximum prediction steps per candidate ("Nmax") */
        uint32_t prediction_max_steps;
        /** Cd table ("Cd_table", "Opening_bp", "Mach_bp") */
        const AeroTable *aero;
    };

    /**
     * @brief Flight state for one cycle.
     */
    struct Inputs
    {
        /** Current altitude, m AGL */
        float altitude_agl_m;
        /** Current vertical velocity, m/s */
        float vertical_velocity_mps;
    };

    /**
     * @brief One cycle of controller output, including the prediction
     */
    struct Outputs
    {
        /** False when the law is holding the brakes closed by rule */
        bool armed;
        /** Commanded opening, percent of full travel ("cmdOpening") */
        float opening_pct;
        /** Cd of the commanded opening at the current Mach ("Cd_cmd") */
        float cd_cmd;
        /** Predicted apogee with the commanded opening, m AGL ("apogeeAlt") */
        float predicted_apogee_m;
        /** Predicted time from now to apogee, s ("t_apogee" - currentTime) */
        float time_to_apogee_s;
        /** Current Mach number */
        float mach;
        /** Apogee predictions run this cycle, for cost monitoring */
        uint8_t candidates_evaluated;
    };

    AirbrakesController() = default;
    explicit AirbrakesController(const Params &params) : _params(params) {}

    /** @brief Replace the per-flight constants. */
    void setParams(const Params &params) { _params = params; }

    /** @brief Current per-flight constants. */
    const Params &params() const { return _params; }

    /**
     * @brief Simplified ISA troposphere.
     *
     * @param altitudeMslM Geometric altitude above mean sea level, clamped to [0, 11000] m.
     * @param densityKgm3 Receives density, kg/m^3.
     * @param speedOfSoundMps Receives speed of sound, m/s.
     */
    static void isaAtmosphere(float altitudeMslM, float &densityKgm3, float &speedOfSoundMps);

    /**
     * @brief Predict apogee for one opening (one Cd(Mach) table row).
     *
     * Requires a table ("params().aero != nullptr").
     *
     * @param openingIndex Row of the aero table to hold for the whole coast.
     * @param altitudeAglM Starting altitude, m AGL.
     * @param verticalVelocityMps Starting vertical velocity, m/s.
     * @param apogeeAglM Receives the predicted apogee, m AGL.
     * @param timeToApogeeS Receives the predicted time to apogee, s.
     */
    void predictApogee(size_t openingIndex,
                       float altitudeAglM,
                       float verticalVelocityMps,
                       float &apogeeAglM,
                       float &timeToApogeeS) const;

    /**
     * @brief Run one cycle of the predictive law.
     *
     * With no table ("params().aero == nullptr") the law stays disarmed.
     */
    Outputs computeCommand(const Inputs &inputs) const;

private:
    /**
     * @brief Find which pair of breakpoints @p value falls between.
     *
     * @p value is first clamped to the table range. Returns the index of the
     * lower breakpoint and sets @p fraction to how far @p value is towards the
     * next one (0 to 1). Same search as in Airbrakes.m.
     */
    static size_t findSegment(const float *breakpoints, size_t count, float value, float &fraction);

    /** Cd along Mach for one opening row (`interpCd`) */
    static float cdAlongMach(const AeroTable &aero, size_t openingIndex, float mach);

    /** Cd at an arbitrary opening for the current Mach (`cdAtMach` + `openingToCd`). */
    static float cdAtOpening(const AeroTable &aero, float openingPct, float mach);

    Params _params{};
};
