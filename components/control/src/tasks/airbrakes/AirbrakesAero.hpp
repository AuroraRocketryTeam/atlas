#pragma once

#include <cmath>
#include <cstddef>

/**
 * @brief Atmosphere model and airbrake aerodynamic tables.
 *
 * Everything here is free-standing math: no FreeRTOS, no ESP-IDF, no model
 * access. That keeps the flight-critical aerodynamics reviewable on its own and
 * lets the GNC team cross-check it against the Simulink blocks it replaces.
 *
 * The two tables mirror the pair used in the Simulink airbrake model:
 *
 *  - `kCdTable`, the aero-provided map $C_d(\text{opening}, M)$. It answers
 *    "what drag do I actually have right now", which is what the apogee
 *    predictor needs.
 *  - `kOpeningTable`, the same map inverted to $\text{opening}(M, C_d)$. It
 *    answers "how far do I open to obtain the drag the controller asked for".
 *
 * Both are placeholders supplied by GNC and are expected to be replaced by the
 * real aero data. Keep the two consistent when they are: they are stored as two
 * tables rather than inverted numerically on the fly because inverting a 2-D
 * map at 50 Hz on the flight controller buys nothing over a second table.
 */
namespace airbrakes
{
    inline constexpr float kGravityMps2 = 9.80665f;
    /** Specific gas constant of dry air, J/(kg*K). */
    inline constexpr float kDryAirGasConstant = 287.0528f;
    /** Ratio of specific heats for air. */
    inline constexpr float kHeatCapacityRatio = 1.4f;
    inline constexpr float kZeroCelsiusInKelvin = 273.15f;

    /** Sea-level ISA reference values, used only by the density fallback. */
    inline constexpr float kIsaSeaLevelDensity = 1.225f;
    inline constexpr float kIsaTemperatureLapseRate = 0.0065f;   // K/m
    inline constexpr float kIsaSeaLevelTemperatureK = 288.15f;

    /* ===================== ATMOSPHERE ===================== */

    /**
     * @brief Air density from the barometer's own pressure and temperature.
     *
     * $$ \rho = \frac{p}{R_{air} T} $$
     *
     * Measuring both terms is better than an ISA table: it tracks the real
     * launch-day atmosphere, and drag scales linearly with the result.
     *
     * @param pressurePa Static pressure in Pa (the MS5611 reports Pa).
     * @param temperatureC Static temperature in degrees Celsius.
     * @return Density in kg/m^3, or 0 when the inputs are not usable.
     */
    inline float airDensity(float pressurePa, float temperatureC)
    {
        const float temperatureK = temperatureC + kZeroCelsiusInKelvin;
        if (!std::isfinite(pressurePa) || !std::isfinite(temperatureK) ||
            pressurePa <= 0.0f || temperatureK <= 1.0f)
        {
            return 0.0f;
        }
        return pressurePa / (kDryAirGasConstant * temperatureK);
    }

    /**
     * @brief ISA troposphere density, used when the barometer sample is unusable.
     *
     * Deliberately altitude-only so it still produces a sane number when the
     * temperature channel is the one that failed.
     */
    inline float isaDensity(float altitudeMsl)
    {
        if (!std::isfinite(altitudeMsl)) return kIsaSeaLevelDensity;
        const float temperatureRatio =
            1.0f - (kIsaTemperatureLapseRate * altitudeMsl / kIsaSeaLevelTemperatureK);
        if (temperatureRatio <= 0.0f) return 0.0f;
        // rho = rho0 * (T/T0)^(g/(L*R) - 1); the exponent is ~4.2559 for ISA air.
        return kIsaSeaLevelDensity * std::pow(temperatureRatio, 4.2558797f);
    }

    /**
     * @brief Speed of sound, $a = \sqrt{\gamma R T}$.
     *
     * @param temperatureC Static temperature in degrees Celsius.
     * @return Speed of sound in m/s, or 0 when the temperature is not usable.
     */
    inline float speedOfSound(float temperatureC)
    {
        const float temperatureK = temperatureC + kZeroCelsiusInKelvin;
        if (!std::isfinite(temperatureK) || temperatureK <= 1.0f) return 0.0f;
        return std::sqrt(kHeatCapacityRatio * kDryAirGasConstant * temperatureK);
    }

    /* ===================== BREAKPOINTS AND TABLES ===================== */

    /**
     * @brief Servo detents, in percent of full airbrake travel.
     *
     * The servo is commanded to one of these positions only; the controller's
     * continuous request is snapped to the nearest one. Denser detents track the
     * requested drag more closely, so this is the array to grow if the servo
     * turns out to resolve finer than 20%.
     */
    inline constexpr float kOpeningBreakpointsPct[] = {0.0f, 20.0f, 40.0f, 60.0f, 80.0f, 100.0f};

    /**
     * @brief Deadband applied when moving between detents, in percent of travel.
     *
     * Without it a request sitting near the midpoint between two detents flips
     * the commanded detent every cycle, and the servo dithers at the control
     * rate for the whole coast. A quarter of the detent spacing is enough to
     * stop that while still tracking a genuinely moving request.
     */
    inline constexpr float kOpeningHysteresisPct = 5.0f;
    inline constexpr size_t kOpeningBreakpointCount =
        sizeof(kOpeningBreakpointsPct) / sizeof(kOpeningBreakpointsPct[0]);

    inline constexpr float kMachBreakpoints[] = {0.0f, 0.2f, 0.4f, 0.6f, 0.8f};
    inline constexpr size_t kMachBreakpointCount =
        sizeof(kMachBreakpoints) / sizeof(kMachBreakpoints[0]);

    /** $C_d$ indexed [opening][Mach]. Rows follow kOpeningBreakpointsPct. */
    inline constexpr float kCdTable[kOpeningBreakpointCount][kMachBreakpointCount] = {
        {0.30f, 0.31f, 0.32f, 0.33f, 0.34f},
        {0.34f, 0.35f, 0.36f, 0.38f, 0.39f},
        {0.39f, 0.40f, 0.42f, 0.44f, 0.46f},
        {0.45f, 0.47f, 0.49f, 0.51f, 0.53f},
        {0.52f, 0.54f, 0.56f, 0.58f, 0.59f},
        {0.58f, 0.59f, 0.60f, 0.60f, 0.60f},
    };

    inline constexpr float kCdBreakpoints[] = {0.30f, 0.35f, 0.40f, 0.45f, 0.50f, 0.55f, 0.60f};
    inline constexpr size_t kCdBreakpointCount =
        sizeof(kCdBreakpoints) / sizeof(kCdBreakpoints[0]);

    /** Opening percentage indexed [Mach][Cd]. Rows follow kMachBreakpoints. */
    inline constexpr float kOpeningTable[kMachBreakpointCount][kCdBreakpointCount] = {
        {0.0f, 22.0f, 42.0f, 60.0f, 72.0f, 86.0f, 100.0f},
        {0.0f, 20.0f, 40.0f, 56.0f, 70.0f, 84.0f, 100.0f},
        {0.0f, 18.0f, 36.0f, 52.0f, 66.0f, 82.0f, 100.0f},
        {0.0f, 15.0f, 32.0f, 48.0f, 62.0f, 78.0f, 100.0f},
        {0.0f, 12.0f, 28.0f, 44.0f, 58.0f, 74.0f, 100.0f},
    };

    /** Lowest and highest drag the tables can produce, for controller clamping. */
    inline constexpr float kTableCdMin = kCdBreakpoints[0];
    inline constexpr float kTableCdMax = kCdBreakpoints[kCdBreakpointCount - 1];

    /* ===================== INTERPOLATION ===================== */

    /**
     * @brief Locate @p value in an ascending breakpoint array.
     *
     * Out-of-range inputs clamp to the end segments rather than extrapolating.
     * Extrapolating an aero table is how a controller ends up commanding drag
     * the airframe cannot produce, so the tables define the whole envelope.
     *
     * @param breakpoints Ascending breakpoints.
     * @param count Number of breakpoints, at least 2.
     * @param value Value to locate.
     * @param fraction Receives the position inside the segment, in [0, 1].
     * @return Index of the segment's lower breakpoint.
     */
    inline size_t findSegment(const float *breakpoints, size_t count, float value, float &fraction)
    {
        if (!std::isfinite(value) || value <= breakpoints[0])
        {
            fraction = 0.0f;
            return 0;
        }
        if (value >= breakpoints[count - 1])
        {
            fraction = 1.0f;
            return count - 2;
        }

        size_t lower = 0;
        while (lower + 2 < count && value >= breakpoints[lower + 1])
        {
            ++lower;
        }

        const float span = breakpoints[lower + 1] - breakpoints[lower];
        fraction = span > 0.0f ? (value - breakpoints[lower]) / span : 0.0f;
        return lower;
    }

    /**
     * @brief Bilinear interpolation over a clamped 2-D table.
     *
     * Equivalent to the Simulink 2-D lookup block with clipped extrapolation.
     */
    template <size_t RowCount, size_t ColCount>
    inline float interpolate2D(const float (&rowBreakpoints)[RowCount],
                               const float (&colBreakpoints)[ColCount],
                               const float (&table)[RowCount][ColCount],
                               float rowValue,
                               float colValue)
    {
        float rowFraction = 0.0f;
        float colFraction = 0.0f;
        const size_t row = findSegment(rowBreakpoints, RowCount, rowValue, rowFraction);
        const size_t col = findSegment(colBreakpoints, ColCount, colValue, colFraction);

        const float lowerEdge = table[row][col] +
                                colFraction * (table[row][col + 1] - table[row][col]);
        const float upperEdge = table[row + 1][col] +
                                colFraction * (table[row + 1][col + 1] - table[row + 1][col]);
        return lowerEdge + rowFraction * (upperEdge - lowerEdge);
    }

    /* ===================== AIRBRAKE MAPS ===================== */

    /**
     * @brief Drag coefficient produced by an airbrake opening at a given Mach.
     *
     * This is the forward direction of the aero table. The controller uses it on
     * the *achieved* servo position, so the apogee prediction is made with the
     * drag the vehicle actually has, not the drag that was requested.
     *
     * @param openingPct Airbrake opening, 0 to 100 percent.
     * @param mach Free-stream Mach number.
     */
    inline float cdForOpening(float openingPct, float mach)
    {
        return interpolate2D(kOpeningBreakpointsPct, kMachBreakpoints, kCdTable, openingPct, mach);
    }

    /**
     * @brief Airbrake opening needed to obtain a drag coefficient at a given Mach.
     *
     * This is the inverted table: the controller thinks in $C_d$, the servo
     * takes an opening.
     *
     * @param mach Free-stream Mach number.
     * @param cd Requested drag coefficient.
     * @return Continuous opening percentage, before snapping to a servo detent.
     */
    inline float openingForCd(float mach, float cd)
    {
        return interpolate2D(kMachBreakpoints, kCdBreakpoints, kOpeningTable, mach, cd);
    }

    /**
     * @brief Snap a continuous opening to the nearest reachable servo detent.
     *
     * Replaces the `approxOpening` MATLAB helper. The breakpoints are sorted and
     * evenly spaced in practice, but the search does not assume either.
     */
    inline float snapToOpeningBreakpoint(float openingPct)
    {
        if (!std::isfinite(openingPct)) return kOpeningBreakpointsPct[0];

        float best = kOpeningBreakpointsPct[0];
        float bestDistance = std::fabs(openingPct - best);
        for (size_t i = 1; i < kOpeningBreakpointCount; ++i)
        {
            const float distance = std::fabs(openingPct - kOpeningBreakpointsPct[i]);
            if (distance < bestDistance)
            {
                bestDistance = distance;
                best = kOpeningBreakpointsPct[i];
            }
        }
        return best;
    }

    /**
     * @brief Snap to a detent, but keep the current one unless the request has
     *        moved decisively toward another.
     *
     * @param openingPct Continuous opening the controller asked for.
     * @param heldPct Detent currently commanded.
     * @param hysteresisPct Extra margin the new detent must win by.
     * @return The detent to command this cycle.
     */
    inline float snapToOpeningBreakpoint(float openingPct, float heldPct, float hysteresisPct)
    {
        const float nearest = snapToOpeningBreakpoint(openingPct);
        if (nearest == heldPct || !std::isfinite(openingPct)) return nearest;

        return (std::fabs(openingPct - nearest) + hysteresisPct < std::fabs(openingPct - heldPct))
                   ? nearest
                   : heldPct;
    }
}
