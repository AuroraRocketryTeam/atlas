#pragma once

#include "AirbrakesAero.hpp"

#include <cmath>

namespace airbrakes
{
    /**
     * @brief Result of a coast-phase apogee prediction.
     */
    struct ApogeePrediction
    {
        /** Predicted apogee altitude, in the same reference as the input altitude. */
        float apogee_altitude_m;
        /** Time from now until apogee, in seconds. */
        float time_to_apogee_s;
    };

    /**
     * @brief Drag parameter $k$ of the coast dynamics, in 1/m.
     *
     * $$ k = \frac{\rho\,C_d\,A_{ref}}{2m} $$
     *
     * Grouping the four constants makes the prediction a one-parameter problem,
     * which is what lets the closed form below exist.
     */
    inline float dragParameter(float cd, float densityKgm3, float refAreaM2, float massKg)
    {
        if (massKg <= 0.0f) return 0.0f;
        const float k = 0.5f * cd * densityKgm3 * refAreaM2 / massKg;
        return (std::isfinite(k) && k > 0.0f) ? k : 0.0f;
    }

    /**
     * @brief Predict apogee for an unpowered, drag-limited vertical coast.
     *
     * The GNC reference implementation integrates
     *
     * $$ \dot v = -\frac{1}{2m}\rho C_d A_{ref} v^2 - g $$
     *
     * with forward Euler at 10 ms for up to 3000 steps, and stops on the first
     * sign change of $v$. Version 1 of the controller does that once per
     * candidate $C_d$, so a single control cycle costs up to 39000 integration
     * steps.
     *
     * That ODE has a closed-form solution, so none of those steps are needed.
     * With $k$ as above and $v$ positive upward, separating variables gives the
     * time to apogee
     *
     * $$ t = \frac{1}{\sqrt{gk}} \arctan\!\left(v\sqrt{\frac{k}{g}}\right) $$
     *
     * and, using $v\,\mathrm{d}v/\mathrm{d}h = \dot v$, the altitude gained
     *
     * $$ \Delta h = \frac{1}{2k}\ln\!\left(1 + \frac{k v^2}{g}\right). $$
     *
     * This is one evaluation instead of thousands, and it is also *more*
     * accurate than the reference: forward Euler accumulates truncation error
     * over the coast, while these expressions are exact for the same dynamics.
     *
     * Like the reference, this holds $\rho$ constant over the coast. That is the
     * dominant modelling approximation, and it makes the prediction slightly
     * conservative (real density drops with altitude, so real drag is lower).
     *
     * @param altitudeM Current altitude.
     * @param verticalVelocityMps Current vertical velocity, positive upward.
     * @param dragParameterPerM The $k$ from dragParameter().
     * @return Predicted apogee altitude and time to reach it.
     */
    inline ApogeePrediction predictApogee(float altitudeM,
                                          float verticalVelocityMps,
                                          float dragParameterPerM)
    {
        // Already descending: apogee is now. Matches the reference loop, which
        // exits on the first negative velocity sample.
        if (!(verticalVelocityMps > 0.0f) || !std::isfinite(verticalVelocityMps))
        {
            return {altitudeM, 0.0f};
        }

        const float v = verticalVelocityMps;
        const float g = kGravityMps2;
        const float k = dragParameterPerM;

        // k*v^2/g is the ratio of drag deceleration to gravity at the current
        // speed. Below this threshold the logarithm and arctangent are within
        // float precision of their drag-free limits, and evaluating 1/(2k)
        // directly would divide by an almost-zero.
        const float dragToWeightRatio = k * v * v / g;
        if (!std::isfinite(dragToWeightRatio) || dragToWeightRatio < 1e-6f)
        {
            return {altitudeM + (v * v) / (2.0f * g), v / g};
        }

        const float altitudeGainM = std::log1p(dragToWeightRatio) / (2.0f * k);
        const float timeToApogeeS = std::atan(std::sqrt(dragToWeightRatio)) / std::sqrt(k * g);

        return {altitudeM + altitudeGainM, timeToApogeeS};
    }

    /**
     * @brief Convenience overload taking the physical parameters directly.
     */
    inline ApogeePrediction predictApogee(float altitudeM,
                                          float verticalVelocityMps,
                                          float cd,
                                          float densityKgm3,
                                          float refAreaM2,
                                          float massKg)
    {
        return predictApogee(altitudeM,
                             verticalVelocityMps,
                             dragParameter(cd, densityKgm3, refAreaM2, massKg));
    }
}
