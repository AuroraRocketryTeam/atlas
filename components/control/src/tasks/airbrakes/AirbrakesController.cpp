#include "AirbrakesController.hpp"

#include "config.h"

#include <algorithm>
#include <cmath>

size_t AirbrakesController::findSegment(const float *breakpoints, size_t count, float value, float &fraction)
{
    const float clamped = std::min(std::max(value, breakpoints[0]), breakpoints[count - 1]);
    size_t k = 0;
    while (k + 2 < count && clamped > breakpoints[k + 1])
    {
        ++k;
    }
    fraction = (clamped - breakpoints[k]) / (breakpoints[k + 1] - breakpoints[k]);
    return k;
}

float AirbrakesController::cdAlongMach(const AeroTable &aero, size_t openingIndex, float mach)
{
    float w = 0.0f;
    const size_t k = findSegment(aero.mach_bp, aero.mach_count, mach, w);
    const float *row = aero.cd[openingIndex];
    return (1.0f - w) * row[k] + w * row[k + 1];
}

float AirbrakesController::cdAtOpening(const AeroTable &aero, float openingPct, float mach)
{
    float wm = 0.0f;
    const size_t km = findSegment(aero.mach_bp, aero.mach_count, mach, wm);
    float wo = 0.0f;
    const size_t ko = findSegment(aero.opening_bp_pct, aero.opening_count, openingPct, wo);

    const float lower = (1.0f - wm) * aero.cd[ko][km] + wm * aero.cd[ko][km + 1];
    const float upper = (1.0f - wm) * aero.cd[ko + 1][km] + wm * aero.cd[ko + 1][km + 1];
    return (1.0f - wo) * lower + wo * upper;
}

void AirbrakesController::isaAtmosphere(float altitudeMslM, float &densityKgm3, float &speedOfSoundMps)
{
    constexpr float pressureExponent = GRAVITY / (ISA_GAS_CONSTANT * ISA_LAPSE_RATE_K_PER_M);

    const float h = std::min(std::max(altitudeMslM, 0.0f), ISA_TROPOPAUSE_M);
    const float temperatureK = ISA_T0_K - ISA_LAPSE_RATE_K_PER_M * h;
    const float pressurePa = ISA_P0_PA * std::pow(temperatureK / ISA_T0_K, pressureExponent);
    densityKgm3 = pressurePa / (ISA_GAS_CONSTANT * temperatureK);
    speedOfSoundMps = std::sqrt(ISA_GAMMA * ISA_GAS_CONSTANT * temperatureK);
}

void AirbrakesController::predictApogee(size_t openingIndex,
                                         float altitudeAglM,
                                         float verticalVelocityMps,
                                         float &apogeeAglM,
                                         float &timeToApogeeS) const
{
    const float dt = _params.prediction_dt_s;
    const float dragFactor = 0.5f * _params.reference_area_m2 / _params.dry_mass_kg;

    float v = verticalVelocityMps;
    float h = altitudeAglM;
    float t = 0.0f;

    for (uint32_t i = 0; i < _params.prediction_max_steps; ++i)
    {
        float rho = 0.0f;
        float speedOfSound = 0.0f;
        isaAtmosphere(h + _params.launch_site_altitude_m, rho, speedOfSound);

        const float cd = cdAlongMach(*_params.aero, openingIndex, v / speedOfSound);
        const float a = -dragFactor * cd * rho * v * v - GRAVITY;
        v += a * dt;
        h += v * dt;
        t += dt;
        if (v < 0.0f)
        {
            break;
        }
    }

    // Also the result when the horizon is exhausted, as in the reference
    apogeeAglM = h;
    timeToApogeeS = t;
}

AirbrakesController::Outputs AirbrakesController::computeCommand(const Inputs &inputs) const
{
    Outputs out{};
    out.armed = false;
    out.apogee_closed_m = NAN;
    if (_params.aero == nullptr)
    {
        return out;
    }

    const AeroTable &aero = *_params.aero;
    const size_t openingCount = aero.opening_count;
    out.opening_pct = aero.opening_bp_pct[0];

    const float h0 = inputs.altitude_agl_m;
    const float v0 = inputs.vertical_velocity_mps;
    if (!std::isfinite(h0) || !std::isfinite(v0))
    {
        out.cd_cmd = aero.cd[0][0];
        return out;
    }

    float rho0 = 0.0f;
    float a0 = 0.0f;
    isaAtmosphere(h0 + _params.launch_site_altitude_m, rho0, a0);
    out.mach = std::fabs(v0) / a0;
    out.cd_cmd = cdAtOpening(aero, aero.opening_bp_pct[0], out.mach);

    if (h0 < _params.opening_altitude_m || v0 <= 0.0f)
    {
        return out;
    }
    out.armed = true;

    // Scan from closed upward
    float openingPct = aero.opening_bp_pct[openingCount - 1];
    float previousApogee = 0.0f;
    float previousTime = 0.0f;

    for (size_t k = 0; k < openingCount; ++k)
    {
        float apogee = 0.0f;
        float timeToApogee = 0.0f;
        predictApogee(k, h0, v0, apogee, timeToApogee);
        ++out.candidates_evaluated;
        if (k == 0)
        {
            out.apogee_closed_m = apogee;
        }

        if (apogee <= _params.target_apogee_m)
        {
            if (k == 0)
            {
                openingPct = aero.opening_bp_pct[0];
                out.predicted_apogee_m = apogee;
                out.time_to_apogee_s = timeToApogee;
            }
            else
            {
                // Interpolate between the last candidate above target
                // and the first one at or below it
                const float w = (previousApogee - _params.target_apogee_m) / (previousApogee - apogee);
                openingPct = aero.opening_bp_pct[k - 1] +
                             w * (aero.opening_bp_pct[k] - aero.opening_bp_pct[k - 1]);
                out.predicted_apogee_m = _params.target_apogee_m;
                out.time_to_apogee_s = previousTime + w * (timeToApogee - previousTime);
            }
            break;
        }

        // Still above target, keep this candidate as the fallback
        previousApogee = apogee;
        previousTime = timeToApogee;
        openingPct = aero.opening_bp_pct[k];
        out.predicted_apogee_m = apogee;
        out.time_to_apogee_s = timeToApogee;
    }

    out.opening_pct = openingPct;
    out.cd_cmd = cdAtOpening(aero, openingPct, out.mach);
    return out;
}
