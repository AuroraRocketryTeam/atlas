#include "AirbrakesController.hpp"

#include <algorithm>
#include <cmath>

namespace airbrakes
{
    namespace
    {
        float clamp01(float value)
        {
            if (!std::isfinite(value)) return 0.0f;
            return std::min(1.0f, std::max(0.0f, value));
        }

        /**
         * @brief Step @p current toward @p target by at most @p maxStep.
         *
         * This is the servo rate limit. It is applied to the normalized travel
         * rather than to the drag coefficient, because it is the servo, not the
         * aerodynamics, that is rate-limited.
         */
        float moveTowards(float current, float target, float maxStep)
        {
            current = clamp01(current);
            target = clamp01(target);
            if (!(maxStep > 0.0f)) return current;

            if (current < target) return std::min(target, current + maxStep);
            if (current > target) return std::max(target, current - maxStep);
            return current;
        }
    }

    void AirbrakesController::configure(const ControllerParams &params)
    {
        _params = params;
        reset();
    }

    void AirbrakesController::reset()
    {
        _integrator = 0.0f;
        _deployment = 0.0f;
        _targetOpeningPct = 0.0f;
    }

    float AirbrakesController::updatePid(float predictedApogeeM, float dtSeconds)
    {
        const float errorM = predictedApogeeM - _params.target_apogee_m;

        // Conditional integration. The GNC reference integrates unconditionally;
        // with a hard saturation downstream that lets the integrator run away
        // while the brakes are already fully open, and the vehicle then keeps
        // braking long after the error has reversed. Only integrate when doing
        // so moves the unsaturated demand back toward the linear range.
        const float candidateIntegrator = _integrator + errorM * dtSeconds;
        const float candidateDemand = _params.kp * errorM + _params.ki * candidateIntegrator;
        const bool windingUp = (candidateDemand > 1.0f && errorM > 0.0f) ||
                               (candidateDemand < 0.0f && errorM < 0.0f);
        if (!windingUp)
        {
            _integrator = candidateIntegrator;
        }

        // Normalized brake demand, then mapped onto the drag range. The
        // reference does the same thing as a gain of 0.3 followed by a sum with
        // Cd_clean, 0.3 being Cd_max - Cd_clean for its airframe.
        const float demand = clamp01(_params.kp * errorM + _params.ki * _integrator);
        return _params.cd_clean + (_params.cd_max - _params.cd_clean) * demand;
    }

    float AirbrakesController::searchCd(const ControllerInputs &inputs) const
    {
        const float cdStep = (_params.cd_max - _params.cd_clean) /
                             static_cast<float>(kCdSearchSteps);

        // Predicted apogee decreases monotonically with drag, so this returns
        // the least drag that brings the vehicle to target: the same choice the
        // reference makes by breaking out of its sweep.
        float cd = _params.cd_clean;
        for (int step = 0; step <= kCdSearchSteps; ++step)
        {
            cd = _params.cd_clean + static_cast<float>(step) * cdStep;
            const ApogeePrediction prediction = predictApogee(inputs.altitude_m,
                                                              inputs.vertical_velocity_mps,
                                                              cd,
                                                              inputs.air_density_kgm3,
                                                              _params.reference_area_m2,
                                                              _params.dry_mass_kg);
            if (prediction.apogee_altitude_m < _params.target_apogee_m)
            {
                break;
            }
        }
        return cd;
    }

    ControllerOutputs AirbrakesController::update(const ControllerInputs &inputs)
    {
        ControllerOutputs out{};

        // Drag the vehicle has right now, from where the servo actually is.
        out.achieved_cd = cdForOpening(_deployment * 100.0f, inputs.mach);

        const ApogeePrediction prediction = predictApogee(inputs.altitude_m,
                                                          inputs.vertical_velocity_mps,
                                                          out.achieved_cd,
                                                          inputs.air_density_kgm3,
                                                          _params.reference_area_m2,
                                                          _params.dry_mass_kg);
        out.predicted_apogee_m = prediction.apogee_altitude_m;
        out.time_to_apogee_s = prediction.time_to_apogee_s;
        out.apogee_error_m = prediction.apogee_altitude_m - _params.target_apogee_m;

        // Arming window. Below the opening altitude the brakes stay shut so the
        // vehicle is not braked while the prediction is still dominated by the
        // burn. Above the closing altitude, and once the vehicle stops climbing,
        // they shut again: braking past apogee only disturbs recovery.
        out.armed = inputs.altitude_m >= _params.opening_altitude_m &&
                    inputs.altitude_m < _params.close_altitude_m &&
                    inputs.vertical_velocity_mps > 0.0f &&
                    std::isfinite(inputs.air_density_kgm3) &&
                    inputs.air_density_kgm3 > 0.0f;

        float targetDeployment = 0.0f;
        if (!out.armed)
        {
            _targetOpeningPct = 0.0f;
            // Hold the integrator cleared while disarmed. Error accumulated
            // before the brakes may open is not error the brakes could have
            // corrected, and carrying it in would cause an opening transient.
            _integrator = 0.0f;
            out.requested_cd = _params.cd_clean;
        }
        else
        {
            switch (_params.mode)
            {
                case ControlMode::PredictivePid:
                    out.requested_cd = updatePid(prediction.apogee_altitude_m, inputs.dt_s);
                    break;

                case ControlMode::PredictiveCdSearch:
                    out.requested_cd = searchCd(inputs);
                    break;

                case ControlMode::AltitudeBangBang:
                default:
                    // The legacy law commands travel directly; there is no drag
                    // request to invert.
                    targetDeployment = 1.0f;
                    out.requested_cd = cdForOpening(100.0f, inputs.mach);
                    break;
            }

            if (_params.mode != ControlMode::AltitudeBangBang)
            {
                // Requested drag outside the table cannot be produced; clamping
                // here keeps the inverse lookup inside its defined envelope.
                const float requestedCd = std::min(kTableCdMax,
                                                   std::max(kTableCdMin, out.requested_cd));
                const float openingPct = openingForCd(inputs.mach, requestedCd);
                _targetOpeningPct = snapToOpeningBreakpoint(openingPct,
                                                            _targetOpeningPct,
                                                            kOpeningHysteresisPct);
                targetDeployment = clamp01(_targetOpeningPct / 100.0f);
            }
        }

        const float ratePerSecond = targetDeployment > _deployment
                                        ? _params.open_rate_per_s
                                        : _params.close_rate_per_s;
        _deployment = moveTowards(_deployment, targetDeployment, ratePerSecond * inputs.dt_s);

        out.commanded_deployment = _deployment;
        out.opening_pct = _deployment * 100.0f;
        out.target_opening_pct = targetDeployment * 100.0f;
        out.integrator = _integrator;
        return out;
    }
}
