#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

/**
 * @brief Detects apogee from accelerometer specific force alone.
 *
 * An accelerometer measures specific force, not acceleration:
 *
 * $$ \mathbf{f} = \mathbf{a} - \mathbf{g} $$
 *
 * At rest on the pad $\mathbf{a} = 0$, so the averaged pad reading
 * $\mathbf{f}_0$ points up in the sensor frame. Each flight reading is
 * projected onto that axis, $\hat{u} = \mathbf{f}_0 / |\mathbf{f}_0|$, and
 * integrated:
 *
 * $$ a_z = \mathbf{f}\cdot\hat{u} - |\mathbf{f}_0|, \qquad v_z = \int a_z\,dt $$
 *
 * Subtracting the measured pad magnitude rather than nominal gravity cancels a
 * constant sensor bias along $\hat{u}$, and a scale-factor error scales $v_z$
 * without moving its zero crossing. Apogee is the first run of confirmed
 * $v_z \le v_{trigger}$ samples after liftoff.
 *
 * Limits:
 * - The projection assumes the launch attitude. Pitch-over during coast leaks
 *   drag into the vertical axis, so flights far from vertical drift.
 * - A clipped sample makes the integral unrecoverable. Sustained saturation, a
 *   sample gap after liftoff, or a missing pad reference mark the estimate
 *   invalid; the caller must then rely on another detector.
 * - Once a parachute opens the attitude no longer matches the pad, so only the
 *   first apogee decision is meaningful.
 *
 * The class has no RTOS or ESP-IDF dependencies so flights can be replayed
 * off-target.
 */
class AccelerometerApogeeDetector {
public:
    enum class Fault : uint8_t {
        None,
        NoPadReference,
        Saturated,
        SampleGap,
    };

    static constexpr float STANDARD_GRAVITY_MPS2 = 9.80665f;
    // |f| must stay this close to the pad reference for a sample to count as rest.
    static constexpr float PAD_STATIONARY_TOLERANCE_MPS2 = 3.0f;
    // Low-pass time constant of the pad reference while waiting for liftoff.
    static constexpr float PAD_REFERENCE_TIME_CONSTANT_S = 2.0f;
    // Rest samples needed to build a pad reference when no calibration seed exists.
    static constexpr uint32_t PAD_REFERENCE_MIN_SAMPLES = 50;
    // A longer hole after liftoff means the integral skipped real motion.
    static constexpr uint32_t MAX_FLIGHT_SAMPLE_GAP_MS = 250;

    static bool isStationary(float x, float y, float z,
                             float referenceMagnitudeMps2 = STANDARD_GRAVITY_MPS2)
    {
        return std::fabs(std::sqrt(x * x + y * y + z * z) - referenceMagnitudeMps2) <
               PAD_STATIONARY_TOLERANCE_MPS2;
    }

    static const char *faultName(Fault fault)
    {
        switch (fault) {
            case Fault::None: return "none";
            case Fault::NoPadReference: return "no pad reference";
            case Fault::Saturated: return "accelerometer saturated";
            case Fault::SampleGap: return "accelerometer sample gap";
        }
        return "unknown";
    }

    // The caller supplies RuntimeConfig-validated values before the first sample.
    void configure(float liftoffAccelMps2, uint32_t liftoffDebounceMs,
                   float triggerVelocityMps, uint8_t confirmationSamples,
                   float saturationLimitMps2)
    {
        _liftoffAccelMps2 = liftoffAccelMps2;
        _liftoffDebounceMs = liftoffDebounceMs;
        _triggerVelocityMps = triggerVelocityMps;
        _confirmationSamples = confirmationSamples;
        _saturationLimitMps2 = saturationLimitMps2;
        reset();
    }

    /**
     * @brief Start from a pad reference averaged during calibration.
     *
     * Call after configure() and before the first update(). Rest samples seen
     * before liftoff keep refining it, so a reference taken minutes earlier
     * converges to the final rail attitude.
     */
    void seedPadReference(float x, float y, float z)
    {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || !isStationary(x, y, z)) {
            return;
        }
        _padReference = {x, y, z};
        _hasPadReference = true;

        // The vehicle was at rest before the first sample. Priming the median
        // with that state delays ignition by the same one sample as burnout;
        // otherwise an immediate ignition would integrate one boost sample twice.
        _rawSamples.fill(_padReference);
        _rawCount = _rawSamples.size();
    }

    void update(float x, float y, float z, uint32_t timestampMs)
    {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return;
        if (_hasTimestamp && timestampMs == _lastTimestampMs) return;

        // Median of three per axis rejects one-sample shocks at the cost of
        // one sample of delay; a real step still passes through unchanged.
        _rawSamples[_rawHead] = {x, y, z};
        _rawHead = (_rawHead + 1) % _rawSamples.size();
        if (_rawCount < _rawSamples.size()) ++_rawCount;

        Vec3 force = {x, y, z};
        if (_rawCount == _rawSamples.size()) {
            for (size_t axis = 0; axis < force.size(); ++axis) {
                force[axis] = median3(_rawSamples[0][axis], _rawSamples[1][axis], _rawSamples[2][axis]);
            }
        }

        // Unsigned subtraction also handles millis() wrap.
        const uint32_t dtMs = _hasTimestamp ? timestampMs - _lastTimestampMs : 0;
        const float dtS = static_cast<float>(dtMs) / 1000.0f;
        _hasTimestamp = true;
        _lastTimestampMs = timestampMs;

        if (!_launched) {
            updateOnPad(force, dtS);
            updateLiftoff(norm(force), timestampMs);
            return;
        }
        updateInFlight(force, dtMs, dtS);
    }

    bool isLaunched() const { return _launched; }
    bool hasPadReference() const { return _hasPadReference; }
    // Valid means the integral covers the whole ascent from a known rest state.
    bool isValid() const { return _launched && _fault == Fault::None; }
    bool isRising() const { return !_apogeeDetected; }
    float getVelocity() const { return _velocityMps; }
    Fault getFault() const { return _fault; }

    void reset()
    {
        _rawCount = 0;
        _rawHead = 0;
        _hasTimestamp = false;
        _lastTimestampMs = 0;
        _hasPadReference = false;
        _padReference = {0.0f, 0.0f, 0.0f};
        _padReferenceSamples = 0;
        _liftoffHigh = false;
        _liftoffHighSinceMs = 0;
        _launched = false;
        _fault = Fault::None;
        _velocityMps = 0.0f;
        _lastVerticalAccelMps2 = 0.0f;
        _consecutiveDescendingSamples = 0;
        _apogeeDetected = false;
    }

private:
    using Vec3 = std::array<float, 3>;

    static float median3(float a, float b, float c)
    {
        return std::max(std::min(a, b), std::min(std::max(a, b), c));
    }

    static float norm(const Vec3 &v)
    {
        return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }

    float verticalAcceleration(const Vec3 &force) const
    {
        const float referenceMagnitude = norm(_padReference);
        const float projection = (force[0] * _padReference[0] +
                                  force[1] * _padReference[1] +
                                  force[2] * _padReference[2]) / referenceMagnitude;
        return projection - referenceMagnitude;
    }

    void integrate(float verticalAccelMps2, float dtS)
    {
        // Trapezoidal rule over the real sample interval.
        _velocityMps += 0.5f * (verticalAccelMps2 + _lastVerticalAccelMps2) * dtS;
        _lastVerticalAccelMps2 = verticalAccelMps2;
    }

    void holdAtRest()
    {
        _velocityMps = 0.0f;
        _lastVerticalAccelMps2 = 0.0f;
    }

    void updateOnPad(const Vec3 &force, float dtS)
    {
        if (!_hasPadReference) {
            // No calibration seed: average rest samples until enough are seen.
            if (isStationary(force[0], force[1], force[2])) {
                ++_padReferenceSamples;
                for (size_t axis = 0; axis < force.size(); ++axis) {
                    _padReference[axis] += (force[axis] - _padReference[axis]) /
                                           static_cast<float>(_padReferenceSamples);
                }
                _hasPadReference = _padReferenceSamples >= PAD_REFERENCE_MIN_SAMPLES;
            }
            holdAtRest();
            return;
        }

        if (isStationary(force[0], force[1], force[2], norm(_padReference))) {
            // At rest: follow slow attitude and bias changes, pin velocity to zero.
            const float alpha = std::min(1.0f, dtS / PAD_REFERENCE_TIME_CONSTANT_S);
            for (size_t axis = 0; axis < force.size(); ++axis) {
                _padReference[axis] += alpha * (force[axis] - _padReference[axis]);
            }
            holdAtRest();
            return;
        }

        // Possible ignition: integrate provisionally so the liftoff debounce
        // window is not lost. The next rest sample discards it.
        integrate(verticalAcceleration(force), dtS);
    }

    void updateLiftoff(float magnitudeMps2, uint32_t timestampMs)
    {
        if (magnitudeMps2 <= _liftoffAccelMps2) {
            _liftoffHigh = false;
            return;
        }
        if (!_liftoffHigh) {
            _liftoffHigh = true;
            _liftoffHighSinceMs = timestampMs;
        }
        if (timestampMs - _liftoffHighSinceMs >= _liftoffDebounceMs) {
            _launched = true;
            if (!_hasPadReference) _fault = Fault::NoPadReference;
        }
    }

    void updateInFlight(const Vec3 &force, uint32_t dtMs, float dtS)
    {
        // Faults freeze at apogee: later deployment shocks must not undo a decision
        // the FSM may already have acted on.
        if (!_apogeeDetected && _fault == Fault::None) {
            const float peakAxis = std::max({std::fabs(force[0]), std::fabs(force[1]), std::fabs(force[2])});
            if (dtMs > MAX_FLIGHT_SAMPLE_GAP_MS) {
                _fault = Fault::SampleGap;
            } else if (peakAxis >= _saturationLimitMps2) {
                _fault = Fault::Saturated;
            }
        }
        if (!_hasPadReference) return;

        integrate(verticalAcceleration(force), dtS);
        if (_apogeeDetected) return;

        // Require consecutive descending samples to reject a noise excursion.
        if (_velocityMps <= _triggerVelocityMps) {
            if (_consecutiveDescendingSamples < _confirmationSamples) {
                ++_consecutiveDescendingSamples;
            }
        } else {
            _consecutiveDescendingSamples = 0;
        }
        _apogeeDetected = _consecutiveDescendingSamples >= _confirmationSamples;
    }

    float _liftoffAccelMps2 = 3.0f * STANDARD_GRAVITY_MPS2;
    uint32_t _liftoffDebounceMs = 250;
    float _triggerVelocityMps = 0.0f;
    uint8_t _confirmationSamples = 5;
    float _saturationLimitMps2 = 16.0f * STANDARD_GRAVITY_MPS2;

    std::array<Vec3, 3> _rawSamples = {};
    size_t _rawHead = 0;
    size_t _rawCount = 0;
    bool _hasTimestamp = false;
    uint32_t _lastTimestampMs = 0;

    bool _hasPadReference = false;
    Vec3 _padReference = {0.0f, 0.0f, 0.0f};
    uint32_t _padReferenceSamples = 0;

    bool _liftoffHigh = false;
    uint32_t _liftoffHighSinceMs = 0;
    bool _launched = false;
    Fault _fault = Fault::None;

    float _velocityMps = 0.0f;
    float _lastVerticalAccelMps2 = 0.0f;
    uint8_t _consecutiveDescendingSamples = 0;
    bool _apogeeDetected = false;
};
