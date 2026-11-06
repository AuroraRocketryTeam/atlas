#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>

/**
 * @file DescentDetector.hpp
 * @brief Tells which way the rocket is going from pressure alone fast enough
 *        to be useful to a board that has just restarted in mid air.
 *
 * Only calculates relative ascending / descending / standing still.
 *
 * The verdict comes from a least squares fit over a short rolling window, after
 * a median of three. 
 */
class DescentDetector
{
public:
    enum class Verdict : uint8_t
    {
        Unknown,    ///< not enough samples yet
        Level,      ///< not enough rising / descending
        Rising,
        Descending,
    };

    DescentDetector() { reset(); }

    /**
     * @param thresholdMps vertical speed to determine movement
     * @param windowMs     window size
     */
    void configure(float thresholdMps, uint32_t windowMs)
    {
        _thresholdMps = thresholdMps;
        _windowMs = windowMs;
        reset();
    }

    void reset()
    {
        _count = 0;
        _head = 0;
        _rawCount = 0;
        _referencePa = 0.0f;
        _rateMps = 0.0f;
        _windowVerdict = Verdict::Unknown;
        _latched = Verdict::Unknown;
        _levelSinceMs = 0;
        _levelNow = false;
        _stuck = false;
    }

    /**
     * @brief Feed one barometer reading.
     * @param pressurePa  raw pressure
     * @param timestampMs the sensor's timestamp
     */
    void addSample(float pressurePa, uint32_t timestampMs)
    {
        if (!(pressurePa > 0.0f) || !std::isfinite(pressurePa)) return;

        // Median of three
        _raw[_rawCount % _raw.size()] = pressurePa;
        _rawCount++;
        if (_rawCount < _raw.size()) return;
        const float filtered = median3(_raw[0], _raw[1], _raw[2]);

        if (_referencePa <= 0.0f) _referencePa = filtered;

        push(altitudeFromPressure(filtered, _referencePa), timestampMs);
        evaluate(timestampMs);
    }

    Verdict verdict() const { return _latched == Verdict::Descending ? _latched : _windowVerdict; }

    /// @brief What the current window alone says
    Verdict windowVerdict() const { return _windowVerdict; }

    /// @brief Fitted vertical speed, positive upwards.
    float rateMps() const { return _rateMps; }

    /// @brief How long the window has been reading "standing still"
    uint32_t levelForMs(uint32_t nowMs) const
    {
        if (!_levelNow || _levelSinceMs == 0) return 0;
        return nowMs - _levelSinceMs;
    }

    /**
     * A live sensor sitting still still jitters in its last bits. If faulty it will be
     * stuck as in "no data".
     */
    bool isStuck() const { return _stuck; }

    bool isReady() const { return _count >= MIN_FIT_SAMPLES; }

private:
    static constexpr size_t CAPACITY = 32;         ///< 400 ms needs 10 samples at 25 Hz
    static constexpr size_t MIN_FIT_SAMPLES = 7; 
    // Partially filled windows are therefore not judged.
    static constexpr float MIN_SPAN_FRACTION = 0.8f;

    // Same atmosphere model as AltitudeTask
    static constexpr float TEMP_GRADIENT = 0.0065f;  // [K/m]
    static constexpr float TEMP_REF = 288.15f;       // [K]
    static constexpr float AIR_GAS_CONST = 287.05f;  // [J/(kg K)]
    // Not named GRAVITY: config.h defines that as a macro
    static constexpr float STD_GRAVITY = 9.80665f;   // [m/s^2]
    static constexpr float N_INV = (AIR_GAS_CONST * TEMP_GRADIENT) / STD_GRAVITY;

    static float median3(float a, float b, float c)
    {
        if (a > b) std::swap(a, b);
        if (b > c) std::swap(b, c);
        if (a > b) std::swap(a, b);
        return b;
    }

    static float altitudeFromPressure(float pressure, float reference)
    {
        return (TEMP_REF / TEMP_GRADIENT) * (1.0f - std::pow(pressure / reference, N_INV));
    }

    void push(float altitudeM, uint32_t timestampMs)
    {
        _altitude[_head] = altitudeM;
        _timestamp[_head] = timestampMs;
        _head = (_head + 1) % CAPACITY;
        if (_count < CAPACITY) _count++;
    }

    void evaluate(uint32_t nowMs)
    {
        // Least squares over whatever falls inside the window, which is what
        // makes the detector independent of the sampling rate.
        // Same as AltitudeTask.
        float sumT = 0.0f, sumZ = 0.0f, sumTT = 0.0f, sumTZ = 0.0f;
        float minZ = 0.0f, maxZ = 0.0f;
        size_t n = 0;
        uint32_t spanMs = 0;

        for (size_t i = 0; i < _count; i++)
        {
            const size_t idx = (_head + CAPACITY - 1 - i) % CAPACITY;
            const uint32_t age = nowMs - _timestamp[idx];
            if (age > _windowMs) break;

            const float t = -static_cast<float>(age) / 1000.0f; // seconds, newest at 0
            sumT += t;
            sumZ += _altitude[idx];
            sumTT += t * t;
            sumTZ += t * _altitude[idx];

            if (n == 0) { minZ = maxZ = _altitude[idx]; }
            else if (_altitude[idx] < minZ) minZ = _altitude[idx];
            else if (_altitude[idx] > maxZ) maxZ = _altitude[idx];
            n++;
            spanMs = age;
        }

        if (n < MIN_FIT_SAMPLES ||
            static_cast<float>(spanMs) < MIN_SPAN_FRACTION * static_cast<float>(_windowMs))
        {
            _windowVerdict = Verdict::Unknown;
            return;
        }

        const float fn = static_cast<float>(n);
        const float denom = fn * sumTT - sumT * sumT;
        if (std::fabs(denom) < 1e-9f)
        {
            _windowVerdict = Verdict::Unknown;
            return;
        }

        _rateMps = (fn * sumTZ - sumT * sumZ) / denom;
        _stuck = (maxZ == minZ);

        if (_rateMps <= -_thresholdMps)      _windowVerdict = Verdict::Descending;
        else if (_rateMps >= _thresholdMps)  _windowVerdict = Verdict::Rising;
        else                                 _windowVerdict = Verdict::Level;

        // Descending is the verdict that has to survive.
        if (_windowVerdict == Verdict::Descending) _latched = Verdict::Descending;

        if (_windowVerdict == Verdict::Level)
        {
            if (!_levelNow) { _levelNow = true; _levelSinceMs = nowMs; }
        }
        else
        {
            _levelNow = false;
            _levelSinceMs = 0;
        }
    }

    std::array<float, CAPACITY> _altitude{};
    std::array<uint32_t, CAPACITY> _timestamp{};
    size_t _count = 0;
    size_t _head = 0;

    std::array<float, 3> _raw{};
    size_t _rawCount = 0;

    float _referencePa = 0.0f;
    float _thresholdMps = 2.0f;
    uint32_t _windowMs = 400;

    float _rateMps = 0.0f;
    Verdict _windowVerdict = Verdict::Unknown;
    Verdict _latched = Verdict::Unknown;
    uint32_t _levelSinceMs = 0;
    bool _levelNow = false;
    bool _stuck = false;
};
