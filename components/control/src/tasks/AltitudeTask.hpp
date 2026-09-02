#pragma once
#include "BaseTask.hpp"

#include <MS561101BA03.hpp>
#include <SerialLogger.hpp>
#include <RocketModel.hpp>
#include <utils.h>
#include <config.h>
#include <array>
#include <algorithm>

// Barometer noise filtering
// ALTITUDE_FILTER_WINDOW: Size of median filter window for pressure/altitude smoothing
// Smaller = faster response but more noise (1 = no filtering)
// Larger = smoother but more lag (recommended: 3-7)
// At 10Hz sampling: window=5 adds 50ms lag
inline constexpr size_t ALTITUDE_FILTER_MAX_WINDOW = 31;
inline constexpr size_t APOGEE_DETECTOR_MAX_WINDOW = 64;
inline constexpr uint32_t ALTITUDE_FILTER_DEFAULT_WINDOW = 11;
inline constexpr uint32_t APOGEE_DETECTOR_DEFAULT_WINDOW = 25;

// TODO: ApogeeDetectorConfig?
struct AltitudeConfig {
    uint32_t filter_window;
    float max_pressure_rate_pa_per_s;
    uint32_t apogee_window_size;
    float apogee_sample_rate_hz;
    float apogee_trigger_velocity_mps;
    uint8_t apogee_confirmation_windows;
    uint8_t selected_barometer;
};

/**
 * @brief RTOS-Safe Median Filter using fixed-size arrays.
 */
template <size_t MaxWindowSize>
class FixedMedianFilter {
public:
    FixedMedianFilter() { buffer.fill(0.0f); }

    float update(float newValue, size_t windowSize) {
        buffer[head] = newValue;
        head = (head + 1) % windowSize;
        
        if (count < windowSize) {
            count++;
            return newValue; 
        }

        std::array<float, MaxWindowSize> sorted = buffer;
        std::sort(sorted.begin(), sorted.begin() + windowSize);

        const size_t mid = windowSize / 2;
        if (windowSize % 2 == 0) {
            return (sorted[mid - 1] + sorted[mid]) / 2.0f;
        } else {
            return sorted[mid];
        }
    }
    
    void reset() { count = 0; head = 0; }
    bool isReady(size_t windowSize) const { return count >= windowSize; }
    
private:
    std::array<float, MaxWindowSize> buffer;
    size_t head = 0;
    size_t count = 0;
};

/**
 * @brief Estimates vertical velocity from a bounded window of altitude samples.
 *
 * Given timestamped samples $(t_i, h_i)$, the detector fits:
 *
 * $$ h_i = v t_i + b $$
 *
 * by ordinary least squares. The fitted slope $v$ is vertical velocity.
 * Real barometer timestamps are used, so a delayed sample changes the sample
 * interval instead of producing a false velocity estimate.
 *
 */
template <size_t MaxWindowSize>
class OLSApogeeDetector {
public:
    OLSApogeeDetector()
    {
        _altitudeSamplesM.fill(0.0f);
        _sampleTimestampsMs.fill(0);
    }

    // The caller supplies RuntimeConfig-validated values before the first sample.
    void configure(size_t windowSize, float apogeeTriggerVelocityMps,
                   uint8_t confirmationWindows)
    {
        _activeWindowSize = windowSize;
        _apogeeTriggerVelocityMps = apogeeTriggerVelocityMps;
        _confirmationWindows = confirmationWindows;
        reset();
    }

    void update(float altitudeM, uint32_t timestampMs)
    {
        const size_t windowSize = _activeWindowSize;

        // `_nextWriteIndex` always points to the oldest element after a full
        // circular buffer wraps.  Iterating from it restores chronological order.
        _altitudeSamplesM[_nextWriteIndex] = altitudeM;
        _sampleTimestampsMs[_nextWriteIndex] = timestampMs;
        _nextWriteIndex = (_nextWriteIndex + 1) % windowSize;
        if (_sampleCount < windowSize) ++_sampleCount;

        // A partially filled regression window has a changing response; do not
        // use it for apogee decisions.
        if (_sampleCount < windowSize) return;

        const uint32_t oldestTimestampMs = _sampleTimestampsMs[_nextWriteIndex];
        float sumTimeS = 0.0f;
        float sumAltitudeM = 0.0f;
        float sumTimeSquaredS2 = 0.0f;
        float sumTimeAltitudeMS = 0.0f;

        for (size_t sampleOffset = 0; sampleOffset < windowSize; ++sampleOffset) {
            const size_t sampleIndex = (_nextWriteIndex + sampleOffset) % windowSize;
            // Unsigned subtraction also handles millis() wrap for this short window.
            const float timeSinceOldestS = static_cast<float>(
                _sampleTimestampsMs[sampleIndex] - oldestTimestampMs) / 1000.0f;
            const float sampleAltitudeM = _altitudeSamplesM[sampleIndex];

            sumTimeS += timeSinceOldestS;
            sumAltitudeM += sampleAltitudeM;
            sumTimeSquaredS2 += timeSinceOldestS * timeSinceOldestS;
            sumTimeAltitudeMS += timeSinceOldestS * sampleAltitudeM;
        }

        // Least-squares slope of altitude over time:
        //
        // $$ v = \frac{N\sum(t_i h_i) - \sum t_i \sum h_i}{N\sum(t_i^2) - (\sum t_i)^2} $$
        //
        // `timeSinceOldestS` is seconds and altitude is metres, so v is m/s.
        const float slopeDenominator =
            (windowSize * sumTimeSquaredS2) - (sumTimeS * sumTimeS);
        if (slopeDenominator > 0.000001f) {
            _estimatedVelocityMps =
                ((windowSize * sumTimeAltitudeMS) - (sumTimeS * sumAltitudeM)) /
                slopeDenominator;

            // Require consecutive descending OLS fits to reject a one-window
            // noise excursion. The configured threshold remains the definition
            // of a descending fit (normally a small negative velocity).
            if (_estimatedVelocityMps <= _apogeeTriggerVelocityMps) {
                if (_consecutiveDescendingFits < _confirmationWindows) {
                    ++_consecutiveDescendingFits;
                }
            } else {
                _consecutiveDescendingFits = 0;
            }
        }
    }

    bool isRising() const
    {
        // Remain safely "rising" until enough descending fits confirm apogee.
        return _sampleCount < _activeWindowSize ||
               _consecutiveDescendingFits < _confirmationWindows;
    }

    float getVelocity() const { return _estimatedVelocityMps; }
    void reset()
    {
        _nextWriteIndex = 0;
        _sampleCount = 0;
        _consecutiveDescendingFits = 0;
        _estimatedVelocityMps = 0.0f;
    }

private:
    std::array<float, MaxWindowSize> _altitudeSamplesM;
    std::array<uint32_t, MaxWindowSize> _sampleTimestampsMs;
    size_t _nextWriteIndex = 0;
    size_t _sampleCount = 0;
    size_t _activeWindowSize = MaxWindowSize;
    float _apogeeTriggerVelocityMps = -0.5f;
    uint8_t _confirmationWindows = 3;
    uint8_t _consecutiveDescendingFits = 0;
    float _estimatedVelocityMps = 0.0f;
};

/**
 * @brief Task responsible for reading pressure, filtering, and calculating AGL altitude.
 */
class AltitudeTask : public BaseTask
{
public:
    AltitudeTask(std::shared_ptr<RocketModel> rocketModel)
        : BaseTask("AltitudeTask"),
          _rocketModel(rocketModel),
          _max_altitude_read(-1000.0f)
    {
    }

    void taskFunction() override;

    /**
     * @brief Clears filter/detector state so a restart without a power cycle
     *        (HIL runs, ground tests) does not inherit the previous run's state.
     */
    void onTaskStart() override;

    ~AltitudeTask() override
    {
        stop();
    }
 
private:
    std::shared_ptr<RocketModel> _rocketModel;

    float _max_altitude_read;
    
    // RTOS-Safe Filters
    FixedMedianFilter<ALTITUDE_FILTER_MAX_WINDOW> pressureFilter;
    
    // OLS Apogee Detector: 
    // Window, sample rate, and trigger velocity come from RuntimeConfig.
    // We could switch the -0.5 to something positive like 1.0/2.0 to try triggering 
    // it before the apogee, but we should be carefull at the end of the motor burnout, 
    // as there is a strong drag force (which should be covered by the RuntimeConfig apogee lockout)
    OLSApogeeDetector<APOGEE_DETECTOR_MAX_WINDOW> apogeeDetector;

    // Atmospheric Constants
    static constexpr float TEMP_GRADIENT = 0.0065f; // [K/m]
    static constexpr float AIR_GAS_CONST = 287.05f; // [J/Kg/K]
    static constexpr float TEMP_REF      = 288.15f; // [K]
    
    // Precomputed inverse exponent for the altitude formula
    static constexpr float N_INV = (AIR_GAS_CONST * TEMP_GRADIENT) / GRAVITY;

    /**
     * @brief Calculates altitude using the hypsometric formula.
     */
    float calculateAltitude(float pressure, float pressureRef);

    /**
     * @brief Updates the trend buffer and evaluates if the rocket is still rising.
     */
    void updateRisingTrend(float currentAltitude, uint32_t timestamp);
};
