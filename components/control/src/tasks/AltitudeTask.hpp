#pragma once
#include "BaseTask.hpp"

#include <MS561101BA03.hpp>
#include <Logger.hpp>
#include <SharedData.hpp>
#include <RocketModel.hpp>
#include <config.h>
#include <array>
#include <algorithm>

// Toggle to switch between Baro 1 and Baro 2
#define BARO_1

/**
 * @brief RTOS-Safe Median Filter using fixed-size arrays.
 */
template <size_t WindowSize>
class FixedMedianFilter {
public:
    FixedMedianFilter() { buffer.fill(0.0f); }

    float update(float newValue) {
        buffer[head] = newValue;
        head = (head + 1) % WindowSize;
        
        if (count < WindowSize) {
            count++;
            return newValue; 
        }

        std::array<float, WindowSize> sorted = buffer;
        std::sort(sorted.begin(), sorted.end());

        constexpr size_t mid = WindowSize / 2;
        if constexpr (WindowSize % 2 == 0) {
            return (sorted[mid - 1] + sorted[mid]) / 2.0f;
        } else {
            return sorted[mid];
        }
    }
    
    void reset() { count = 0; head = 0; }
    bool isReady() const { return count >= WindowSize; }
    
private:
    std::array<float, WindowSize> buffer;
    size_t head = 0;
    size_t count = 0;
};

/**
 * @brief RTOS-Safe Windowed Ordinary Least Squares Apogee Detector
 * Calculates the slope of the line of best fit over the last N altitude samples.
 */
template <size_t WindowSize>
class OLSApogeeDetector {
public:
    /**
     * @param sample_rate_hz Expected frequency of the task calling update()
     * @param trigger_velocity_ms Vertical velocity threshold (m/s) to trigger apogee. 
     * Set slightly negative to avoid false positives at apex.
     */
    OLSApogeeDetector(float sample_rate_hz = 50.0f, float trigger_velocity_ms = -1.0f) 
        : _dt(1.0f / sample_rate_hz), _trigger_velocity(trigger_velocity_ms) 
    {
        _y_buffer.fill(0.0f);
        
        // Precompute the X-axis (time/index) constants for the OLS formula:
        // slope = [ N*sum(xy) - sum(x)*sum(y) ] / [ N*sum(x^2) - (sum(x))^2 ]
        _sum_x = (WindowSize * (WindowSize - 1)) / 2.0f;
        _sum_x2 = (WindowSize * (WindowSize - 1) * (2 * WindowSize - 1)) / 6.0f;
        _denominator = (WindowSize * _sum_x2) - (_sum_x * _sum_x);
    }

    void update(float newAltitude) {
        _y_buffer[_head] = newAltitude;
        _head++;
        
        if (_head >= WindowSize) {
            _head = 0;
            _is_full = true;
        }

        // Do not calculate velocity until we have a full window of data
        if (!_is_full) return;

        float sum_y = 0.0f;
        float sum_xy = 0.0f;

        // Iterate through buffer from oldest (x=0) to newest (x=WindowSize-1)
        for (size_t i = 0; i < WindowSize; ++i) {
            size_t idx = (_head + i) % WindowSize;
            float y = _y_buffer[idx];
            
            sum_y += y;
            sum_xy += (i * y);
        }

        // Calculate slope (change in altitude per sample)
        float slope_per_sample = ((WindowSize * sum_xy) - (_sum_x * sum_y)) / _denominator;

        // Convert sample slope to physical velocity (m/s)
        _estimated_velocity = slope_per_sample / _dt;
    }

    // Returns true if the rocket is still going up (velocity is > trigger threshold)
    bool isRising() const {
        if (!_is_full) return true; 
        return _estimated_velocity > _trigger_velocity;
    }

    float getVelocity() const { return _estimated_velocity; }
    bool isReady() const { return _is_full; }
    void reset() { _head = 0; _is_full = false; _estimated_velocity = 0.0f; }

private:
    std::array<float, WindowSize> _y_buffer;
    size_t _head = 0;
    bool _is_full = false;

    float _dt;
    float _trigger_velocity;
    float _estimated_velocity = 0.0f;
    
    float _sum_x;
    float _sum_x2;
    float _denominator;
};

/**
 * @brief Task responsible for reading pressure, filtering, and calculating AGL altitude.
 */
class AltitudeTask : public BaseTask
{
public:
    AltitudeTask(std::shared_ptr<RocketModel> rocketModel, SemaphoreHandle_t modelMutex)
        : BaseTask("AltitudeTask"),
          _rocketModel(rocketModel),
          _modelMutex(modelMutex),
          _max_altitude_read(-1000.0f)
    {
    }

    void taskFunction() override;

    ~AltitudeTask() override
    {
        stop();
    }
 
private:
    std::shared_ptr<RocketModel> _rocketModel;
    SemaphoreHandle_t _modelMutex;

    float _max_altitude_read;
    
    // RTOS-Safe Filters
    FixedMedianFilter<ALTITUDE_FILTER_WINDOW> pressureFilter;
    
    // OLS Apogee Detector: 
    // Uses the window size defined in config.h. Assuming 50Hz task rate (20ms delay).
    // Triggers when estimated vertical velocity drops below -0.5 m/s.
    // We could switch the -0.5 to something positive like 1.0/2.0 to try triggering 
    // it before the apogee, but we should be carefull at the end of the motor burnout, 
    // as there is a strong drag force (which shouldn't be a problem thanks to the APOGEE_LOCKOUT_MS timer)
    OLSApogeeDetector<APOGEE_DETECTION_WINDOW_SIZE> apogeeDetector{50.0f, -0.5f};

    // Atmospheric Constants
    static constexpr float TEMP_GRADIENT = 0.0065f; // [K/m]
    static constexpr float AIR_GAS_CONST = 287.05f; // [J/Kg/K]
    static constexpr float TEMP_REF      = 288.15f; // [K]
    
    // Precomputed inverse exponent for the altitude formula
    static constexpr float N_INV = (AIR_GAS_CONST * TEMP_GRADIENT) / GRAVITY;

    // Maximum physically possible pressure change per 20ms tick.
    // 80 Pa ≈ 6.8 meters ≈ 340 m/s (Mach 1).
    static constexpr float MAX_DELTA_P_PER_TICK = 80.0f; 

    /**
     * @brief Calculates altitude using the hypsometric formula.
     */
    float calculateAltitude(float pressure, float pressureRef);

    /**
     * @brief Updates the trend buffer and evaluates if the rocket is still rising.
     */
    void updateRisingTrend(float currentAltitude);
};