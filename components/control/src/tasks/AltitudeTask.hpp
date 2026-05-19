#pragma once
#include "BaseTask.hpp"

#include <MS561101BA03.hpp>
#include <Logger.hpp>
#include <SharedData.hpp>
#include <RocketModel.hpp>
#include <config.h>
#include <vector>
#include <algorithm>

// Toggle to switch between Baro 1 and Baro 2
#define BARO_1

/**
 * @brief Class to implement a median filter.
 */
class MedianFilter {
public:
    explicit MedianFilter(size_t windowSize) : windowSize(windowSize) {
        buffer.reserve(windowSize);
    }

    float update(float newValue) {
        buffer.push_back(newValue);
        if (buffer.size() > windowSize) {
            buffer.erase(buffer.begin());
        }
        
        std::vector<float> sorted = buffer;
        std::sort(sorted.begin(), sorted.end());
        
        size_t mid = sorted.size() / 2;
        return (sorted.size() % 2 == 0) 
            ? (sorted[mid - 1] + sorted[mid]) / 2.0f 
            : sorted[mid];
    }
    
    void reset() { buffer.clear(); }
    bool isReady() const { return buffer.size() >= windowSize; }
    
private:
    size_t windowSize;
    std::vector<float> buffer;
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
        altitudeTrendBuffer.reserve(trendBufferSize);
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
    
    // Noise reduction filter
    MedianFilter pressureFilter{ALTITUDE_FILTER_WINDOW};

    // Buffer for tendency filtering (stores altitudes instead of pressures for performance)
    std::vector<float> altitudeTrendBuffer;
    size_t trendBufferSize = APOGEE_DETECTION_WINDOW_SIZE;

    // Atmospheric Constants
    static constexpr float TEMP_GRADIENT = 0.0065f; // [K/m]
    static constexpr float AIR_GAS_CONST = 287.05f; // [J/Kg/K]
    static constexpr float TEMP_REF      = 288.15f; // [K]
    
    // Precomputed inverse exponent for the altitude formula
    static constexpr float N_INV = (AIR_GAS_CONST * TEMP_GRADIENT) / GRAVITY;

    /**
     * @brief Reads the raw pressure from the configured hardware barometer.
     * @return float Pressure in Pa, or 0.0f if data is unavailable.
     */
    float readPressure();

    /**
     * @brief Calculates altitude using the hypsometric formula.
     */
    float calculateAltitude(float pressure, float pressureRef);

    /**
     * @brief Updates the trend buffer and evaluates if the rocket is still rising.
     */
    void updateRisingTrend(float currentAltitude);
};