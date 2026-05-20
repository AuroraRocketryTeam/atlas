#include "AltitudeTask.hpp"
#include <cmath>
#include <config.h>
#include <Logger.hpp>

void AltitudeTask::taskFunction()
{
    LOG_INFO("AltitudeTask", "Starting Altitude task pipeline...");

    while (running)
    {
        esp_task_wdt_reset();
        if(!running) break;
        
        // Pressure Reading and Filtering
        float rawPressure = readPressure();
        if (rawPressure <= 0.0f) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        
        float filteredPressure = pressureFilter.update(rawPressure);
        
        // Establish Baseline (Fallback to sea-level if not zeroed)
        float pressureRef = _rocketModel->isBarometerZeroed() ? 
                            _rocketModel->getLaunchpadBasePressure() : 
                            101325.0f;

        float currentAltitude = calculateAltitude(filteredPressure, pressureRef);

        // Apogee & Trend Detection
        updateRisingTrend(currentAltitude);
        
        if (currentAltitude > _max_altitude_read) {
            _max_altitude_read = currentAltitude;
        }

        // Update Model
        if (auto heightPtr = _rocketModel->getCurrentHeight()) {
            *heightPtr = currentAltitude;
        }

        LOG_INFO("AltitudeTask", "Alt: %0.2f m | Max: %0.2f m", currentAltitude, _max_altitude_read);
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

float AltitudeTask::readPressure()
{
#ifdef BARO_1
    auto baroData = _rocketModel->getMS561101BA03Data_1();
    if (!baroData) {
        LOG_ERROR("AltitudeTask", "Barometer 1 data not available");
        return 0.0f;
    }
#else
    auto baroData = _rocketModel->getMS561101BA03Data_2();
    if (!baroData) {
        LOG_ERROR("AltitudeTask", "Barometer 2 data not available");
        return 0.0f;
    }
#endif
    return baroData->pressure;
}

float AltitudeTask::calculateAltitude(float pressure, float pressureRef)
{
    if (pressureRef <= 0.0f || pressure <= 0.0f) return 0.0f;
    
    float tempRef = TEMP_REF;
    if (_rocketModel->isTemperatureZeroed()) {
        tempRef = _rocketModel->getLaunchpadBaseTemperature();
    }

    // Hypsometric formula
    return (tempRef / TEMP_GRADIENT) * (1.0f - powf(pressure / pressureRef, N_INV));
}

void AltitudeTask::updateRisingTrend(float currentAltitude)
{
    auto isRisingPtr = _rocketModel->getIsRising();
    if (!isRisingPtr) return;

    if (altitudeTrendBuffer.size() >= trendBufferSize) {
        altitudeTrendBuffer.erase(altitudeTrendBuffer.begin());
    }
    altitudeTrendBuffer.push_back(currentAltitude);

    // Evaluate trend
    if (altitudeTrendBuffer.size() < trendBufferSize) {
        *isRisingPtr = true;
    } else {
        bool currentlyRising = false;
        for (float alt : altitudeTrendBuffer) {
            if (alt >= _max_altitude_read) {
                currentlyRising = true;
                break;
            }
        }
        *isRisingPtr = currentlyRising;
    }
}