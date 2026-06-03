#include "AltitudeTask.hpp"
#include <cmath>
#include <config.h>
#include <Logger.hpp>

void AltitudeTask::taskFunction()
{
    LOG_INFO("AltitudeTask", "Starting Altitude task pipeline...");

    uint32_t lastTimestamp = 0;
    
    // Baseline for the slew rate limiter
    static float lastValidPressure = -1.0f; 

    while (running)
    {
        esp_task_wdt_reset();
        if(!running) break;

        PressureSensorData baroData("Barometer");
#ifdef BARO_1
        bool result = _rocketModel->getMS561101BA03Data_1(baroData);
#else
        bool result = _rocketModel->getMS561101BA03Data_2(baroData);
#endif
        // Reject identical simulated packets
        if (baroData.pressure <= 0.0f || baroData.timestamp == lastTimestamp) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        lastTimestamp = baroData.timestamp;
        float rawPressure = baroData.pressure;

        // Physics Lock, if the pressure change is too extreme, clamp it 
        // to a maximum plausible change based on physical limits of the 
        // atmosphere and the sampling rate (prevents spikes instability errors)
        if (lastValidPressure < 0.0f) {
            lastValidPressure = rawPressure;
        } else {
            float deltaP = rawPressure - lastValidPressure;
            
            // Clamp the pressure change to physical reality
            if (deltaP > MAX_DELTA_P_PER_TICK) {
                rawPressure = lastValidPressure + MAX_DELTA_P_PER_TICK;
            } else if (deltaP < -MAX_DELTA_P_PER_TICK) {
                rawPressure = lastValidPressure - MAX_DELTA_P_PER_TICK;
            }
            lastValidPressure = rawPressure;
        }
        
        // Median Filter (removes isolated outliers)
        float filteredPressure = pressureFilter.update(rawPressure);

        if (!pressureFilter.isReady()) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        
        // Altitude Calculation
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
        _rocketModel->setCurrentHeight(currentAltitude);
        
        float currentVelocity = apogeeDetector.getVelocity();

        LOG_INFO("AltitudeTask", "Alt: %0.2f m | Vz: %0.2f m/s | Max: %0.2f m", 
                 currentAltitude, currentVelocity, _max_altitude_read);
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

float AltitudeTask::calculateAltitude(float pressure, float pressureRef)
{
    if (pressureRef <= 0.0f || pressure <= 0.0f) return 0.0f;
    
    float tempRef = TEMP_REF;
    if (_rocketModel->isTemperatureZeroed()) {
        tempRef = _rocketModel->getLaunchpadBaseTemperature();
    }

    return (tempRef / TEMP_GRADIENT) * (1.0f - powf(pressure / pressureRef, N_INV));
}

void AltitudeTask::updateRisingTrend(float currentAltitude)
{
    apogeeDetector.update(currentAltitude);   
    _rocketModel->setIsRising(apogeeDetector.isRising());
}