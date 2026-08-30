#include "AltitudeTask.hpp"
#include <cmath>
#include <config.h>
#include <SerialLogger.hpp>
#include "RuntimeConfig.hpp"

static constexpr uint32_t ALTITUDE_LOG_PERIOD_MS = 1000;

void AltitudeTask::taskFunction()
{
    LOG_INFO("AltitudeTask", "Starting Altitude task pipeline...");

    uint32_t lastTimestamp = 0;
    const RuntimeConfig &flightConfig = runtime_config_get_flight_snapshot();

    while (running)
    {
        esp_task_wdt_reset();
        if(!running) break;

        PressureSensorData baroData;
#ifdef BARO_1
        SensorReadStatus baro_status = _rocketModel->getMS561101BA03Data_1(baroData);
#else
        SensorReadStatus baro_status = _rocketModel->getMS561101BA03Data_2(baroData);
#endif
        // Reject identical simulated packets
        if ((baro_status != SensorReadStatus::OK) || baroData.pressure <= 0.0f || baroData.timestamp == lastTimestamp) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        const uint32_t sampleDeltaMs = (lastTimestamp == 0) ? 0U : (baroData.timestamp - lastTimestamp);
        lastTimestamp = baroData.timestamp;
        float rawPressure = baroData.pressure;

        // Physics Lock, if the pressure change is too extreme, clamp it 
        // to a maximum plausible change based on physical limits of the 
        // atmosphere and the sampling rate (prevents spikes instability errors)
        if (_lastValidPressure < 0.0f) {
            _lastValidPressure = rawPressure;
        } else {
            float deltaP = rawPressure - _lastValidPressure;
            
            // Clamp the pressure change to physical reality
            if (deltaP > MAX_DELTA_P_PER_TICK) {
                rawPressure = _lastValidPressure + MAX_DELTA_P_PER_TICK;
            } else if (deltaP < -MAX_DELTA_P_PER_TICK) {
                rawPressure = _lastValidPressure - MAX_DELTA_P_PER_TICK;
            }
            _lastValidPressure = rawPressure;
        }
        
        // Median Filter (removes isolated outliers)
        float filteredPressure = pressureFilter.update(rawPressure);

        if (!pressureFilter.isReady()) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        
        // Altitude Calculation
        float currentAltitude = 0.0f;
        if (_rocketModel->isBarometerZeroed()) {
            currentAltitude = calculateAltitude(filteredPressure, _rocketModel->getLaunchpadBasePressure());
        } else {
            const float seaLevelPressurePa = flightConfig.sea_level_pressure_hpa * 100.0f;
            currentAltitude = calculateAltitude(filteredPressure, seaLevelPressurePa) - flightConfig.launch_site_altitude_m;
        }

        // Apogee & Trend Detection
        updateRisingTrend(currentAltitude, baroData.timestamp);
        
        if (currentAltitude > _max_altitude_read) {
            _max_altitude_read = currentAltitude;
        }

        // Update Model
        _rocketModel->setCurrentHeight(currentAltitude);
        
        float currentVelocity = apogeeDetector.getVelocity();

        _rocketModel->setHeightGainSpeed(currentVelocity);

        LOG_EVERY_MS(ALTITUDE_LOG_PERIOD_MS, INFO, "AltitudeTask", "Alt: %0.2f m | Vz: %0.2f m/s | Max: %0.2f m",
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

void AltitudeTask::updateRisingTrend(float currentAltitude, uint32_t timestampMs)
{
    apogeeDetector.update(currentAltitude, timestampMs);

    _belowThresholdCount = apogeeDetector.isRising() ? 0 : (_belowThresholdCount + 1);
    const bool confirmedDescending = _belowThresholdCount >= APOGEE_CONFIRM_SAMPLES;

    _rocketModel->setIsRising(!confirmedDescending);
}

void AltitudeTask::onTaskStart()
{
    // Clear state left over from a previous run so a HIL/ground-test restart
    // (no power cycle) doesn't inherit filter/detector state from the last flight.
    pressureFilter.reset();
    apogeeDetector.reset();
    _lastValidPressure = -1.0f;
    _max_altitude_read = -1000.0f;
    LOG_INFO("AltitudeTask", "State reset for new run");
}
