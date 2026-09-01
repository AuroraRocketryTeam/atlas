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
    uint32_t last_packet_log_ms = Utils::realMillis() - ALTITUDE_LOG_PERIOD_MS;
    
    const RuntimeConfig runtimeConfig = runtime_config_get_flight_snapshot();
    const TickType_t taskPeriod = pdMS_TO_TICKS(static_cast<uint32_t>(1000.0f / runtimeConfig.altitude.apogee_sample_rate_hz));
    
    // Baseline for the slew rate limiter
    static float lastValidPressure = -1.0f;

    while (running)
    {
        esp_task_wdt_reset();
        if(!running) break;

        PressureSensorData baroData;
        SensorReadStatus baro_status = runtimeConfig.altitude.selected_barometer == 1
            ? _rocketModel->getMS561101BA03Data_1(baroData)
            : _rocketModel->getMS561101BA03Data_2(baroData);
        // Reject identical simulated packets
        if ((baro_status != SensorReadStatus::OK) || baroData.pressure <= 0.0f || baroData.timestamp == lastTimestamp) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        lastTimestamp = baroData.timestamp;
        float rawPressure = baroData.pressure;

        // Clamp pressure steps to suppress corrupt sensor samples.
        if (lastValidPressure < 0.0f) {
            lastValidPressure = rawPressure;
        } else {
            float deltaP = rawPressure - lastValidPressure;
            if (deltaP > MAX_DELTA_P_PER_TICK) {
                rawPressure = lastValidPressure + MAX_DELTA_P_PER_TICK;
            } else if (deltaP < -MAX_DELTA_P_PER_TICK) {
                rawPressure = lastValidPressure - MAX_DELTA_P_PER_TICK;
            }
            lastValidPressure = rawPressure;
        }
        
        // Median Filter (removes isolated outliers)
        float filteredPressure = pressureFilter.update(rawPressure, runtimeConfig.altitude.filter_window);

        if (!pressureFilter.isReady(runtimeConfig.altitude.filter_window)) {
            vTaskDelay(taskPeriod);
            continue;
        }
        
        // Altitude Calculation
        float currentAltitude = 0.0f;
        if (_rocketModel->isBarometerZeroed()) {
            currentAltitude = calculateAltitude(filteredPressure, _rocketModel->getLaunchpadBasePressure());
        } else {
            const float seaLevelPressurePa = runtimeConfig.mission.sea_level_pressure_hpa * 100.0f;
            currentAltitude = calculateAltitude(filteredPressure, seaLevelPressurePa) - runtimeConfig.mission.launch_site_altitude_m;
        }

        // Apogee & Trend Detection
        updateRisingTrend(currentAltitude);
        
        if (currentAltitude > _max_altitude_read) {
            _max_altitude_read = currentAltitude;
        }

        // Update Model
        _rocketModel->setCurrentHeight(currentAltitude);
        
        float currentVelocity = apogeeDetector.getVelocity();

        _rocketModel->setHeightGainSpeed(currentVelocity);

        const uint32_t now_ms = Utils::realMillis();
        if (now_ms - last_packet_log_ms >= ALTITUDE_LOG_PERIOD_MS) {
            LOG_INFO("AltitudeTask", "Alt: %0.2f m | Vz: %0.2f m/s | Max: %0.2f m", 
                 currentAltitude, currentVelocity, _max_altitude_read);
            last_packet_log_ms = now_ms;
        }

        vTaskDelay(taskPeriod);
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
