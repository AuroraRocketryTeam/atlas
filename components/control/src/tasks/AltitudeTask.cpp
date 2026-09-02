#include "AltitudeTask.hpp"
#include <cmath>
#include <config.h>
#include <SerialLogger.hpp>
#include "RuntimeConfig.hpp"

static constexpr uint32_t ALTITUDE_LOG_PERIOD_MS = 1000;
static constexpr uint32_t ALTITUDE_DIAGNOSTIC_PERIOD_MS = 1000;
static constexpr uint32_t UNUSUAL_BAROMETER_DT_MS = 75;

void AltitudeTask::taskFunction()
{
    LOG_INFO("AltitudeTask", "Starting Altitude task pipeline...");

    uint32_t lastTimestamp = 0;
    const RuntimeConfig runtimeConfig = runtime_config_get_flight_snapshot();
    const TickType_t taskPeriod = pdMS_TO_TICKS(static_cast<uint32_t>(1000.0f / runtimeConfig.altitude.apogee_sample_rate_hz));
    apogeeDetector.configure(runtimeConfig.altitude.apogee_window_size,
                             runtimeConfig.altitude.apogee_trigger_velocity_mps,
                             runtimeConfig.altitude.apogee_confirmation_windows);
    
    // Baseline for the slew rate limiter
    float lastValidPressure = -1.0f;

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
        
        const uint32_t sampleDtMs = lastTimestamp == 0 ? 0 : baroData.timestamp - lastTimestamp;
        lastTimestamp = baroData.timestamp;
        float rawPressure = baroData.pressure;

        // Clamp by elapsed time so a delayed SPI sample retains its legitimate pressure change.
        if (lastValidPressure < 0.0f) {
            lastValidPressure = rawPressure;
        } else {
            float deltaP = rawPressure - lastValidPressure;
            const float maxDeltaPa = runtimeConfig.altitude.max_pressure_rate_pa_per_s *
                                     (static_cast<float>(sampleDtMs) / 1000.0f);
            if (deltaP > maxDeltaPa) {
                rawPressure = lastValidPressure + maxDeltaPa;
            } else if (deltaP < -maxDeltaPa) {
                rawPressure = lastValidPressure - maxDeltaPa;
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

        const uint32_t sampleAgeMs = Utils::millis() - baroData.timestamp;
        LOG_EVERY_MS(ALTITUDE_DIAGNOSTIC_PERIOD_MS, INFO, "AltitudeTask", "Baro timing: dt=%lu ms age=%lu ms Vz=%0.2f m/s%s",
                     static_cast<unsigned long>(sampleDtMs),
                     static_cast<unsigned long>(sampleAgeMs),
                     currentVelocity,
                     sampleDtMs > UNUSUAL_BAROMETER_DT_MS ? " (delayed)" : "");

        vTaskDelay(taskPeriod);
    }
}

void AltitudeTask::onTaskStart()
{
    // Clear state left over from a previous run so a HIL/ground-test restart
    // (no power cycle) doesn't inherit filter state from the last flight.
    pressureFilter.reset();
    apogeeDetector.reset();
    _max_altitude_read = -1000.0f;
    LOG_INFO("AltitudeTask", "State reset for new run");
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

void AltitudeTask::updateRisingTrend(float currentAltitude, uint32_t timestamp)
{
    apogeeDetector.update(currentAltitude, timestamp);
    _rocketModel->setIsRising(apogeeDetector.isRising());
}
