#include "AltitudeTask.hpp"
#include <cmath>
#include <config.h>
#include <SerialLogger.hpp>
#include "RuntimeConfig.hpp"
#include "RocketLogger.hpp"
#include "AltitudeData.hpp"
#include <LIS3DHTRSensor.hpp>

static constexpr uint32_t ALTITUDE_LOG_PERIOD_MS = 1000;
static constexpr uint32_t ALTITUDE_DIAGNOSTIC_PERIOD_MS = 1000;
static constexpr uint32_t UNUSUAL_BAROMETER_DT_MS = 75;
// Flight-recorder diagnostic rate: log every processed altitude sample by default.
// Increase this value to reduce flash/SPI load without changing estimator behavior.
static constexpr uint32_t ALTITUDE_LOG_INTERVAL_SAMPLES = 1;
static_assert(ALTITUDE_LOG_INTERVAL_SAMPLES > 0);
// Readings this close to full scale are treated as clipped.
static constexpr float ACCEL_SATURATION_FRACTION = 0.98f;

void AltitudeTask::taskFunction()
{
    LOG_INFO("AltitudeTask", "Starting Altitude task pipeline...");

    uint32_t lastTimestamp = 0;
    uint32_t altitudeLogSampleCounter = 0;
    RuntimeConfig runtimeConfig = runtime_config_get_flight_snapshot();
    TickType_t taskPeriod = pdMS_TO_TICKS(static_cast<uint32_t>(1000.0f / runtimeConfig.altitude.apogee_sample_rate_hz));
    TickType_t lastWakeTime = xTaskGetTickCount();
    apogeeDetector.configure(runtimeConfig.altitude.apogee_window_size,
                             runtimeConfig.altitude.apogee_trigger_velocity_mps,
                             runtimeConfig.altitude.apogee_confirmation_windows);
    LOG_INFO("AltitudeTask", "Apogee detection: first of barometer and accelerometer; accelerometer arms with the flight config lock");
    
    // Baseline for the slew rate limiter
    float lastValidPressure = -1.0f;

    while (running)
    {
        esp_task_wdt_reset();
        if(!running) break;

        if (runtime_config_get_flight_revision() != runtimeConfig.config_revision) {
            const RuntimeConfig updatedConfig = runtime_config_get_flight_snapshot();
            const bool filterChanged =
                updatedConfig.altitude.filter_window != runtimeConfig.altitude.filter_window;
            const bool barometerChanged =
                updatedConfig.altitude.selected_barometer != runtimeConfig.altitude.selected_barometer;
            const bool detectorChanged = filterChanged || barometerChanged ||
                updatedConfig.altitude.apogee_window_size != runtimeConfig.altitude.apogee_window_size ||
                updatedConfig.altitude.apogee_trigger_velocity_mps != runtimeConfig.altitude.apogee_trigger_velocity_mps ||
                updatedConfig.altitude.apogee_confirmation_windows != runtimeConfig.altitude.apogee_confirmation_windows ||
                updatedConfig.mission.sea_level_pressure_hpa != runtimeConfig.mission.sea_level_pressure_hpa ||
                updatedConfig.mission.launch_site_altitude_m != runtimeConfig.mission.launch_site_altitude_m;
            const bool periodChanged =
                updatedConfig.altitude.apogee_sample_rate_hz != runtimeConfig.altitude.apogee_sample_rate_hz;

            runtimeConfig = updatedConfig;
            if (filterChanged || barometerChanged) pressureFilter.reset();
            if (barometerChanged) {
                lastTimestamp = 0;
                lastValidPressure = -1.0f;
            }
            if (detectorChanged) {
                apogeeDetector.configure(runtimeConfig.altitude.apogee_window_size,
                                         runtimeConfig.altitude.apogee_trigger_velocity_mps,
                                         runtimeConfig.altitude.apogee_confirmation_windows);
                _max_altitude_read = -1000.0f;
            }
            if (periodChanged) {
                taskPeriod = pdMS_TO_TICKS(static_cast<uint32_t>(1000.0f / runtimeConfig.altitude.apogee_sample_rate_hz));
                lastWakeTime = xTaskGetTickCount();
            }
            LOG_INFO("AltitudeTask", "Applied RuntimeConfig revision %lu",
                     static_cast<unsigned long>(runtimeConfig.config_revision));
        }

        // The inertial detector must not wait for barometer samples.
        updateAccelerometerApogee();

        PressureSensorData baroData;
        SensorReadStatus baro_status = runtimeConfig.altitude.selected_barometer == 1
            ? _rocketModel->getMS561101BA03Data_1(baroData)
            : _rocketModel->getMS561101BA03Data_2(baroData);
        // Reject identical simulated packets
        if ((baro_status != SensorReadStatus::OK) || baroData.pressure <= 0.0f || baroData.timestamp == lastTimestamp) {
            vTaskDelayUntil(&lastWakeTime, taskPeriod);
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
            vTaskDelayUntil(&lastWakeTime, taskPeriod);
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

        _rocketModel->setHeightGainSpeed(currentVelocity, baroData.timestamp);

        // Preserve the exact data consumed by the apogee detector so flight
        // analysis can compare raw pressure with the filtered estimator input.
        if (_logger && ++altitudeLogSampleCounter >= ALTITUDE_LOG_INTERVAL_SAMPLES) {
            altitudeLogSampleCounter = 0;
            _logger->logSensorData(AltitudeData{
                filteredPressure,
                currentAltitude,
                currentVelocity,
                apogeeDetector.isReady(),
                apogeeDetector.isRising(),
                baroData.timestamp,
                accelApogeeDetector.getVelocity(),
                accelApogeeDetector.isValid(),
                accelApogeeDetector.isRising(),
            });
        }

        LOG_EVERY_MS(ALTITUDE_LOG_PERIOD_MS, INFO, "AltitudeTask", "Alt: %0.2f m | Vz: %0.2f m/s | Max: %0.2f m | Accel Vz: %0.2f m/s%s",
                     currentAltitude, currentVelocity, _max_altitude_read,
                     accelApogeeDetector.getVelocity(), accelApogeeDetector.isValid() ? "" : " (invalid)");

        const uint32_t sampleAgeMs = Utils::millis() - baroData.timestamp;
        LOG_EVERY_MS(ALTITUDE_DIAGNOSTIC_PERIOD_MS, INFO, "AltitudeTask", "Baro timing: dt=%lu ms age=%lu ms Vz=%0.2f m/s%s",
                     static_cast<unsigned long>(sampleDtMs),
                     static_cast<unsigned long>(sampleAgeMs),
                     currentVelocity,
                     sampleDtMs > UNUSUAL_BAROMETER_DT_MS ? " (delayed)" : "");

        vTaskDelayUntil(&lastWakeTime, taskPeriod);
    }
}

void AltitudeTask::onTaskStart()
{
    // Clear state left over from a previous run so a HIL/ground-test restart
    // (no power cycle) doesn't inherit filter state from the last flight.
    pressureFilter.reset();
    apogeeDetector.reset();
    accelApogeeDetector.reset();
    _accelArmed = false;
    _accelWasLaunched = false;
    _accelWasRising = true;
    _accelLastFault = AccelerometerApogeeDetector::Fault::None;
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
    publishRisingState();
}

void AltitudeTask::armAccelerometerApogee()
{
    // Configure from the locked snapshot, which the FSM also uses in flight.
    const RuntimeConfig runtimeConfig = runtime_config_get_flight_snapshot();

    // Liftoff uses the FSM's thresholds so both latch on the same event.
    accelApogeeDetector.configure(runtimeConfig.flight.liftoff_accel_threshold_mps2,
                                  runtimeConfig.flight.liftoff_timeout_ms,
                                  runtimeConfig.apogee_detection.accel_trigger_velocity_mps,
                                  runtimeConfig.apogee_detection.accel_confirmation_samples,
                                  ACCEL_SATURATION_FRACTION * LIS3DHTRSensor::FULL_SCALE_MPS2);
    float padX = 0.0f, padY = 0.0f, padZ = 0.0f;
    if (_rocketModel->getLaunchpadSpecificForce(padX, padY, padZ)) {
        accelApogeeDetector.seedPadReference(padX, padY, padZ);
    }
    _accelArmed = true;

    char message[96];
    snprintf(message, sizeof(message), "Accelerometer apogee armed; pad reference %s",
             accelApogeeDetector.hasPadReference() ? "seeded from calibration" : "collecting on pad");
    LOG_INFO("AltitudeTask", "%s", message);
    if (_logger) _logger->logInfo("AltitudeTask", message);
}

void AltitudeTask::updateAccelerometerApogee()
{
    // Ground handling must not latch liftoff: stay idle until the FSM can
    // detect liftoff too, which needs the flight config lock.
    if (!_accelArmed) {
        if (!runtime_config_is_locked()) return;
        armAccelerometerApogee();
    }

    AccelerometerSensorData accelData;
    if (_rocketModel->getLIS3DHTRData(accelData) != SensorReadStatus::OK) return;

    // The detector ignores a repeated timestamp, so polling faster than the
    // sensor updates is harmless.
    accelApogeeDetector.update(accelData.acceleration_x,
                               accelData.acceleration_y,
                               accelData.acceleration_z,
                               accelData.timestamp);
    reportAccelerometerApogeeTransitions();
    publishRisingState();
}

void AltitudeTask::reportAccelerometerApogeeTransitions()
{
    char message[96];

    if (!_accelWasLaunched && accelApogeeDetector.isLaunched()) {
        _accelWasLaunched = true;
        // Repeats the arming state: pre-launch log entries can be overwritten
        // before the flight recorder starts.
        snprintf(message, sizeof(message), "Accelerometer detector latched liftoff; pad reference %s",
                 accelApogeeDetector.hasPadReference() ? "ok" : "missing");
        LOG_INFO("AltitudeTask", "%s", message);
        if (_logger) _logger->logInfo("AltitudeTask", message);
    }

    const AccelerometerApogeeDetector::Fault fault = accelApogeeDetector.getFault();
    if (fault != _accelLastFault) {
        _accelLastFault = fault;
        snprintf(message, sizeof(message), "Accelerometer apogee invalid: %s; barometer only",
                 AccelerometerApogeeDetector::faultName(fault));
        LOG_WARNING("AltitudeTask", "%s", message);
        if (_logger) _logger->logWarning("AltitudeTask", message);
    }

    if (_accelWasRising && !accelApogeeDetector.isRising()) {
        _accelWasRising = false;
        snprintf(message, sizeof(message), "Accelerometer apogee at Vz=%.2f m/s (%s)",
                 static_cast<double>(accelApogeeDetector.getVelocity()),
                 accelApogeeDetector.isValid() ? "valid" : "invalid");
        LOG_INFO("AltitudeTask", "%s", message);
        if (_logger) _logger->logInfo("AltitudeTask", message);
    }
}

void AltitudeTask::publishRisingState()
{
    // Either detector may end the ascent. The accelerometer votes only while
    // its integral covers the whole ascent from rest.
    const bool barometerApogee = !apogeeDetector.isRising();
    const bool accelerometerApogee = accelApogeeDetector.isValid() && !accelApogeeDetector.isRising();
    uint8_t votes = 0;
    if (barometerApogee) votes |= RocketModel::APOGEE_VOTE_BAROMETER;
    if (accelerometerApogee) votes |= RocketModel::APOGEE_VOTE_ACCELEROMETER;
    // Votes first: the FSM reads them after seeing isRising drop.
    _rocketModel->setApogeeVotes(votes);
    _rocketModel->setIsRising(votes == 0);
}
