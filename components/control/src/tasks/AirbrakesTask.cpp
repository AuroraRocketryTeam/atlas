#include "AirbrakesTask.hpp"
#include "AirbrakesData.hpp"
#include "RuntimeConfig.hpp"
#include <SerialLogger.hpp>
#include <utils.h>
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include <cstdio>

static const char* TAG = "Airbrakes";

namespace
{
    // Control-command rate.
    static constexpr uint32_t TASK_PERIOD_MS = 50;

    // The vertical velocity is published by AltitudeTask with every barometer
    // sample (~25 Hz), if it stops changing for this long the estimate is not
    // trustworthy, so the brakes are driven closed until it resumes.
    static constexpr uint32_t VELOCITY_STALE_MS = 1000;

    // Log one AIRBRAKES record every N control cycles (10 Hz at 50 ms)
    static constexpr uint32_t LOG_INTERVAL_CYCLES = 2;

    // Warn once if the control law eats most of the control period.
    static constexpr uint32_t COMPUTE_WARNING_US = 40000;

    /*
     * Deployment command is normalized:
     *
     *   0.0f = fully closed
     *   1.0f = fully open
     */
    float clamp01(float value)
    {
        if (!(value > 0.0f)) return 0.0f;
        if (value > 1.0f) return 1.0f;
        return value;
    }

    // Puts the Cd-table load result in the flight log as a system event, so the
    // flash record shows whether the predictive controller could run.
    void logCdTableStatus(RocketLogger &logger, const airbrakes::AeroTableLoad &aero)
    {
        if (!aero.ok) {
            logger.logError(TAG, aero.error);
            return;
        }
        char message[96];
        std::snprintf(message, sizeof(message), "Cd table loaded: %u openings x %u Mach%s",
                      static_cast<unsigned>(aero.table.opening_count), static_cast<unsigned>(aero.table.mach_count),
                      aero.cd_monotonic ? "" : " (Cd not monotonic in opening)");
        logger.logInfo(TAG, message);
    }
}

AirbrakesTask::AirbrakesTask(std::shared_ptr<RocketModel> rocketModel,
                             std::shared_ptr<RocketLogger> logger)
    : BaseTask("AirbrakesTask"),
      _rocketModel(rocketModel),
      _logger(logger)
{
    // Parse the embedded Cd table at boot (tasks are constructed once by
    // TaskManager), so a bad CSV shows up in the boot log
    const airbrakes::AeroTableLoad &aero = airbrakes::embeddedAeroTable();
    if (aero.ok) {
        LOG_INFO(TAG, "Cd table loaded: %u openings x %u Mach breakpoints%s",
                 static_cast<unsigned>(aero.table.opening_count), static_cast<unsigned>(aero.table.mach_count),
                 aero.cd_monotonic ? "" : " (warning: Cd decreases with opening somewhere)");
    } else {
        LOG_ERROR(TAG, "Cd table invalid, airbrakes will stay closed: %s", aero.error);
    }
    // Boot events wait in RAM and are written at the start of the flight file.
    if (_logger) logCdTableStatus(*_logger, aero);
}

void AirbrakesTask::onTaskStart()
{
}

void AirbrakesTask::onTaskStop()
{
    _rocketModel->setAirbrakesCommand(0.0f);
    LOG_INFO(TAG, "airbrakes task stopped, commanding closed (stack free: %lu bytes)",
             static_cast<unsigned long>(uxTaskGetStackHighWaterMark(nullptr)));
}

void AirbrakesTask::taskFunction()
{
    const airbrakes::AeroTableLoad &aeroTable = airbrakes::embeddedAeroTable();
    if (_logger) logCdTableStatus(*_logger, aeroTable);

    if (!aeroTable.ok)
    {
        // Without the Cd table the control law has nothing to run on: hold the
        // brakes closed and only keep the watchdog fed until the FSM stops us.
        _rocketModel->setAirbrakesCommand(0.0f);
        if (_logger) _logger->logError(TAG, "airbrakes disabled for this flight, brakes held closed");
        LOG_ERROR(TAG, "Cd table invalid, airbrakes disabled for this flight");
        while (running)
        {
            esp_task_wdt_reset();
            vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_MS));
        }
        return;
    }

    {
        // Scoped so the full RuntimeConfig copy does not stay on the stack.
        const RuntimeConfig runtimeConfig = runtime_config_get_flight_snapshot();
        const AirbrakesConfig &cfg = runtimeConfig.airbrakes;
        _params.opening_altitude_m = cfg.opening_altitude_m;
        _params.target_apogee_m = cfg.target_apogee_m;
        _params.launch_site_altitude_m = runtimeConfig.mission.launch_site_altitude_m;
        _params.dry_mass_kg = cfg.dry_mass_kg;
        _params.reference_area_m2 = cfg.reference_area_m2;
        _params.prediction_dt_s = cfg.prediction_dt_s;
        _params.prediction_max_steps = cfg.prediction_max_steps;
    }
    _params.aero = &aeroTable.table;

    const TickType_t taskPeriod = pdMS_TO_TICKS(TASK_PERIOD_MS);
    TickType_t lastWakeTime = xTaskGetTickCount();
    TickType_t lastControlTime = lastWakeTime;

    bool openingAltitudeReached = false;
    bool computeWarningIssued = false;
    uint32_t logCycleCounter = 0;

    while (running)
    {
        esp_task_wdt_reset();

        const TickType_t controlTime = xTaskGetTickCount();
        const TickType_t elapsedTicks = controlTime - lastControlTime;
        lastControlTime = controlTime;

        /* ===== READ MODEL ===== */
        const float altitude = _rocketModel->getCurrentHeight();
        const float verticalVelocity = _rocketModel->getHeightGainSpeed();
        const uint32_t velocityTimestamp = _rocketModel->getHeightGainSpeedTimestamp();

        /* ===== OPENING-ALTITUDE GATE ===== */
        // Nothing is computed or commanded until the opening altitude is
        // first reached (Euroc Rules: 1500m)
        if (!openingAltitudeReached)
        {
            if (!(altitude >= _params.opening_altitude_m))
            {
                vTaskDelayUntil(&lastWakeTime, taskPeriod);
                continue;
            }
            openingAltitudeReached = true;
            LOG_INFO(TAG, "opening altitude reached (%.1f m AGL), predictive control active",
                     static_cast<double>(altitude));
        }

        // Same freshness rule as RocketFSM: judge the sample by its own age.
        const bool velocityFresh = velocityTimestamp != 0 &&
                                   Utils::millis() - velocityTimestamp <= VELOCITY_STALE_MS;

        /* ===== CONTROL LAW ===== */
        airbrakes::ControllerOutputs out{};
        uint32_t computeUs = 0;
        if (velocityFresh)
        {
            const int64_t start = esp_timer_get_time();
            out = airbrakes::computeCommand(_params, {altitude, verticalVelocity});
            computeUs = static_cast<uint32_t>(esp_timer_get_time() - start);
        }
        else
        {
            out.armed = false;
        }

        if (computeUs > COMPUTE_WARNING_US && !computeWarningIssued)
        {
            computeWarningIssued = true;
            LOG_WARNING(TAG, "control law took %lu us of the %lu ms period",
                        static_cast<unsigned long>(computeUs), static_cast<unsigned long>(TASK_PERIOD_MS));
        }

        // Opening breakpoints are percent of full travel, so the command is a
        // direct scale of the control-law output.
        const float deployment = out.armed ? clamp01(out.opening_pct / 100.0f) : 0.0f;

        /* ===== WRITE COMMAND ===== */
        _rocketModel->setAirbrakesCommand(deployment);

        /* ===== LOG ===== */
        if (_logger && ++logCycleCounter >= LOG_INTERVAL_CYCLES)
        {
            logCycleCounter = 0;
            AirbrakesData record;
            record.altitude_m = altitude;
            record.vertical_velocity_mps = verticalVelocity;
            record.mach = out.mach;
            record.deployment = deployment;
            record.cd_cmd = out.cd_cmd;
            record.predicted_apogee_m = out.predicted_apogee_m;
            record.time_to_apogee_s = out.time_to_apogee_s;
            record.compute_us = computeUs;
            record.candidates = out.candidates_evaluated;
            record.armed = out.armed;
            record.velocity_fresh = velocityFresh;
            record.timestamp = Utils::millis();
            _logger->logSensorData(record);
        }

        // Re-anchor after a missed period so vTaskDelayUntil() cannot run catch-up
        // iterations back-to-back and spam identical control commands.
        if (elapsedTicks >= taskPeriod * 2) {
            lastWakeTime = controlTime;
        }
        vTaskDelayUntil(&lastWakeTime, taskPeriod);
    }
}
