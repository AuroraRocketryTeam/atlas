#include "AirbrakesTask.hpp"
#include "esp_task_wdt.h"

static const char* TAG = "Airbrakes";

namespace
{
    static constexpr float AIRBRAKES_OPEN_ALTITUDE_M  = 1000.0f;
    static constexpr float AIRBRAKES_CLOSE_ALTITUDE_M = 2000.0f;

    static constexpr uint32_t TASK_PERIOD_MS = 50;
    static constexpr float TASK_PERIOD_SEC   = TASK_PERIOD_MS / 1000.0f;

    /*
     * Deployment command is normalized:
     *
     *   0.0f = fully closed
     *   1.0f = fully open
     *
     * Rate is expressed in normalized units per second.
     *
     * Examples:
     *   1.0f -> full 0 to 1 movement in 1 second
     *   0.5f -> full 0 to 1 movement in 2 seconds
     *   2.0f -> full 0 to 1 movement in 0.5 seconds
     */
    static constexpr float AIRBRAKES_OPEN_RATE_PER_SEC  = 0.5f;
    static constexpr float AIRBRAKES_CLOSE_RATE_PER_SEC = 1.0f;

    enum class AirbrakesState
    {
        WAITING_TO_OPEN,
        OPEN,
        CLOSED_FOREVER
    };

    float clamp01(float value)
    {
        if (value > 1.0f) return 1.0f;
        if (value < 0.0f) return 0.0f;
        return value;
    }

    float moveTowards(float current, float target, float maxStep)
    {
        current = clamp01(current);
        target  = clamp01(target);

        if (current < target)
        {
            current += maxStep;

            if (current > target)
            {
                current = target;
            }
        }
        else if (current > target)
        {
            current -= maxStep;

            if (current < target)
            {
                current = target;
            }
        }

        return clamp01(current);
    }
}

AirbrakesTask::AirbrakesTask(std::shared_ptr<RocketModel> rocketModel,
                             SemaphoreHandle_t modelMutex,
                             std::shared_ptr<RocketLogger> logger)
    : BaseTask("AirbrakesTask"),
      _rocketModel(rocketModel),
      _modelMutex(modelMutex),
      _logger(logger)
{
    // LOG_INFO(TAG, "Constructor initialized");
}

void AirbrakesTask::onTaskStart()
{
    // LOG_INFO(TAG, "onTaskStart");
}

void AirbrakesTask::onTaskStop()
{
    // LOG_INFO(TAG, "onTaskStop");
}

void AirbrakesTask::taskFunction()
{
    AirbrakesState airbrakesState = AirbrakesState::WAITING_TO_OPEN;

    float targetDeployment = 0.0f;

    while (running)
    {
        esp_task_wdt_reset();

        float altitude = 0.0f;
        float currentLevel = 0.0f;

        /* ===== READ MODEL ===== */
        if (_rocketModel &&
            _modelMutex &&
            xSemaphoreTake(_modelMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            auto alt_ptr = _rocketModel->getCurrentHeight();

            if (alt_ptr)
            {
                altitude = *alt_ptr;
            }

            currentLevel = _rocketModel->getCommand().getAirbrakes();

            xSemaphoreGive(_modelMutex);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        currentLevel = clamp01(currentLevel);

        /* ===== AIRBRAKES FSM: DECIDE TARGET ===== */
        switch (airbrakesState)
        {
            case AirbrakesState::WAITING_TO_OPEN:
            {
                targetDeployment = 0.0f;

                if (altitude >= AIRBRAKES_OPEN_ALTITUDE_M)
                {
                    LOG_INFO(TAG, "opening airbrakes");

                    targetDeployment = 1.0f;
                    airbrakesState = AirbrakesState::OPEN;
                }

                break;
            }

            case AirbrakesState::OPEN:
            {
                targetDeployment = 1.0f;

                if (altitude >= AIRBRAKES_CLOSE_ALTITUDE_M)
                {
                    LOG_INFO(TAG, "closing airbrakes");

                    targetDeployment = 0.0f;
                    airbrakesState = AirbrakesState::CLOSED_FOREVER;
                }

                break;
            }

            case AirbrakesState::CLOSED_FOREVER:
            {
                targetDeployment = 0.0f;
                break;
            }
        }

        targetDeployment = clamp01(targetDeployment);

        /* ===== SLEW-RATE LIMIT COMMAND ===== */
        const float ratePerSec =
            targetDeployment > currentLevel
                ? AIRBRAKES_OPEN_RATE_PER_SEC
                : AIRBRAKES_CLOSE_RATE_PER_SEC;

        const float maxStep = ratePerSec * TASK_PERIOD_SEC;

        const float newDeployment = moveTowards(
            currentLevel,
            targetDeployment,
            maxStep
        );

        // TODO: we still need real actuation!

        /* ===== WRITE COMMAND ===== */
        if (_rocketModel && _modelMutex && xSemaphoreTake(_modelMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            _rocketModel->setAirbrakesCommand(newDeployment);
            xSemaphoreGive(_modelMutex);
        }

        /*
         * Optional debug:
         *
         * LOG_INFO(TAG,
         *          "alt=%.1f current=%.2f target=%.2f cmd=%.2f state=%d",
         *          altitude,
         *          currentLevel,
         *          targetDeployment,
         *          newDeployment,
         *          static_cast<int>(airbrakesState));
         */

        vTaskDelay(pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}