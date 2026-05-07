#include "AirbrakesTask.hpp"
#include "esp_task_wdt.h"

static const char* TAG = "Airbrakes";

AirbrakesTask::AirbrakesTask(std::shared_ptr<RocketModel> rocketModel,
                             SemaphoreHandle_t modelMutex,
                             std::shared_ptr<RocketLogger> logger,
                             SemaphoreHandle_t loggerMutex)
    : BaseTask("AirbrakesTask"),
      _rocketModel(rocketModel),
      _modelMutex(modelMutex),
      _logger(logger),
      _loggerMutex(loggerMutex)
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
    float deployment = 0.0f;

    while (running)
    {
        esp_task_wdt_reset();

        float altitude = 0.0f;

        /* ===== READ MODEL ===== */
        if (_rocketModel &&
            xSemaphoreTake(_modelMutex, 50 / portTICK_PERIOD_MS) == pdTRUE)
        {
            auto alt_ptr = _rocketModel->getCurrentHeight();
            if (alt_ptr) altitude = *alt_ptr;

            xSemaphoreGive(_modelMutex);
        }
        else
        {
            vTaskDelay(10 / portTICK_PERIOD_MS);
            continue;
        }


        if (altitude > 1000 && deployment < 1) { // open once
            LOG_INFO(TAG, "opening airbrakes");
            deployment = 1.0f;
        }

        if (altitude > 2000 && deployment >= 0.75) { // close once
            LOG_INFO(TAG, "closing airbrakes");
            deployment = 0.0f;
        }

        /* ===== CLAMP ===== */
        if (deployment > 1.0f) deployment = 1.0f;
        if (deployment < 0.0f) deployment = 0.0f;

        /* ===== WRITE COMMAND ===== */
        if (_rocketModel &&
            xSemaphoreTake(_modelMutex, 50 / portTICK_PERIOD_MS) == pdTRUE)
        {
            _rocketModel->setAirbrakesCommand(deployment);
            xSemaphoreGive(_modelMutex);
        }

        /* ===== LOG ===== */
        // LOG_INFO(TAG, "alt=%.1f cmd=%.2f", altitude, deployment);

        vTaskDelay(pdMS_TO_TICKS(50)); // 50ms, 20Hz
    }
}