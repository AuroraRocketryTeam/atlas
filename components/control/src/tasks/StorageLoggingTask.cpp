#include "StorageLoggingTask.hpp"
#include "esp_task_wdt.h"

StorageLoggingTask::StorageLoggingTask(std::shared_ptr<RocketModel> rocketModel,
                       std::shared_ptr<RocketLogger> logger)
    : BaseTask("StorageLoggingTask"),
      rocketModel(rocketModel),
      logger(logger)
{
    storageInitialized = this->rocketModel && this->rocketModel->isStorageInitialized();

    if (!storageInitialized) {
        LOG_INFO("StorageLoggingTask", "Storage initialization failed or no storage provided!");
    } else {
        LOG_INFO("StorageLoggingTask", "Storage initialized successfully. Ready for JSONL logging.");
    }
}

StorageLoggingTask::~StorageLoggingTask() {
    stop();
}

void StorageLoggingTask::taskFunction() {
    TickType_t lastWriteTicks = xTaskGetTickCount();

    while (running) {
        esp_task_wdt_reset();

        storageInitialized = rocketModel && rocketModel->isStorageInitialized();

        // Handle Storage Disconnection
        if (!storageInitialized) {            
            // Drop pending old data
            pendingBytesToWrite = 0;
            
            vTaskDelay(pdMS_TO_TICKS(100)); 
            continue;
        }

        // Evaluate if we need to trigger a write cycle
        int currentLogCount = logger ? logger->getLogCount() : 0;
        TickType_t currentTicks = xTaskGetTickCount();
        bool timeoutReached = (currentTicks - lastWriteTicks) * portTICK_PERIOD_MS >= FLUSH_TIMEOUT_MS;

        // We write if we have leftover data, OR if data is getting stale, OR if we have "enough" logs waiting
        bool shouldWrite = (pendingBytesToWrite > 0) || 
                        (timeoutReached && currentLogCount > 0) ||
                        (currentLogCount >= 10);

        // Execute Write
        if (shouldWrite && running) {
            LOG_DEBUG("StorageLoggingTask", "Initiating write cycle. pendingBytesToWrite=%zu, currentLogCount=%d\n", pendingBytesToWrite, currentLogCount);
            if (pendingBytesToWrite == 0 && logger) {
                // Just hand over the entire available buffer space!
                pendingBytesToWrite = logger->consumeBatch(writeBuffer, WRITE_BUFFER_SIZE);
            }

            if (pendingBytesToWrite > 0) {
                if (rocketModel->storageAppendFile(TELEMETRY_FILENAME, writeBuffer, pendingBytesToWrite)) {
                    pendingBytesToWrite = 0;
                    lastWriteTicks = xTaskGetTickCount();
                } else {
                    LOG_DEBUG("StorageLoggingTask", "Write failed. Retrying next loop.");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}