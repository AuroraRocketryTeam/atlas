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
    if (pendingDataToWrite != nullptr) {
        free(pendingDataToWrite);
        pendingDataToWrite = nullptr;
    }
}

void StorageLoggingTask::taskFunction() {
    while (running) {
        esp_task_wdt_reset();

        if (!running) break;

        storageInitialized = rocketModel && rocketModel->isStorageInitialized();
        int currentLogCount = logger ? logger->getLogCount() : 0;

        // Trigger write if we hit batch size OR if we have leftover data from a failed write
        if (currentLogCount >= BATCH_SIZE || pendingDataToWrite != nullptr) {
            if (storageInitialized && running) {
                
                if (pendingDataToWrite == nullptr) {
                    if (!logger || !logger->consumeAllAsJsonChar(&pendingDataToWrite, BATCH_SIZE)) {
                        continue;
                    }
                }

                if (!running) break;

                // We use storageAppendFile to append the data block to the end of the JSONL file
                if (!rocketModel->storageAppendFile(TELEMETRY_FILENAME, pendingDataToWrite)) {
                    LOG_ERROR("StorageLoggingTask", "Failed to append batch to JSONL. Retaining data in memory for retry.");
                } else {
                    free(pendingDataToWrite);
                    pendingDataToWrite = nullptr;
                }
            } else if (!storageInitialized) {
                if (logger) {
                    logger->clearData();
                }
                // Flush memory if storage completely disconnects to prevent memory leaks
                if (pendingDataToWrite != nullptr) {
                    free(pendingDataToWrite);
                    pendingDataToWrite = nullptr;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}