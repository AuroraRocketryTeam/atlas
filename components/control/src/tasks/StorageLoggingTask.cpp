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
        LOG_INFO("StorageLoggingTask", "Storage initialized successfully.");
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

        if (currentLogCount >= BATCH_SIZE || pendingDataToWrite != nullptr) {
            if (storageInitialized && running) {
                LOG_INFO("Storage", "Creating unique filename for batch...");
                String filename;
                int currentFileCounter = file_counter;

                do {
                    filename = "JSON_data_" + String(currentFileCounter) + ".json";
                    currentFileCounter++;
                } while (rocketModel->storageFileExists(filename.c_str()) && running);

                if (!running) break;

                if (pendingDataToWrite == nullptr) {
                    if (!logger || !logger->consumeAllAsJsonChar(&pendingDataToWrite, BATCH_SIZE)) {
                        continue;
                    }
                }

                if (!running) break;

                if (!rocketModel->storageWriteFile(filename.c_str(), pendingDataToWrite)) {
                    LOG_ERROR("StorageLoggingTask", "Failed to write batch to file. Retaining data in memory for retry.");
                } else {
                    file_counter = currentFileCounter;
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