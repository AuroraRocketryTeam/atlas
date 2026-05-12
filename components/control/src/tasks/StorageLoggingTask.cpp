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
}

void StorageLoggingTask::taskFunction() {
    while (running) {
        esp_task_wdt_reset();

        if (!running) break;

        storageInitialized = rocketModel && rocketModel->isStorageInitialized();
        int currentLogCount = logger ? logger->getLogCount() : 0;

        if (currentLogCount >= BATCH_SIZE) {
            if (storageInitialized && running) {
                LOG_INFO("Storage", "Creating unique filename for batch...");
                String filename;
                int currentFileCounter = file_counter;

                do {
                    filename = "JSON_data_" + String(currentFileCounter) + ".json";
                    currentFileCounter++;
                } while (rocketModel->storageFileExists(filename.c_str()) && running);

                if (!running) break;

                file_counter = currentFileCounter;
                char* dataToWrite = nullptr;

                // Limit the number of entries serialized to BATCH_SIZE so we don't run out of memory
                if (!logger || !logger->consumeAllAsJsonChar(&dataToWrite, BATCH_SIZE)) {
                    continue;
                }

                if (!running) break;

                if (!rocketModel->storageWriteFile(filename.c_str(), dataToWrite)) {
                    LOG_ERROR("StorageLoggingTask", "Failed to write batch to file.");
                }

                if (dataToWrite != nullptr) {
                    free(dataToWrite);
                }
            } else if (!storageInitialized) {
                if (logger) {
                    logger->clearData();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
