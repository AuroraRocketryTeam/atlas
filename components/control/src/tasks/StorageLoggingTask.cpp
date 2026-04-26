#include "StorageLoggingTask.hpp"
#include <cstdlib>
#include <stdexcept>
#include "esp_task_wdt.h"

StorageLoggingTask::StorageLoggingTask(std::shared_ptr<RocketModel> rocketModel)
    : BaseTask("StorageLoggingTask"),
            rocketModel(rocketModel)
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
        size_t currentLogCount = rocketModel ? rocketModel->getLogCountThreadSafe() : 0;

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

                try {
                    if (!rocketModel->consumeLogsAsJson(&dataToWrite)) {
                        continue;
                    }
                } catch (const std::exception& e) {
                    LOG_ERROR("StorageLoggingTask", "JSON serialization failed: %s", e.what());
                    continue;
                }

                if (!running) break;

                try {
                    rocketModel->storageOpenFile(filename.c_str());
                    if (!rocketModel->storageWriteFile(filename.c_str(), dataToWrite)) {
                        LOG_ERROR("StorageLoggingTask", "Failed to write batch to file.");
                    }
                    rocketModel->storageCloseFile();
                } catch (const std::exception& e) {
                    LOG_ERROR("StorageLoggingTask", "Storage operation failed: %s", e.what());
                    rocketModel->storageCloseFile();
                }

                if (dataToWrite != nullptr) {
                    free(dataToWrite);
                }
            } else if (!storageInitialized) {
                if (rocketModel) rocketModel->clearLogsThreadSafe();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
