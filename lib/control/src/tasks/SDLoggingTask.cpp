#include "SDLoggingTask.hpp"

SDLoggingTask::SDLoggingTask(std::shared_ptr<RocketLogger> rocketLogger, 
                               SemaphoreHandle_t loggerMutex,
                               std::shared_ptr<SD> sdCard)
    : BaseTask("SDLoggingTask"),
      rocketLogger(rocketLogger),
      loggerMutex(loggerMutex),
      sdCard(sdCard)
{
    sdInitialized = static_cast<bool>(sdCard);
    
    if (!sdInitialized) {
        LOG_INFO("SDLoggingTask", "SD card Failed!");
        if (xSemaphoreTake(loggerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            rocketLogger->logError("SD card initialization failed!");
            xSemaphoreGive(loggerMutex);
        } else {
            LOG_ERROR("SDLoggingTask", "Failed to acquire mutex for logging SD error");
        }
    } else {
        LOG_INFO("SDLoggingTask", "SD card initialized successfully.");
        if (xSemaphoreTake(loggerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
            rocketLogger->logInfo("SD card initialized successfully.");
            xSemaphoreGive(loggerMutex);
        } else {
            LOG_ERROR("SDLoggingTask", "Failed to acquire mutex for logging SD success");
        }
    }
}

SDLoggingTask::~SDLoggingTask() {
    stop();
}

void SDLoggingTask::taskFunction() {
    LOG_INFO("SDLoggingTask", "Task started, batch size: %d", BATCH_SIZE);
    
    while (true) {
        size_t currentLogCount = rocketLogger->getLogCount();
        
        if (currentLogCount >= BATCH_SIZE) {
            LOG_INFO("SDLoggingTask", "Batch size reached (%zu >= %d), processing...", currentLogCount, BATCH_SIZE);
            
            if (sdInitialized) {
                
                // Create a unique filename for this batch, checking if file already exists
                // note: the check should not be useful, but if the system somehow restarts, the check could prevent overwriting
                String filename;
                int currentFileCounter = file_counter;
                do {
                    filename = "JSON_data_" + String(currentFileCounter) + ".json";
                    currentFileCounter++;
                } while (sdCard->fileExists(filename.c_str()));
                
                // Update file_counter to the next available number
                file_counter = currentFileCounter;

                std::string dataToWrite = "";
                if (xSemaphoreTake(loggerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    // Convert the entire batch of JSON data to a string
                    dataToWrite = rocketLogger->getJSONAll().dump();
                    rocketLogger->clearData();
                    xSemaphoreGive(loggerMutex);
                } else {
                    LOG_ERROR("SDLoggingTask", "Failed to acquire logger mutex for SD write");
                    continue; // Skip this iteration if we can't get the mutex
                }

                // Write the string to the SD card
                // Serial.println("Opening file...");
                sdCard->openFile(filename.c_str());

                // Serial.println("Writing to SD card...");
                if (!sdCard->writeFile(filename.c_str(), dataToWrite)) {
                    rocketLogger->logError("Failed to write batch to SD card.");
                    LOG_ERROR("SDLoggingTask", "Failed to write batch to file: %s", filename.c_str());
                } else {
                    LOG_INFO("SDLoggingTask", "Successfully wrote batch to file: %s (size: %zu bytes)", filename.c_str(), dataToWrite.length());
                }
                
                sdCard->closeFile();                
            } else {
                if (xSemaphoreTake(loggerMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
                    rocketLogger->clearData();
                    xSemaphoreGive(loggerMutex);
                } else {
                    LOG_ERROR("SDLoggingTask", "Failed to acquire logger mutex for clearing data (SD not initialized)");
                }   
            }
        } else {
            // Periodic logging to show the task is running
            static unsigned long lastPeriodicLog = 0;
            if (millis() - lastPeriodicLog > 5000) { // Log every 5 seconds
                LOG_DEBUG("SDLoggingTask", "Task running - Log count: %zu/%d", currentLogCount, BATCH_SIZE);
                lastPeriodicLog = millis();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}