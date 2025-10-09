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
    while (true) {
        if (rocketLogger->getLogCount() >= BATCH_SIZE) {
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
                if (xSemaphoreTake(loggerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    // Convert the entire batch of JSON data to a string
                    dataToWrite = rocketLogger->getJSONAll().dump();
                    rocketLogger->clearData();
                    xSemaphoreGive(loggerMutex);
                }

                // Write the string to the SD card
                // Serial.println("Opening file...");
                sdCard->openFile(filename.c_str());

                // Serial.println("Writing to SD card...");
                if (!sdCard->writeFile(filename.c_str(), dataToWrite)) {
                    rocketLogger->logError("Failed to write batch to SD card.");
                }
                
                sdCard->closeFile();                
            } else {
                if (xSemaphoreTake(loggerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    rocketLogger->clearData();
                    xSemaphoreGive(loggerMutex);
                }   
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}