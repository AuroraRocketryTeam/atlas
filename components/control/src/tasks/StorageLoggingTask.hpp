#pragma once

#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "BaseTask.hpp"
#include "RocketModel.hpp"
#include "RocketLogger.hpp"
#include <config.h>
#include <SerialLogger.hpp>

/**
 * @brief Class to implement a generic storage logging task using JSONL architecture.
 */
class StorageLoggingTask : public BaseTask {
public:
    /**
     * @brief Construct a new StorageLoggingTask object
     *
     * @param rocketModel The shared pointer to the rocket model storage wrapper
     * @param logger The shared pointer to the RocketLogger instance
     */
    StorageLoggingTask(std::shared_ptr<RocketModel> rocketModel,
                       std::shared_ptr<RocketLogger> logger);

    ~StorageLoggingTask() override;

protected:
    void taskFunction() override;

private:
    std::shared_ptr<RocketModel> rocketModel;
    std::shared_ptr<RocketLogger> logger;
    bool storageInitialized = false;
    
    // Single append-only file for all flight telemetry
    const char* TELEMETRY_FILENAME = "flight_telemetry.jsonl";
    
    // Buffer to hold data if a write fails
    char* pendingDataToWrite = nullptr; 
};