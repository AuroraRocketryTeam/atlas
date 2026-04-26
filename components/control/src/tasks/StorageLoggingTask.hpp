#pragma once

#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "BaseTask.hpp"
#include "RocketModel.hpp"
#include <config.h>
#include <Logger.hpp>

/**
 * @brief Class to implement a generic storage logging task.
 */
class StorageLoggingTask : public BaseTask {
public:
    /**
     * @brief Construct a new StorageLoggingTask object
     *
     * @param logger The shared pointer to the RocketLogger instance
     * @param loggerMutex The semaphore handle to protect access to the logger
     * @param storage The shared pointer to the storage backend
     */
    StorageLoggingTask(std::shared_ptr<RocketModel> rocketModel);

    ~StorageLoggingTask() override;

protected:
    void taskFunction() override;

private:
    std::shared_ptr<RocketModel> rocketModel;
    bool storageInitialized = false;
    int file_counter = 0;
};
