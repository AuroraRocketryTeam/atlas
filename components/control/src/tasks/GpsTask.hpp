#pragma once

#include "BaseTask.hpp"
#include <RocketModel.hpp>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <GPS.hpp>
#include "Logger.hpp"
#include <RocketLogger.hpp>

/**
 * @brief Class to implement a GPS task.
 * 
 */
class GpsTask : public BaseTask
{
public:
    /**
     * @brief Construct a new Gps Task object
     * 
     * @param rocketModel The shared pointer to the rocket model
     * @param modelMutex The semaphore handle to protect access to the model
     * @param logger The shared pointer to the RocketLogger instance
     * @param loggerMutex The semaphore handle to protect access to the logger
     */
    GpsTask(std::shared_ptr<RocketModel> rocketModel,
            SemaphoreHandle_t modelMutex,
            std::shared_ptr<RocketLogger> logger, 
            SemaphoreHandle_t loggerMutex
        );

    ~GpsTask() override
    {
        stop();
    }

protected:
    void taskFunction() override;
    void onTaskStart() override;
    void onTaskStop() override;
private:
    std::shared_ptr<RocketModel> _rocketModel;
    SemaphoreHandle_t _modelMutex;

    std::shared_ptr<RocketLogger> _logger;
    SemaphoreHandle_t _loggerMutex;
};