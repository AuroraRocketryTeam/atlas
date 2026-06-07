#pragma once

#include "BaseTask.hpp"
#include <RocketModel.hpp>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <GPS.hpp>
#include "SerialLogger.hpp"
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
     * @param logger The shared pointer to the RocketLogger instance
     */
    GpsTask(std::shared_ptr<RocketModel> rocketModel,
            std::shared_ptr<RocketLogger> logger
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

    std::shared_ptr<RocketLogger> _logger;
};