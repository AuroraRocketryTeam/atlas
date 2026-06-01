#pragma once

#include "BaseTask.hpp"
#include "SharedData.hpp"
#include "Logger.hpp"
#include "RocketLogger.hpp"
#include <ISensor.hpp>
#include <memory>
#include "RocketModel.hpp"

/**
 * @brief Class to implement a Sensor task.
 * 
 */
class SensorTask : public BaseTask
{
public:
    /**
     * @brief Construct a new Sensor Task object
     * 
     * @param rocketModel The shared pointer to the rocket model
     * @param logger The shared pointer to the RocketLogger instance
     */
    SensorTask(std::shared_ptr<RocketModel> rocketModel,
               std::shared_ptr<RocketLogger> logger);

protected:
    void taskFunction() override;
    void onTaskStart() override;
    void onTaskStop() override;


private:
    std::shared_ptr<RocketModel> rocketModel;

    std::shared_ptr<RocketLogger> logger;
};