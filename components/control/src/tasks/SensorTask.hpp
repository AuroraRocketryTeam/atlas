#pragma once

#include "BaseTask.hpp"
#include "SerialLogger.hpp"
#include "RocketLogger.hpp"
#include <ISensor.hpp>
#include <memory>
#include "RocketModel.hpp"
#include "config.h"

// Acquisition and recording cadence. HilSimulationTask mirrors these so a
// simulated run produces the same recording rate as a real sensor run.
static constexpr uint32_t SENSOR_LOOP_RATE_HZ = 50;
static constexpr uint32_t SENSOR_LOG_RATE_HZ = 25;
static constexpr uint32_t SENSOR_LOOP_PERIOD_MS = 1000 / SENSOR_LOOP_RATE_HZ;
static constexpr uint32_t SENSOR_LOG_INTERVAL_LOOPS = SENSOR_LOOP_RATE_HZ / SENSOR_LOG_RATE_HZ;
static constexpr uint32_t SENSOR_LOG_PERIOD_MS = 1000 / SENSOR_LOG_RATE_HZ;

static_assert(SENSOR_LOOP_RATE_HZ % SENSOR_LOG_RATE_HZ == 0,
              "Sensor log rate must divide the sensor loop rate");

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