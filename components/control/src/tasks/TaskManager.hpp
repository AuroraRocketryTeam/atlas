#pragma once

#include "ITask.hpp"
#include <memory>
#include <map>
#include <string>
#include "TaskConfig.hpp"
#include "SerialLogger.hpp"
#include "SD-master.hpp"
#include "IStorage.hpp"
#include "RocketModel.hpp"
#include "IStateMachine.hpp"
#include "config.h"

#include "SensorTask.hpp"
#include "AirbrakesTask.hpp"
#include "StorageLoggingTask.hpp"
#include "GpsTask.hpp"

#if CONFIG_AURORA_HIL_SUPPORT
#include "HilSimulationTask.hpp"
#endif

#include "TelemetryTask.hpp"
#include "AltitudeTask.hpp"
#include <E220LoRaTransmitter.hpp>

/**
 * @brief Class to manage tasks in the system.
 * 
 */
class TaskManager {    
public:
    /**
     * @brief Construct a new Task Manager object
     * 
     * @param rocketModel The shared pointer to the rocket model
     * @param sd The shared pointer to the SD card
     * @param logger The shared pointer to the RocketLogger instance
     */
    TaskManager(std::shared_ptr<RocketModel> rocketModel,
            std::shared_ptr<SD> sd,
            std::shared_ptr<RocketLogger> logger,
            IStateMachine* fsm = nullptr);
    
    /**
     * @brief Destroy the Task Manager object
     * 
     */
    ~TaskManager();

    /**
     * @brief Initialize all tasks in the manager
     * 
     */
    void initializeTasks();

    /**
     * @brief Start a task in the manager
     * 
     * @param type The type of task to start
     * @param config The configuration parameters for the task
     * @return true if the task was started successfully, false otherwise
     */
    bool startTask(TaskType type, const TaskConfig& config);
    
    /**
     * @brief Stop a task in the manager
     * 
     * @param type The type of task to stop
     */
    void stopTask(TaskType type);

    /**
     * @brief Stop all running tasks in the manager
     * 
     */
    void stopAllTasks();

    /**
     * @brief Get the count of currently running tasks
     * 
     * @return int The number of running tasks
     */
    int getRunningTaskCount();

    /**
     * @brief Check if a task is currently running
     * 
     * @param type The type of task to check
     * @return true if the task is running, false otherwise
     */
    bool isTaskRunning(TaskType type) const;

    /**
     * @brief Get the stack usage of a task
     * 
     * @param type The type of task to check
     * @return uint32_t The stack usage of the task
     */
    uint32_t getTaskStackUsage(TaskType type) const;
    
    /**
     * @brief Print the status of all tasks
     * 
     */
    void printTaskStatus() const;

private:
    // Map of task type to task instance
    std::map<TaskType, std::unique_ptr<ITask>> _tasks;
    
    // Shared resources
    std::shared_ptr<RocketModel> _rocketModel;
    std::shared_ptr<RocketLogger> _logger;

    std::shared_ptr<SD> _sd;
    // Telemetry
    std::shared_ptr<E220LoRaTransmitter> _loraTransmitter;

    IStateMachine* _fsm;
};