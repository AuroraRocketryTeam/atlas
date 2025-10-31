#pragma once

#include "ITask.hpp"
#include "SharedData.hpp"
#include <memory>
#include <map>
#include <string>
#include "TaskConfig.hpp"
#include "Logger.hpp"
#include "SD-master.hpp"
#include "Nemesis.hpp"

#include "SensorTask.hpp"
#include "SDLoggingTask.hpp"
#include "EkfTask.hpp"
#include "GpsTask.hpp"
#include "SimulationTask.hpp"
#include "TelemetryTask.hpp"
#include "BarometerTask.hpp"
#include <EspNowTransmitter.hpp>

//#define SIMULATION_DATA // Comment this out to use real sensors

class TaskManager {
private:
    std::map<TaskType, std::unique_ptr<ITask>> _tasks;
    std::shared_ptr<Nemesis> _model;
    std::shared_ptr<RocketLogger> _logger;
    SemaphoreHandle_t _modelMutex;
    SemaphoreHandle_t _loggerMutex;

    std::shared_ptr<SD> _sd;
    
    // Telemetry
    std::shared_ptr<EspNowTransmitter> _espNowTransmitter;
    
public:
    TaskManager(std::shared_ptr<Nemesis> model,
            SemaphoreHandle_t modelMutex,
            std::shared_ptr<SD> sd,
            std::shared_ptr<RocketLogger> logger,
            SemaphoreHandle_t loggerMutex);
    
    ~TaskManager();
    
    void initializeTasks();
    bool startTask(TaskType type, const TaskConfig& config);
    void stopTask(TaskType type);
    void stopAllTasks();
    int getRunningTaskCount();
    
    bool isTaskRunning(TaskType type) const;
    uint32_t getTaskStackUsage(TaskType type) const;
    
    void printTaskStatus() const;
};