#include "TaskManager.hpp"
#include <config.h>
#include <config.h>

TaskManager::TaskManager(std::shared_ptr<Nemesis> model,
                         SemaphoreHandle_t modelMutex,
                         std::shared_ptr<SD> sd,
                         std::shared_ptr<RocketLogger> logger,
                         SemaphoreHandle_t loggerMutex) : 
                         _model(model),
                         _modelMutex(modelMutex),
                         _sd(sd),
                         _logger(logger),
                         _loggerMutex(loggerMutex)
{
    LOG_INFO("TaskMgr", "Initialized with model");

    // Initialize ESP-NOW transmitter
    uint8_t peerMac[] = ESPNOW_PEER_MAC;
    _espNowTransmitter = std::make_shared<EspNowTransmitter>(peerMac, ESPNOW_CHANNEL);

    // Initialize transmitter
    ResponseStatusContainer initResult = _espNowTransmitter->init();
    if (initResult.getCode() != 0)
    {
        LOG_ERROR("TaskMgr", "Failed to initialize ESP-NOW: %s", initResult.getDescription().c_str());
    }
    else
    {
        LOG_INFO("TaskMgr", "ESP-NOW transmitter initialized successfully");
    }
}

TaskManager::~TaskManager()
{
    stopAllTasks();
    LOG_INFO("TaskManager", "Destroyed");
}

void TaskManager::initializeTasks()
{
    LOG_INFO("TaskManager", "Creating task instances...");

    // Create all task instances but don't start them yet
    // Note: Most tasks still need refactoring to use the model-based architecture
    // For now, only BarometerTask has been updated to use the Nemesis model
    
    _tasks[TaskType::SENSOR] = std::make_unique<SensorTask>(
        _model,
        _modelMutex,
        _logger,
        _loggerMutex);
    _tasks[TaskType::GPS] = std::make_unique<GpsTask>(
        _model,
        _modelMutex,
        _logger,
        _loggerMutex);
    // _tasks[TaskType::EKF] = std::make_unique<EkfTask>(
    //     _model,
    //     _modelMutex,
    //     _kalmanFilter);
    _tasks[TaskType::SD_LOGGING] = std::make_unique<SDLoggingTask>(
        _logger,
        _loggerMutex,
        _sd);
    _tasks[TaskType::SIMULATION] = std::make_unique<SimulationTask>(
        // Using a different simulation file where at the end of each line there is a
        // pipe symbol, this was needed as the readLine function had problem recognizing 
        // the \n character, so separating each line 
        "/simulated_sensors_full_piped.csv",
        _model,
        _modelMutex,
        _logger,
        _loggerMutex);

    // Create TelemetryTask with ESP-NOW transmitter
    // We should probably change this, such that the transmitted data aligns better with the ones saved in the sd!!!
    _tasks[TaskType::TELEMETRY] = std::make_unique<TelemetryTask>(
        _model,
        _modelMutex,
        _espNowTransmitter,
        TELEMETRY_INTERVAL_MS);

    _tasks[TaskType::BAROMETER] = std::make_unique<BarometerTask>(
        _model,
        _modelMutex);

    LOG_INFO("TaskManager", "Created %d task instances", _tasks.size());
}

bool TaskManager::startTask(TaskType type, const TaskConfig &config)
{
    LOG_INFO("TaskManager", "Starting task type %d with config: %s",
             static_cast<int>(type), config.name);

    // Check if task exists
    auto it = _tasks.find(type);
    if (it == _tasks.end())
    {
        LOG_ERROR("TaskManager", "ERROR: Task type %d not found", static_cast<int>(type));
        return false;
    }

    // Check if already running
    if (it->second->isRunning())
    {
        LOG_WARNING("TaskManager", "WARNING: Task %s already running", config.name);
        return true; // Not an error, just already running
    }

    // Check available memory before starting
    uint32_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < config.stackSize + 1024)
    { // Reserve 1KB buffer
        LOG_ERROR("TaskManager", "ERROR: Insufficient memory. Free: %u, Need: %u",
                  freeHeap, config.stackSize + 1024);
        return false;
    }

    // Start the task
    bool result = it->second->start(config);
    if (result)
    {
        LOG_INFO("TaskManager", "Successfully started task: %s", config.name);
    }
    else
    {
        LOG_ERROR("TaskManager", "FAILED to start task: %s", config.name);
    }

    return result;
}

void TaskManager::stopTask(TaskType type)
{
    if (_tasks.find(type) == _tasks.end())
    {
        LOG_WARNING("TaskManager", "Task type %d not found", static_cast<int>(type));
        return;
    }

    auto &task = _tasks[type];
    if (task && task->isRunning())
    {
        LOG_INFO("TaskManager", "Stopping task: %s", task->getName());

        // Record pre-stop status
        LOG_DEBUG("TaskManager", "Pre-stop: %s isRunning=%d, StackHWM=%u, FreeHeap=%u",
                  task->getName(), task->isRunning(), task->getStackHighWaterMark(), ESP.getFreeHeap());

        // Request cooperative stop - BaseTask::stop() now handles waiting
        task->stop();

        // Quick verification
        if (task->isRunning())
        {
            LOG_ERROR("TaskManager", "Task %s still running after stop()", task->getName());
        }
        else
        {
            LOG_INFO("TaskManager", "Task %s stopped - FreeHeap=%u", task->getName(), ESP.getFreeHeap());
        }
    }
}

void TaskManager::stopAllTasks()
{
    LOG_INFO("TaskManager", "Stopping all tasks...");

    // Iterate over a snapshot of task types to avoid iterator invalidation during stops
    std::vector<TaskType> taskTypes;
    for (const auto &p : _tasks)
        taskTypes.push_back(p.first);

    for (auto type : taskTypes)
    {
        auto it = _tasks.find(type);
        if (it != _tasks.end())
        {
            auto &task = it->second;
            if (task && task->isRunning())
            {
                LOG_DEBUG("TaskManager", "About to stop task type %d (%s)", static_cast<int>(type), task->getName());
                stopTask(type);
            }
        }
    }

    // Minimal settle delay since BaseTask::stop() already waits
    vTaskDelay(pdMS_TO_TICKS(50));

    LOG_INFO("TaskManager", "All tasks stopped - FreeHeap: %u bytes", ESP.getFreeHeap());
}

bool TaskManager::isTaskRunning(TaskType type) const
{
    auto it = _tasks.find(type);
    return (it != _tasks.end()) && it->second->isRunning();
}

uint32_t TaskManager::getTaskStackUsage(TaskType type) const
{
    auto it = _tasks.find(type);
    if (it != _tasks.end())
    {
        return it->second->getStackHighWaterMark();
    }
    return 0;
}

void TaskManager::printTaskStatus() const
{
    LOG_INFO("TaskManager", "\n=== TASK STATUS ===");
    LOG_INFO("TaskManager", "Free heap: %u bytes", ESP.getFreeHeap());
    LOG_INFO("TaskManager", "Task Status:");

    const char *taskNames[] = {
        "SENSOR", "EKF", "APOGEE_DETECTION", "RECOVERY",
        "DATA_COLLECTION", "TELEMETRY", "GPS", "LOGGING"};

    int index = 0;
    for (const auto &[type, task] : _tasks)
    {
        if (task)
        {
            LOG_INFO("TaskManager", "  %s: %s (Stack HWM: %u)",
                     taskNames[index],
                     task->isRunning() ? "RUNNING" : "STOPPED",
                     task->getStackHighWaterMark());
        }
        index++;
    }
    LOG_INFO("TaskManager", "=================");
}

int TaskManager::getRunningTaskCount()
{
    int count = 0;
    for (const auto &[type, task] : _tasks)
    {
        if (task && task->isRunning())
        {
            count++;
        }
    }
    return count;
}