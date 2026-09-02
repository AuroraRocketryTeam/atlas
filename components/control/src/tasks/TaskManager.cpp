// TODO: LoRa E220 init uses Arduino Serial in this library.
// Will be replaced with ESP-IDF UART (uart_driver_install / uart_write_bytes).

#include "TaskManager.hpp"
#include <config.h>
#include <board.h>

TaskManager::TaskManager(std::shared_ptr<RocketModel> rocketModel,
                         std::shared_ptr<SD> sd,
                         std::shared_ptr<RocketLogger> logger,
                         IBoardHardware* board,
                         IStateMachine* fsm,
                         std::shared_ptr<IGroundTestRunner> testRunner) :
                         _rocketModel(rocketModel),
                         _logger(logger),
                         _sd(sd),
                         _fsm(fsm),
                         _board(board),
                         _testRunner(testRunner)
{
    LOG_INFO("TaskMgr", "Initialized with model");

    
    LOG_INFO("TaskMgr", "Skipping ESP-NOW init. We are not using it and it conflicts with WiFi SoftAP for GroundService and HIL.");
    _espNowTransmitter = nullptr;

    // Initialize ESP-NOW transmitter. ESP-NOW uses board-managed WiFi STA mode.
    // uint8_t peerMac[] = ESPNOW_PEER_MAC;
    // _espNowTransmitter = std::make_shared<EspNowTransmitter>(_board, peerMac, ESPNOW_CHANNEL);

    // // Initialize transmitter
    // ResponseStatusContainer initResult = _espNowTransmitter->init();
    // if (initResult.getCode() != 0)
    // {
    //     LOG_ERROR("TaskMgr", "Failed to initialize ESP-NOW: %s", initResult.getDescription().c_str());
    //     _espNowTransmitter = nullptr;
    // }
    // else
    // {
    //     LOG_INFO("TaskMgr", "ESP-NOW transmitter initialized successfully");
    // }


    // Initialize and configure LoRa transmitter
    // Serial1 is used by GPS, so LoRa uses Serial2
    _loraTransmitter = std::make_shared<E220LoRaTransmitter>(Serial2, MANNY_LORA_TX_PIN, MANNY_LORA_RX_PIN, MANNY_LORA_AUX_PIN, MANNY_LORA_M0_PIN, MANNY_LORA_M1_PIN);

    auto loraInit = _loraTransmitter->init(E220LoRaTransmitter::defaultConfiguration());
    if (loraInit.getCode() == E220_SUCCESS)
    {
        LOG_INFO("TaskMgr", "LoRa E220 initialized and configured successfully");
    }
    else
    {
        LOG_ERROR("TaskMgr", "LoRa E220 init failed: %s", loraInit.getDescription().c_str());
        _loraTransmitter = nullptr; // disable LoRa if init failed
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
    // For now, only AltitudeTask has been updated to use the Nemesis model
    
    _tasks[TaskType::SENSOR] = std::make_unique<SensorTask>(
        _rocketModel,
        _logger);
    if (_rocketModel->hasGPS())
    {
        _tasks[TaskType::GPS] = std::make_unique<GpsTask>(
            _rocketModel,
            _logger);
    } else LOG_INFO("TaskManager", "GPS unavailable. Skipping GPS task");
    _tasks[TaskType::STORAGE] = std::make_unique<StorageLoggingTask>(
        _rocketModel,
        _logger);

#if AURORA_HIL_ENABLED
        _tasks[TaskType::HIL_SIMULATION] = std::make_unique<HilSimulationTask>(
        _rocketModel,
        _logger,
        _board,
        _fsm);
#endif
        
    _tasks[TaskType::AIRBRAKES] = std::make_unique<AirbrakesTask>(
        _rocketModel,
        _logger);
    

    // Create TelemetryTask with ESP-NOW and LoRa transmitters
    // We should probably change this, such that the transmitted data aligns better with the ones saved in the sd!!!
    auto telemetryTask = std::make_unique<TelemetryTask>(
        _rocketModel,
        _espNowTransmitter,
        _fsm);
    if (_loraTransmitter)
    {
        telemetryTask->setLoRaTransmitter(_loraTransmitter);
    }
    _tasks[TaskType::TELEMETRY] = std::move(telemetryTask);

    _tasks[TaskType::ALTITUDE] = std::make_unique<AltitudeTask>(
        _rocketModel);

    _tasks[TaskType::GROUND_SERVICES] = std::make_unique<GroundServicesTask>(
        _rocketModel,
        _logger,
        _board,
        _fsm,
        _testRunner);

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

    for (const auto &[type, task] : _tasks)
    {
        if (task)
        {
            LOG_INFO("TaskManager", "  [%s] %s: %s (Stack HWM: %u)",
                     taskTypeToString(type),
                     task->getName(),
                     task->isRunning() ? "RUNNING" : "STOPPED",
                     task->getStackHighWaterMark());
        }
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
