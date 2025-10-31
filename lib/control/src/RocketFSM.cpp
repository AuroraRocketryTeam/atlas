#include "RocketFSM.hpp"
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"
#include <Arduino.h>
#include <pins.h>

// Event queue size
static const size_t EVENT_QUEUE_SIZE = 10;

RocketFSM::RocketFSM(std::shared_ptr<Nemesis> model,
                     std::shared_ptr<SD> sd,
                     std::shared_ptr<RocketLogger> logger)
    : _fsmTaskHandle(nullptr), _eventQueue(nullptr), _stateMutex(nullptr),
      _currentState(RocketState::INACTIVE), _previousState(RocketState::INACTIVE),
      _stateStartTime(0), _isRunning(false), _isTransitioning(false),
      _model(model), _sd(sd), _logger(logger)
{
    LOG_INFO("FSM", "Constructor called");
    LOG_INFO("FSM", "Variables check: model=%s, SD=%s, Logger=%s",
             _model ? "OK" : "NULL",
             _sd ? "OK" : "NULL",
             _logger ? "OK" : "NULL");

    LOG_INFO("FSM", "Constructor completed");
}

RocketFSM::~RocketFSM()
{
    LOG_INFO("RocketFSM", "Destructor called");
    stop();

    // Clean up FreeRTOS objects
    if (_eventQueue)
    {
        vQueueDelete(_eventQueue);
        _eventQueue = nullptr;
    }

    if (_stateMutex)
    {
        vSemaphoreDelete(_stateMutex);
        _stateMutex = nullptr;
    }

    if (_modelMutex)
    {
        vSemaphoreDelete(_modelMutex);
        _modelMutex = nullptr;
    }

    if (_loggerMutex)
    {
        vSemaphoreDelete(_loggerMutex);
        _loggerMutex = nullptr;
    }

    LOG_INFO("RocketFSM", "Destructor completed");
}

void RocketFSM::init()
{
    LOG_INFO("RocketFSM", "Initializing...");

    // Initialize watchdog timer
    esp_task_wdt_init(60000, true); // 60 second timeout

    // Create FreeRTOS objects
    _eventQueue = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(FSMEventData));
    if (!_eventQueue)
    {
        LOG_ERROR("RocketFSM", "ERROR: Failed to create event queue");
        return;
    }

    _stateMutex = xSemaphoreCreateMutex();
    if (!_stateMutex)
    {
        LOG_ERROR("RocketFSM", "ERROR: Failed to create state mutex");
        vQueueDelete(_eventQueue);
        _eventQueue = nullptr;
        return;
    }
    _modelMutex = xSemaphoreCreateMutex();
    if (!_modelMutex)
    {
        LOG_ERROR("RocketFSM", "ERROR: Failed to create sensor data mutex");
        vQueueDelete(_eventQueue);
        _eventQueue = nullptr;
        vSemaphoreDelete(_stateMutex);
        _stateMutex = nullptr;
        return;
    }
    _loggerMutex = xSemaphoreCreateMutex();
    if (!_loggerMutex)
    {
        LOG_ERROR("RocketFSM", "ERROR: Failed to create loggerMutex");
        vQueueDelete(_eventQueue);
        _eventQueue = nullptr;
        vSemaphoreDelete(_stateMutex);
        _stateMutex = nullptr;
        vSemaphoreDelete(_modelMutex);
        _modelMutex = nullptr;
        return;
    }
    // Initialize managers
    LOG_INFO("RocketFSM", "Initializing TaskManager...");
    _taskManager = std::make_unique<TaskManager>(
        _model,     // model
        _modelMutex,// modelMutex
        _sd,             // sdCard
        _logger,         // _logger
        _loggerMutex   // loggerMutex
    );
    LOG_INFO("RocketFSM", "INITIALIZING TASKS...");
    _taskManager->initializeTasks();
    LOG_INFO("RocketFSM", "INITIALIZING TRANSITION MANAGER...");
    _transitionManager = std::make_unique<TransitionManager>();

    // Setup state machine configuration
    setupStateActions();
    setupTransitions();

    // Initialize state
    _currentState = RocketState::INACTIVE;
    _previousState = RocketState::INACTIVE;
    _stateStartTime = millis();

    LOG_INFO("RocketFSM", "Initialization complete - Free heap: %u bytes", ESP.getFreeHeap());
}

void RocketFSM::start()
{
    LOG_INFO("RocketFSM", "Starting...");

    if (_isRunning)
    {
        LOG_WARNING("RocketFSM", "Already running");
        return;
    }

    // CRITICAL: Set _isRunning BEFORE creating task to prevent race condition
    // where the new task starts before this flag is set
    _isRunning = true;

    // Create main FSM task
    BaseType_t result = xTaskCreate(
        fsmTaskWrapper,
        "FSM_Task",
        4096, // Stack size
        this, // Parameter
        2,    // Priority
        &_fsmTaskHandle);

    if (result == pdPASS)
    {
        LOG_INFO("RocketFSM", "Started successfully");
    }
    else
    {
        // Task creation failed: revert the flag
        _isRunning = false;
        LOG_ERROR("RocketFSM", "ERROR: Failed to create FSM task");
    }
}

void RocketFSM::stop()
{
    LOG_INFO("RocketFSM", "Stopping...");

    if (!_isRunning)
    {
        LOG_WARNING("RocketFSM", "Already stopped");
        return;
    }

    // Signal task to stop
    _isRunning = false;

    // Stop all managed tasks first
    if (_taskManager)
    {
        _taskManager->stopAllTasks();
    }

    // Wait cooperatively for main FSM task to finish (up to 3 seconds)
    if (_fsmTaskHandle)
    {
        LOG_DEBUG("RocketFSM", "Waiting for FSM task to exit cooperatively...");


        TaskHandle_t localHandle = _fsmTaskHandle;
        int maxWaits = 60; // 60 * 50ms = 3000ms
        int waits = 0;


        while (waits < maxWaits)
        {
            eTaskState taskState = eTaskGetState(localHandle);


            // Task has self-deleted or is invalid
            if (taskState == eDeleted || taskState == eInvalid)
            {
                LOG_DEBUG("RocketFSM", "FSM task exited after %d ms", waits * 50);
                break;
            }


            vTaskDelay(pdMS_TO_TICKS(50));
            waits++;
        }


        // Check final state
        eTaskState finalState = eTaskGetState(localHandle);
        if (finalState != eDeleted && finalState != eInvalid)
        {
            // CRITICAL: Do NOT force delete - this causes mutex deadlock
            LOG_ERROR("RocketFSM", "FSM task FAILED to exit after %d ms - SYSTEM MAY BE UNSTABLE", maxWaits * 50);
            LOG_ERROR("RocketFSM", "Task state: %d (0=Running, 1=Ready, 2=Blocked, 3=Suspended, 4=Deleted)", finalState);
            // Leave handle as-is for debugging
            return;
        }


        _fsmTaskHandle = nullptr;
    }

    LOG_INFO("RocketFSM", "Stopped");
}

bool RocketFSM::sendEvent(FSMEvent event, RocketState targetState, void *eventData)
{
    if (!_eventQueue)
    {
        LOG_ERROR("RocketFSM", "ERROR: Event queue not initialized");
        return false;
    }

    FSMEventData eventMsg(event, targetState, eventData);

    BaseType_t result = xQueueSend(_eventQueue, &eventMsg, 0);
    if (result == pdPASS)
    {
        LOG_INFO("RocketFSM", "Event %d sent successfully", static_cast<int>(event));
        return true;
    }
    else
    {
        LOG_ERROR("RocketFSM", "ERROR: Failed to send event %d (queue full?)", static_cast<int>(event));
        return false;
    }
}

RocketState RocketFSM::getCurrentState()
{
    if (_stateMutex && xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        RocketState state = _currentState;
        xSemaphoreGive(_stateMutex);
        return state;
    }
    return _currentState; // Fallback without mutex
}

FlightPhase RocketFSM::getCurrentPhase()
{
    RocketState state = getCurrentState();

    switch (state)
    {
    case RocketState::INACTIVE:
    case RocketState::CALIBRATING:
    case RocketState::READY_FOR_LAUNCH:
        return FlightPhase::PRE_FLIGHT;

    case RocketState::LAUNCH:
    case RocketState::ACCELERATED_FLIGHT:
    case RocketState::BALLISTIC_FLIGHT:
    case RocketState::APOGEE:
        return FlightPhase::FLIGHT;

    case RocketState::STABILIZATION:
    case RocketState::DECELERATION:
    case RocketState::LANDING:
    case RocketState::RECOVERED:
        return FlightPhase::RECOVERY;

    default:
        return FlightPhase::PRE_FLIGHT;
    }
}

void RocketFSM::forceTransition(RocketState newState)
{
    LOG_INFO("RocketFSM", "Force transition to %s", getStateString(newState));
    sendEvent(FSMEvent::FORCE_TRANSITION, newState);
}

bool RocketFSM::isFinished()
{
    return getCurrentState() == RocketState::RECOVERED;
}

void RocketFSM::setupStateActions()
{
    LOG_INFO("RocketFSM", "Setting up state actions...");

    // INACTIVE state
    _stateActions[RocketState::INACTIVE] = std::make_unique<StateAction>(RocketState::INACTIVE);
    _stateActions[RocketState::INACTIVE]->setEntryAction([this]()
                                                        { LOG_INFO("RocketFSM", "Entering INACTIVE"); });

    // CALIBRATING state
    _stateActions[RocketState::CALIBRATING] = std::make_unique<StateAction>(RocketState::CALIBRATING);
    _stateActions[RocketState::CALIBRATING]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering CALIBRATING"); })
        .setExitAction([this]()
                       { LOG_INFO("RocketFSM", "Exiting CALIBRATING"); })
        #ifdef SIMULATION_DATA
        // This state should be deleted eventually, also commenting out the simulation part, as it would just burn samples from the file
        //.addTask(TaskConfig(TaskType::SIMULATION, "Simulation_1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::BAROMETER, "Barometer_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Launch", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::SD_LOGGING, "Started_SD_Logging", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true));
    // READY_FOR_LAUNCH state
    _stateActions[RocketState::READY_FOR_LAUNCH] = std::make_unique<StateAction>(RocketState::READY_FOR_LAUNCH);
    _stateActions[RocketState::READY_FOR_LAUNCH]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering READY_FOR_LAUNCH"); })
        #ifdef SIMULATION_DATA
        .addTask(TaskConfig(TaskType::SIMULATION, "Simulation_2", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::SD_LOGGING, "Started_SD_Logging_2", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::BAROMETER, "Barometer_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask((TaskConfig(TaskType::TELEMETRY, "Telemetry_Ready", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true)));

    // LAUNCH state
    _stateActions[RocketState::LAUNCH] = std::make_unique<StateAction>(RocketState::LAUNCH);
    _stateActions[RocketState::LAUNCH]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering LAUNCH"); })
        #ifdef SIMULATION_DATA
        .addTask(TaskConfig(TaskType::SIMULATION, "Simulation_3", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::SD_LOGGING, "SD_Logging_Launch_3", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::BAROMETER, "Barometer_Launch", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Launch", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));
    _stateActions[RocketState::ACCELERATED_FLIGHT] = std::make_unique<StateAction>(RocketState::ACCELERATED_FLIGHT);
    _stateActions[RocketState::ACCELERATED_FLIGHT]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering ACCELERATED_FLIGHT"); })
        .addTask(TaskConfig(TaskType::BAROMETER, "Barometer_Accel", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        #ifdef SIMULATION_DATA
        .addTask(TaskConfig(TaskType::SIMULATION, "Simulation_4", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::SD_LOGGING, "SD_Logging_Accel_4", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Accel", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));

    _stateActions[RocketState::BALLISTIC_FLIGHT] = std::make_unique<StateAction>(RocketState::BALLISTIC_FLIGHT);
    _stateActions[RocketState::BALLISTIC_FLIGHT]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering BALLISTIC_FLIGHT"); })
        #ifdef SIMULATION_DATA
        .addTask(TaskConfig(TaskType::SIMULATION, "Simulation_5", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::SD_LOGGING, "SD_Logging_Ballistic_5", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::BAROMETER, "Barometer_Ballistic", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Ballistic", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));

    _stateActions[RocketState::APOGEE] = std::make_unique<StateAction>(RocketState::APOGEE);
    _stateActions[RocketState::APOGEE]
        ->setEntryAction([this]()
                         {
                             LOG_INFO("RocketFSM", "Entering APOGEE");
                             digitalWrite(DROGUE_ACTUATOR_PIN, HIGH); // Activate drogue deployment
                             tone(BUZZER_PIN, 1000, 500);             // Sound buzzer at 1kHz for 500ms
                         })
        #ifdef SIMULATION_DATA
        .addTask(TaskConfig(TaskType::SIMULATION, "Simulation_6", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::BAROMETER, "Barometer_Apogee", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::SD_LOGGING, "SD_Logging_Apogee_6", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Apogee", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));

    _stateActions[RocketState::STABILIZATION] = std::make_unique<StateAction>(RocketState::STABILIZATION);
    _stateActions[RocketState::STABILIZATION]
        ->setExitAction([this]()
                         {
                             LOG_INFO("RocketFSM", "Exiting STABILIZATION");
                             digitalWrite(MAIN_ACTUATOR_PIN, HIGH); // Activate main deployment
                             tone(BUZZER_PIN, 1000, 500);           // Sound buzzer at 1kHz for 500ms
                         })
        #ifdef SIMULATION_DATA
        .addTask(TaskConfig(TaskType::SIMULATION, "Simulation_7", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::BAROMETER, "Barometer_Stabilization", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::SD_LOGGING, "SD_Logging_Stabilization_7", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Stabilization", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));

    _stateActions[RocketState::DECELERATION] = std::make_unique<StateAction>(RocketState::DECELERATION);
    _stateActions[RocketState::DECELERATION]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering DECELERATION"); })
        #ifdef SIMULATION_DATA
        .addTask(TaskConfig(TaskType::SIMULATION, "Simulation_8", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::BAROMETER, "Barometer_Deceleration", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::SD_LOGGING, "SD_Logging_Deceleration_8", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Deceleration", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));
    _stateActions[RocketState::LANDING] = std::make_unique<StateAction>(RocketState::LANDING);
    _stateActions[RocketState::LANDING]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering LANDING"); })
        #ifdef SIMULATION_DATA
        .addTask(TaskConfig(TaskType::SIMULATION, "Simulation_9", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::BAROMETER, "Barometer_Landing", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::SD_LOGGING, "SD_Logging_Landing_9", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Landing", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));
    _stateActions[RocketState::RECOVERED] = std::make_unique<StateAction>(RocketState::RECOVERED);
    _stateActions[RocketState::RECOVERED]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering RECOVERED"); })
        #ifdef SIMULATION_DATA
        .addTask(TaskConfig(TaskType::SIMULATION, "Simulation_10", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::SD_LOGGING, "SD_Logging_PostFlight_10", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_PostFlight", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));
    LOG_INFO("RocketFSM", "State actions setup complete");
}

void RocketFSM::setupTransitions()
{
    LOG_INFO("RocketFSM", "Setting up transitions...");

    // Basic event-driven transitions
    _transitionManager->addTransition(Transition(
        RocketState::INACTIVE,
        RocketState::CALIBRATING,
        FSMEvent::START_CALIBRATION));

    _transitionManager->addTransition(Transition(
        RocketState::CALIBRATING,
        RocketState::READY_FOR_LAUNCH,
        FSMEvent::CALIBRATION_COMPLETE));

    _transitionManager->addTransition(Transition(
        RocketState::READY_FOR_LAUNCH,
        RocketState::LAUNCH,
        FSMEvent::LAUNCH_DETECTED));

    _transitionManager->addTransition(Transition(
        RocketState::LAUNCH,
        RocketState::ACCELERATED_FLIGHT,
        FSMEvent::LIFTOFF_STARTED));

    _transitionManager->addTransition(Transition(
        RocketState::ACCELERATED_FLIGHT,
        RocketState::BALLISTIC_FLIGHT,
        FSMEvent::ACCELERATION_COMPLETE));

    _transitionManager->addTransition(Transition(
        RocketState::BALLISTIC_FLIGHT,
        RocketState::APOGEE,
        FSMEvent::APOGEE_REACHED));

    _transitionManager->addTransition(Transition(
        RocketState::APOGEE,
        RocketState::STABILIZATION,
        FSMEvent::DROGUE_READY));

    _transitionManager->addTransition(Transition(
        RocketState::STABILIZATION,
        RocketState::DECELERATION,
        FSMEvent::STABILIZATION_COMPLETE));

    _transitionManager->addTransition(Transition(
        RocketState::DECELERATION,
        RocketState::LANDING,
        FSMEvent::DECELERATION_COMPLETE));

    _transitionManager->addTransition(Transition(
        RocketState::LANDING,
        RocketState::RECOVERED,
        FSMEvent::LANDING_COMPLETE));

    // Emergency abort transition from any state to INACTIVE
    for (int state = static_cast<int>(RocketState::INACTIVE); state <= static_cast<int>(RocketState::RECOVERED); ++state)
    {
        _transitionManager->addTransition(Transition(
            static_cast<RocketState>(state),
            RocketState::INACTIVE,
            FSMEvent::EMERGENCY_ABORT));
    }

    // Add more transitions as needed...

    LOG_INFO("RocketFSM", "Transitions setup complete");
}

void RocketFSM::transitionTo(RocketState newState)
{
    //play buzzer
    tone(BUZZER_PIN, 2000, 100);
    if (_isTransitioning)
    {
        LOG_WARNING("RocketFSM", "Already transitioning, ignoring");
        return;
    }

    if (newState == _currentState)
    {
        return;
    }

    _isTransitioning = true;

    if (xSemaphoreTake(_stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        LOG_INFO("RocketFSM", "[TRANSITION] %s -> %s",
                 getStateString(_currentState),
                 getStateString(newState));

        // Execute exit action for current state
        if (_stateActions[_currentState])
        {
            _stateActions[_currentState]->onExit();
        }

        // Stop current tasks
        if (_taskManager)
        {
            try
            {
                _taskManager->stopAllTasks();
            }
            catch (const std::exception &e)
            {
                LOG_ERROR("RocketFSM", "[TRANSITION] ERROR: Exception while stopping tasks: %s", e.what());
            }
        }

        // Update state
        _previousState = _currentState;
        _currentState = newState;
        _stateStartTime = millis();

        // Execute entry action for new state
        if (_stateActions[_currentState])
        {
            _stateActions[_currentState]->onEntry();
        }

        // Start new tasks
        if (_stateActions[_currentState])
        {
            for (const auto &taskConfig : _stateActions[_currentState]->getTaskConfigs())
            {
                _taskManager->startTask(taskConfig.type, taskConfig);
            }
        }

        xSemaphoreGive(_stateMutex);

        LOG_INFO("RocketFSM", "[TRANSITION] Complete - Free heap: %u bytes", ESP.getFreeHeap());
    }
    else
    {
        LOG_ERROR("RocketFSM", "[TRANSITION] ERROR: Failed to acquire state mutex");
    }
    _isTransitioning = false;
}

void RocketFSM::processEvent(const FSMEventData &eventData)
{
    LOG_INFO("RocketFSM", "Processing event %d in state %s",
             static_cast<int>(eventData.event),
             getStateString(_currentState));

    if (eventData.event == FSMEvent::FORCE_TRANSITION)
    {
        LOG_INFO("RocketFSM", "Force transition to %s",
                 getStateString(eventData.targetState));
        transitionTo(eventData.targetState);
        return;
    }

    // Find valid transition
    auto newState = _transitionManager->findTransition(_currentState, eventData.event);
    if (newState.has_value())
    {
        transitionTo(newState.value());
    }
    else
    {
        LOG_WARNING("RocketFSM", "No valid transition for event %d in state %s",
                    static_cast<int>(eventData.event),
                    getStateString(_currentState));
    }
}

void RocketFSM::checkTransitions()
{
    // Check for automatic transitions quickly (fast paths first)
    if (_transitionManager->checkAutomaticTransitions(_currentState, _stateStartTime))
    {
        return; // automatic transition handled
    }

    // Cache some persistent values across calls to avoid repeated allocations
    static bool haveAccel = false;
    static float accelZ = 0.0f;
    static unsigned long launchHighSince = 0;
    static unsigned long decelSince = 0;

    // Get accelerometer data before switch statement
    std::shared_ptr<BNO055Data> bno055Data = _model->getBNO055Data();
    auto accX = bno055Data->acceleration_x;
    auto accY = bno055Data->acceleration_y;
    auto accZ = bno055Data->acceleration_z;

    // Fast state-based checks
    switch (_currentState)
    {
    case RocketState::INACTIVE:
        // Kick off calibration immediately
        sendEvent(FSMEvent::START_CALIBRATION);
        break;

    case RocketState::CALIBRATING:
        if (millis() - _stateStartTime > 5000U)
        {
            sendEvent(FSMEvent::CALIBRATION_COMPLETE);
        }
        // Calibration timeout fallback
        break;
    case RocketState::READY_FOR_LAUNCH:
        try {
            auto accMag = sqrt(accX * accX + accY * accY + accZ * accZ);

            if (accMag > LIFTOFF_ACCELERATION_THRESHOLD)
            {
                if (launchHighSince == 0)
                {
                    launchHighSince = millis();
                }
                else if (millis() - launchHighSince > static_cast<unsigned long>(LIFTOFF_TIMEOUT_MS))
                {
                    _launchDetectionTime = millis();
                    sendEvent(FSMEvent::LAUNCH_DETECTED);
                    launchHighSince = 0;
                }
            }
        } catch (const std::exception& e) {
            LOG_ERROR("RocketFSM", "READY_FOR_LAUNCH: Exception occurred: %s", e.what());
        }
        
        break;

    case RocketState::LAUNCH:
        // After a short delay consider liftoff started (rocket left the launch pad and is accelerating)
        LOG_INFO("RocketFSM", "Now: %lu, stateStartTime: %lu, LAUNCH: elapsed=%lu ms", millis(), _stateStartTime, millis() - _stateStartTime);
        sendEvent(FSMEvent::LIFTOFF_STARTED);
        break;

    case RocketState::ACCELERATED_FLIGHT:
        if (millis() - _launchDetectionTime >= LAUNCH_TO_BALLISTIC_THRESHOLD)
        {
            sendEvent(FSMEvent::ACCELERATION_COMPLETE);
        }

        //LOG_INFO("ACC_FLIGHT", "Checking condition");
        /*if (accOpt.has_value() && std::holds_alternative<std::map<std::string, float>>(accOpt.value()))
        {
            const auto &accMap = std::get<std::map<std::string, float>>(accOpt.value());
            auto accVal = accMap.at("magnitude");

            LOG_INFO("RocketFSM", "ACCELERATED_FLIGHT: accelZ=%.3f", accVal);
            // Detect sustained deceleration to switch to ballistic
            if (accVal <= 0)
            {
                if (decelSince == 0)
                {
                    decelSince = millis();
                }
                else if (millis() - decelSince > 200U)
                {
                    sendEvent(FSMEvent::ACCELERATION_COMPLETE);
                    decelSince = 0;
                }
            }
            else
            {
                decelSince = 0;
            }
        } */
        break;

    case RocketState::BALLISTIC_FLIGHT:
    {
        //LOG_INFO("RocketFSM", "BALLISTIC_FLIGHT: is rising = %u", *isRising);
        auto elapsed = millis() - _launchDetectionTime;
        auto isRising = _model->getIsRising();
        //LOG_INFO("RocketFSM", "now: %.3lu, stateStartTime: %.3lu, evaluated: %.3lu, treshold: %.3lu", millis(), stateStartTime, elapsed, LAUNCH_TO_APOGEE_THRESHOLD);
        if(!*isRising || (elapsed >= LAUNCH_TO_APOGEE_THRESHOLD)){
            sendEvent(FSMEvent::APOGEE_REACHED);
        }

        break;
    }

    case RocketState::APOGEE:
        if (millis() - _stateStartTime > DROGUE_APOGEE_TIMEOUT)
        {
            sendEvent(FSMEvent::DROGUE_READY);
        }
        break;

    case RocketState::STABILIZATION:
    {
        auto currentHeight = _model->getCurrentHeight();
        LOG_INFO("RocketFSM", "STABILIZATION: altitude=%.3f", *currentHeight);
        if (*currentHeight < MAIN_ALTITUDE_THRESHOLD)
        {
            LOG_INFO("RocketFSM", "STABILIZATION: condition met (altitude=%.3f, elapsed=%lu ms)", *currentHeight, millis() - _stateStartTime);
            sendEvent(FSMEvent::STABILIZATION_COMPLETE);
        }
    
        break;
    }

    case RocketState::DECELERATION:
    {
        // In DECELERATION state, vertical velocity in heightGainSpeed will still be tracked, but it should be negative (falling)
        // !!! choose if chenge the control to be with negative values or to invert the value here

        auto currentHeight = _model->getCurrentHeight();
        if (*currentHeight < TOUCHDOWN_ALTITUDE_THRESHOLD)
        {
            sendEvent(FSMEvent::DECELERATION_COMPLETE);
        }
        
        break;
    }

    case RocketState::LANDING:
        if (millis() - _stateStartTime > 2000U)
        {
            sendEvent(FSMEvent::LANDING_COMPLETE);
        }
        break;

    case RocketState::RECOVERED:
        // Nothing to do
        break;

    default:
        break;
    }
}

void RocketFSM::fsmTaskWrapper(void *parameter)
{

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (fsm)
    {
        fsm->fsmTask();
    }

    vTaskDelete(NULL);
}

void RocketFSM::fsmTask()
{
    LOG_INFO("RocketFSM", "Main task started");

    FSMEventData eventData(FSMEvent::NONE);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned long loopCounter = 0;
    esp_task_wdt_add(NULL);

    while (_isRunning)
    {
        esp_task_wdt_reset();

        if (_isTransitioning)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        // Check for automatic transitions
        checkTransitions();

        // Process events from queue
        if (_eventQueue && xQueueReceive(_eventQueue, &eventData, pdMS_TO_TICKS(20)) == pdPASS)
        {
            if (!_isTransitioning)
            {
                processEvent(eventData);
            }
        }

        loopCounter++;
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20)); // 50Hz
    }
    esp_task_wdt_delete(NULL);

    LOG_INFO("RocketFSM", "Main task ended");
}

const char* RocketFSM::getStateString(RocketState state) const
{
    return rocketStateToString(state);
}
