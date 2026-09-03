#include "RocketFSM.hpp"
#include "tasks/RuntimeConfig.hpp"
#include "esp_task_wdt.h"
#include <utils.h>
#include <algorithm>
#include <cmath>

// Event queue size
static const size_t EVENT_QUEUE_SIZE = 10;
static constexpr uint32_t ACTUATOR_PULSE_COUNT = 3;
static constexpr uint32_t ACTUATOR_PULSE_DURATION_MS = 3;

namespace {
void fireActuator(gpio_num_t pin)
{
    for (uint32_t pulse = 0; pulse < ACTUATOR_PULSE_COUNT; ++pulse)
    {
        gpio_set_level(pin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(ACTUATOR_PULSE_DURATION_MS));
        gpio_set_level(pin, LOW);
    }
}
} // namespace

RocketFSM::RocketFSM(std::shared_ptr<RocketModel> rocketModel,
                     std::shared_ptr<SD> sd,
                     std::shared_ptr<RocketLogger> logger,
                     IBoardHardware* board,
                     std::shared_ptr<IGroundTestRunner> testRunner)
    : _fsmTaskHandle(nullptr), _eventQueue(nullptr), _stateMutex(nullptr),
      _currentState(RocketState::INACTIVE), _previousState(RocketState::INACTIVE),
      _stateStartTime(0), _isRunning(false), _isTransitioning(false),
      _rocketModel(rocketModel), _logger(logger), _testRunner(testRunner), _sd(sd), _board(board)
{
    LOG_INFO("FSM", "Constructor called");
    LOG_INFO("FSM", "Variables check: model=%s, SD=%s, Logger=%s",
             _rocketModel ? "OK" : "NULL",
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

    LOG_INFO("RocketFSM", "Destructor completed");
}

void RocketFSM::init()
{
    LOG_INFO("RocketFSM", "Initializing...");

    // Initialize watchdog timer
    esp_task_wdt_config_t config;
    config.timeout_ms = 60000;
    config.idle_core_mask = (1 << portNUM_PROCESSORS) - 1;
    config.trigger_panic = true;
    esp_task_wdt_init(&config); // 60 second timeout

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
    
    // Initialize managers
    LOG_INFO("RocketFSM", "Initializing TaskManager...");
    _taskManager = std::make_unique<TaskManager>(
        _rocketModel,
        _sd,
        _logger,
        _board,
        this,
        _testRunner
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
    _stateStartTime = Utils::millis();

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
        4096,
        this,
        static_cast<UBaseType_t>(TaskPriority::TASK_REAL_TIME),
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
    case RocketState::GROUND_SERVICES:
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

void RocketFSM::deployMain()
{
    if (_mainDeploymentCommanded)
    {
        LOG_INFO("RocketFSM", "MAIN already commanded, skipping");
        return;
    }

    LOG_INFO("RocketFSM", "Deploying MAIN");
    if (_logger) {
        _logger->logInfo("RocketFSM", "Main deployment commanded");
    }

    fireActuator(_board->get_main_actuator_pin());
    _mainDeploymentCommanded = true;

    _rocketModel->setOpenMainCommand();
    
}


void RocketFSM::deployDrogue()
{
    if (_drogueDeploymentCommanded)
    {
        LOG_INFO("RocketFSM", "DROGUE already commanded, skipping");
        return;
    }

    
    LOG_INFO("RocketFSM", "Deploying DROGUE");
    if (_logger) {
        _logger->logInfo("RocketFSM", "Drogue deployment commanded");
    }
    
    fireActuator(_board->get_drogue_actuator_pin());
    _drogueDeploymentCommanded = true;
    
    _rocketModel->setOpenDrogueCommand();
}


void RocketFSM::deployApogeeRecovery()
{
    const auto recoveryMode = static_cast<RecoveryMode>(runtime_config_get_flight_snapshot().recovery.mode);
    if (recoveryMode == RecoveryMode::OneParachuteMode)
    {
        LOG_INFO("RocketFSM", "APOGEE recovery policy: OneParachuteMode -> deploy MAIN (and DROGUE)");
        deployMain();
        deployDrogue(); // the parachute is attached to both main-pin and drogue-pin. so for safety command both.
    }
    else if (recoveryMode == RecoveryMode::TwoParachuteMode)
    {
        LOG_INFO("RocketFSM", "APOGEE recovery policy: TwoParachuteMode -> deploy DROGUE");
        deployDrogue();
    }
}


void RocketFSM::deployStabilizationExitRecovery()
{
    const auto recoveryMode = static_cast<RecoveryMode>(runtime_config_get_flight_snapshot().recovery.mode);
    if (recoveryMode == RecoveryMode::OneParachuteMode)
    {
        LOG_INFO("RocketFSM", "STABILIZATION exit recovery policy: OneParachuteMode -> no deployment");
        return;
    }
    else if (recoveryMode == RecoveryMode::TwoParachuteMode)
    {
        LOG_INFO("RocketFSM", "STABILIZATION exit recovery policy: TwoParachuteMode -> deploy MAIN");
        deployMain();
    }
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
        #if AURORA_HIL_ENABLED
        .addTask(TaskConfig(TaskType::HIL_SIMULATION, "HIL_Calib", (4096+1024), TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Calib1", 4096, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Calib", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true))
        //.addTask((TaskConfig(TaskType::AIRBRAKES, "Airbrakes_Calib", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)))
        //.addTask(TaskConfig(TaskType::STORAGE, "Storage_Calib", 8192, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        ;
    // GROUND_SERVICES state
    _stateActions[RocketState::GROUND_SERVICES] = std::make_unique<StateAction>(RocketState::GROUND_SERVICES);
    _stateActions[RocketState::GROUND_SERVICES]
        ->setEntryAction([this]() {
            LOG_INFO("RocketFSM", "Entering GROUND_SERVICES");
            // TODO: move startWifiStaForEspNow() here?
        })
        .setExitAction([this]() {
            LOG_INFO("RocketFSM", "Exiting GROUND_SERVICES");
            // TODO: move stopWifiStaForEspNow() here?
        })
        /*
         * Keep Ground Services lean on ESP32-S3-WROOM-1-N16.
         * HTTPS/TLS needs contiguous internal RAM per connection, so sensor,
         * GPS, and telemetry tasks are intentionally stopped in this state.
         */
        .addTask(TaskConfig(TaskType::GROUND_SERVICES, "GroundServices", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true))
        #if AURORA_HIL_ENABLED
        .addTask(TaskConfig(TaskType::HIL_SIMULATION, "HIL_Ground", (4096+1024), TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Ground", 4096, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Ground", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Ground", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true))
        ;
    // READY_FOR_LAUNCH state
    _stateActions[RocketState::READY_FOR_LAUNCH] = std::make_unique<StateAction>(RocketState::READY_FOR_LAUNCH);
    _stateActions[RocketState::READY_FOR_LAUNCH]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering READY_FOR_LAUNCH"); })
        #if AURORA_HIL_ENABLED
        // TODO: do not treat HilSimulationTask as a normal fsm task, start in ONCE from rocketfsm.
        .addTask(TaskConfig(TaskType::HIL_SIMULATION, "HIL_Ready", (4096+1024), TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Ready", 4096, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Ready", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        //.addTask(TaskConfig(TaskType::STORAGE, "Storage_Ready", 8192, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        //.addTask((TaskConfig(TaskType::AIRBRAKES, "Airbrakes_Ready", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)))
        .addTask(TaskConfig(TaskType::ALTITUDE, "Altitude_Ready", 4096, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask((TaskConfig(TaskType::TELEMETRY, "Telemetry_Ready", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true)))
        ;

    // LAUNCH state
    _stateActions[RocketState::LAUNCH] = std::make_unique<StateAction>(RocketState::LAUNCH);
    _stateActions[RocketState::LAUNCH]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering LAUNCH"); })
        #if AURORA_HIL_ENABLED
        .addTask(TaskConfig(TaskType::HIL_SIMULATION, "HIL_Launch", (4096+1024), TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Launch", 4096, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Launch", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::STORAGE, "Storage_Launch", 8192, TaskPriority::TASK_CRITICAL, TaskCore::CORE_1, true))
        //.addTask((TaskConfig(TaskType::AIRBRAKES, "Airbrakes_Launch", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)))
        .addTask(TaskConfig(TaskType::ALTITUDE, "Altitude_Launch", 4096, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Launch", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));
    _stateActions[RocketState::ACCELERATED_FLIGHT] = std::make_unique<StateAction>(RocketState::ACCELERATED_FLIGHT);
    _stateActions[RocketState::ACCELERATED_FLIGHT]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering ACCELERATED_FLIGHT"); })
        .addTask(TaskConfig(TaskType::ALTITUDE, "Altitude_Accel", 4096, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        #if AURORA_HIL_ENABLED
        .addTask(TaskConfig(TaskType::HIL_SIMULATION, "HIL_Accel", (4096+1024), TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)) // Might need way more memory
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Accel", 4096, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Accel", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::STORAGE, "Storage_Accel", 8192, TaskPriority::TASK_CRITICAL, TaskCore::CORE_1, true))
        //.addTask((TaskConfig(TaskType::AIRBRAKES, "Airbrakes_Accel", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Accel", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));

    _stateActions[RocketState::BALLISTIC_FLIGHT] = std::make_unique<StateAction>(RocketState::BALLISTIC_FLIGHT);
    _stateActions[RocketState::BALLISTIC_FLIGHT]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering BALLISTIC_FLIGHT"); })
        #if AURORA_HIL_ENABLED
        .addTask(TaskConfig(TaskType::HIL_SIMULATION, "HIL_Ballistic", (4096+1024), TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Ballistic", 4096, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Ballistic", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::STORAGE, "Storage_Ballistic", 8192, TaskPriority::TASK_CRITICAL, TaskCore::CORE_1, true))
        //.addTask((TaskConfig(TaskType::AIRBRAKES, "Airbrakes_Ballistic", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)))
        .addTask(TaskConfig(TaskType::ALTITUDE, "Altitude_Ballistic", 4096, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Ballistic", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));

    _stateActions[RocketState::APOGEE] = std::make_unique<StateAction>(RocketState::APOGEE);
    _stateActions[RocketState::APOGEE]
        ->setEntryAction([this]()
                        {
                            LOG_INFO("RocketFSM", "Entering APOGEE");
                            deployApogeeRecovery();
                            // tone(BUZZER_PIN, 1000, 500);             // Sound buzzer at 1kHz for 500ms
                        })
        #if AURORA_HIL_ENABLED
        .addTask(TaskConfig(TaskType::HIL_SIMULATION, "HIL_Apogee", (4096+1024), TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Apogee", 4096, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Apogee", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::ALTITUDE, "Altitude_Apogee", 4096, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::STORAGE, "Storage_Apogee", 8192, TaskPriority::TASK_CRITICAL, TaskCore::CORE_1, true))
        // .addTask((TaskConfig(TaskType::AIRBRAKES, "Airbrakes_Apogee", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Apogee", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));

    _stateActions[RocketState::STABILIZATION] = std::make_unique<StateAction>(RocketState::STABILIZATION);
    _stateActions[RocketState::STABILIZATION]
        ->setExitAction([this]()
                        {
                            LOG_INFO("RocketFSM", "Exiting STABILIZATION");
                            deployStabilizationExitRecovery();
                            // tone(BUZZER_PIN, 1000, 500);           // Sound buzzer at 1kHz for 500ms
                        })
        #if AURORA_HIL_ENABLED
        .addTask(TaskConfig(TaskType::HIL_SIMULATION, "HIL_Stabilization", (4096+1024), TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Stabilization", 4096, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Stabilization", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::ALTITUDE, "Altitude_Stabilization", 4096, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::STORAGE, "Storage_Stabilization", 8192, TaskPriority::TASK_CRITICAL, TaskCore::CORE_1, true))
        // .addTask((TaskConfig(TaskType::AIRBRAKES, "Airbrakes_Stabilization", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Stabilization", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));

    _stateActions[RocketState::DECELERATION] = std::make_unique<StateAction>(RocketState::DECELERATION);
    _stateActions[RocketState::DECELERATION]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering DECELERATION"); })
        #if AURORA_HIL_ENABLED
        .addTask(TaskConfig(TaskType::HIL_SIMULATION, "HIL_Deceleration", (4096+1024), TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Deceleration", 4096, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Deceleration", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::ALTITUDE, "Altitude_Deceleration", 4096, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::STORAGE, "Storage_Deceleration_8", 8192, TaskPriority::TASK_CRITICAL, TaskCore::CORE_1, true))
        // .addTask((TaskConfig(TaskType::AIRBRAKES, "Airbrakes_Deceleration", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Deceleration", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));
    _stateActions[RocketState::LANDING] = std::make_unique<StateAction>(RocketState::LANDING);
    _stateActions[RocketState::LANDING]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering LANDING"); })
        #if AURORA_HIL_ENABLED
        .addTask(TaskConfig(TaskType::HIL_SIMULATION, "HIL_Landing", (4096+1024), TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Landing", 4096, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Landing", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        .addTask(TaskConfig(TaskType::ALTITUDE, "Altitude_Landing", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::STORAGE, "Storage_Landing", 8192, TaskPriority::TASK_CRITICAL, TaskCore::CORE_1, true))
        // .addTask((TaskConfig(TaskType::AIRBRAKES, "Airbrakes_Landing", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Landing", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));
    _stateActions[RocketState::RECOVERED] = std::make_unique<StateAction>(RocketState::RECOVERED);
    _stateActions[RocketState::RECOVERED]
        ->setEntryAction([this]()
                         { LOG_INFO("RocketFSM", "Entering RECOVERED"); })
        #if AURORA_HIL_ENABLED
        .addTask(TaskConfig(TaskType::HIL_SIMULATION, "HIL_Recovered", (4096+1024), TaskPriority::TASK_HIGH, TaskCore::CORE_0, true))
        #else
        .addTask(TaskConfig(TaskType::SENSOR, "Sensor_Recovered", 4096, TaskPriority::TASK_CRITICAL, TaskCore::CORE_0, true))
        .addTask(TaskConfig(TaskType::GPS, "Gps_Recovered", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_1, true))
        #endif
        // .addTask(TaskConfig(TaskType::STORAGE, "Storage_Recovered", 8192, TaskPriority::TASK_CRITICAL, TaskCore::CORE_1, true))
        // .addTask((TaskConfig(TaskType::AIRBRAKES, "Airbrakes_Recovered", 4096, TaskPriority::TASK_HIGH, TaskCore::CORE_0, true)))
        .addTask(TaskConfig(TaskType::TELEMETRY, "Telemetry_Recovered", 4096, TaskPriority::TASK_MEDIUM, TaskCore::CORE_1, true));
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
        RocketState::GROUND_SERVICES,
        FSMEvent::CALIBRATION_COMPLETE));

    _transitionManager->addTransition(Transition(
        RocketState::GROUND_SERVICES,
        RocketState::READY_FOR_LAUNCH,
        FSMEvent::START_READY_FOR_LAUNCH));

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
        if (_logger) {
            char message[64];
            snprintf(message, sizeof(message), "%.24s -> %.24s",
                     getStateString(_currentState), getStateString(newState));
            _logger->logInfo("RocketFSM", message);
        }

        // Execute exit action for current state
        if (_stateActions[_currentState])
        {
            _stateActions[_currentState]->onExit();
        }

        
        // Fetch configurations for current (old) state and target (new) state
        std::vector<TaskConfig> oldTaskConfigs;
        if (_stateActions[_currentState])
        {
            oldTaskConfigs = _stateActions[_currentState]->getTaskConfigs();
        }

        std::vector<TaskConfig> newTaskConfigs;
        if (_stateActions[newState])
        {
            newTaskConfigs = _stateActions[newState]->getTaskConfigs();
        }

        if (_taskManager)
        {
            // Stop tasks that are no longer needed in the new state
            for (const auto& oldTask : oldTaskConfigs)
            {
                auto it = std::find_if(newTaskConfigs.begin(), newTaskConfigs.end(),
                                       [&](const TaskConfig& c) { return c.type == oldTask.type; });

                if (it == newTaskConfigs.end())
                {
                    try {
                        LOG_INFO("RocketFSM", "[TRANSITION] Task Diff: Stopping no longer needed task type %d", static_cast<int>(oldTask.type));
                        _taskManager->stopTask(oldTask.type);
                    } catch (const std::exception &e) {
                        LOG_ERROR("RocketFSM", "[TRANSITION] ERROR: Exception stopping task: %s", e.what());
                    }
                }
            }
        }

        // Update internal FSM state tracking
        _previousState = _currentState;
        _currentState = newState;
        _stateStartTime = Utils::millis();

        // Execute entry action for new state
        if (_stateActions[_currentState])
        {
            _stateActions[_currentState]->onEntry();
        }

        // Start tasks that are newly required in the new state
        if (_taskManager)
        {
            for (const auto& newTask : newTaskConfigs)
            {
                auto it = std::find_if(oldTaskConfigs.begin(), oldTaskConfigs.end(),
                                       [&](const TaskConfig& c) { return c.type == newTask.type; });

                if (it == oldTaskConfigs.end())
                {
                    try {
                        LOG_INFO("RocketFSM", "[TRANSITION] Task Diff: Starting newly required task type %d", static_cast<int>(newTask.type));
                        _taskManager->startTask(newTask.type, newTask);
                    } catch (const std::exception &e) {
                        LOG_ERROR("RocketFSM", "[TRANSITION] ERROR: Exception starting task: %s", e.what());
                    }
                }
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
    if (!newState.has_value())
    {
        LOG_WARNING("RocketFSM", "No valid transition for event %d in state %s",
                    static_cast<int>(eventData.event),
                    getStateString(_currentState));
        return;
    }

    if (_currentState == RocketState::GROUND_SERVICES && eventData.event == FSMEvent::START_READY_FOR_LAUNCH)
    {
        char reason[128] = {};
        if (runtime_config_lock_for_flight(reason, sizeof(reason)) != ESP_OK)
        {
            LOG_ERROR("RocketFSM", "Refusing READY_FOR_LAUNCH: config lock failed: %s", reason);
            return;
        }
    }

    transitionTo(newState.value());
}

void RocketFSM::checkTransitions()
{
    // Check for automatic transitions quickly (fast paths first)
    if (_transitionManager->checkAutomaticTransitions(_currentState, _stateStartTime))
    {
        return; // automatic transition handled
    }

    // Cache some persistent values across calls to avoid repeated allocations
    static uint32_t launchHighSince = 0;

    // Get accelerometer data before switch statement
    auto accX = 0.0f;
    auto accY = 0.0f;
    auto accZ = 0.0f;
    IMUData outBno055Data;
    SensorReadStatus imuStatus = _rocketModel->getBNO055Data(outBno055Data);
    
    if(imuStatus == SensorReadStatus::OK) {
        accX = outBno055Data.acceleration_x;
        accY = outBno055Data.acceleration_y;
        accZ = outBno055Data.acceleration_z;
    }

    const RuntimeConfig runtimeCfg = runtime_config_get_flight_snapshot();

    // Fast state-based checks
    switch (_currentState)
    {
    case RocketState::INACTIVE:
        sendEvent(FSMEvent::START_CALIBRATION);
        break;

    case RocketState::GROUND_SERVICES:
        // Wait for an explicit authenticated dashboard or operator event.
        break;

    case RocketState::CALIBRATING:
    {
        PressureSensorData outBaroData;
        const SensorReadStatus baroStatus = runtimeCfg.altitude.selected_barometer == 1
            ? _rocketModel->getMS561101BA03Data_1(outBaroData)
            : _rocketModel->getMS561101BA03Data_2(outBaroData);
        bool useBaroSample = baroStatus == SensorReadStatus::OK;

#if AURORA_HIL_ENABLED
        static bool hasLastBaroTimestamp = false;
        static uint32_t lastBaroTimestamp = 0;

        if (useBaroSample)
        {
            useBaroSample = !hasLastBaroTimestamp || (outBaroData.timestamp != lastBaroTimestamp);

            if (useBaroSample)
            {
                hasLastBaroTimestamp = true;
                lastBaroTimestamp = outBaroData.timestamp;
            }
        }
#endif

        if (useBaroSample)
        {
            if (outBaroData.pressure > 0.0f)
            {
                _rocketModel->addBarometerSample(outBaroData.pressure, runtimeCfg.calibration.barometer_samples);
            }

            _rocketModel->addTemperatureSample(outBaroData.temperature + 273.15f, runtimeCfg.calibration.temperature_samples);
        }

        const bool isBaroReady = _rocketModel->isBarometerZeroed();
        const bool isBnoReady  = _rocketModel->isSensorSystemCalibrated();

        if (useBaroSample)
        {
            LOG_INFO("RocketFSM", "CALIBRATING: Baro Ready=%d (samples=%u), IMU Ready=%d",
                    isBaroReady, _rocketModel->getBarometerSampleCount(), isBnoReady);
        }

        if (isBaroReady && isBnoReady)
        {
            LOG_INFO("RocketFSM", "Calibration complete! Baro zeroed & IMU calibrated.");
            sendEvent(FSMEvent::CALIBRATION_COMPLETE);
        }
        else if (Utils::millis() - _stateStartTime > runtimeCfg.calibration.timeout_ms)
        {
            LOG_WARNING("RocketFSM", "Calibration timeout! Forcing completion. Baro Ready: %d, IMU Ready: %d",
                        isBaroReady, isBnoReady);
            sendEvent(FSMEvent::CALIBRATION_COMPLETE);
        }

        break;
    }
    case RocketState::READY_FOR_LAUNCH:
        try {
            auto accMag = sqrt(accX * accX + accY * accY + accZ * accZ);

            if (accMag > runtimeCfg.flight.liftoff_accel_threshold_mps2)
            {
                if (launchHighSince == 0)
                {
                    launchHighSince = Utils::millis();
                }
                else if (Utils::millis() - launchHighSince >= runtimeCfg.flight.liftoff_timeout_ms)
                {
                    _launchDetectionTime = Utils::millis();
                    sendEvent(FSMEvent::LAUNCH_DETECTED);
                }
            }
            else 
            {
                launchHighSince = 0;
            }
        } catch (const std::exception& e) {
            LOG_ERROR("RocketFSM", "READY_FOR_LAUNCH: Exception occurred: %s", e.what());
        }
        
        break;

    case RocketState::LAUNCH:
        // After a short delay consider liftoff started (rocket left the launch pad and is accelerating)
        LOG_INFO("RocketFSM", "Now: %lu, stateStartTime: %lu, LAUNCH: elapsed=%lu ms", Utils::millis(), _stateStartTime, Utils::millis() - _stateStartTime);
        sendEvent(FSMEvent::LIFTOFF_STARTED);
        break;

    case RocketState::ACCELERATED_FLIGHT:
        
        if (Utils::millis() - _launchDetectionTime >= runtimeCfg.flight.launch_to_ballistic_threshold_ms)
        {
            sendEvent(FSMEvent::ACCELERATION_COMPLETE);
        }
        break;

    case RocketState::BALLISTIC_FLIGHT:
    {
        auto elapsed = Utils::millis() - _launchDetectionTime;
        
        // Ignore all sensor apogee logic until the initial chaotic burn phase ends
        if (elapsed > runtimeCfg.flight.apogee_lockout_ms)
        {
            if (!_rocketModel->getIsRising() || (elapsed >= runtimeCfg.flight.launch_to_apogee_threshold_ms))
            {
                LOG_INFO("RocketFSM", "Apogee detected! Elapsed: %lu ms", elapsed);
                sendEvent(FSMEvent::APOGEE_REACHED);
            }
        } 
        // If we somehow haven't hit the lockout but the max time elapsed, trigger anyway
        else if (elapsed >= runtimeCfg.flight.launch_to_apogee_threshold_ms)
        {
            LOG_WARNING("RocketFSM", "Apogee Lockout bypassed due to absolute max time limit!");
            sendEvent(FSMEvent::APOGEE_REACHED);
        }

        break;
    }

    case RocketState::APOGEE:
        LOG_INFO("RocketFSM", "Drogue Opened! %d", Utils::millis()-_launchDetectionTime);
        if (Utils::millis() - _stateStartTime >= runtimeCfg.flight.drogue_apogee_timeout_ms)
        {
            sendEvent(FSMEvent::DROGUE_READY);
        }
        break;

    case RocketState::STABILIZATION:
    {
        auto currentHeight = _rocketModel->getCurrentHeight();
        
        // LOG_INFO("RocketFSM", "STABILIZATION: altitude=%.3f", currentHeight);
        
        if (currentHeight < runtimeCfg.flight.main_altitude_threshold_m)
        {
            LOG_INFO("RocketFSM", "STABILIZATION: condition met (altitude=%.3f, elapsed=%lu ms)", currentHeight, Utils::millis() - _stateStartTime);
            sendEvent(FSMEvent::STABILIZATION_COMPLETE);
        }
    
        break;
    }

    case RocketState::DECELERATION:
    {
        const float currentHeight = _rocketModel->getCurrentHeight();

        if (currentHeight < runtimeCfg.flight.touchdown_altitude_threshold_m)
        {
            sendEvent(FSMEvent::DECELERATION_COMPLETE);
        }
        
        break;
    }

    case RocketState::LANDING:
        if (Utils::millis() - _stateStartTime > 2000U)
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
