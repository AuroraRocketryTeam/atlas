#include "RocketFSM.hpp"
#include <config.h>
#include <KalmanFilter.hpp>

using TransmitDataType = std::variant<char *, String, std::string, nlohmann::json>;

// External references to your system components
extern ISensor *bno055;
extern ISensor *baro1;
extern ISensor *baro2;
extern ISensor *gps;
extern ILogger *rocketLogger;
extern ITransmitter<TransmitDataType> *loraTransmitter;

RocketFSM::RocketFSM()
    : currentState(RocketState::INACTIVE), previousState(RocketState::INACTIVE), stateStartTime(0), lastUpdateTime(0), isRunning(false), fsmTaskHandle(nullptr), sensorTaskHandle(nullptr), ekfTaskHandle(nullptr), apogeeDetectionTaskHandle(nullptr), recoveryTaskHandle(nullptr), dataCollectionTaskHandle(nullptr), telemetryTaskHandle(nullptr), gpsTaskHandle(nullptr), loggingTaskHandle(nullptr), eventQueue(nullptr), stateMutex(nullptr), sharedData{SensorData("bno055"), SensorData("baro1"), SensorData("baro2"), SensorData("gps"), 0, false}
{
}

RocketFSM::~RocketFSM()
{
    stop();

    if (eventQueue)
    {
        vQueueDelete(eventQueue);
    }
    if (stateMutex)
    {
        vSemaphoreDelete(stateMutex);
    }
}

void RocketFSM::init()
{
    // Create FreeRTOS objects
    eventQueue = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(FSMEventData));
    stateMutex = xSemaphoreCreateMutex();

    if (!eventQueue || !stateMutex)
    {
        if (rocketLogger)
        {
            rocketLogger->logError("Failed to create FSM FreeRTOS objects");
        }
        return;
    }

    setupStateActions();
    setupTransitions();

    currentState = RocketState::INACTIVE;
    stateStartTime = millis();
    lastUpdateTime = millis();

    if (rocketLogger)
    {
        rocketLogger->logInfo("FSM initialized in INACTIVE state");
    }
}

void RocketFSM::start()
{
    if (isRunning)
        return;

    BaseType_t result = xTaskCreatePinnedToCore(
        fsmTaskWrapper,
        "FSM_Task",
        FSM_TASK_STACK_SIZE,
        this,
        FSM_TASK_PRIORITY,
        &fsmTaskHandle,
        1 // Run FSM coordinator on Core 1
    );

    if (result == pdPASS)
    {
        isRunning = true;
        if (rocketLogger)
        {
            rocketLogger->logInfo("FSM task started on Core 1");
        }

        // Execute entry action for initial state
        if (stateActions[currentState].onEntry)
        {
            stateActions[currentState].onEntry();
        }
        startStateTasks();
    }
    else
    {
        if (rocketLogger)
        {
            rocketLogger->logError("Failed to start FSM task");
        }
    }
}

void RocketFSM::stop()
{
    if (!isRunning)
        return;

    isRunning = false;
    stopAllTasks();

    if (fsmTaskHandle)
    {
        vTaskDelete(fsmTaskHandle);
        fsmTaskHandle = nullptr;
    }

    if (rocketLogger)
    {
        rocketLogger->logInfo("FSM stopped");
    }
}

bool RocketFSM::sendEvent(FSMEvent event, RocketState targetState, void *eventData)
{
    if (!eventQueue)
        return false;

    FSMEventData eventStruct(event, targetState, eventData);
    return xQueueSend(eventQueue, &eventStruct, pdMS_TO_TICKS(100)) == pdPASS;
}

RocketState RocketFSM::getCurrentState()
{
    RocketState state = currentState;
    if (stateMutex && xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE)
    {
        state = currentState;
        xSemaphoreGive(stateMutex);
    }
    return state;
}

FlightPhase RocketFSM::getCurrentPhase()
{
    RocketState state = getCurrentState();

    if (state == RocketState::CALIBRATING ||
        state == RocketState::READY_FOR_LAUNCH)
    {
        return FlightPhase::PRE_FLIGHT;
    }

    if (state >= RocketState::LAUNCH &&
        state <= RocketState::APOGEE)
    {
        return FlightPhase::FLIGHT;
    }

    if (state >= RocketState::STABILIZATION &&
        state <= RocketState::RECOVERED)
    {
        return FlightPhase::RECOVERY;
    }

    return FlightPhase::PRE_FLIGHT;
}

void RocketFSM::fsmTaskWrapper(void *parameter)
{
    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    fsm->fsmTask();
}

void RocketFSM::fsmTask()
{
    FSMEventData eventData(FSMEvent::CALIBRATION_COMPLETE);
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (isRunning)
    {
        // Check for events (non-blocking with timeout)
        if (xQueueReceive(eventQueue, &eventData, pdMS_TO_TICKS(50)) == pdPASS)
        {
            processEvent(eventData);
        }

        // Update last update time
        lastUpdateTime = millis();

        // Task delay to prevent busy waiting
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50)); // 20Hz update rate
    }
}

void RocketFSM::setupStateActions()
{
    // INACTIVE state
    stateActions[RocketState::INACTIVE] = StateActions(
        [this]()
        { onInactiveEntry(); });

    // CALIBRATING state
    stateActions[RocketState::CALIBRATING] = StateActions(
        [this]()
        { onCalibratingEntry(); },
        [this]()
        { onCalibratingExit(); });
    stateActions[RocketState::CALIBRATING].sensorTask = TaskConfig("Sensor_Calib", 3072, 4, 0, true);
    stateActions[RocketState::CALIBRATING].loggingTask = TaskConfig("Log_Calib", 2048, 1, 1, true);

    // READY_FOR_LAUNCH state
    stateActions[RocketState::READY_FOR_LAUNCH] = StateActions(
        [this]()
        { onReadyForLaunchEntry(); });
    stateActions[RocketState::READY_FOR_LAUNCH].sensorTask = TaskConfig("Sensor_Ready", 3072, 4, 0, true);
    stateActions[RocketState::READY_FOR_LAUNCH].telemetryTask = TaskConfig("Telemetry_Ready", 3072, 2, 1, true);
    stateActions[RocketState::READY_FOR_LAUNCH].loggingTask = TaskConfig("Log_Ready", 2048, 1, 1, true);

    // LAUNCH state
    stateActions[RocketState::LAUNCH] = StateActions(
        [this]()
        { onLaunchEntry(); });
    stateActions[RocketState::LAUNCH].sensorTask = TaskConfig("Sensor_Launch", 3072, 5, 0, true);
    stateActions[RocketState::LAUNCH].dataCollectionTask = TaskConfig("DataCol_Launch", 2048, 2, 1, true);
    stateActions[RocketState::LAUNCH].loggingTask = TaskConfig("Log_Launch", 2048, 1, 1, true);

    // ACCELERATED_FLIGHT state
    stateActions[RocketState::ACCELERATED_FLIGHT] = StateActions(
        [this]()
        { onAcceleratedFlightEntry(); });
    stateActions[RocketState::ACCELERATED_FLIGHT].sensorTask = TaskConfig("Sensor_AccFlight", 3072, 5, 0, true);
    stateActions[RocketState::ACCELERATED_FLIGHT].ekfTask = TaskConfig("EKF_AccFlight", 4096, 4, 0, true);
    stateActions[RocketState::ACCELERATED_FLIGHT].telemetryTask = TaskConfig("Telemetry_AccFlight", 3072, 2, 1, true);
    stateActions[RocketState::ACCELERATED_FLIGHT].dataCollectionTask = TaskConfig("DataCol_AccFlight", 2048, 2, 1, true);

    // BALLISTIC_FLIGHT state
    stateActions[RocketState::BALLISTIC_FLIGHT] = StateActions(
        [this]()
        { onBallisticFlightEntry(); });
    stateActions[RocketState::BALLISTIC_FLIGHT].sensorTask = TaskConfig("Sensor_BalFlight", 3072, 5, 0, true);
    stateActions[RocketState::BALLISTIC_FLIGHT].ekfTask = TaskConfig("EKF_BalFlight", 4096, 4, 0, true);
    stateActions[RocketState::BALLISTIC_FLIGHT].apogeeDetectionTask = TaskConfig("Apogee_Detection", 2048, 4, 0, true);
    stateActions[RocketState::BALLISTIC_FLIGHT].telemetryTask = TaskConfig("Telemetry_BalFlight", 3072, 2, 1, true);
    stateActions[RocketState::BALLISTIC_FLIGHT].dataCollectionTask = TaskConfig("DataCol_BalFlight", 2048, 2, 1, true);

    // APOGEE state
    stateActions[RocketState::APOGEE] = StateActions(
        [this]()
        { onApogeeEntry(); });
    stateActions[RocketState::APOGEE].sensorTask = TaskConfig("Sensor_Apogee", 3072, 5, 0, true);
    stateActions[RocketState::APOGEE].recoveryTask = TaskConfig("Recovery_Apogee", 2048, 5, 0, true);
    stateActions[RocketState::APOGEE].dataCollectionTask = TaskConfig("DataCol_Apogee", 2048, 2, 1, true);
    stateActions[RocketState::APOGEE].telemetryTask = TaskConfig("Telemetry_Apogee", 3072, 2, 1, true);

    // STABILIZATION state
    stateActions[RocketState::STABILIZATION] = StateActions(
        [this]()
        { onStabilizationEntry(); });
    stateActions[RocketState::STABILIZATION].sensorTask = TaskConfig("Sensor_Stab", 3072, 4, 0, true);
    stateActions[RocketState::STABILIZATION].recoveryTask = TaskConfig("Recovery_Stab", 2048, 4, 0, true);
    stateActions[RocketState::STABILIZATION].telemetryTask = TaskConfig("Telemetry_Stab", 3072, 2, 1, true);
    stateActions[RocketState::STABILIZATION].dataCollectionTask = TaskConfig("DataCol_Stab", 2048, 2, 1, true);
    stateActions[RocketState::STABILIZATION].gpsTask = TaskConfig("GPS_Stab", 2048, 1, 1, true);

    // DECELERATION state
    stateActions[RocketState::DECELERATION] = StateActions(
        [this]()
        { onDecelerationEntry(); });
    stateActions[RocketState::DECELERATION].sensorTask = TaskConfig("Sensor_Decel", 3072, 4, 0, true);
    stateActions[RocketState::DECELERATION].recoveryTask = TaskConfig("Recovery_Decel", 2048, 4, 0, true);
    stateActions[RocketState::DECELERATION].telemetryTask = TaskConfig("Telemetry_Decel", 3072, 2, 1, true);
    stateActions[RocketState::DECELERATION].dataCollectionTask = TaskConfig("DataCol_Decel", 2048, 2, 1, true);
    stateActions[RocketState::DECELERATION].gpsTask = TaskConfig("GPS_Decel", 2048, 1, 1, true);

    // LANDING state
    stateActions[RocketState::LANDING] = StateActions(
        [this]()
        { onLandingEntry(); },
        [this]()
        { onLandingExit(); });
    stateActions[RocketState::LANDING].sensorTask = TaskConfig("Sensor_Landing", 2048, 3, 0, true);
    stateActions[RocketState::LANDING].dataCollectionTask = TaskConfig("DataCol_Landing", 2048, 2, 1, true);
    stateActions[RocketState::LANDING].gpsTask = TaskConfig("GPS_Landing", 2048, 1, 1, true);
    stateActions[RocketState::LANDING].loggingTask = TaskConfig("Log_Landing", 3072, 1, 1, true);

    // RECOVERED state
    stateActions[RocketState::RECOVERED] = StateActions(
        [this]()
        { onRecoveredEntry(); },
        [this]()
        { onRecoveredExit(); });
    stateActions[RocketState::RECOVERED].gpsTask = TaskConfig("GPS_Recovered", 2048, 1, 1, true);
    stateActions[RocketState::RECOVERED].loggingTask = TaskConfig("Log_Recovered", 3072, 1, 1, true);
}

void RocketFSM::setupTransitions()
{
    transitions.emplace_back(RocketState::INACTIVE, RocketState::CALIBRATING, FSMEvent::CALIBRATION_COMPLETE);
    transitions.emplace_back(RocketState::CALIBRATING, RocketState::READY_FOR_LAUNCH, FSMEvent::CALIBRATION_COMPLETE);
    transitions.emplace_back(RocketState::READY_FOR_LAUNCH, RocketState::LAUNCH, FSMEvent::LAUNCH_DETECTED);
    transitions.emplace_back(RocketState::LAUNCH, RocketState::ACCELERATED_FLIGHT, FSMEvent::LIFTOFF_STARTED);
    transitions.emplace_back(RocketState::ACCELERATED_FLIGHT, RocketState::BALLISTIC_FLIGHT, FSMEvent::ACCELERATION_COMPLETE);
    transitions.emplace_back(RocketState::BALLISTIC_FLIGHT, RocketState::APOGEE, FSMEvent::APOGEE_REACHED);
    transitions.emplace_back(RocketState::APOGEE, RocketState::STABILIZATION, FSMEvent::DROGUE_READY);
    transitions.emplace_back(RocketState::STABILIZATION, RocketState::DECELERATION, FSMEvent::STABILIZATION_COMPLETE);
    transitions.emplace_back(RocketState::DECELERATION, RocketState::LANDING, FSMEvent::DECELERATION_COMPLETE);
    transitions.emplace_back(RocketState::LANDING, RocketState::RECOVERED, FSMEvent::LANDING_COMPLETE);
}

void RocketFSM::transitionTo(RocketState newState)
{
    if (newState == currentState)
        return;

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        if (rocketLogger)
        {
            String transitionMsg = "FSM transition: " + getStateString(currentState) + " -> " + getStateString(newState);
            rocketLogger->logInfo(transitionMsg.c_str());
        }

        // Execute exit action for current state
        if (stateActions[currentState].onExit)
        {
            stateActions[currentState].onExit();
        }

        // Stop current state tasks
        stopAllTasks();

        previousState = currentState;
        currentState = newState;
        stateStartTime = millis();

        // Execute entry action for new state
        if (stateActions[currentState].onEntry)
        {
            stateActions[currentState].onEntry();
        }

        // Start new state tasks
        startStateTasks();

        xSemaphoreGive(stateMutex);
    }
}

void RocketFSM::processEvent(const FSMEventData &eventData)
{
    if (eventData.event == FSMEvent::FORCE_TRANSITION)
    {
        transitionTo(eventData.targetState);
        return;
    }

    // Find matching transition
    for (const auto &transition : transitions)
    {
        if (transition.fromState == currentState && transition.triggerEvent == eventData.event)
        {
            transitionTo(transition.toState);
            break;
        }
    }
}

void RocketFSM::startStateTasks()
{
    const StateActions &actions = stateActions[currentState];

    // Start Core 0 tasks (Critical)
    if (actions.sensorTask.shouldRun)
    {
        xTaskCreatePinnedToCore(sensorTaskWrapper, actions.sensorTask.name, actions.sensorTask.stackSize,
                                this, actions.sensorTask.priority, &sensorTaskHandle, actions.sensorTask.coreId);
    }

    if (actions.ekfTask.shouldRun)
    {
        xTaskCreatePinnedToCore(ekfTaskWrapper, actions.ekfTask.name, actions.ekfTask.stackSize,
                                this, actions.ekfTask.priority, &ekfTaskHandle, actions.ekfTask.coreId);
    }

    if (actions.apogeeDetectionTask.shouldRun)
    {
        xTaskCreatePinnedToCore(apogeeDetectionTaskWrapper, actions.apogeeDetectionTask.name, actions.apogeeDetectionTask.stackSize,
                                this, actions.apogeeDetectionTask.priority, &apogeeDetectionTaskHandle, actions.apogeeDetectionTask.coreId);
    }

    if (actions.recoveryTask.shouldRun)
    {
        xTaskCreatePinnedToCore(recoveryTaskWrapper, actions.recoveryTask.name, actions.recoveryTask.stackSize,
                                this, actions.recoveryTask.priority, &recoveryTaskHandle, actions.recoveryTask.coreId);
    }

    // Start Core 1 tasks (Non-critical)
    if (actions.dataCollectionTask.shouldRun)
    {
        xTaskCreatePinnedToCore(dataCollectionTaskWrapper, actions.dataCollectionTask.name, actions.dataCollectionTask.stackSize,
                                this, actions.dataCollectionTask.priority, &dataCollectionTaskHandle, actions.dataCollectionTask.coreId);
    }

    if (actions.telemetryTask.shouldRun)
    {
        xTaskCreatePinnedToCore(telemetryTaskWrapper, actions.telemetryTask.name, actions.telemetryTask.stackSize,
                                this, actions.telemetryTask.priority, &telemetryTaskHandle, actions.telemetryTask.coreId);
    }

    if (actions.gpsTask.shouldRun)
    {
        xTaskCreatePinnedToCore(gpsTaskWrapper, actions.gpsTask.name, actions.gpsTask.stackSize,
                                this, actions.gpsTask.priority, &gpsTaskHandle, actions.gpsTask.coreId);
    }

    if (actions.loggingTask.shouldRun)
    {
        xTaskCreatePinnedToCore(loggingTaskWrapper, actions.loggingTask.name, actions.loggingTask.stackSize,
                                this, actions.loggingTask.priority, &loggingTaskHandle, actions.loggingTask.coreId);
    }
}

void RocketFSM::stopAllTasks()
{
    // Stop Core 0 tasks
    if (sensorTaskHandle)
    {
        vTaskDelete(sensorTaskHandle);
        sensorTaskHandle = nullptr;
    }
    if (ekfTaskHandle)
    {
        vTaskDelete(ekfTaskHandle);
        ekfTaskHandle = nullptr;
    }
    if (apogeeDetectionTaskHandle)
    {
        vTaskDelete(apogeeDetectionTaskHandle);
        apogeeDetectionTaskHandle = nullptr;
    }
    if (recoveryTaskHandle)
    {
        vTaskDelete(recoveryTaskHandle);
        recoveryTaskHandle = nullptr;
    }

    // Stop Core 1 tasks
    if (dataCollectionTaskHandle)
    {
        vTaskDelete(dataCollectionTaskHandle);
        dataCollectionTaskHandle = nullptr;
    }
    if (telemetryTaskHandle)
    {
        vTaskDelete(telemetryTaskHandle);
        telemetryTaskHandle = nullptr;
    }
    if (gpsTaskHandle)
    {
        vTaskDelete(gpsTaskHandle);
        gpsTaskHandle = nullptr;
    }
    if (loggingTaskHandle)
    {
        vTaskDelete(loggingTaskHandle);
        loggingTaskHandle = nullptr;
    }
}

// Task wrappers for Core 0 (Critical)
void RocketFSM::sensorTaskWrapper(void *parameter)
{
    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    fsm->sensorTask();
}

void RocketFSM::ekfTaskWrapper(void *parameter)
{
    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    fsm->ekfTask();
}

void RocketFSM::apogeeDetectionTaskWrapper(void *parameter)
{
    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    fsm->apogeeDetectionTask();
}

void RocketFSM::recoveryTaskWrapper(void *parameter)
{
    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    fsm->recoveryTask();
}

// Task wrappers for Core 1 (Non-critical)
void RocketFSM::dataCollectionTaskWrapper(void *parameter)
{
    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    fsm->dataCollectionTask();
}

void RocketFSM::telemetryTaskWrapper(void *parameter)
{
    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    fsm->telemetryTask();
}

void RocketFSM::gpsTaskWrapper(void *parameter)
{
    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    fsm->gpsTask();
}

void RocketFSM::loggingTaskWrapper(void *parameter)
{
    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    fsm->loggingTask();
}

// Core 0 Task implementations (Critical)
void RocketFSM::sensorTask()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (isRunning)
    {
        // High-frequency sensor reading and condition checking
        checkTransitions();

        if(bno055)
        {
            auto data = bno055->getData();
            if (data.has_value())
            {
                sharedData.imuData = data.value();
            }
        }
        if(baro1)
        {
            auto data = baro1->getData();
            if (data.has_value())
            {
                sharedData.baroData1 = data.value();
            }
        }
        if(baro2)
        {
            auto data = baro2->getData();
            if (data.has_value())
            {
                sharedData.baroData2 = data.value();
            }
        }
        // GPS is read in its own task due to lower frequency.
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // 100Hz for critical sensors
    }
}

void RocketFSM::ekfTask()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (isRunning)
    {
        // Run EKF processing
        // TODO: Integrate your KalmanFilter 

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(20)); // 50Hz for EKF
    }
}

void RocketFSM::apogeeDetectionTask()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (isRunning)
    {
        // Critical apogee detection algorithms
        if (isApogeeReached())
        {
            sendEvent(FSMEvent::APOGEE_REACHED);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10)); // 100Hz for apogee detection
    }
}

void RocketFSM::recoveryTask()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (isRunning)
    {
        // Handle recovery system operations
        // TODO: Implement recovery system control

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50)); // 20Hz for recovery
    }
}

// Core 1 Task implementations (Non-critical)
void RocketFSM::dataCollectionTask()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (isRunning)
    {
        rocketLogger->logSensorData(sharedData.imuData);
        rocketLogger->logSensorData(sharedData.baroData1);
        rocketLogger->logSensorData(sharedData.baroData2);
        rocketLogger->logSensorData(sharedData.gpsData);
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50)); // 20Hz for data collection
    }
}

void RocketFSM::telemetryTask()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (isRunning)
    {
        if (loraTransmitter && rocketLogger)
        {
            auto response = loraTransmitter->transmit(rocketLogger->getJSONAll());
            // Process response if needed
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000)); // 1Hz for telemetry
    }
}

void RocketFSM::gpsTask()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (isRunning)
    {
        if (gps)
        {
            auto data = gps->getData();
            if (data.has_value())
            {
                sharedData.gpsData = data.value();
            }
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(200)); // 5Hz for GPS
    }
}

void RocketFSM::loggingTask()
{
    TickType_t xLastWakeTime = xTaskGetTickCount();

    while (isRunning)
    {
        // log to SD card
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(500)); // 2Hz for logging
    }
}

// Legacy compatibility method
void RocketFSM::update()
{
    lastUpdateTime = millis();
    // This method is kept for compatibility but functionality is now handled by tasks
}

void RocketFSM::checkTransitions()
{
    RocketState state = getCurrentState();

    switch (state)
    {
    case RocketState::INACTIVE:
        // Auto-transition to calibrating
        sendEvent(FSMEvent::CALIBRATION_COMPLETE);
        break;

    case RocketState::CALIBRATING:
        if (isCalibrationComplete())
        {
            sendEvent(FSMEvent::CALIBRATION_COMPLETE);
        }
        break;

    case RocketState::READY_FOR_LAUNCH:
        if (isLaunchDetected())
        {
            sendEvent(FSMEvent::LAUNCH_DETECTED);
        }
        break;

    case RocketState::LAUNCH:
        if (isLiftoffStarted())
        {
            sendEvent(FSMEvent::LIFTOFF_STARTED);
        }
        break;

    case RocketState::ACCELERATED_FLIGHT:
        if (isAccelerationPhaseComplete())
        {
            sendEvent(FSMEvent::ACCELERATION_COMPLETE);
        }
        break;

    case RocketState::BALLISTIC_FLIGHT:
        if (isBallisticPhaseComplete())
        {
            sendEvent(FSMEvent::BALLISTIC_COMPLETE);
        }
        break;

    case RocketState::APOGEE:
        if (isDrogueReady())
        {
            sendEvent(FSMEvent::DROGUE_READY);
        }
        break;

    case RocketState::STABILIZATION:
        if (isStabilizationComplete())
        {
            sendEvent(FSMEvent::STABILIZATION_COMPLETE);
        }
        break;

    case RocketState::DECELERATION:
        if (isDecelerationComplete())
        {
            sendEvent(FSMEvent::DECELERATION_COMPLETE);
        }
        break;

    case RocketState::LANDING:
        if (isLandingComplete())
        {
            sendEvent(FSMEvent::LANDING_COMPLETE);
        }
        break;

    default:
        break;
    }
}

// Keep all your existing state entry/exit and condition checking methods unchanged
void RocketFSM::onInactiveEntry()
{
    if (rocketLogger)
        rocketLogger->logInfo("System initialization");
}

void RocketFSM::onCalibratingEntry()
{
    if (rocketLogger)
        rocketLogger->logInfo("Starting calibration");
}

void RocketFSM::onReadyForLaunchEntry()
{
    if (rocketLogger)
    {
        rocketLogger->logInfo("Logger enabled");
        rocketLogger->logInfo("Data transmission enabled");
    }
}

void RocketFSM::onLaunchEntry()
{
    if (rocketLogger)
    {
        rocketLogger->logInfo("Flight timer started");
        rocketLogger->logInfo("LAUNCH DETECTED!");
    }
}

void RocketFSM::onAcceleratedFlightEntry()
{
    if (rocketLogger)
        rocketLogger->logInfo("Accelerated flight phase");
}

void RocketFSM::onBallisticFlightEntry()
{
    if (rocketLogger)
        rocketLogger->logInfo("Ballistic flight phase - EKF started");
}

void RocketFSM::onApogeeEntry()
{
    if (rocketLogger)
        rocketLogger->logInfo("APOGEE REACHED");
}

void RocketFSM::onStabilizationEntry()
{
    if (rocketLogger)
        rocketLogger->logInfo("Stabilization phase - Drogue deployed");
}

void RocketFSM::onDecelerationEntry()
{
    if (rocketLogger)
        rocketLogger->logInfo("Deceleration phase - Main chute deployed");
}

void RocketFSM::onLandingEntry()
{
    if (rocketLogger)
        rocketLogger->logInfo("Landing phase");
}

void RocketFSM::onRecoveredEntry()
{
    if (rocketLogger)
        rocketLogger->logInfo("ROCKET RECOVERED");
}

void RocketFSM::onCalibratingExit()
{
    if (rocketLogger)
        rocketLogger->logInfo("Calibration complete");
}

void RocketFSM::onLandingExit()
{
    if (rocketLogger)
        rocketLogger->logInfo("Saving data to SD card");
    // TODO: Implement SD card saving
}

void RocketFSM::onRecoveredExit()
{
    if (rocketLogger)
        rocketLogger->logInfo("System shutdown");
}

// Keep all your existing condition checking methods
bool RocketFSM::isCalibrationComplete()
{
    return millis() - stateStartTime > 5000; // 5 seconds for calibration
}

bool RocketFSM::isSystemReady()
{
    return true;
}

bool RocketFSM::isLaunchDetected()
{
    // TODO: Implement launch detection logic
    return false;
}

bool RocketFSM::isLiftoffStarted()
{
    return millis() - stateStartTime > LIFTOFF_TIMEOUT_MS;
}

bool RocketFSM::isAccelerationPhaseComplete()
{
    // TODO: Implement acceleration phase detection
    return false;
}

bool RocketFSM::isBallisticPhaseComplete()
{
    // TODO: Implement ballistic phase detection
    return false;
}

bool RocketFSM::isApogeeReached()
{
    // TODO: Implement apogee detection
    return false;
}

bool RocketFSM::isDrogueReady()
{
    return millis() - stateStartTime > DROGUE_APOGEE_TIMEOUT;
}

bool RocketFSM::isStabilizationComplete()
{
    // TODO: Implement stabilization detection
    return false;
}

bool RocketFSM::isDecelerationComplete()
{
    // TODO: Implement deceleration detection
    return false;
}

bool RocketFSM::isLandingComplete()
{
    // TODO: Implement landing detection
    return false;
}

String RocketFSM::getStateString(RocketState state) const
{
    switch (state)
    {
    case RocketState::INACTIVE:
        return "INACTIVE";
    case RocketState::CALIBRATING:
        return "CALIBRATING";
    case RocketState::READY_FOR_LAUNCH:
        return "READY_FOR_LAUNCH";
    case RocketState::LAUNCH:
        return "LAUNCH";
    case RocketState::ACCELERATED_FLIGHT:
        return "ACCELERATED_FLIGHT";
    case RocketState::BALLISTIC_FLIGHT:
        return "BALLISTIC_FLIGHT";
    case RocketState::APOGEE:
        return "APOGEE";
    case RocketState::STABILIZATION:
        return "STABILIZATION";
    case RocketState::DECELERATION:
        return "DECELERATION";
    case RocketState::LANDING:
        return "LANDING";
    case RocketState::RECOVERED:
        return "RECOVERED";
    default:
        return "NULL";
    }
}

void RocketFSM::forceTransition(RocketState newState)
{
    sendEvent(FSMEvent::FORCE_TRANSITION, newState);
}

bool RocketFSM::isFinished()
{
    return getCurrentState() == RocketState::RECOVERED;
}

void RocketFSM::onFlightUpdate()
{
    // Legacy method - functionality now handled by tasks
}

void RocketFSM::onRecoveryUpdate()
{
    // Legacy method - functionality now handled by tasks
}