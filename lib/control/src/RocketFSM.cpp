#include "RocketFSM.hpp"
#include <config.h>
#include <KalmanFilter.hpp>
#include "esp_task_wdt.h"
#include "esp_heap_caps.h"

using TransmitDataType = std::variant<char *, String, std::string, nlohmann::json>;

// External references to your system components
extern ISensor *bno055;
extern ISensor *baro1;
extern ISensor *baro2;
extern ISensor *gps;
extern ILogger *rocketLogger;
extern ITransmitter<TransmitDataType> *loraTransmitter;
extern KalmanFilter *ekf;

// Debug macro that only prints when __DEBUG__ is defined
#ifdef __DEBUG__
    #define DEBUG_PRINT(x) Serial.println(x)
    #define DEBUG_PRINTF(format, ...) Serial.printf(format, ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(x)
    #define DEBUG_PRINTF(format, ...)
#endif

void debugMemory(const char *location)
{
    DEBUG_PRINTF("\n=== MEMORY DEBUG [%s] ===\n", location);
    DEBUG_PRINTF("Free heap: %u bytes\n", ESP.getFreeHeap());
    DEBUG_PRINTF("Min free heap: %u bytes\n", ESP.getMinFreeHeap());
    DEBUG_PRINTF("Max alloc heap: %u bytes\n", ESP.getMaxAllocHeap());
    DEBUG_PRINTF("Largest free block: %u bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    DEBUG_PRINTF("==========================\n\n");
}

RocketFSM::RocketFSM()
    : currentState(RocketState::INACTIVE), previousState(RocketState::INACTIVE), stateStartTime(0), lastUpdateTime(0), isRunning(false), fsmTaskHandle(nullptr), sensorTaskHandle(nullptr), ekfTaskHandle(nullptr), apogeeDetectionTaskHandle(nullptr), recoveryTaskHandle(nullptr), dataCollectionTaskHandle(nullptr), telemetryTaskHandle(nullptr), gpsTaskHandle(nullptr), loggingTaskHandle(nullptr), eventQueue(nullptr), stateMutex(nullptr), sharedData{SensorData("bno055"), SensorData("baro1"), SensorData("baro2"), SensorData("gps"), 0, false}
{
    DEBUG_PRINT("RocketFSM constructor called");
    debugMemory("Constructor");
}

RocketFSM::~RocketFSM()
{
    DEBUG_PRINT("RocketFSM destructor called");
    debugMemory("Destructor start");
    stop();

    if (eventQueue)
    {
        vQueueDelete(eventQueue);
    }
    if (stateMutex)
    {
        vSemaphoreDelete(stateMutex);
    }
    debugMemory("Destructor end");
}

void RocketFSM::init()
{
    DEBUG_PRINT("FSM init() called");
    debugMemory("Init start");

    // Create FreeRTOS objects with error checking
    DEBUG_PRINT("Creating event queue...");
    eventQueue = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(FSMEventData));
    if (!eventQueue)
    {
        DEBUG_PRINT("ERROR: Failed to create event queue");
        debugMemory("Failed event queue creation");
        return;
    }
    DEBUG_PRINT("Event queue created successfully");
    debugMemory("After event queue creation");

    DEBUG_PRINT("Creating state mutex...");
    stateMutex = xSemaphoreCreateMutex();
    if (!stateMutex)
    {
        DEBUG_PRINT("ERROR: Failed to create state mutex");
        vQueueDelete(eventQueue);
        eventQueue = nullptr;
        debugMemory("Failed mutex creation");
        return;
    }
    DEBUG_PRINT("State mutex created successfully");
    debugMemory("After mutex creation");

    if (!eventQueue || !stateMutex)
    {
        // if (rocketLogger)
        {
            // rocketLogger->logError("Failed to create FSM FreeRTOS objects");
        }
        DEBUG_PRINT("ERROR: FreeRTOS objects creation failed");
        return;
    }

    DEBUG_PRINT("Setting up state actions...");
    debugMemory("Before setupStateActions");
    setupStateActions();
    DEBUG_PRINT("State actions setup complete");
    debugMemory("After setupStateActions");

    DEBUG_PRINT("Setting up transitions...");
    debugMemory("Before setupTransitions");
    setupTransitions();
    DEBUG_PRINT("Transitions setup complete");
    debugMemory("After setupTransitions");

    currentState = RocketState::INACTIVE;
    stateStartTime = millis();
    lastUpdateTime = millis();

    debugMemory("Init end");

    // if (rocketLogger)
    {
        // rocketLogger->logInfo("FSM initialized in INACTIVE state");
    }

    DEBUG_PRINT("FSM init() completed successfully");
}

void RocketFSM::start()
{
    DEBUG_PRINT("FSM start() called");
    debugMemory("Start begin");

    if (isRunning)
    {
        DEBUG_PRINT("FSM already running, returning");
        return;
    }

    // Now try with the actual task
    DEBUG_PRINT("Creating FSM task with global wrapper...");
    debugMemory("Before FSM task creation");

    BaseType_t result = xTaskCreatePinnedToCore(
        fsmTaskWrapper,
        "FSM_Task",
        4096, // Increased stack size
        this, // Pass this pointer
        3,    // Higher priority for FSM
        &fsmTaskHandle,
        1); // Pin to Core 1

    DEBUG_PRINTF("FSM task creation result: %d\n", result);
    debugMemory("After FSM task creation");

    if (result == pdPASS)
    {
        DEBUG_PRINT("SUCCESS: FSM task created");
        isRunning = true;
    }
    else
    {
        DEBUG_PRINT("FAILED: FSM task creation");
        debugMemory("FSM task creation failed");
        return;
    }

    // Only if task created successfully, proceed with state task startup
    DEBUG_PRINT("Starting state tasks...");
    debugMemory("Before startStateTasks");
    startStateTasks();
    debugMemory("After startStateTasks");
}

// Also modify the stop() method to add more debugging
void RocketFSM::stop()
{
    DEBUG_PRINT("FSM stop() called");
    DEBUG_PRINT("Call stack trace (if available):");
    // This won't give you a real stack trace but might help identify patterns
    DEBUG_PRINTF("Current state: %s, Previous state: %s\n",
                  getStateString(currentState).c_str(),
                  getStateString(previousState).c_str());
    debugMemory("Stop begin");

    if (!isRunning)
    {
        DEBUG_PRINT("FSM already stopped, returning");
        return;
    }

    DEBUG_PRINT("Setting isRunning to false...");
    isRunning = false;

    DEBUG_PRINT("Stopping all tasks...");
    stopAllTasks();

    if (fsmTaskHandle)
    {
        DEBUG_PRINT("Deleting FSM task...");
        vTaskDelete(fsmTaskHandle);
        fsmTaskHandle = nullptr;
    }

    // if (rocketLogger)
    {
        // rocketLogger->logInfo("FSM stopped");
    }

    debugMemory("Stop end");
}

bool RocketFSM::sendEvent(FSMEvent event, RocketState targetState, void *eventData)
{
    if (!eventQueue)
        return false;

    FSMEventData eventStruct(event, targetState, eventData);
    bool result = xQueueSend(eventQueue, &eventStruct, pdMS_TO_TICKS(100)) == pdPASS;
    DEBUG_PRINTF("Event sent: %d, result: %d\n", static_cast<int>(event), result);
    return result;
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

// Change this function definition
void IRAM_ATTR RocketFSM::fsmTaskWrapper(void *parameter)
{
    DEBUG_PRINT("Global FSM task wrapper started");
    debugMemory("FSM task wrapper start");

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (!fsm)
    {
        DEBUG_PRINT("ERROR: Invalid FSM pointer");
        debugMemory("Invalid FSM pointer");
        vTaskDelete(NULL);
        return;
    }

    DEBUG_PRINT("Calling fsm->fsmTask()");
    fsm->fsmTask();

    // Should never reach here unless task exits
    DEBUG_PRINT("Global FSM task wrapper exiting");
    debugMemory("FSM task wrapper end");
    vTaskDelete(NULL);
}

void RocketFSM::fsmTask()
{
    DEBUG_PRINT("fsmTask started");
    debugMemory("fsmTask start");

    DEBUG_PRINTF("isRunning = %d\n", isRunning);
    DEBUG_PRINTF("Current state: %s\n", getStateString(currentState).c_str());

    // Create a proper event structure
    FSMEventData eventData(FSMEvent::NONE);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned long loopCounter = 0;

    // Register with watchdog timer for safety
    esp_task_wdt_add(NULL);

    // Critical: manually check transitions for INACTIVE state immediately
    if (currentState == RocketState::INACTIVE)
    {
        DEBUG_PRINT("Initial state is INACTIVE, sending START_CALIBRATION event");
        sendEvent(FSMEvent::START_CALIBRATION);
        if (!isRunning)
        {
            DEBUG_PRINT("WARNING: isRunning is false after sending START_CALIBRATION!");
            isRunning = true;
            DEBUG_PRINT("Re-enabling FSM task execution...");
        }
    }

    while (isRunning)
    {
        // Reset watchdog
        esp_task_wdt_reset();

        // Check if isRunning was changed externally
        if (!isRunning)
        {
            DEBUG_PRINT("WARNING: isRunning changed to false unexpectedly!");
            DEBUG_PRINT("Re-enabling FSM task execution...");
            isRunning = true;
        }

        // Memory check every 100 loops
        if (loopCounter % 100 == 0)
        {
            debugMemory("FSM main loop");
            DEBUG_PRINTF("FSM running status: %d\n", isRunning);
        }

        checkTransitions();

        // Check for events (non-blocking with timeout)
        if (xQueueReceive(eventQueue, &eventData, pdMS_TO_TICKS(50)) == pdPASS)
        {
            DEBUG_PRINTF("Received event: %d for state: %s\n",
                          static_cast<int>(eventData.event),
                          getStateString(currentState).c_str());

            try
            {
                debugMemory("Before processEvent");
                processEvent(eventData);
                debugMemory("After processEvent");
            }
            catch (...)
            {
                DEBUG_PRINT("EXCEPTION: Error in processEvent!");
                debugMemory("processEvent-exception");
            }
        }

        // Periodically output the current state for debugging
        if (loopCounter % 20 == 0)
        {
            DEBUG_PRINTF("FSM Loop #%lu: Current state: %s, Time in state: %lu ms, isRunning: %d\n",
                          loopCounter, getStateString(currentState).c_str(),
                          millis() - stateStartTime, isRunning);
        }

        // Update last update time
        lastUpdateTime = millis();
        loopCounter++;

        // Task delay to prevent busy waiting
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(50)); // 20Hz update rate
    }

    // We should never reach here unless stop() was explicitly called
    DEBUG_PRINT("fsmTask ended because isRunning became false!");
    DEBUG_PRINT("Stack high water mark: " + String(uxTaskGetStackHighWaterMark(NULL)));
    debugMemory("fsmTask end");

    // Clean up watchdog
    esp_task_wdt_delete(NULL);
}

void RocketFSM::setupStateActions()
{
    DEBUG_PRINT("Setting up state actions...");
    debugMemory("setupStateActions start");

    // INACTIVE state
    stateActions[RocketState::INACTIVE] = StateActions(
        [this]()
        { onInactiveEntry(); });
    DEBUG_PRINT("INACTIVE state action set");

    // CALIBRATING state
    stateActions[RocketState::CALIBRATING] = StateActions(
        [this]()
        { onCalibratingEntry(); },
        [this]()
        { onCalibratingExit(); });
    stateActions[RocketState::CALIBRATING].sensorTask = TaskConfig("Sensor_Calib", 3072, 4, 0, true);
    stateActions[RocketState::CALIBRATING].loggingTask = TaskConfig("Log_Calib", 2048, 1, 1, true);
    DEBUG_PRINT("CALIBRATING state action set");

    // READY_FOR_LAUNCH state
    stateActions[RocketState::READY_FOR_LAUNCH] = StateActions(
        [this]()
        { onReadyForLaunchEntry(); });
    stateActions[RocketState::READY_FOR_LAUNCH].sensorTask = TaskConfig("Sensor_Ready", 3072, 4, 0, true);
    stateActions[RocketState::READY_FOR_LAUNCH].telemetryTask = TaskConfig("Telemetry_Ready", 3072, 2, 1, true);
    stateActions[RocketState::READY_FOR_LAUNCH].loggingTask = TaskConfig("Log_Ready", 2048, 1, 1, true);
    DEBUG_PRINT("READY_FOR_LAUNCH state action set");

    // LAUNCH state
    stateActions[RocketState::LAUNCH] = StateActions(
        [this]()
        { onLaunchEntry(); });
    stateActions[RocketState::LAUNCH].sensorTask = TaskConfig("Sensor_Launch", 3072, 5, 0, true);
    stateActions[RocketState::LAUNCH].dataCollectionTask = TaskConfig("DataCol_Launch", 2048, 2, 1, true);
    stateActions[RocketState::LAUNCH].loggingTask = TaskConfig("Log_Launch", 2048, 1, 1, true);
    DEBUG_PRINT("LAUNCH state action set");

    // ACCELERATED_FLIGHT state
    stateActions[RocketState::ACCELERATED_FLIGHT] = StateActions(
        [this]()
        { onAcceleratedFlightEntry(); });
    stateActions[RocketState::ACCELERATED_FLIGHT].sensorTask = TaskConfig("Sensor_AccFlight", 3072, 5, 0, true);
    stateActions[RocketState::ACCELERATED_FLIGHT].ekfTask = TaskConfig("EKF_AccFlight", 4096, 4, 0, true);
    stateActions[RocketState::ACCELERATED_FLIGHT].telemetryTask = TaskConfig("Telemetry_AccFlight", 3072, 2, 1, true);
    stateActions[RocketState::ACCELERATED_FLIGHT].dataCollectionTask = TaskConfig("DataCol_AccFlight", 2048, 2, 1, true);
    DEBUG_PRINT("ACCELERATED_FLIGHT state action set");

    // BALLISTIC_FLIGHT state
    stateActions[RocketState::BALLISTIC_FLIGHT] = StateActions(
        [this]()
        { onBallisticFlightEntry(); });
    stateActions[RocketState::BALLISTIC_FLIGHT].sensorTask = TaskConfig("Sensor_BalFlight", 3072, 5, 0, true);
    stateActions[RocketState::BALLISTIC_FLIGHT].ekfTask = TaskConfig("EKF_BalFlight", 4096, 4, 0, true);
    stateActions[RocketState::BALLISTIC_FLIGHT].apogeeDetectionTask = TaskConfig("Apogee_Detection", 2048, 4, 0, true);
    stateActions[RocketState::BALLISTIC_FLIGHT].telemetryTask = TaskConfig("Telemetry_BalFlight", 3072, 2, 1, true);
    stateActions[RocketState::BALLISTIC_FLIGHT].dataCollectionTask = TaskConfig("DataCol_BalFlight", 2048, 2, 1, true);
    DEBUG_PRINT("BALLISTIC_FLIGHT state action set");

    // APOGEE state
    stateActions[RocketState::APOGEE] = StateActions(
        [this]()
        { onApogeeEntry(); });
    stateActions[RocketState::APOGEE].sensorTask = TaskConfig("Sensor_Apogee", 3072, 5, 0, true);
    stateActions[RocketState::APOGEE].recoveryTask = TaskConfig("Recovery_Apogee", 2048, 5, 0, true);
    stateActions[RocketState::APOGEE].dataCollectionTask = TaskConfig("DataCol_Apogee", 2048, 2, 1, true);
    stateActions[RocketState::APOGEE].telemetryTask = TaskConfig("Telemetry_Apogee", 3072, 2, 1, true);
    DEBUG_PRINT("APOGEE state action set");

    // STABILIZATION state
    stateActions[RocketState::STABILIZATION] = StateActions(
        [this]()
        { onStabilizationEntry(); });
    stateActions[RocketState::STABILIZATION].sensorTask = TaskConfig("Sensor_Stab", 3072, 4, 0, true);
    stateActions[RocketState::STABILIZATION].recoveryTask = TaskConfig("Recovery_Stab", 2048, 4, 0, true);
    stateActions[RocketState::STABILIZATION].telemetryTask = TaskConfig("Telemetry_Stab", 3072, 2, 1, true);
    stateActions[RocketState::STABILIZATION].dataCollectionTask = TaskConfig("DataCol_Stab", 2048, 2, 1, true);
    stateActions[RocketState::STABILIZATION].gpsTask = TaskConfig("GPS_Stab", 2048, 1, 1, true);
    DEBUG_PRINT("STABILIZATION state action set");

    // DECELERATION state
    stateActions[RocketState::DECELERATION] = StateActions(
        [this]()
        { onDecelerationEntry(); });
    stateActions[RocketState::DECELERATION].sensorTask = TaskConfig("Sensor_Decel", 3072, 4, 0, true);
    stateActions[RocketState::DECELERATION].recoveryTask = TaskConfig("Recovery_Decel", 2048, 4, 0, true);
    stateActions[RocketState::DECELERATION].telemetryTask = TaskConfig("Telemetry_Decel", 3072, 2, 1, true);
    stateActions[RocketState::DECELERATION].dataCollectionTask = TaskConfig("DataCol_Decel", 2048, 2, 1, true);
    stateActions[RocketState::DECELERATION].gpsTask = TaskConfig("GPS_Decel", 2048, 1, 1, true);
    DEBUG_PRINT("DECELERATION state action set");

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
    DEBUG_PRINT("LANDING state action set");

    // RECOVERED state
    stateActions[RocketState::RECOVERED] = StateActions(
        [this]()
        { onRecoveredEntry(); },
        [this]()
        { onRecoveredExit(); });
    stateActions[RocketState::RECOVERED].gpsTask = TaskConfig("GPS_Recovered", 2048, 1, 1, true);
    stateActions[RocketState::RECOVERED].loggingTask = TaskConfig("Log_Recovered", 3072, 1, 1, true);
    DEBUG_PRINT("RECOVERED state action set");

    debugMemory("setupStateActions end");
}

void RocketFSM::setupTransitions()
{
    DEBUG_PRINT("Setting up transitions...");
    debugMemory("setupTransitions start");

    transitions.emplace_back(RocketState::INACTIVE, RocketState::CALIBRATING, FSMEvent::START_CALIBRATION);
    transitions.emplace_back(RocketState::CALIBRATING, RocketState::READY_FOR_LAUNCH, FSMEvent::CALIBRATION_COMPLETE);
    transitions.emplace_back(RocketState::READY_FOR_LAUNCH, RocketState::LAUNCH, FSMEvent::LAUNCH_DETECTED);
    transitions.emplace_back(RocketState::LAUNCH, RocketState::ACCELERATED_FLIGHT, FSMEvent::LIFTOFF_STARTED);
    transitions.emplace_back(RocketState::ACCELERATED_FLIGHT, RocketState::BALLISTIC_FLIGHT, FSMEvent::ACCELERATION_COMPLETE);
    transitions.emplace_back(RocketState::BALLISTIC_FLIGHT, RocketState::APOGEE, FSMEvent::APOGEE_REACHED);
    transitions.emplace_back(RocketState::APOGEE, RocketState::STABILIZATION, FSMEvent::DROGUE_READY);
    transitions.emplace_back(RocketState::STABILIZATION, RocketState::DECELERATION, FSMEvent::STABILIZATION_COMPLETE);
    transitions.emplace_back(RocketState::DECELERATION, RocketState::LANDING, FSMEvent::DECELERATION_COMPLETE);
    transitions.emplace_back(RocketState::LANDING, RocketState::RECOVERED, FSMEvent::LANDING_COMPLETE);

    debugMemory("setupTransitions end");
}

void RocketFSM::transitionTo(RocketState newState)
{
    if (newState == currentState)
        return;

    DEBUG_PRINTF("Attempting transition from %s to %s\n", getStateString(currentState).c_str(), getStateString(newState).c_str());
    debugMemory("Before transition mutex take");

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        DEBUG_PRINT("\n==================== STATE TRANSITION ====================");
        DEBUG_PRINTF("Time: %lu ms\n", millis());
        DEBUG_PRINTF("Transition: %s -> %s\n", getStateString(currentState).c_str(), getStateString(newState).c_str());
        debugMemory("Transition start");

        // if (rocketLogger)
        {
            String transitionMsg = "FSM transition: " + getStateString(currentState) + " -> " + getStateString(newState);
            // rocketLogger->logInfo(transitionMsg.c_str());
        }

        // Execute exit action for current state
        if (stateActions[currentState].onExit)
        {
            DEBUG_PRINT("Executing exit action for current state...");
            debugMemory("Before exit action");
            stateActions[currentState].onExit();
            debugMemory("After exit action");
        }

        // Stop current state tasks
        DEBUG_PRINT("Stopping current state tasks...");
        debugMemory("Before stopAllTasks");
        stopAllTasks();
        debugMemory("After stopAllTasks");

        previousState = currentState;
        currentState = newState;
        stateStartTime = millis();

        // Execute entry action for new state
        if (stateActions[currentState].onEntry)
        {
            DEBUG_PRINT("Executing entry action for new state...");
            debugMemory("Before entry action");
            stateActions[currentState].onEntry();
            debugMemory("After entry action");
        }

        // Start new state tasks
        DEBUG_PRINT("Starting new state tasks...");
        debugMemory("Before startStateTasks");
        startStateTasks();
        debugMemory("After startStateTasks");

        DEBUG_PRINT("==========================================================\n");

        xSemaphoreGive(stateMutex);
        debugMemory("After transition mutex give");
    }
    else
    {
        DEBUG_PRINT("ERROR: Failed to take mutex for transition");
        debugMemory("Failed mutex take");
    }
}

void RocketFSM::processEvent(const FSMEventData &eventData)
{
    DEBUG_PRINTF("Processing event: %d\n", static_cast<int>(eventData.event));
    debugMemory("processEvent start");

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
            DEBUG_PRINTF("Found matching transition: %s -> %s\n",
                          getStateString(transition.fromState).c_str(),
                          getStateString(transition.toState).c_str());
            transitionTo(transition.toState);
            break;
        }
    }

    debugMemory("processEvent end");
}

void RocketFSM::startStateTasks()
{
    DEBUG_PRINTF("Starting tasks for state: %s\n", getStateString(currentState).c_str());
    debugMemory("startStateTasks begin");

    const StateActions &actions = stateActions[currentState];

    // Start Core 0 tasks (Critical)
    if (actions.sensorTask.shouldRun)
    {
        DEBUG_PRINTF("Creating sensor task: %s\n", actions.sensorTask.name);
        debugMemory("Before sensor task creation");

        BaseType_t result = xTaskCreatePinnedToCore(sensorTaskWrapper, actions.sensorTask.name, actions.sensorTask.stackSize,
                                                    this, actions.sensorTask.priority, &sensorTaskHandle, actions.sensorTask.coreId);
        if (result != pdPASS)
        {
            DEBUG_PRINTF("Failed to create sensor task: %d\n", result);
            debugMemory("Sensor task creation failed");
        }
        else
        {
            DEBUG_PRINT("Sensor task created successfully");
            debugMemory("After sensor task creation");
        }
    }

    if (actions.ekfTask.shouldRun)
    {
        DEBUG_PRINTF("Creating EKF task: %s\n", actions.ekfTask.name);
        debugMemory("Before EKF task creation");

        BaseType_t result = xTaskCreatePinnedToCore(ekfTaskWrapper, actions.ekfTask.name, actions.ekfTask.stackSize,
                                                    this, actions.ekfTask.priority, &ekfTaskHandle, actions.ekfTask.coreId);
        if (result != pdPASS)
        {
            DEBUG_PRINTF("Failed to create EKF task: %d\n", result);
            debugMemory("EKF task creation failed");
        }
        else
        {
            DEBUG_PRINT("EKF task created successfully");
            debugMemory("After EKF task creation");
        }
    }

    if (actions.apogeeDetectionTask.shouldRun)
    {
        DEBUG_PRINTF("Creating apogee detection task: %s\n", actions.apogeeDetectionTask.name);
        debugMemory("Before apogee task creation");

        BaseType_t result = xTaskCreatePinnedToCore(apogeeDetectionTaskWrapper, actions.apogeeDetectionTask.name, actions.apogeeDetectionTask.stackSize,
                                                    this, actions.apogeeDetectionTask.priority, &apogeeDetectionTaskHandle, actions.apogeeDetectionTask.coreId);
        if (result != pdPASS)
        {
            DEBUG_PRINTF("Failed to create apogee detection task: %d\n", result);
            debugMemory("Apogee task creation failed");
        }
        else
        {
            DEBUG_PRINT("Apogee detection task created successfully");
            debugMemory("After apogee task creation");
        }
    }

    if (actions.recoveryTask.shouldRun)
    {
        DEBUG_PRINTF("Creating recovery task: %s\n", actions.recoveryTask.name);
        debugMemory("Before recovery task creation");

        BaseType_t result = xTaskCreatePinnedToCore(recoveryTaskWrapper, actions.recoveryTask.name, actions.recoveryTask.stackSize,
                                                    this, actions.recoveryTask.priority, &recoveryTaskHandle, actions.recoveryTask.coreId);
        if (result != pdPASS)
        {
            DEBUG_PRINTF("Failed to create recovery task: %d\n", result);
            debugMemory("Recovery task creation failed");
        }
        else
        {
            DEBUG_PRINT("Recovery task created successfully");
            debugMemory("After recovery task creation");
        }
    }

    // Start Core 1 tasks (Non-critical)
    if (actions.dataCollectionTask.shouldRun)
    {
        DEBUG_PRINTF("Creating data collection task: %s\n", actions.dataCollectionTask.name);
        debugMemory("Before data collection task creation");

        BaseType_t result = xTaskCreatePinnedToCore(dataCollectionTaskWrapper, actions.dataCollectionTask.name, actions.dataCollectionTask.stackSize,
                                                    this, actions.dataCollectionTask.priority, &dataCollectionTaskHandle, actions.dataCollectionTask.coreId);
        if (result != pdPASS)
        {
            DEBUG_PRINTF("Failed to create data collection task: %d\n", result);
            debugMemory("Data collection task creation failed");
        }
        else
        {
            DEBUG_PRINT("Data collection task created successfully");
            debugMemory("After data collection task creation");
        }
    }

    if (actions.telemetryTask.shouldRun)
    {
        DEBUG_PRINTF("Creating telemetry task: %s\n", actions.telemetryTask.name);
        debugMemory("Before telemetry task creation");

        BaseType_t result = xTaskCreatePinnedToCore(telemetryTaskWrapper, actions.telemetryTask.name, actions.telemetryTask.stackSize,
                                                    this, actions.telemetryTask.priority, &telemetryTaskHandle, actions.telemetryTask.coreId);
        if (result != pdPASS)
        {
            DEBUG_PRINTF("Failed to create telemetry task: %d\n", result);
            debugMemory("Telemetry task creation failed");
        }
        else
        {
            DEBUG_PRINT("Telemetry task created successfully");
            debugMemory("After telemetry task creation");
        }
    }

    if (actions.gpsTask.shouldRun)
    {
        DEBUG_PRINTF("Creating GPS task: %s\n", actions.gpsTask.name);
        debugMemory("Before GPS task creation");

        BaseType_t result = xTaskCreatePinnedToCore(gpsTaskWrapper, actions.gpsTask.name, actions.gpsTask.stackSize,
                                                    this, actions.gpsTask.priority, &gpsTaskHandle, actions.gpsTask.coreId);
        if (result != pdPASS)
        {
            DEBUG_PRINTF("Failed to create GPS task: %d\n", result);
            debugMemory("GPS task creation failed");
        }
        else
        {
            DEBUG_PRINT("GPS task created successfully");
            debugMemory("After GPS task creation");
        }
    }

    if (actions.loggingTask.shouldRun)
    {
        DEBUG_PRINTF("Creating logging task: %s\n", actions.loggingTask.name);
        debugMemory("Before logging task creation");

        BaseType_t result = xTaskCreatePinnedToCore(loggingTaskWrapper, actions.loggingTask.name, actions.loggingTask.stackSize,
                                                    this, actions.loggingTask.priority, &loggingTaskHandle, actions.loggingTask.coreId);
        if (result != pdPASS)
        {
            DEBUG_PRINTF("Failed to create logging task: %d\n", result);
            debugMemory("Logging task creation failed");
        }
        else
        {
            DEBUG_PRINT("Logging task created successfully");
            debugMemory("After logging task creation");
        }
    }

    debugMemory("startStateTasks end");
    DEBUG_PRINT("All state tasks creation completed");
}

void RocketFSM::stopAllTasks()
{
    DEBUG_PRINT("Stopping all tasks...");
    debugMemory("stopAllTasks begin");

    // Stop Core 0 tasks
    if (sensorTaskHandle)
    {
        DEBUG_PRINT("Deleting sensor task");
        vTaskDelete(sensorTaskHandle);
        sensorTaskHandle = nullptr;
        debugMemory("After sensor task deletion");
    }
    if (ekfTaskHandle)
    {
        DEBUG_PRINT("Deleting EKF task");
        vTaskDelete(ekfTaskHandle);
        ekfTaskHandle = nullptr;
        debugMemory("After EKF task deletion");
    }
    if (apogeeDetectionTaskHandle)
    {
        DEBUG_PRINT("Deleting apogee detection task");
        vTaskDelete(apogeeDetectionTaskHandle);
        apogeeDetectionTaskHandle = nullptr;
        debugMemory("After apogee task deletion");
    }
    if (recoveryTaskHandle)
    {
        DEBUG_PRINT("Deleting recovery task");
        vTaskDelete(recoveryTaskHandle);
        recoveryTaskHandle = nullptr;
        debugMemory("After recovery task deletion");
    }

    // Stop Core 1 tasks
    if (dataCollectionTaskHandle)
    {
        DEBUG_PRINT("Deleting data collection task");
        vTaskDelete(dataCollectionTaskHandle);
        dataCollectionTaskHandle = nullptr;
        debugMemory("After data collection task deletion");
    }
    if (telemetryTaskHandle)
    {
        DEBUG_PRINT("Deleting telemetry task");
        vTaskDelete(telemetryTaskHandle);
        telemetryTaskHandle = nullptr;
        debugMemory("After telemetry task deletion");
    }
    if (gpsTaskHandle)
    {
        DEBUG_PRINT("Deleting GPS task");
        vTaskDelete(gpsTaskHandle);
        gpsTaskHandle = nullptr;
        debugMemory("After GPS task deletion");
    }
    if (loggingTaskHandle)
    {
        DEBUG_PRINT("Deleting logging task");
        vTaskDelete(loggingTaskHandle);
        loggingTaskHandle = nullptr;
        debugMemory("After logging task deletion");
    }

    // Add small delay to ensure tasks are cleaned up
    vTaskDelay(pdMS_TO_TICKS(100));
    debugMemory("stopAllTasks end");
    DEBUG_PRINT("All tasks stopped");
}

// Task wrappers for Core 0 (Critical)
void IRAM_ATTR RocketFSM::sensorTaskWrapper(void *parameter)
{
    DEBUG_PRINT("Sensor task wrapper started");
    debugMemory("Sensor task wrapper start");

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (fsm)
    {
        fsm->sensorTask();
    }

    DEBUG_PRINT("Sensor task wrapper ending");
    debugMemory("Sensor task wrapper end");
    vTaskDelete(NULL);
}

void IRAM_ATTR RocketFSM::ekfTaskWrapper(void *parameter)
{
    DEBUG_PRINT("EKF task wrapper started");
    debugMemory("EKF task wrapper start");

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (fsm)
    {
        fsm->ekfTask();
    }

    DEBUG_PRINT("EKF task wrapper ending");
    debugMemory("EKF task wrapper end");
    vTaskDelete(NULL);
}

void IRAM_ATTR RocketFSM::apogeeDetectionTaskWrapper(void *parameter)
{
    DEBUG_PRINT("Apogee detection task wrapper started");
    debugMemory("Apogee task wrapper start");

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (fsm)
    {
        fsm->apogeeDetectionTask();
    }

    DEBUG_PRINT("Apogee detection task wrapper ending");
    debugMemory("Apogee task wrapper end");
    vTaskDelete(NULL);
}

void IRAM_ATTR RocketFSM::recoveryTaskWrapper(void *parameter)
{
    DEBUG_PRINT("Recovery task wrapper started");
    debugMemory("Recovery task wrapper start");

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (fsm)
    {
        fsm->recoveryTask();
    }

    DEBUG_PRINT("Recovery task wrapper ending");
    debugMemory("Recovery task wrapper end");
    vTaskDelete(NULL);
}

// Task wrappers for Core 1 (Non-critical)
void IRAM_ATTR RocketFSM::dataCollectionTaskWrapper(void *parameter)
{
    DEBUG_PRINT("Data collection task wrapper started");
    debugMemory("Data collection task wrapper start");

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (fsm)
    {
        fsm->dataCollectionTask();
    }

    DEBUG_PRINT("Data collection task wrapper ending");
    debugMemory("Data collection task wrapper end");
    vTaskDelete(NULL);
}

void IRAM_ATTR RocketFSM::telemetryTaskWrapper(void *parameter)
{
    DEBUG_PRINT("Telemetry task wrapper started");
    debugMemory("Telemetry task wrapper start");

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (fsm)
    {
        fsm->telemetryTask();
    }

    DEBUG_PRINT("Telemetry task wrapper ending");
    debugMemory("Telemetry task wrapper end");
    vTaskDelete(NULL);
}

void IRAM_ATTR RocketFSM::gpsTaskWrapper(void *parameter)
{
    DEBUG_PRINT("GPS task wrapper started");
    debugMemory("GPS task wrapper start");

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (fsm)
    {
        fsm->gpsTask();
    }

    DEBUG_PRINT("GPS task wrapper ending");
    debugMemory("GPS task wrapper end");
    vTaskDelete(NULL);
}

void IRAM_ATTR RocketFSM::loggingTaskWrapper(void *parameter)
{
    DEBUG_PRINT("Logging task wrapper started");
    debugMemory("Logging task wrapper start");

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (fsm)
    {
        fsm->loggingTask();
    }

    DEBUG_PRINT("Logging task wrapper ending");
    debugMemory("Logging task wrapper end");
    vTaskDelete(NULL);
}

// Core 0 Task implementations (Critical) - OPTIMIZED
void RocketFSM::sensorTask()
{
    DEBUG_PRINT("Sensor task started");
    debugMemory("Sensor task start");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz
    unsigned long loopCount = 0;

    while (isRunning)
    {
        // Memory check every 1000 loops
        if (loopCount % 1000 == 0)
        {
            debugMemory("Sensor task loop");
        }

        // Read sensors efficiently
        if (bno055)
        {
            auto data = bno055->getData();
            if (data.has_value())
            {
                sharedData.imuData = data.value();
            }
        }
        if (baro1)
        {
            auto data = baro1->getData();
            if (data.has_value())
            {
                sharedData.baroData1 = data.value();
            }
        }
        if (baro2)
        {
            auto data = baro2->getData();
            if (data.has_value())
            {
                sharedData.baroData2 = data.value();
            }
        }

        loopCount++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    DEBUG_PRINT("Sensor task ended");
    debugMemory("Sensor task end");
}

void RocketFSM::ekfTask()
{
    DEBUG_PRINT("EKF task started");
    debugMemory("EKF task start");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 50Hz
    unsigned long loopCount = 0;

    while (isRunning)
    {
        // Memory check every 500 loops
        if (loopCount % 500 == 0)
        {
            debugMemory("EKF task loop");
        }

        // EKF processing (placeholder)
        if (ekf)
        {
            // Process Kalman filter with current sensor data
            // ekf->update(sharedData.imuData, sharedData.baroData1);
        }

        loopCount++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    DEBUG_PRINT("EKF task ended");
    debugMemory("EKF task end");
}

void RocketFSM::apogeeDetectionTask()
{
    DEBUG_PRINT("Apogee detection task started");
    debugMemory("Apogee detection task start");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 100Hz
    unsigned long loopCount = 0;

    while (isRunning)
    {
        // Memory check every 1000 loops
        if (loopCount % 1000 == 0)
        {
            debugMemory("Apogee detection task loop");
        }

        // Critical apogee detection
        if (isApogeeReached())
        {
            DEBUG_PRINT("Apogee reached! Sending event...");
            sendEvent(FSMEvent::APOGEE_REACHED);
        }

        loopCount++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    DEBUG_PRINT("Apogee detection task ended");
    debugMemory("Apogee detection task end");
}

void RocketFSM::recoveryTask()
{
    DEBUG_PRINT("Recovery task started");
    debugMemory("Recovery task start");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz
    unsigned long loopCount = 0;

    while (isRunning)
    {
        // Memory check every 200 loops
        if (loopCount % 200 == 0)
        {
            debugMemory("Recovery task loop");
        }

        // Recovery system control
        // Handle parachute deployment, etc.

        loopCount++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    DEBUG_PRINT("Recovery task ended");
    debugMemory("Recovery task end");
}

// Core 1 Task implementations (Non-critical) - OPTIMIZED
void RocketFSM::dataCollectionTask()
{
    DEBUG_PRINT("Data collection task started");
    debugMemory("Data collection task start");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(50); // 20Hz
    unsigned long loopCount = 0;

    while (isRunning)
    {
        // Memory check every 200 loops
        if (loopCount % 200 == 0)
        {
            debugMemory("Data collection task loop");
        }

        // if (rocketLogger)
        {
            // rocketLogger->logSensorData(sharedData.imuData);
            // rocketLogger->logSensorData(sharedData.baroData1);
            // rocketLogger->logSensorData(sharedData.baroData2);
            // rocketLogger->logSensorData(sharedData.gpsData);
        }

        loopCount++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    DEBUG_PRINT("Data collection task ended");
    debugMemory("Data collection task end");
}

void RocketFSM::telemetryTask()
{
    DEBUG_PRINT("Telemetry task started");
    debugMemory("Telemetry task start");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200); // 5Hz
    unsigned long loopCount = 0;

    while (isRunning)
    {
        // Memory check every 50 loops
        if (loopCount % 50 == 0)
        {
            debugMemory("Telemetry task loop");
        }

        if (loraTransmitter && rocketLogger)
        {
            // auto response = loraTransmitter->transmit(rocketLogger->getJSONAll());
            // Process response if needed
        }

        loopCount++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    DEBUG_PRINT("Telemetry task ended");
    debugMemory("Telemetry task end");
}

void RocketFSM::gpsTask()
{
    DEBUG_PRINT("GPS task started");
    debugMemory("GPS task start");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200); // 5Hz
    unsigned long loopCount = 0;

    while (isRunning)
    {
        // Memory check every 50 loops
        if (loopCount % 50 == 0)
        {
            debugMemory("GPS task loop");
        }

        if (gps)
        {
            auto data = gps->getData();
            if (data.has_value())
            {
                sharedData.gpsData = data.value();
            }
        }

        loopCount++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    DEBUG_PRINT("GPS task ended");
    debugMemory("GPS task end");
}

void RocketFSM::loggingTask()
{
    DEBUG_PRINT("Logging task started");
    debugMemory("Logging task start");

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(500); // 2Hz
    unsigned long loopCount = 0;

    while (isRunning)
    {
        // Memory check every 20 loops
        if (loopCount % 20 == 0)
        {
            debugMemory("Logging task loop");
        }

        // SD card logging operations
        // TODO: Implement actual SD card operations

        loopCount++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    DEBUG_PRINT("Logging task ended");
    debugMemory("Logging task end");
}

void RocketFSM::checkTransitions()
{
    RocketState state = getCurrentState();

    switch (state)
    {
    case RocketState::INACTIVE:
        DEBUG_PRINT("In INACTIVE state, sending START_CALIBRATION event");
        DEBUG_PRINTF("START_CALIBRATION event sent result: %d\n", sendEvent(FSMEvent::START_CALIBRATION));
        break;

    case RocketState::CALIBRATING:
        if (isCalibrationComplete())
        {
            DEBUG_PRINT("Calibration complete! Sending event...");
            sendEvent(FSMEvent::CALIBRATION_COMPLETE);
        }
        break;

    case RocketState::READY_FOR_LAUNCH:
    {
        unsigned long currentTime = millis();
        unsigned long timeInState = currentTime - stateStartTime;

        if (isLaunchDetected())
        {
            DEBUG_PRINT("*** LAUNCH DETECTED! Sending LAUNCH_DETECTED event ***");
            bool sent = sendEvent(FSMEvent::LAUNCH_DETECTED);
            DEBUG_PRINTF("Event sent result: %d\n", sent);
        }
    }
    break;

    case RocketState::LAUNCH:
        if (isLiftoffStarted())
        {
            DEBUG_PRINT("Liftoff started! Sending event...");
            sendEvent(FSMEvent::LIFTOFF_STARTED);
        }
        break;

    case RocketState::ACCELERATED_FLIGHT:
        if (isAccelerationPhaseComplete())
        {
            DEBUG_PRINT("Acceleration phase complete! Sending event...");
            sendEvent(FSMEvent::ACCELERATION_COMPLETE);
        }
        break;

    case RocketState::BALLISTIC_FLIGHT:
        if (isApogeeReached())
        {
            DEBUG_PRINT("Apogee reached! Sending APOGEE_REACHED event");
            sendEvent(FSMEvent::APOGEE_REACHED);
        }
        break;

    case RocketState::APOGEE:
        if (isDrogueReady())
        {
            DEBUG_PRINT("Drogue ready! Sending event...");
            sendEvent(FSMEvent::DROGUE_READY);
        }
        break;

    case RocketState::STABILIZATION:
        if (isStabilizationComplete())
        {
            DEBUG_PRINT("Stabilization complete! Sending event...");
            sendEvent(FSMEvent::STABILIZATION_COMPLETE);
        }
        break;

    case RocketState::DECELERATION:
        if (isDecelerationComplete())
        {
            DEBUG_PRINT("Deceleration complete! Sending event...");
            sendEvent(FSMEvent::DECELERATION_COMPLETE);
        }
        break;

    case RocketState::LANDING:
        if (isLandingComplete())
        {
            DEBUG_PRINT("Landing complete! Sending event...");
            sendEvent(FSMEvent::LANDING_COMPLETE);
        }
        break;

    default:
        DEBUG_PRINTF("No transition check for state: %s\n", getStateString(state).c_str());
        break;
    }
}

// Keep all your existing state entry/exit and condition checking methods unchanged
void RocketFSM::onInactiveEntry()
{
    DEBUG_PRINT("Inactive state entered");
    debugMemory("onInactiveEntry");

    // if (rocketLogger)
        // rocketLogger->logInfo("System initialization");
}

void RocketFSM::onCalibratingEntry()
{
    DEBUG_PRINT("Calibrating state entered");
    debugMemory("onCalibratingEntry");

    // if (rocketLogger)
        // rocketLogger->logInfo("Starting calibration");
}

void RocketFSM::onReadyForLaunchEntry()
{
    DEBUG_PRINT("Ready for launch state entered");
    debugMemory("onReadyForLaunchEntry");

    // if (rocketLogger)
    {
        // rocketLogger->logInfo("Logger enabled");
        // rocketLogger->logInfo("Data transmission enabled");
    }
}

void RocketFSM::onLaunchEntry()
{
    DEBUG_PRINT("Launch state entered");
    debugMemory("onLaunchEntry");

    // if (rocketLogger)
    {
        // rocketLogger->logInfo("Flight timer started");
        // rocketLogger->logInfo("LAUNCH DETECTED!");
    }
}

void RocketFSM::onAcceleratedFlightEntry()
{
    DEBUG_PRINT("Accelerated flight state entered");
    debugMemory("onAcceleratedFlightEntry");

    // if (rocketLogger)
        // rocketLogger->logInfo("Accelerated flight phase");
}

void RocketFSM::onBallisticFlightEntry()
{
    DEBUG_PRINT("Ballistic flight state entered");
    debugMemory("onBallisticFlightEntry");

    // if (rocketLogger)
        // rocketLogger->logInfo("Ballistic flight phase - EKF started");
}

void RocketFSM::onApogeeEntry()
{
    DEBUG_PRINT("Apogee state entered");
    debugMemory("onApogeeEntry");

    // if (rocketLogger)
        // rocketLogger->logInfo("APOGEE REACHED");
}

void RocketFSM::onStabilizationEntry()
{
    DEBUG_PRINT("Stabilization state entered");
    debugMemory("onStabilizationEntry");

    // if (rocketLogger)
        // rocketLogger->logInfo("Stabilization phase - Drogue deployed");
}

void RocketFSM::onDecelerationEntry()
{
    DEBUG_PRINT("Deceleration state entered");
    debugMemory("onDecelerationEntry");

    // if (rocketLogger)
        // rocketLogger->logInfo("Deceleration phase - Main chute deployed");
}

void RocketFSM::onLandingEntry()
{
    DEBUG_PRINT("Landing state entered");
    debugMemory("onLandingEntry");

    // if (rocketLogger)
        // rocketLogger->logInfo("Landing phase");
}

void RocketFSM::onRecoveredEntry()
{
    DEBUG_PRINT("Recovered state entered");
    debugMemory("onRecoveredEntry");

    // if (rocketLogger)
        // rocketLogger->logInfo("ROCKET RECOVERED");
}

void RocketFSM::onCalibratingExit()
{
    DEBUG_PRINT("Exiting Calibrating state");
    debugMemory("onCalibratingExit");

    // if (rocketLogger)
        // rocketLogger->logInfo("Calibration complete");
}

void RocketFSM::onLandingExit()
{
    DEBUG_PRINT("Exiting Landing state");
    debugMemory("onLandingExit");

    // if (rocketLogger)
        // rocketLogger->logInfo("Saving data to SD card");
    // TODO: Implement SD card saving
}

void RocketFSM::onRecoveredExit()
{
    DEBUG_PRINT("Exiting Recovered state");
    debugMemory("onRecoveredExit");

    // if (rocketLogger)
        // rocketLogger->logInfo("System shutdown");
}

// Modifica questi metodi per avere transizioni automatiche basate sul tempo

bool RocketFSM::isCalibrationComplete()
{
    // Già implementato come timeout, ma riduciamo a 3 secondi per i test
    bool result = millis() - stateStartTime > 3000;
    if (result)
    {
        DEBUG_PRINT("DEBUG: Calibration timeout reached, triggering transition");
    }
    return result;
}

bool RocketFSM::isLaunchDetected()
{
    // Modifica per transizione automatica dopo 4 secondi
    bool result = millis() - stateStartTime > 4000;
    if (result)
    {
        DEBUG_PRINT("DEBUG: Launch detection timeout reached, triggering transition");
    }
    return result;
}

bool RocketFSM::isLiftoffStarted()
{
    // Modifica per transizione automatica dopo 2 secondi
    bool result = millis() - stateStartTime > 2000;
    if (result)
    {
        DEBUG_PRINT("DEBUG: Liftoff timeout reached, triggering transition");
    }
    return result;
}

bool RocketFSM::isAccelerationPhaseComplete()
{
    // Modifica per transizione automatica dopo 5 secondi
    bool result = millis() - stateStartTime > 5000;
    if (result)
    {
        DEBUG_PRINT("DEBUG: Acceleration phase timeout reached, triggering transition");
    }
    return result;
}

bool RocketFSM::isBallisticPhaseComplete()
{
    // Modifica per transizione automatica dopo 6 secondi
    bool result = millis() - stateStartTime > 6000;
    if (result)
    {
        DEBUG_PRINT("DEBUG: Ballistic phase timeout reached, triggering transition");
    }
    return result;
}

bool RocketFSM::isApogeeReached()
{
    // Modifica per transizione automatica dopo 5 secondi di volo balistico
    RocketState state = getCurrentState();
    if (state == RocketState::BALLISTIC_FLIGHT)
    {
        bool result = millis() - stateStartTime > 5000;
        if (result)
        {
            DEBUG_PRINT("DEBUG: Apogee detection timeout reached, triggering transition");
        }
        return result;
    }
    return false;
}

bool RocketFSM::isDrogueReady()
{
    // Già implementato come timeout, ma assicuriamoci sia un valore ragionevole
    bool result = millis() - stateStartTime > 2000;
    if (result)
    {
        DEBUG_PRINT("DEBUG: Drogue ready timeout reached, triggering transition");
    }
    return result;
}

bool RocketFSM::isStabilizationComplete()
{
    // Modifica per transizione automatica dopo 4 secondi
    bool result = millis() - stateStartTime > 4000;
    if (result)
    {
        DEBUG_PRINT("DEBUG: Stabilization timeout reached, triggering transition");
    }
    return result;
}

bool RocketFSM::isDecelerationComplete()
{
    // Modifica per transizione automatica dopo 5 secondi
    bool result = millis() - stateStartTime > 5000;
    if (result)
    {
        DEBUG_PRINT("DEBUG: Deceleration timeout reached, triggering transition");
    }
    return result;
}

bool RocketFSM::isLandingComplete()
{
    // Modifica per transizione automatica dopo 6 secondi
    bool result = millis() - stateStartTime > 6000;
    if (result)
    {
        DEBUG_PRINT("DEBUG: Landing timeout reached, triggering transition");
    }
    return result;
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