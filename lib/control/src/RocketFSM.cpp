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

// Add this utility method to RocketFSM class header
void debugMemory(const char *location)
{
    Serial.printf("\n=== MEMORY DEBUG [%s] ===\n", location);
    Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("Min free heap: %u bytes\n", ESP.getMinFreeHeap());
    Serial.printf("Max alloc heap: %u bytes\n", ESP.getMaxAllocHeap());
    Serial.printf("Largest free block: %u bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    Serial.printf("==========================\n\n");
}

RocketFSM::RocketFSM()
    : currentState(RocketState::INACTIVE), previousState(RocketState::INACTIVE), stateStartTime(0), lastUpdateTime(0), isRunning(false), fsmTaskHandle(nullptr), sensorTaskHandle(nullptr), ekfTaskHandle(nullptr), apogeeDetectionTaskHandle(nullptr), recoveryTaskHandle(nullptr), dataCollectionTaskHandle(nullptr), telemetryTaskHandle(nullptr), gpsTaskHandle(nullptr), loggingTaskHandle(nullptr), eventQueue(nullptr), stateMutex(nullptr), sharedData{SensorData("bno055"), SensorData("baro1"), SensorData("baro2"), SensorData("gps"), 0, false}
{
    Serial.println("RocketFSM constructor called");
    debugMemory("Constructor");
}

RocketFSM::~RocketFSM()
{
    Serial.println("RocketFSM destructor called");
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
    Serial.println("FSM init() called");
    debugMemory("Init start");

    // Create FreeRTOS objects with error checking
    Serial.println("Creating event queue...");
    eventQueue = xQueueCreate(EVENT_QUEUE_SIZE, sizeof(FSMEventData));
    if (!eventQueue)
    {
        Serial.println("ERROR: Failed to create event queue");
        debugMemory("Failed event queue creation");
        return;
    }
    Serial.println("Event queue created successfully");
    debugMemory("After event queue creation");

    Serial.println("Creating state mutex...");
    stateMutex = xSemaphoreCreateMutex();
    if (!stateMutex)
    {
        Serial.println("ERROR: Failed to create state mutex");
        vQueueDelete(eventQueue);
        eventQueue = nullptr;
        debugMemory("Failed mutex creation");
        return;
    }
    Serial.println("State mutex created successfully");
    debugMemory("After mutex creation");

    if (!eventQueue || !stateMutex)
    {
        if (rocketLogger)
        {
            rocketLogger->logError("Failed to create FSM FreeRTOS objects");
        }
        Serial.println("ERROR: FreeRTOS objects creation failed");
        return;
    }

    Serial.println("Setting up state actions...");
    debugMemory("Before setupStateActions");
    setupStateActions();
    Serial.println("State actions setup complete");
    debugMemory("After setupStateActions");

    Serial.println("Setting up transitions...");
    debugMemory("Before setupTransitions");
    setupTransitions();
    Serial.println("Transitions setup complete");
    debugMemory("After setupTransitions");

    currentState = RocketState::INACTIVE;
    stateStartTime = millis();
    lastUpdateTime = millis();

    debugMemory("Init end");

    if (rocketLogger)
    {
        rocketLogger->logInfo("FSM initialized in INACTIVE state");
    }

    Serial.println("FSM init() completed successfully");
}

void RocketFSM::start()
{
    Serial.println("FSM start() called");
    debugMemory("Start begin");

    if (isRunning)
    {
        Serial.println("FSM already running, returning");
        return;
    }

    // Now try with the actual task
    Serial.println("Creating FSM task with global wrapper...");
    debugMemory("Before FSM task creation");

    BaseType_t result = xTaskCreatePinnedToCore(
        fsmTaskWrapper,
        "FSM_Task",
        4096, // Increased stack size
        this, // Pass this pointer
        3,    // Higher priority for FSM
        &fsmTaskHandle,
        1); // Pin to Core 1

    Serial.printf("FSM task creation result: %d\n", result);
    debugMemory("After FSM task creation");

    if (result == pdPASS)
    {
        Serial.println("SUCCESS: FSM task created");
        isRunning = true;
    }
    else
    {
        Serial.println("FAILED: FSM task creation");
        debugMemory("FSM task creation failed");
        return;
    }

    // Only if task created successfully, proceed with state task startup
    Serial.println("Starting state tasks...");
    debugMemory("Before startStateTasks");
    startStateTasks();
    debugMemory("After startStateTasks");
}

// Also modify the stop() method to add more debugging
void RocketFSM::stop()
{
    Serial.println("FSM stop() called");
    Serial.println("Call stack trace (if available):");
    // This won't give you a real stack trace but might help identify patterns
    Serial.printf("Current state: %s, Previous state: %s\n",
                  getStateString(currentState).c_str(),
                  getStateString(previousState).c_str());
    debugMemory("Stop begin");

    if (!isRunning)
    {
        Serial.println("FSM already stopped, returning");
        return;
    }

    Serial.println("Setting isRunning to false...");
    isRunning = false;

    Serial.println("Stopping all tasks...");
    stopAllTasks();

    if (fsmTaskHandle)
    {
        Serial.println("Deleting FSM task...");
        vTaskDelete(fsmTaskHandle);
        fsmTaskHandle = nullptr;
    }

    if (rocketLogger)
    {
        rocketLogger->logInfo("FSM stopped");
    }

    debugMemory("Stop end");
}

bool RocketFSM::sendEvent(FSMEvent event, RocketState targetState, void *eventData)
{
    if (!eventQueue)
        return false;

    FSMEventData eventStruct(event, targetState, eventData);
    bool result = xQueueSend(eventQueue, &eventStruct, pdMS_TO_TICKS(100)) == pdPASS;
    Serial.printf("Event sent: %d, result: %d\n", static_cast<int>(event), result);
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
    Serial.println("Global FSM task wrapper started");
    debugMemory("FSM task wrapper start");

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (!fsm)
    {
        Serial.println("ERROR: Invalid FSM pointer");
        debugMemory("Invalid FSM pointer");
        vTaskDelete(NULL);
        return;
    }

    Serial.println("Calling fsm->fsmTask()");
    fsm->fsmTask();

    // Should never reach here unless task exits
    Serial.println("Global FSM task wrapper exiting");
    debugMemory("FSM task wrapper end");
    vTaskDelete(NULL);
}

void RocketFSM::fsmTask()
{
    Serial.println("fsmTask started");
    debugMemory("fsmTask start");

    Serial.printf("isRunning = %d\n", isRunning);
    Serial.printf("Current state: %s\n", getStateString(currentState).c_str());

    // Create a proper event structure
    FSMEventData eventData(FSMEvent::NONE);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    unsigned long loopCounter = 0;

    // Register with watchdog timer for safety
    esp_task_wdt_add(NULL);

    // Critical: manually check transitions for INACTIVE state immediately
    if (currentState == RocketState::INACTIVE)
    {
        Serial.println("Initial state is INACTIVE, sending START_CALIBRATION event");
        sendEvent(FSMEvent::START_CALIBRATION);
        if (!isRunning)
        {
            Serial.println("WARNING: isRunning is false after sending START_CALIBRATION!");
            isRunning = true;
            Serial.println("Re-enabling FSM task execution...");
        }
    }

    while (isRunning)
    {
        // Reset watchdog
        esp_task_wdt_reset();

        // Check if isRunning was changed externally
        if (!isRunning)
        {
            Serial.println("WARNING: isRunning changed to false unexpectedly!");
            Serial.println("Re-enabling FSM task execution...");
            isRunning = true;
        }

        // Memory check every 100 loops
        if (loopCounter % 100 == 0)
        {
            debugMemory("FSM main loop");
            Serial.printf("FSM running status: %d\n", isRunning);
        }

        checkTransitions();

        // Check for events (non-blocking with timeout)
        if (xQueueReceive(eventQueue, &eventData, pdMS_TO_TICKS(50)) == pdPASS)
        {
            Serial.printf("Received event: %d for state: %s\n",
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
                Serial.println("EXCEPTION: Error in processEvent!");
                debugMemory("processEvent-exception");
            }
        }

        // Periodically output the current state for debugging
        if (loopCounter % 20 == 0)
        {
            Serial.printf("FSM Loop #%lu: Current state: %s, Time in state: %lu ms, isRunning: %d\n",
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
    Serial.println("fsmTask ended because isRunning became false!");
    Serial.println("Stack high water mark: " + String(uxTaskGetStackHighWaterMark(NULL)));
    debugMemory("fsmTask end");

    // Clean up watchdog
    esp_task_wdt_delete(NULL);
}

void RocketFSM::setupStateActions()
{
    Serial.println("Setting up state actions...");
    debugMemory("setupStateActions start");

    // INACTIVE state
    stateActions[RocketState::INACTIVE] = StateActions(
        [this]()
        { onInactiveEntry(); });
    Serial.println("INACTIVE state action set");

    // CALIBRATING state
    stateActions[RocketState::CALIBRATING] = StateActions(
        [this]()
        { onCalibratingEntry(); },
        [this]()
        { onCalibratingExit(); });
    stateActions[RocketState::CALIBRATING].sensorTask = TaskConfig("Sensor_Calib", 3072, 4, 0, true);
    stateActions[RocketState::CALIBRATING].loggingTask = TaskConfig("Log_Calib", 2048, 1, 1, true);
    Serial.println("CALIBRATING state action set");

    // READY_FOR_LAUNCH state
    stateActions[RocketState::READY_FOR_LAUNCH] = StateActions(
        [this]()
        { onReadyForLaunchEntry(); });
    stateActions[RocketState::READY_FOR_LAUNCH].sensorTask = TaskConfig("Sensor_Ready", 3072, 4, 0, true);
    stateActions[RocketState::READY_FOR_LAUNCH].telemetryTask = TaskConfig("Telemetry_Ready", 3072, 2, 1, true);
    stateActions[RocketState::READY_FOR_LAUNCH].loggingTask = TaskConfig("Log_Ready", 2048, 1, 1, true);
    Serial.println("READY_FOR_LAUNCH state action set");

    // LAUNCH state
    stateActions[RocketState::LAUNCH] = StateActions(
        [this]()
        { onLaunchEntry(); });
    stateActions[RocketState::LAUNCH].sensorTask = TaskConfig("Sensor_Launch", 3072, 5, 0, true);
    stateActions[RocketState::LAUNCH].dataCollectionTask = TaskConfig("DataCol_Launch", 2048, 2, 1, true);
    stateActions[RocketState::LAUNCH].loggingTask = TaskConfig("Log_Launch", 2048, 1, 1, true);
    Serial.println("LAUNCH state action set");

    // ACCELERATED_FLIGHT state
    stateActions[RocketState::ACCELERATED_FLIGHT] = StateActions(
        [this]()
        { onAcceleratedFlightEntry(); });
    stateActions[RocketState::ACCELERATED_FLIGHT].sensorTask = TaskConfig("Sensor_AccFlight", 3072, 5, 0, true);
    stateActions[RocketState::ACCELERATED_FLIGHT].ekfTask = TaskConfig("EKF_AccFlight", 4096, 4, 0, true);
    stateActions[RocketState::ACCELERATED_FLIGHT].telemetryTask = TaskConfig("Telemetry_AccFlight", 3072, 2, 1, true);
    stateActions[RocketState::ACCELERATED_FLIGHT].dataCollectionTask = TaskConfig("DataCol_AccFlight", 2048, 2, 1, true);
    Serial.println("ACCELERATED_FLIGHT state action set");

    // BALLISTIC_FLIGHT state
    stateActions[RocketState::BALLISTIC_FLIGHT] = StateActions(
        [this]()
        { onBallisticFlightEntry(); });
    stateActions[RocketState::BALLISTIC_FLIGHT].sensorTask = TaskConfig("Sensor_BalFlight", 3072, 5, 0, true);
    stateActions[RocketState::BALLISTIC_FLIGHT].ekfTask = TaskConfig("EKF_BalFlight", 4096, 4, 0, true);
    stateActions[RocketState::BALLISTIC_FLIGHT].apogeeDetectionTask = TaskConfig("Apogee_Detection", 2048, 4, 0, true);
    stateActions[RocketState::BALLISTIC_FLIGHT].telemetryTask = TaskConfig("Telemetry_BalFlight", 3072, 2, 1, true);
    stateActions[RocketState::BALLISTIC_FLIGHT].dataCollectionTask = TaskConfig("DataCol_BalFlight", 2048, 2, 1, true);
    Serial.println("BALLISTIC_FLIGHT state action set");

    // APOGEE state
    stateActions[RocketState::APOGEE] = StateActions(
        [this]()
        { onApogeeEntry(); });
    stateActions[RocketState::APOGEE].sensorTask = TaskConfig("Sensor_Apogee", 3072, 5, 0, true);
    stateActions[RocketState::APOGEE].recoveryTask = TaskConfig("Recovery_Apogee", 2048, 5, 0, true);
    stateActions[RocketState::APOGEE].dataCollectionTask = TaskConfig("DataCol_Apogee", 2048, 2, 1, true);
    stateActions[RocketState::APOGEE].telemetryTask = TaskConfig("Telemetry_Apogee", 3072, 2, 1, true);
    Serial.println("APOGEE state action set");

    // STABILIZATION state
    stateActions[RocketState::STABILIZATION] = StateActions(
        [this]()
        { onStabilizationEntry(); });
    stateActions[RocketState::STABILIZATION].sensorTask = TaskConfig("Sensor_Stab", 3072, 4, 0, true);
    stateActions[RocketState::STABILIZATION].recoveryTask = TaskConfig("Recovery_Stab", 2048, 4, 0, true);
    stateActions[RocketState::STABILIZATION].telemetryTask = TaskConfig("Telemetry_Stab", 3072, 2, 1, true);
    stateActions[RocketState::STABILIZATION].dataCollectionTask = TaskConfig("DataCol_Stab", 2048, 2, 1, true);
    stateActions[RocketState::STABILIZATION].gpsTask = TaskConfig("GPS_Stab", 2048, 1, 1, true);
    Serial.println("STABILIZATION state action set");

    // DECELERATION state
    stateActions[RocketState::DECELERATION] = StateActions(
        [this]()
        { onDecelerationEntry(); });
    stateActions[RocketState::DECELERATION].sensorTask = TaskConfig("Sensor_Decel", 3072, 4, 0, true);
    stateActions[RocketState::DECELERATION].recoveryTask = TaskConfig("Recovery_Decel", 2048, 4, 0, true);
    stateActions[RocketState::DECELERATION].telemetryTask = TaskConfig("Telemetry_Decel", 3072, 2, 1, true);
    stateActions[RocketState::DECELERATION].dataCollectionTask = TaskConfig("DataCol_Decel", 2048, 2, 1, true);
    stateActions[RocketState::DECELERATION].gpsTask = TaskConfig("GPS_Decel", 2048, 1, 1, true);
    Serial.println("DECELERATION state action set");

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
    Serial.println("LANDING state action set");

    // RECOVERED state
    stateActions[RocketState::RECOVERED] = StateActions(
        [this]()
        { onRecoveredEntry(); },
        [this]()
        { onRecoveredExit(); });
    stateActions[RocketState::RECOVERED].gpsTask = TaskConfig("GPS_Recovered", 2048, 1, 1, true);
    stateActions[RocketState::RECOVERED].loggingTask = TaskConfig("Log_Recovered", 3072, 1, 1, true);
    Serial.println("RECOVERED state action set");

    debugMemory("setupStateActions end");
}

void RocketFSM::setupTransitions()
{
    Serial.println("Setting up transitions...");
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

    Serial.printf("Attempting transition from %s to %s\n", getStateString(currentState).c_str(), getStateString(newState).c_str());
    debugMemory("Before transition mutex take");

    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(1000)) == pdTRUE)
    {
        Serial.println("\n==================== STATE TRANSITION ====================");
        Serial.printf("Time: %lu ms\n", millis());
        Serial.printf("Transition: %s -> %s\n", getStateString(currentState).c_str(), getStateString(newState).c_str());
        debugMemory("Transition start");

        if (rocketLogger)
        {
            String transitionMsg = "FSM transition: " + getStateString(currentState) + " -> " + getStateString(newState);
            rocketLogger->logInfo(transitionMsg.c_str());
        }

        // Execute exit action for current state
        if (stateActions[currentState].onExit)
        {
            Serial.println("Executing exit action for current state...");
            debugMemory("Before exit action");
            stateActions[currentState].onExit();
            debugMemory("After exit action");
        }

        // Stop current state tasks
        Serial.println("Stopping current state tasks...");
        debugMemory("Before stopAllTasks");
        stopAllTasks();
        debugMemory("After stopAllTasks");

        previousState = currentState;
        currentState = newState;
        stateStartTime = millis();

        // Execute entry action for new state
        if (stateActions[currentState].onEntry)
        {
            Serial.println("Executing entry action for new state...");
            debugMemory("Before entry action");
            stateActions[currentState].onEntry();
            debugMemory("After entry action");
        }

        // Start new state tasks
        Serial.println("Starting new state tasks...");
        debugMemory("Before startStateTasks");
        startStateTasks();
        debugMemory("After startStateTasks");

        Serial.println("==========================================================\n");

        xSemaphoreGive(stateMutex);
        debugMemory("After transition mutex give");
    }
    else
    {
        Serial.println("ERROR: Failed to take mutex for transition");
        debugMemory("Failed mutex take");
    }
}

void RocketFSM::processEvent(const FSMEventData &eventData)
{
    Serial.printf("Processing event: %d\n", static_cast<int>(eventData.event));
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
            Serial.printf("Found matching transition: %s -> %s\n",
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
    Serial.printf("Starting tasks for state: %s\n", getStateString(currentState).c_str());
    debugMemory("startStateTasks begin");

    const StateActions &actions = stateActions[currentState];

    // Start Core 0 tasks (Critical)
    if (actions.sensorTask.shouldRun)
    {
        Serial.printf("Creating sensor task: %s\n", actions.sensorTask.name);
        debugMemory("Before sensor task creation");

        BaseType_t result = xTaskCreatePinnedToCore(sensorTaskWrapper, actions.sensorTask.name, actions.sensorTask.stackSize,
                                                    this, actions.sensorTask.priority, &sensorTaskHandle, actions.sensorTask.coreId);
        if (result != pdPASS)
        {
            Serial.printf("Failed to create sensor task: %d\n", result);
            debugMemory("Sensor task creation failed");
        }
        else
        {
            Serial.println("Sensor task created successfully");
            debugMemory("After sensor task creation");
        }
    }

    if (actions.ekfTask.shouldRun)
    {
        Serial.printf("Creating EKF task: %s\n", actions.ekfTask.name);
        debugMemory("Before EKF task creation");

        BaseType_t result = xTaskCreatePinnedToCore(ekfTaskWrapper, actions.ekfTask.name, actions.ekfTask.stackSize,
                                                    this, actions.ekfTask.priority, &ekfTaskHandle, actions.ekfTask.coreId);
        if (result != pdPASS)
        {
            Serial.printf("Failed to create EKF task: %d\n", result);
            debugMemory("EKF task creation failed");
        }
        else
        {
            Serial.println("EKF task created successfully");
            debugMemory("After EKF task creation");
        }
    }

    if (actions.apogeeDetectionTask.shouldRun)
    {
        Serial.printf("Creating apogee detection task: %s\n", actions.apogeeDetectionTask.name);
        debugMemory("Before apogee task creation");

        BaseType_t result = xTaskCreatePinnedToCore(apogeeDetectionTaskWrapper, actions.apogeeDetectionTask.name, actions.apogeeDetectionTask.stackSize,
                                                    this, actions.apogeeDetectionTask.priority, &apogeeDetectionTaskHandle, actions.apogeeDetectionTask.coreId);
        if (result != pdPASS)
        {
            Serial.printf("Failed to create apogee detection task: %d\n", result);
            debugMemory("Apogee task creation failed");
        }
        else
        {
            Serial.println("Apogee detection task created successfully");
            debugMemory("After apogee task creation");
        }
    }

    if (actions.recoveryTask.shouldRun)
    {
        Serial.printf("Creating recovery task: %s\n", actions.recoveryTask.name);
        debugMemory("Before recovery task creation");

        BaseType_t result = xTaskCreatePinnedToCore(recoveryTaskWrapper, actions.recoveryTask.name, actions.recoveryTask.stackSize,
                                                    this, actions.recoveryTask.priority, &recoveryTaskHandle, actions.recoveryTask.coreId);
        if (result != pdPASS)
        {
            Serial.printf("Failed to create recovery task: %d\n", result);
            debugMemory("Recovery task creation failed");
        }
        else
        {
            Serial.println("Recovery task created successfully");
            debugMemory("After recovery task creation");
        }
    }

    // Start Core 1 tasks (Non-critical)
    if (actions.dataCollectionTask.shouldRun)
    {
        Serial.printf("Creating data collection task: %s\n", actions.dataCollectionTask.name);
        debugMemory("Before data collection task creation");

        BaseType_t result = xTaskCreatePinnedToCore(dataCollectionTaskWrapper, actions.dataCollectionTask.name, actions.dataCollectionTask.stackSize,
                                                    this, actions.dataCollectionTask.priority, &dataCollectionTaskHandle, actions.dataCollectionTask.coreId);
        if (result != pdPASS)
        {
            Serial.printf("Failed to create data collection task: %d\n", result);
            debugMemory("Data collection task creation failed");
        }
        else
        {
            Serial.println("Data collection task created successfully");
            debugMemory("After data collection task creation");
        }
    }

    if (actions.telemetryTask.shouldRun)
    {
        Serial.printf("Creating telemetry task: %s\n", actions.telemetryTask.name);
        debugMemory("Before telemetry task creation");

        BaseType_t result = xTaskCreatePinnedToCore(telemetryTaskWrapper, actions.telemetryTask.name, actions.telemetryTask.stackSize,
                                                    this, actions.telemetryTask.priority, &telemetryTaskHandle, actions.telemetryTask.coreId);
        if (result != pdPASS)
        {
            Serial.printf("Failed to create telemetry task: %d\n", result);
            debugMemory("Telemetry task creation failed");
        }
        else
        {
            Serial.println("Telemetry task created successfully");
            debugMemory("After telemetry task creation");
        }
    }

    if (actions.gpsTask.shouldRun)
    {
        Serial.printf("Creating GPS task: %s\n", actions.gpsTask.name);
        debugMemory("Before GPS task creation");

        BaseType_t result = xTaskCreatePinnedToCore(gpsTaskWrapper, actions.gpsTask.name, actions.gpsTask.stackSize,
                                                    this, actions.gpsTask.priority, &gpsTaskHandle, actions.gpsTask.coreId);
        if (result != pdPASS)
        {
            Serial.printf("Failed to create GPS task: %d\n", result);
            debugMemory("GPS task creation failed");
        }
        else
        {
            Serial.println("GPS task created successfully");
            debugMemory("After GPS task creation");
        }
    }

    if (actions.loggingTask.shouldRun)
    {
        Serial.printf("Creating logging task: %s\n", actions.loggingTask.name);
        debugMemory("Before logging task creation");

        BaseType_t result = xTaskCreatePinnedToCore(loggingTaskWrapper, actions.loggingTask.name, actions.loggingTask.stackSize,
                                                    this, actions.loggingTask.priority, &loggingTaskHandle, actions.loggingTask.coreId);
        if (result != pdPASS)
        {
            Serial.printf("Failed to create logging task: %d\n", result);
            debugMemory("Logging task creation failed");
        }
        else
        {
            Serial.println("Logging task created successfully");
            debugMemory("After logging task creation");
        }
    }

    debugMemory("startStateTasks end");
    Serial.println("All state tasks creation completed");
}

void RocketFSM::stopAllTasks()
{
    Serial.println("Stopping all tasks...");
    debugMemory("stopAllTasks begin");

    // Stop Core 0 tasks
    if (sensorTaskHandle)
    {
        Serial.println("Deleting sensor task");
        vTaskDelete(sensorTaskHandle);
        sensorTaskHandle = nullptr;
        debugMemory("After sensor task deletion");
    }
    if (ekfTaskHandle)
    {
        Serial.println("Deleting EKF task");
        vTaskDelete(ekfTaskHandle);
        ekfTaskHandle = nullptr;
        debugMemory("After EKF task deletion");
    }
    if (apogeeDetectionTaskHandle)
    {
        Serial.println("Deleting apogee detection task");
        vTaskDelete(apogeeDetectionTaskHandle);
        apogeeDetectionTaskHandle = nullptr;
        debugMemory("After apogee task deletion");
    }
    if (recoveryTaskHandle)
    {
        Serial.println("Deleting recovery task");
        vTaskDelete(recoveryTaskHandle);
        recoveryTaskHandle = nullptr;
        debugMemory("After recovery task deletion");
    }

    // Stop Core 1 tasks
    if (dataCollectionTaskHandle)
    {
        Serial.println("Deleting data collection task");
        vTaskDelete(dataCollectionTaskHandle);
        dataCollectionTaskHandle = nullptr;
        debugMemory("After data collection task deletion");
    }
    if (telemetryTaskHandle)
    {
        Serial.println("Deleting telemetry task");
        vTaskDelete(telemetryTaskHandle);
        telemetryTaskHandle = nullptr;
        debugMemory("After telemetry task deletion");
    }
    if (gpsTaskHandle)
    {
        Serial.println("Deleting GPS task");
        vTaskDelete(gpsTaskHandle);
        gpsTaskHandle = nullptr;
        debugMemory("After GPS task deletion");
    }
    if (loggingTaskHandle)
    {
        Serial.println("Deleting logging task");
        vTaskDelete(loggingTaskHandle);
        loggingTaskHandle = nullptr;
        debugMemory("After logging task deletion");
    }

    // Add small delay to ensure tasks are cleaned up
    vTaskDelay(pdMS_TO_TICKS(100));
    debugMemory("stopAllTasks end");
    Serial.println("All tasks stopped");
}

// Task wrappers for Core 0 (Critical)
void IRAM_ATTR RocketFSM::sensorTaskWrapper(void *parameter)
{
    Serial.println("Sensor task wrapper started");
    debugMemory("Sensor task wrapper start");

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (fsm)
    {
        fsm->sensorTask();
    }

    Serial.println("Sensor task wrapper ending");
    debugMemory("Sensor task wrapper end");
    vTaskDelete(NULL);
}

void IRAM_ATTR RocketFSM::ekfTaskWrapper(void *parameter)
{
    Serial.println("EKF task wrapper started");
    debugMemory("EKF task wrapper start");

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (fsm)
    {
        fsm->ekfTask();
    }

    Serial.println("EKF task wrapper ending");
    debugMemory("EKF task wrapper end");
    vTaskDelete(NULL);
}

void IRAM_ATTR RocketFSM::apogeeDetectionTaskWrapper(void *parameter)
{
    Serial.println("Apogee detection task wrapper started");
    debugMemory("Apogee task wrapper start");

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (fsm)
    {
        fsm->apogeeDetectionTask();
    }

    Serial.println("Apogee detection task wrapper ending");
    debugMemory("Apogee task wrapper end");
    vTaskDelete(NULL);
}

void IRAM_ATTR RocketFSM::recoveryTaskWrapper(void *parameter)
{
    Serial.println("Recovery task wrapper started");
    debugMemory("Recovery task wrapper start");

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (fsm)
    {
        fsm->recoveryTask();
    }

    Serial.println("Recovery task wrapper ending");
    debugMemory("Recovery task wrapper end");
    vTaskDelete(NULL);
}

// Task wrappers for Core 1 (Non-critical)
void IRAM_ATTR RocketFSM::dataCollectionTaskWrapper(void *parameter)
{
    Serial.println("Data collection task wrapper started");
    debugMemory("Data collection task wrapper start");

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (fsm)
    {
        fsm->dataCollectionTask();
    }

    Serial.println("Data collection task wrapper ending");
    debugMemory("Data collection task wrapper end");
    vTaskDelete(NULL);
}

void IRAM_ATTR RocketFSM::telemetryTaskWrapper(void *parameter)
{
    Serial.println("Telemetry task wrapper started");
    debugMemory("Telemetry task wrapper start");

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (fsm)
    {
        fsm->telemetryTask();
    }

    Serial.println("Telemetry task wrapper ending");
    debugMemory("Telemetry task wrapper end");
    vTaskDelete(NULL);
}

void IRAM_ATTR RocketFSM::gpsTaskWrapper(void *parameter)
{
    Serial.println("GPS task wrapper started");
    debugMemory("GPS task wrapper start");

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (fsm)
    {
        fsm->gpsTask();
    }

    Serial.println("GPS task wrapper ending");
    debugMemory("GPS task wrapper end");
    vTaskDelete(NULL);
}

void IRAM_ATTR RocketFSM::loggingTaskWrapper(void *parameter)
{
    Serial.println("Logging task wrapper started");
    debugMemory("Logging task wrapper start");

    RocketFSM *fsm = static_cast<RocketFSM *>(parameter);
    if (fsm)
    {
        fsm->loggingTask();
    }

    Serial.println("Logging task wrapper ending");
    debugMemory("Logging task wrapper end");
    vTaskDelete(NULL);
}

// Core 0 Task implementations (Critical) - OPTIMIZED
void RocketFSM::sensorTask()
{
    Serial.println("Sensor task started");
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

    Serial.println("Sensor task ended");
    debugMemory("Sensor task end");
}

void RocketFSM::ekfTask()
{
    Serial.println("EKF task started");
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

    Serial.println("EKF task ended");
    debugMemory("EKF task end");
}

void RocketFSM::apogeeDetectionTask()
{
    Serial.println("Apogee detection task started");
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
            Serial.println("Apogee reached! Sending event...");
            sendEvent(FSMEvent::APOGEE_REACHED);
        }

        loopCount++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    Serial.println("Apogee detection task ended");
    debugMemory("Apogee detection task end");
}

void RocketFSM::recoveryTask()
{
    Serial.println("Recovery task started");
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

    Serial.println("Recovery task ended");
    debugMemory("Recovery task end");
}

// Core 1 Task implementations (Non-critical) - OPTIMIZED
void RocketFSM::dataCollectionTask()
{
    Serial.println("Data collection task started");
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

        if (rocketLogger)
        {
            rocketLogger->logSensorData(sharedData.imuData);
            rocketLogger->logSensorData(sharedData.baroData1);
            rocketLogger->logSensorData(sharedData.baroData2);
            rocketLogger->logSensorData(sharedData.gpsData);
        }

        loopCount++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    Serial.println("Data collection task ended");
    debugMemory("Data collection task end");
}

void RocketFSM::telemetryTask()
{
    Serial.println("Telemetry task started");
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
            auto response = loraTransmitter->transmit(rocketLogger->getJSONAll());
            // Process response if needed
        }

        loopCount++;
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    Serial.println("Telemetry task ended");
    debugMemory("Telemetry task end");
}

void RocketFSM::gpsTask()
{
    Serial.println("GPS task started");
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

    Serial.println("GPS task ended");
    debugMemory("GPS task end");
}

void RocketFSM::loggingTask()
{
    Serial.println("Logging task started");
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

    Serial.println("Logging task ended");
    debugMemory("Logging task end");
}

void RocketFSM::checkTransitions()
{
    RocketState state = getCurrentState();

    switch (state)
    {
    case RocketState::INACTIVE:
        Serial.println("In INACTIVE state, sending START_CALIBRATION event");
        Serial.printf("START_CALIBRATION event sent result: %d\n", sendEvent(FSMEvent::START_CALIBRATION));
        break;

    case RocketState::CALIBRATING:
        if (isCalibrationComplete())
        {
            Serial.println("Calibration complete! Sending event...");
            sendEvent(FSMEvent::CALIBRATION_COMPLETE);
        }
        break;

    case RocketState::READY_FOR_LAUNCH:
    {
        unsigned long currentTime = millis();
        unsigned long timeInState = currentTime - stateStartTime;

        if (isLaunchDetected())
        {
            Serial.println("*** LAUNCH DETECTED! Sending LAUNCH_DETECTED event ***");
            bool sent = sendEvent(FSMEvent::LAUNCH_DETECTED);
            Serial.printf("Event sent result: %d\n", sent);
        }
    }
    break;

    case RocketState::LAUNCH:
        if (isLiftoffStarted())
        {
            Serial.println("Liftoff started! Sending event...");
            sendEvent(FSMEvent::LIFTOFF_STARTED);
        }
        break;

    case RocketState::ACCELERATED_FLIGHT:
        if (isAccelerationPhaseComplete())
        {
            Serial.println("Acceleration phase complete! Sending event...");
            sendEvent(FSMEvent::ACCELERATION_COMPLETE);
        }
        break;

    case RocketState::BALLISTIC_FLIGHT:
        Serial.println("Checking for apogee...");
        if (isApogeeReached())
        {
            Serial.println("Apogee reached! Sending APOGEE_REACHED event");
            sendEvent(FSMEvent::APOGEE_REACHED);
        }
        break;

    case RocketState::APOGEE:
        if (isDrogueReady())
        {
            Serial.println("Drogue ready! Sending event...");
            sendEvent(FSMEvent::DROGUE_READY);
        }
        break;

    case RocketState::STABILIZATION:
        if (isStabilizationComplete())
        {
            Serial.println("Stabilization complete! Sending event...");
            sendEvent(FSMEvent::STABILIZATION_COMPLETE);
        }
        break;

    case RocketState::DECELERATION:
        if (isDecelerationComplete())
        {
            Serial.println("Deceleration complete! Sending event...");
            sendEvent(FSMEvent::DECELERATION_COMPLETE);
        }
        break;

    case RocketState::LANDING:
        if (isLandingComplete())
        {
            Serial.println("Landing complete! Sending event...");
            sendEvent(FSMEvent::LANDING_COMPLETE);
        }
        break;

    default:
        Serial.printf("No transition check for state: %s\n", getStateString(state).c_str());
        break;
    }
}

// Keep all your existing state entry/exit and condition checking methods unchanged
void RocketFSM::onInactiveEntry()
{
    Serial.println("Inactive state entered");
    debugMemory("onInactiveEntry");

    if (rocketLogger)
        rocketLogger->logInfo("System initialization");
}

void RocketFSM::onCalibratingEntry()
{
    Serial.println("Calibrating state entered");
    debugMemory("onCalibratingEntry");

    if (rocketLogger)
        rocketLogger->logInfo("Starting calibration");
}

void RocketFSM::onReadyForLaunchEntry()
{
    Serial.println("Ready for launch state entered");
    debugMemory("onReadyForLaunchEntry");

    if (rocketLogger)
    {
        rocketLogger->logInfo("Logger enabled");
        rocketLogger->logInfo("Data transmission enabled");
    }
}

void RocketFSM::onLaunchEntry()
{
    Serial.println("Launch state entered");
    debugMemory("onLaunchEntry");

    if (rocketLogger)
    {
        rocketLogger->logInfo("Flight timer started");
        rocketLogger->logInfo("LAUNCH DETECTED!");
    }
}

void RocketFSM::onAcceleratedFlightEntry()
{
    Serial.println("Accelerated flight state entered");
    debugMemory("onAcceleratedFlightEntry");

    if (rocketLogger)
        rocketLogger->logInfo("Accelerated flight phase");
}

void RocketFSM::onBallisticFlightEntry()
{
    Serial.println("Ballistic flight state entered");
    debugMemory("onBallisticFlightEntry");

    if (rocketLogger)
        rocketLogger->logInfo("Ballistic flight phase - EKF started");
}

void RocketFSM::onApogeeEntry()
{
    Serial.println("Apogee state entered");
    debugMemory("onApogeeEntry");

    if (rocketLogger)
        rocketLogger->logInfo("APOGEE REACHED");
}

void RocketFSM::onStabilizationEntry()
{
    Serial.println("Stabilization state entered");
    debugMemory("onStabilizationEntry");

    if (rocketLogger)
        rocketLogger->logInfo("Stabilization phase - Drogue deployed");
}

void RocketFSM::onDecelerationEntry()
{
    Serial.println("Deceleration state entered");
    debugMemory("onDecelerationEntry");

    if (rocketLogger)
        rocketLogger->logInfo("Deceleration phase - Main chute deployed");
}

void RocketFSM::onLandingEntry()
{
    Serial.println("Landing state entered");
    debugMemory("onLandingEntry");

    if (rocketLogger)
        rocketLogger->logInfo("Landing phase");
}

void RocketFSM::onRecoveredEntry()
{
    Serial.println("Recovered state entered");
    debugMemory("onRecoveredEntry");

    if (rocketLogger)
        rocketLogger->logInfo("ROCKET RECOVERED");
}

void RocketFSM::onCalibratingExit()
{
    Serial.println("Exiting Calibrating state");
    debugMemory("onCalibratingExit");

    if (rocketLogger)
        rocketLogger->logInfo("Calibration complete");
}

void RocketFSM::onLandingExit()
{
    Serial.println("Exiting Landing state");
    debugMemory("onLandingExit");

    if (rocketLogger)
        rocketLogger->logInfo("Saving data to SD card");
    // TODO: Implement SD card saving
}

void RocketFSM::onRecoveredExit()
{
    Serial.println("Exiting Recovered state");
    debugMemory("onRecoveredExit");

    if (rocketLogger)
        rocketLogger->logInfo("System shutdown");
}

// Modifica questi metodi per avere transizioni automatiche basate sul tempo

bool RocketFSM::isCalibrationComplete()
{
    // Già implementato come timeout, ma riduciamo a 3 secondi per i test
    bool result = millis() - stateStartTime > 3000;
    if (result)
    {
        Serial.println("DEBUG: Calibration timeout reached, triggering transition");
    }
    return result;
}

bool RocketFSM::isLaunchDetected()
{
    // Modifica per transizione automatica dopo 4 secondi
    bool result = millis() - stateStartTime > 4000;
    if (result)
    {
        Serial.println("DEBUG: Launch detection timeout reached, triggering transition");
    }
    return result;
}

bool RocketFSM::isLiftoffStarted()
{
    // Modifica per transizione automatica dopo 2 secondi
    bool result = millis() - stateStartTime > 2000;
    if (result)
    {
        Serial.println("DEBUG: Liftoff timeout reached, triggering transition");
    }
    return result;
}

bool RocketFSM::isAccelerationPhaseComplete()
{
    // Modifica per transizione automatica dopo 5 secondi
    bool result = millis() - stateStartTime > 5000;
    if (result)
    {
        Serial.println("DEBUG: Acceleration phase timeout reached, triggering transition");
    }
    return result;
}

bool RocketFSM::isBallisticPhaseComplete()
{
    // Modifica per transizione automatica dopo 6 secondi
    bool result = millis() - stateStartTime > 6000;
    if (result)
    {
        Serial.println("DEBUG: Ballistic phase timeout reached, triggering transition");
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
            Serial.println("DEBUG: Apogee detection timeout reached, triggering transition");
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
        Serial.println("DEBUG: Drogue ready timeout reached, triggering transition");
    }
    return result;
}

bool RocketFSM::isStabilizationComplete()
{
    // Modifica per transizione automatica dopo 4 secondi
    bool result = millis() - stateStartTime > 4000;
    if (result)
    {
        Serial.println("DEBUG: Stabilization timeout reached, triggering transition");
    }
    return result;
}

bool RocketFSM::isDecelerationComplete()
{
    // Modifica per transizione automatica dopo 5 secondi
    bool result = millis() - stateStartTime > 5000;
    if (result)
    {
        Serial.println("DEBUG: Deceleration timeout reached, triggering transition");
    }
    return result;
}

bool RocketFSM::isLandingComplete()
{
    // Modifica per transizione automatica dopo 6 secondi
    bool result = millis() - stateStartTime > 6000;
    if (result)
    {
        Serial.println("DEBUG: Landing timeout reached, triggering transition");
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