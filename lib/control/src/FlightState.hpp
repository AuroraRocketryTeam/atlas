#pragma once
#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

/**
 * @brief Enum representing all possible rocket states
 */
enum class RocketState
{
    // Initial state
    INACTIVE,

    // Pre-flight phase
    CALIBRATING,
    READY_FOR_LAUNCH,

    // Flight phase
    LAUNCH,
    ACCELERATED_FLIGHT,
    BALLISTIC_FLIGHT,
    APOGEE,

    // Recovery phase
    STABILIZATION,
    DECELERATION,
    LANDING,
    RECOVERED
};

/**
 * @brief Enum representing flight phases for better organization
 */
enum class FlightPhase
{
    PRE_FLIGHT,
    FLIGHT,
    RECOVERY
};

/**
 * @brief Enum for FSM events
 */
enum class FSMEvent
{
    NONE,
    START_CALIBRATION,
    CALIBRATION_COMPLETE,
    LAUNCH_DETECTED,
    LIFTOFF_STARTED,
    ACCELERATION_COMPLETE,
    BALLISTIC_COMPLETE,
    APOGEE_REACHED,
    DROGUE_READY,
    STABILIZATION_COMPLETE,
    DECELERATION_COMPLETE,
    LANDING_COMPLETE,
    FORCE_TRANSITION,
    EMERGENCY_ABORT
};

/**
 * @brief Structure for FSM event data
 */
struct FSMEventData
{
    FSMEvent event;
    RocketState targetState; // Used for force transitions
    void *eventData;         // Optional event data

    FSMEventData(FSMEvent e, RocketState target = RocketState::INACTIVE, void *data = nullptr)
        : event(e), targetState(target), eventData(data) {}
};

/**
 * @brief Structure to hold state transition information
 */
struct StateTransition
{
    RocketState fromState;
    RocketState toState;
    FSMEvent triggerEvent;

    StateTransition(RocketState from, RocketState to, FSMEvent event)
        : fromState(from), toState(to), triggerEvent(event) {}
};

/**
 * @brief Structure to hold FreeRTOS task configurations for each state
 */
struct TaskConfig
{
    const char *name;
    uint32_t stackSize;
    UBaseType_t priority;
    BaseType_t coreId;
    bool shouldRun;

    TaskConfig(const char *n = nullptr, uint32_t stack = 2048, UBaseType_t prio = 1,
               BaseType_t core = tskNO_AFFINITY, bool run = false)
        : name(n), stackSize(stack), priority(prio), coreId(core), shouldRun(run) {}
};

/**
 * @brief Structure to hold state actions with FreeRTOS task management
 */
struct StateActions
{
    std::function<void()> onEntry;
    std::function<void()> onExit;
    TaskConfig dataCollectionTask;
    TaskConfig telemetryTask;
    TaskConfig sensorTask;
    TaskConfig ekfTask;
    TaskConfig recoveryTask;
    TaskConfig gpsTask;
    TaskConfig loggingTask;
    TaskConfig apogeeDetectionTask;

    StateActions() : onEntry(nullptr), onExit(nullptr) {}

    StateActions(const std::function<void()> &entry,
                 const std::function<void()> &exit = nullptr)
        : onEntry(entry), onExit(exit) {}
};