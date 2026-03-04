#pragma once

#include <functional>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

/**
 * @brief Enum representing all possible rocket states during mission lifecycle
 *
 * The rocket progresses through these states in a defined sequence from
 * initial setup through recovery. Each state represents a distinct phase
 * of operation with specific tasks and behaviors.
 */
enum class RocketState
{
    // Initial state
    INACTIVE, // System startup, no active operations

    // Pre-flight phase
    CALIBRATING,      // Sensor calibration and system checks
    READY_FOR_LAUNCH, // Armed and waiting for launch detection

    // Flight phase
    LAUNCH,             // Launch sequence initiated
    ACCELERATED_FLIGHT, // Active propulsion phase
    BALLISTIC_FLIGHT,   // Coasting phase after motor burnout
    APOGEE,             // Highest point reached

    // Recovery phase
    STABILIZATION, // Drogue chute deployment and stabilization
    DECELERATION,  // Main chute deployment phase
    LANDING,       // Final descent and landing
    RECOVERED      // Mission complete, rocket recovered
};

/**
 * @brief High-level flight phases grouping related states
 *
 * Used for simplified logic and resource management decisions.
 * Multiple states can belong to the same phase.
 */
enum class FlightPhase
{
    PRE_FLIGHT, // INACTIVE, CALIBRATING, READY_FOR_LAUNCH
    FLIGHT,     // LAUNCH, ACCELERATED_FLIGHT, BALLISTIC_FLIGHT, APOGEE
    RECOVERY    // STABILIZATION, DECELERATION, LANDING, RECOVERED
};

/**
 * @brief Events that can trigger state transitions
 *
 * Events are the primary mechanism for triggering state changes.
 * They can be generated automatically by conditions or sent explicitly
 * by system components.
 */
enum class FSMEvent
{
    NONE = 0, // No event (default/invalid)

    // Pre-flight events
    START_CALIBRATION,    // Begin sensor calibration process
    CALIBRATION_COMPLETE, // Calibration finished successfully

    // Launch events
    LAUNCH_DETECTED, // Launch sequence initiated
    LIFTOFF_STARTED, // Physical liftoff detected

    // Flight events
    ACCELERATION_COMPLETE, // Motor burnout detected
    APOGEE_REACHED,        // Maximum altitude reached

    // Recovery events
    DROGUE_READY,           // Drogue chute ready for deployment
    STABILIZATION_COMPLETE, // Vehicle stabilized after drogue deployment
    DECELERATION_COMPLETE,  // Main chute phase complete
    LANDING_COMPLETE,       // Touchdown detected

    // System events
    FORCE_TRANSITION, // Manual override transition
    EMERGENCY_ABORT   // Emergency abort sequence
};

/**
 * @brief Container for event data passed through the FSM event system
 *
 * Encapsulates an event with optional target state (for force transitions)
 * and arbitrary event data payload.
 */
struct FSMEventData
{
    FSMEvent event;          // The event type
    RocketState targetState; // Target state for FORCE_TRANSITION events
    void *eventData;         // Optional event payload data

    /**
     * @brief Construct FSM event data
     *
     * @param e Event type
     * @param target Target state for force transitions (default: INACTIVE)
     * @param data Optional event payload (default: nullptr)
     */
    FSMEventData(FSMEvent e, RocketState target = RocketState::INACTIVE, void *data = nullptr)
        : event(e), targetState(target), eventData(data) {}
};

/**
 * @brief Represents a simple state transition rule (legacy)
 *
 * @deprecated Use TransitionManager and Transition struct instead
 * @see TransitionManager
 * @see Transition
 */
struct StateTransition
{
    RocketState fromState; // Source state
    RocketState toState;   // Destination state
    FSMEvent triggerEvent; // Triggering event

    StateTransition(RocketState from, RocketState to, FSMEvent event)
        : fromState(from), toState(to), triggerEvent(event) {}
};

/**
 * @brief Convert RocketState enum to string representation
 * 
 * Uses switch statement to ensure compile-time checking - if a new state
 * is added to the enum, the compiler will warn about missing case.
 * 
 * @param state The rocket state to convert
 * @return const char* String representation of the state
 */
inline const char* rocketStateToString(RocketState state) {
    switch (state) {
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
            return "UNKNOWN STATE (missing rocketStateToString function's case implementation)";
    }
}

// Forward declaration of TaskConfig (defined in ITask.hpp)
struct TaskConfig;