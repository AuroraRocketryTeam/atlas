#pragma once

#include <FlightState.hpp>
#include <Arduino.h>
#include <map>
/**
 * @brief Rocket Finite State Machine class
 */
class RocketFSM
{
public:
    RocketFSM();

    /**
     * @brief Initialize the FSM and set up states and transitions
     */
    void init();

    /**
     * @brief Update the FSM - should be called periodically by scheduler
     */
    void update();

    /**
     * @brief Get current state
     */
    RocketState getCurrentState() const { return currentState; }

    /**
     * @brief Get current flight phase
     */
    FlightPhase getCurrentPhase() const;

    /**
     * @brief Get state name as string for debugging
     */
    String getStateString(RocketState state) const;

    /**
     * @brief Force a state transition (for testing/emergency)
     */
    void forceTransition(RocketState newState);

    /**
     * @brief Check if FSM is in final state
     */
    bool isFinished() const { return currentState == RocketState::RECOVERED; }

private:
    RocketState currentState;
    RocketState previousState;
    unsigned long stateStartTime;
    unsigned long lastUpdateTime;

    std::map<RocketState, StateActions> stateActions;
    std::vector<StateTransition> transitions;

    /**
     * @brief Setup all state actions
     */
    void setupStateActions();

    /**
     * @brief Setup all state transitions
     */
    void setupTransitions();

    /**
     * @brief Execute state transition
     */
    void transitionTo(RocketState newState);

    /**
     * @brief Check all transition conditions
     */
    void checkTransitions();

    // State entry actions
    void onInactiveEntry();
    void onCalibratingEntry();
    void onReadyForLaunchEntry();
    void onLaunchEntry();
    void onAcceleratedFlightEntry();
    void onBallisticFlightEntry();
    void onApogeeEntry();
    void onStabilizationEntry();
    void onDecelerationEntry();
    void onLandingEntry();
    void onRecoveredEntry();

    // State exit actions
    void onCalibratingExit();
    void onLandingExit();
    void onRecoveredExit();

    // State update actions (continuous execution while in state)
    void onFlightUpdate();
    void onRecoveryUpdate();

    // Transition conditions
    bool isCalibrationComplete();
    bool isSystemReady();
    bool isLaunchDetected();
    bool isLiftoffStarted();
    bool isAccelerationPhaseComplete();
    bool isBallisticPhaseComplete();
    bool isApogeeReached();
    bool isDrogueReady();
    bool isStabilizationComplete();
    bool isDecelerationComplete();
    bool isLandingComplete();
};