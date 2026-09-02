#pragma once

#include "IStateMachine.hpp"
#include "tasks/TaskManager.hpp"
#include "states/StateAction.hpp"
#include "states/TransitionManager.hpp"
#include <SerialLogger.hpp>
#include "RocketLogger.hpp"
#include "config.h"
#include <SD-master.hpp>
#include <memory>
#include <map>
#include <RocketModel.hpp>
#include <IBoardHardware.hpp>
#include "tasks/FlightParametersConfig.hpp"

class IGroundTestRunner;

/**
 * @brief Enumeration for the different rocket states.
 * 
 */
class RocketFSM : public IStateMachine
{
public:
    /**
     * @brief Construct a new Rocket FSM object
     * 
     * @param model The shared pointer to the rocket model
     * @param sd The shared pointer to the SD card
     * @param logger The shared pointer to the RocketLogger instance
     */
    RocketFSM(std::shared_ptr<RocketModel> rocketModel,
              std::shared_ptr<SD> sd,
              std::shared_ptr<RocketLogger> logger,
              IBoardHardware* board,
              std::shared_ptr<IGroundTestRunner> testRunner = nullptr
            );
    
    /**
     * @brief Destroy the Rocket FSM object
     * 
     */
    ~RocketFSM();

    // IStateMachine interface
    void init() override;
    void start() override;
    void stop() override;

    /**
     * @brief Send an event to the FSM
     * 
     * @param event The event to send
     * @param targetState The target state to transition to
     * @param eventData Optional data associated with the event
     * @return true if the event was processed successfully, false otherwise
     */
    bool sendEvent(FSMEvent event, RocketState targetState = RocketState::INACTIVE, void *eventData = nullptr) override;

    /**
     * @brief Get the current state of the FSM
     * 
     * @return RocketState The current state
     */
    RocketState getCurrentState() override;

    /**
     * @brief Get the current flight phase of the FSM
     * 
     * @return FlightPhase The current flight phase
     */
    FlightPhase getCurrentPhase() override;

    /**
     * @brief Force a transition to a new state
     * 
     * @param newState The new state to transition to
     */
    void forceTransition(RocketState newState) override;

    /**
     * @brief Check if the FSM has finished its current task
     * 
     * @return true if the FSM is finished, false otherwise
     */
    bool isFinished() override;

    // Utility methods
    const char* getStateString(RocketState state) const;

private:
    void setupStateActions();
    void setupTransitions();
    void transitionTo(RocketState newState);
    void processEvent(const FSMEventData &eventData);
    void checkTransitions();

    // FreeRTOS task function
    static void fsmTaskWrapper(void *parameter);
    void fsmTask();

    // Core FSM components
    std::unique_ptr<TaskManager> _taskManager;
    std::unique_ptr<TransitionManager> _transitionManager;
    std::map<RocketState, std::unique_ptr<StateAction>> _stateActions;

    // FreeRTOS components
    TaskHandle_t _fsmTaskHandle;
    QueueHandle_t _eventQueue;
    SemaphoreHandle_t _stateMutex;

    // State management
    RocketState _currentState;
    RocketState _previousState;
    unsigned long _stateStartTime;
    volatile bool _isRunning;
    volatile bool _isTransitioning;

    // Shared data
    std::shared_ptr<RocketModel> _rocketModel;
    std::shared_ptr<RocketLogger> _logger;
    std::shared_ptr<IGroundTestRunner> _testRunner;

    std::shared_ptr<SD> _sd;
    IBoardHardware* _board;

    // Parachutes states
    bool _mainDeploymentCommanded = false;
    bool _drogueDeploymentCommanded = false;

    void deployMain();
    void deployDrogue();
    void deployApogeeRecovery();
    void deployStabilizationExitRecovery();

    // Important timers and tresholds
    unsigned long _launchDetectionTime = 0;
};
