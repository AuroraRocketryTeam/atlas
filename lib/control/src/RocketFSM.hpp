#pragma once

#include "IStateMachine.hpp"
#include "tasks/TaskManager.hpp"
#include "states/StateAction.hpp"
#include "states/TransitionManager.hpp"
#include <Logger.hpp>
#include "RocketLogger.hpp"
#include "config.h"
#include <SD-master.hpp>
#include <memory>
#include <map>
#include <Nemesis.hpp>

class RocketFSM : public IStateMachine
{
public:
    RocketFSM(std::shared_ptr<Nemesis> model,
              std::shared_ptr<SD> sd,
              std::shared_ptr<RocketLogger> logger
            );
    ~RocketFSM();

    // IStateMachine interface
    void init() override;
    void start() override;
    void stop() override;
    bool sendEvent(FSMEvent event, RocketState targetState = RocketState::INACTIVE, void *eventData = nullptr) override;
    RocketState getCurrentState() override;
    FlightPhase getCurrentPhase() override;
    void forceTransition(RocketState newState) override;
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
    std::shared_ptr<Nemesis> _model;
    std::shared_ptr<RocketLogger> _logger;
    SemaphoreHandle_t _modelMutex;
    SemaphoreHandle_t _loggerMutex;

    std::shared_ptr<SD> _sd;

    // Important timers and tresholds
    const unsigned long LAUNCH_TO_BALLISTIC_THRESHOLD = 6000;
    const unsigned long LAUNCH_TO_APOGEE_THRESHOLD = 27000; //24850 + 2150 = 27000
    unsigned long _launchDetectionTime = 0;
};