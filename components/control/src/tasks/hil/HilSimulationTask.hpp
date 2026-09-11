#pragma once

#include "BaseTask.hpp"
#include "IStateMachine.hpp"
#include "SerialLogger.hpp"
#include "RocketLogger.hpp"
#include <ISensor.hpp>
#include <IBoardHardware.hpp>
#include <RocketModel.hpp>
#include <IMUData.hpp>
#include <AccelerometerSensorData.hpp>
#include <PressureSensorData.hpp>
#include <GPSData.hpp>
#include <utils.h>

/**
 * @brief Hardware-In-The-Loop Simulation Task
 *
 * This task:
 *  - opens a TCP server
 *  - receives simulation data (RocketPy)
 *  - injects sensor data into RocketModel
 *  - reads command produced by RocketFSM
 *  - sends command back to simulator
 */
class HilSimulationTask : public BaseTask {
public:

    /**
     * @brief Construct a new Simulation Task object
     * 
     * @param rocketModel The shared pointer to the rocket model
     * @param logger The shared pointer to the RocketLogger instance
     * @param fsm The FSM object of the rocket
     */
    HilSimulationTask(
        std::shared_ptr<RocketModel> rocketModel,
        std::shared_ptr<RocketLogger> logger,
        IBoardHardware* board,
        IStateMachine* fsm
    );

    ~HilSimulationTask();

    void onTaskStart() override;
    void onTaskStop() override;
    void taskFunction() override;
    
    /**
     * @brief Reset the simulation task to its initial state
     * 
     */
    void reset();

private:
    std::shared_ptr<RocketModel> _rocketModel;

    std::shared_ptr<RocketLogger> _logger;

    IBoardHardware* _board = nullptr;

    IStateMachine* _fsm = nullptr;

    int _listen_sock = -1;
    int _client_sock = -1;
    bool _softApAcquired = false;
};
