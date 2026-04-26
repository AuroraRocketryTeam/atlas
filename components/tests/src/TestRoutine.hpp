#pragma once

#include <memory>
#include <IBoardHardware.hpp>
#include <RocketModel.hpp>
#include <SD-master.hpp>
#include <Flash.hpp>
#include <LEDController.hpp>
#include <BuzzerController.hpp>
#include <StatusManager.hpp>
#include <RocketFSM.hpp>

typedef struct TestOption {
    const char *name;
    std::function<bool()> func;
    bool run_all_flag;
} TestOption;

class TestRoutine {
public:
    TestRoutine(IBoardHardware& board,
                std::shared_ptr<RocketModel> model,
                std::shared_ptr<SD> sdCard,
                StatusManager& statusManager,
                LEDController& ledController,
                BuzzerController& buzzerController);

    void run();
    void testFSMTransitions(RocketFSM& fsm);
    static void printSystemInfo();

private:
    IBoardHardware&             _board;
    std::shared_ptr<RocketModel> _model;
    std::shared_ptr<SD>          _sdCard;
    std::shared_ptr<Flash>       _flash;
    StatusManager&               _statusManager;
    LEDController&               _ledController;
    BuzzerController&            _buzzerController;

    bool waitForUserInput(const char* message);
    void showTestPattern(int testNumber);

    bool testPowerAndLEDs();
    bool testSensors();
    bool testActuators();
    bool testSDCard();
    bool testFlashMemory();
    bool testTelemetry();
    bool testI2CScan();
    bool configureE220();
    bool testE220Connector();
};
