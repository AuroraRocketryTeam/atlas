#pragma once

#include <functional>
#include <memory>
#include <IBoardHardware.hpp>
#include <RocketModel.hpp>
#include <SD-master.hpp>
#include <Flash.hpp>
#include <LEDController.hpp>
#include <BuzzerController.hpp>
#include <StatusManager.hpp>
#include <RocketFSM.hpp>
#include <IGroundTestRunner.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

struct TestOption {
    const char *name;
    std::function<bool()> func;
    bool run_all_flag;
};

class TestRoutine : public IGroundTestRunner {
public:
    TestRoutine(IBoardHardware& board,
                std::shared_ptr<RocketModel> model,
                std::shared_ptr<SD> sdCard,
                std::shared_ptr<Flash> flash,
                StatusManager& statusManager,
                LEDController& ledController,
                BuzzerController& buzzerController);

    void run();
    void testFSMTransitions(RocketFSM& fsm);
    static void printSystemInfo();

    size_t getGroundTestCount() const override;
    const GroundTestDescriptor* getGroundTest(size_t index) const override;
    bool startGroundTest(int id, char* error, size_t error_size) override;
    bool submitGroundTestVerdict(GroundTestVerdict verdict, char* error, size_t error_size) override;
    void getGroundTestStatus(GroundTestStatus* out) const override;

private:
    IBoardHardware&             _board;
    std::shared_ptr<RocketModel> _model;
    std::shared_ptr<SD>          _sdCard;
    std::shared_ptr<Flash>       _flash;
    StatusManager&               _statusManager;
    LEDController&               _ledController;
    BuzzerController&            _buzzerController;
    SemaphoreHandle_t            _webMutex;
    TaskHandle_t                 _webTask;
    GroundTestStatus             _webStatus;
    GroundTestVerdict            _pendingVerdict;
    GroundTestVerdict            _lastInputVerdict;
    bool                         _hasPendingVerdict;

    bool waitForUserInput(const char* message);
    void showTestPattern(int testNumber);
    bool runSingleTestById(int id);
    bool isWebTestContext() const;
    void setWebStatus(const char* state, const char* message);
    void setWebPrompt(const char* prompt);
    const GroundTestDescriptor* findGroundTest(int id) const;
    static void webTestTaskEntry(void* arg);

    bool testPowerAndLEDs();
    bool testSensors();
    bool testActuators();
    bool testSDCard();
    bool testFlashMemory();
    bool testTelemetry();
    bool testTelemetryCommand();
    bool testI2CScan();
    bool configureE220();
    bool testE220Connector();
    bool clearFlashMemory();
    bool dumpFlashJsonFiles();
    bool calibrateAndSaveIMU();
};
