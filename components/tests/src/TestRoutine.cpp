#include "TestRoutine.hpp"

#include <cstring>
#include <string>
#include <algorithm>
#include <cctype>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <esp_system.h>
#include <cstdio>

#include <config.h>
#include <board.h>
#include <utils.h>
#include <SerialLogger.hpp>
#include <E220LoRaTransmitter.hpp>
#include <I2CBus.hpp>

TestRoutine::TestRoutine(IBoardHardware& board,
                         std::shared_ptr<RocketModel> model,
                         std::shared_ptr<SD> sdCard,
                         std::shared_ptr<Flash> flash,
                         StatusManager& statusManager,
                         LEDController& ledController,
                         BuzzerController& buzzerController)
    : _board(board),
      _model(model),
      _sdCard(sdCard),
      _flash(flash),
      _statusManager(statusManager),
      _ledController(ledController),
      _buzzerController(buzzerController),
      _webMutex(xSemaphoreCreateMutex()),
      _webTask(nullptr),
      _pendingVerdict(GroundTestVerdict::FAILED),
      _lastInputVerdict(GroundTestVerdict::FAILED),
      _hasPendingVerdict(false)
{
    snprintf(_webStatus.state, sizeof(_webStatus.state), "%s", "idle");
    snprintf(_webStatus.message, sizeof(_webStatus.message), "%s", "idle");
}

static const GroundTestDescriptor GROUND_TESTS[] = {
    {1,  "Power",     "Power and LEDs",          "Blink status LEDs and ask the operator to confirm visual behavior.", false, ""},
    {2,  "Sensors",   "Sensor Readout",          "Read IMU, barometers, and accelerometer through the real RocketModel.", false, ""},
    {5,  "Sensors",   "I2C Scan",                "Probe the I2C bus and report known/unknown devices on serial logs.", false, ""},
    {14, "Sensors",   "IMU Calibration to NVS",  "Run BNO055 calibration and save offsets to internal NVS.", false, ""},
    {3,  "Actuators", "Actuators and Buzzer",    "Energize drogue/main outputs and buzzer for operator verification.", true, "ARM_ACTUATORS"},
    {7,  "Telemetry", "LoRa Telemetry Send",     "Initialize E220 LoRa and transmit three test packets.", false, ""},
    {8,  "Telemetry", "E220 Connector GPIO",     "Pulse E220 connector pins in a visible sequence.", false, ""},
    {10, "Telemetry", "LoRa Command Receive",    "Wait up to 30 seconds for a command from the ground station.", false, ""},
    {6,  "Telemetry", "Configure E220",          "Write the default E220 radio configuration.", true, "CONFIGURE_E220"},
    {4,  "Storage",   "SD Card",                 "Write and read a test file on the SD card when available.", false, ""},
    {9,  "Storage",   "External Flash",          "Initialize external flash, write/read/append, then clear test data.", true, "TEST_FLASH"},
    {12, "Storage",   "Dump Flash Telemetry",    "Print JSONL telemetry from external flash to serial output.", false, ""},
    {13, "Storage",   "Format Flash",            "Format the external flash memory. Data will be lost.", true, "FORMAT_FLASH"},
};

size_t TestRoutine::getGroundTestCount() const
{
    return sizeof(GROUND_TESTS) / sizeof(GROUND_TESTS[0]);
}

const GroundTestDescriptor* TestRoutine::getGroundTest(size_t index) const
{
    if (index >= getGroundTestCount()) return nullptr;
    return &GROUND_TESTS[index];
}

const GroundTestDescriptor* TestRoutine::findGroundTest(int id) const
{
    for (size_t i = 0; i < getGroundTestCount(); ++i) {
        if (GROUND_TESTS[i].id == id) return &GROUND_TESTS[i];
    }
    return nullptr;
}

void TestRoutine::getGroundTestStatus(GroundTestStatus* out) const
{
    if (out == nullptr) return;
    if (_webMutex != nullptr) xSemaphoreTake(_webMutex, portMAX_DELAY);
    *out = _webStatus;
    if (_webMutex != nullptr) xSemaphoreGive(_webMutex);
}

void TestRoutine::setWebStatus(const char* state, const char* message)
{
    if (_webMutex != nullptr) xSemaphoreTake(_webMutex, portMAX_DELAY);
    snprintf(_webStatus.state, sizeof(_webStatus.state), "%s", state ? state : "");
    snprintf(_webStatus.message, sizeof(_webStatus.message), "%s", message ? message : "");
    _webStatus.waiting_for_verdict = false;
    _webStatus.prompt[0] = '\0';
    if (_webMutex != nullptr) xSemaphoreGive(_webMutex);
}

void TestRoutine::setWebPrompt(const char* prompt)
{
    if (_webMutex != nullptr) xSemaphoreTake(_webMutex, portMAX_DELAY);
    _webStatus.waiting_for_verdict = true;
    snprintf(_webStatus.state, sizeof(_webStatus.state), "%s", "waiting");
    snprintf(_webStatus.prompt, sizeof(_webStatus.prompt), "%s", prompt ? prompt : "");
    snprintf(_webStatus.message, sizeof(_webStatus.message), "%s", "waiting for operator verdict");
    if (_webMutex != nullptr) xSemaphoreGive(_webMutex);
}

bool TestRoutine::isWebTestContext() const
{
    return _webTask != nullptr && xTaskGetCurrentTaskHandle() == _webTask;
}

bool TestRoutine::startGroundTest(int id, char* error, size_t error_size)
{
    const GroundTestDescriptor* descriptor = findGroundTest(id);
    if (descriptor == nullptr) {
        if (error && error_size > 0) snprintf(error, error_size, "%s", "unknown test id");
        return false;
    }

    if (_webMutex != nullptr) xSemaphoreTake(_webMutex, portMAX_DELAY);
    if (_webStatus.running) {
        if (_webMutex != nullptr) xSemaphoreGive(_webMutex);
        if (error && error_size > 0) snprintf(error, error_size, "%s", "a test is already running");
        return false;
    }

    _webStatus = GroundTestStatus{};
    _webStatus.running = true;
    _webStatus.active_id = id;
    _webStatus.started_ms = Utils::millis();
    snprintf(_webStatus.state, sizeof(_webStatus.state), "%s", "starting");
    snprintf(_webStatus.active_name, sizeof(_webStatus.active_name), "%s", descriptor->name);
    snprintf(_webStatus.message, sizeof(_webStatus.message), "%s", "starting test");
    _hasPendingVerdict = false;
    if (_webMutex != nullptr) xSemaphoreGive(_webMutex);

    if (xTaskCreate(webTestTaskEntry, "ground_test", 8192, this, 5, &_webTask) != pdPASS) {
        if (_webMutex != nullptr) xSemaphoreTake(_webMutex, portMAX_DELAY);
        _webStatus.running = false;
        snprintf(_webStatus.state, sizeof(_webStatus.state), "%s", "failed");
        snprintf(_webStatus.message, sizeof(_webStatus.message), "%s", "failed to create test task");
        if (_webMutex != nullptr) xSemaphoreGive(_webMutex);
        _webTask = nullptr;
        if (error && error_size > 0) snprintf(error, error_size, "%s", "failed to create test task");
        return false;
    }

    return true;
}

bool TestRoutine::submitGroundTestVerdict(GroundTestVerdict verdict, char* error, size_t error_size)
{
    if (_webMutex != nullptr) xSemaphoreTake(_webMutex, portMAX_DELAY);
    if (!_webStatus.running || !_webStatus.waiting_for_verdict) {
        if (_webMutex != nullptr) xSemaphoreGive(_webMutex);
        if (error && error_size > 0) snprintf(error, error_size, "%s", "no test is waiting for a verdict");
        return false;
    }
    _pendingVerdict = verdict;
    _hasPendingVerdict = true;
    snprintf(_webStatus.message, sizeof(_webStatus.message), "%s", "operator verdict received");
    if (_webMutex != nullptr) xSemaphoreGive(_webMutex);
    return true;
}

void TestRoutine::webTestTaskEntry(void* arg)
{
    auto* self = static_cast<TestRoutine*>(arg);
    int id = 0;
    if (self->_webMutex != nullptr) xSemaphoreTake(self->_webMutex, portMAX_DELAY);
    id = self->_webStatus.active_id;
    if (self->_webMutex != nullptr) xSemaphoreGive(self->_webMutex);

    bool passed = false;
    do {
        self->setWebStatus("running", "test running");
        self->_lastInputVerdict = GroundTestVerdict::FAILED;
        passed = self->runSingleTestById(id);

        if (!passed && self->_lastInputVerdict == GroundTestVerdict::RETRY) {
            self->setWebStatus("retrying", "retrying test");
            vTaskDelay(pdMS_TO_TICKS(250));
        }
    } while (!passed && self->_lastInputVerdict == GroundTestVerdict::RETRY);

    if (self->_webMutex != nullptr) xSemaphoreTake(self->_webMutex, portMAX_DELAY);
    self->_webStatus.running = false;
    self->_webStatus.waiting_for_verdict = false;
    self->_webStatus.last_passed = passed;
    self->_webStatus.finished_ms = Utils::millis();
    if (passed) {
        snprintf(self->_webStatus.state, sizeof(self->_webStatus.state), "%s", "passed");
        snprintf(self->_webStatus.message, sizeof(self->_webStatus.message), "%s", "test passed");
    } else if (self->_lastInputVerdict == GroundTestVerdict::EXIT) {
        snprintf(self->_webStatus.state, sizeof(self->_webStatus.state), "%s", "exited");
        snprintf(self->_webStatus.message, sizeof(self->_webStatus.message), "%s", "test exited");
    } else {
        snprintf(self->_webStatus.state, sizeof(self->_webStatus.state), "%s", "failed");
        snprintf(self->_webStatus.message, sizeof(self->_webStatus.message), "%s", "test failed");
    }
    self->_webStatus.prompt[0] = '\0';
    self->_webTask = nullptr;
    if (self->_webMutex != nullptr) xSemaphoreGive(self->_webMutex);

    vTaskDelete(nullptr);
}

bool TestRoutine::runSingleTestById(int id)
{
    showTestPattern(id);

    switch (id)
    {
    case 1:  return testPowerAndLEDs();
    case 2:  return testSensors();
    case 3:  return testActuators();
    case 4:  return testSDCard();
    case 5:  return testI2CScan();
    case 6:  return configureE220();
    case 7:  return testTelemetry();
    case 8:  return testE220Connector();
    case 9:  return testFlashMemory();
    case 10: return testTelemetryCommand();
    case 12: return dumpFlashJsonFiles();
    case 13: return clearFlashMemory();
    case 14: return calibrateAndSaveIMU();
    default: return false;
    }
}

// ── String helpers ────────────────────────────────────────────────────────────

static void trimString(std::string& s)
{
    auto notSpace = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), notSpace));
    s.erase(std::find_if(s.rbegin(), s.rend(), notSpace).base(), s.end());
}

static void toUpperString(std::string& s)
{
    for (char& c : s)
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
}

bool TestRoutine::waitForUserInput(const char* message)
{
    if (isWebTestContext()) {
        setWebPrompt(message);

        while (true) {
            GroundTestVerdict verdict = GroundTestVerdict::FAILED;
            bool hasVerdict = false;

            if (_webMutex != nullptr) xSemaphoreTake(_webMutex, portMAX_DELAY);
            if (_hasPendingVerdict) {
                verdict = _pendingVerdict;
                _hasPendingVerdict = false;
                hasVerdict = true;
                _webStatus.waiting_for_verdict = false;
            }
            if (_webMutex != nullptr) xSemaphoreGive(_webMutex);

            if (hasVerdict) {
                _lastInputVerdict = verdict;
                if (verdict == GroundTestVerdict::PASSED) {
                    _statusManager.playBlockingPattern(TEST_SUCCESS, 1000);
                    return true;
                }
                if (verdict == GroundTestVerdict::REBOOT) {
                    LOG_WARNING("Test", "Web operator requested reboot");
                    esp_restart();
                }
                if (verdict == GroundTestVerdict::RETRY) {
                    _statusManager.playBlockingPattern(TEST_FAILURE, 1000);
                    return false;
                }
                if (verdict == GroundTestVerdict::EXIT) {
                    LOG_INFO("Test", "Web operator exited test");
                    return false;
                }
                _statusManager.playBlockingPattern(TEST_FAILURE, 1000);
                return false;
            }

            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    printf("%s\n(Shortcuts: 'P' = Passed, 'F' = Failed, 'R' = Reboot)\n", message);
    _statusManager.setSystemCode(WAITING_INPUT);

    while (true)
    {
        char buffer[64] = {0};
        Utils::readLine(buffer, sizeof(buffer));
        std::string input(buffer);
        trimString(input);
        toUpperString(input);

        if (input == "PASSED" || input == "P")
        {
            _statusManager.playBlockingPattern(TEST_SUCCESS, 1000);
            return true;
        }
        if (input == "FAILED" || input == "F")
        {
            _statusManager.playBlockingPattern(TEST_FAILURE, 1000);
            return false;
        }
        if (input == "REBOOT" || input == "R")
        {
            LOG_WARNING("Test", "System is going to reboot, are you sure?");
            LOG_WARNING("Test", "Type REBOOT (or 'R') to confirm or anything else to cancel.");

            char confirmBuffer[64] = {0};
            Utils::readLine(confirmBuffer, sizeof(confirmBuffer));
            std::string confirm(confirmBuffer);

            if (confirm.empty()) {
                LOG_INFO("Test", "No confirmation - continuing normal operation.");
                continue;
            }
            trimString(confirm);
            toUpperString(confirm);

            if (confirm == "REBOOT" || confirm == "R") {
                LOG_WARNING("Test", "Rebooting system...");
                esp_restart();
            } else {
                LOG_INFO("Test", "Reboot cancelled.");
            }
        }
        else if (!input.empty())
        {
            LOG_WARNING("Test", "Unrecognized input. Please type P, F, or R.");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void TestRoutine::showTestPattern(int testNumber)
{
    switch (testNumber)
    {
    case 1:  _statusManager.playBlockingPattern(TEST_POWER,     1000); break;
    case 2:  _statusManager.playBlockingPattern(TEST_SENSORS,   1000); break;
    case 3:  _statusManager.playBlockingPattern(TEST_ACTUATORS, 1000); break;
    case 4:  _statusManager.playBlockingPattern(TEST_SD,        1000); break;
    case 5:  _statusManager.playBlockingPattern(TEST_SENSORS,   1000); break;
    case 6:  _statusManager.playBlockingPattern(TEST_TELEMETRY, 1000); break;
    case 7:  _statusManager.playBlockingPattern(TEST_TELEMETRY, 1000); break;
    case 8:  _statusManager.playBlockingPattern(TEST_TELEMETRY, 1000); break;
    case 9:  _statusManager.playBlockingPattern(TEST_SD,        1000); break;
    case 10: _statusManager.playBlockingPattern(TEST_TELEMETRY, 1000); break;
    case 11: _statusManager.playBlockingPattern(TEST_ALL,       2000); break;
    default: break;
    }
}

bool TestRoutine::testPowerAndLEDs()
{
    LOG_INFO("Test", "\n[STEP 1] Verifica alimentazione e LED di stato");
    LOG_INFO("Test", "3 lampeggi rossi...");

    for (int i = 0; i < 3; i++) {
        _ledController.setColor(ART_LED_RED);
        vTaskDelay(pdMS_TO_TICKS(500));
        _ledController.setOff();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
}

bool TestRoutine::testSensors()
{
    LOG_INFO("Test", "\n[STEP 2] Test sensori");

    _board.init_sensor_test_pins();
    
    IMUData imuData;
    SensorReadStatus imuStatus = _model->getBNO055Data(imuData);
    if (imuStatus != SensorReadStatus::OK) {
        _statusManager.playBlockingPattern(IMU_FAIL, 2000);
        LOG_ERROR("Test", "Errore: IMU non inizializzata.");
    } else {
        LOG_INFO("Test", "IMU Accelerometer: x=%.2f, y=%.2f, z=%.2f m/s^2",
                 (double)imuData.acceleration_x,
                 (double)imuData.acceleration_y,
                 (double)imuData.acceleration_z);
    }


    _model->updateMS561101BA03_1();
    _model->updateMS561101BA03_2();
    delay(20);
    _model->updateMS561101BA03_1();
    _model->updateMS561101BA03_2();
    delay(20);
    
    _model->updateMS561101BA03_1();
    _model->updateMS561101BA03_2();

    PressureSensorData baro1Data;
    SensorReadStatus baro1Status = _model->getMS561101BA03Data_1(baro1Data);
    if (baro1Status != SensorReadStatus::OK) {
        _statusManager.playBlockingPattern(BARO1_FAIL, 2000);
        LOG_ERROR("Test", "Errore: Barometro 1 non inizializzato.");
    } else {
        LOG_INFO("Test", "Barometer 1 Pressure: %.2f Pa",
                 (double)baro1Data.pressure);
    }


    PressureSensorData baro2Data;
    SensorReadStatus baro2Status = _model->getMS561101BA03Data_2(baro2Data);
    if (baro2Status != SensorReadStatus::OK) {
        _statusManager.playBlockingPattern(BARO2_FAIL, 2000);
        LOG_ERROR("Test", "Errore: Barometro 2 non inizializzato.");
    } else {
        LOG_INFO("Test", "Barometer 2 Pressure: %.2f Pa",
                 (double)baro2Data.pressure);
    }

    AccelerometerSensorData acclData;
    SensorReadStatus acclStatus  = _model->getLIS3DHTRData(acclData);
    if (acclStatus != SensorReadStatus::OK) {
        _statusManager.playBlockingPattern(IMU_FAIL, 2000);
        LOG_ERROR("Test", "Errore: Accelerometro non inizializzato.");
    } else {
        _board.signal_sensor_ok(IBoardHardware::Sensor::ACC);
    }

    return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
}

bool TestRoutine::testActuators()
{
    LOG_INFO("Test", "[STEP 3] Test attuatori");
    LOG_INFO("Test", "Attivazione attuatori per 5 secondi...");
    _statusManager.playBlockingPattern(TEST_ACTUATORS, 500);
    gpio_set_level(_board.get_drogue_actuator_pin(), HIGH);
    gpio_set_level(_board.get_main_actuator_pin(),   HIGH);
    _buzzerController.playTone(TONE_MID, 1000);
    vTaskDelay(pdMS_TO_TICKS(5000));

    LOG_INFO("Test", "Spegnimento attuatori...");
    _statusManager.playBlockingPattern(TEST_ACTUATORS, 500);
    gpio_set_level(_board.get_drogue_actuator_pin(), LOW);
    gpio_set_level(_board.get_main_actuator_pin(),   LOW);
    _buzzerController.playTone(TONE_MID, 1000);

    return waitForUserInput("Verificare accensione LED e tensione in uscita da DROGUE e MAIN, verificare funzionamento Buzzer. Scrivi PASSED o FAILED");
}

bool TestRoutine::testSDCard()
{
    static const char* TEST_FILE = "/test.txt";
    LOG_INFO("Test", "[STEP 4] Test SD Card");

    if (!_sdCard) {
        LOG_WARNING("Test", "SD card not available on this board");
        return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
    }

    if (_sdCard->openFile(TEST_FILE)) {
        std::string content = "SD card write test successful! Timestamp: " + std::to_string(Utils::millis()) + " ms\n";
        const uint8_t* data = reinterpret_cast<const uint8_t*>(content.c_str());
        size_t length = content.length();
        if (_sdCard->writeFile(TEST_FILE, data, length)) {
            LOG_INFO("Test", "SD card write test successful");
            std::string readContent = _sdCard->readFile(TEST_FILE);
            if (!readContent.empty())
            {
                LOG_INFO("Test", "Read from SD card: %s", readContent.c_str());
            }
            else
            {
                _statusManager.playBlockingPattern(SD_READ_FAIL, 2000);
                LOG_ERROR("Test", "Failed to read back from SD card");
            }
        } else {
            _statusManager.playBlockingPattern(SD_WRITE_FAIL, 2000);
            LOG_ERROR("Test", "SD card write test failed");
        }
        _sdCard->closeFile();
    } else {
        _statusManager.playBlockingPattern(SD_MOUNT_FAIL, 2000);
        LOG_ERROR("Test", "Failed to open test file on SD card");
    }

    return waitForUserInput("Type PASSED to continue or FAILED to repeat");
}

bool TestRoutine::testFlashMemory()
{

    // Print _board.get_spi_bus(), _board.get_flash_cs_pin(), _board.get_flash_hold_pin(), _board.get_flash_wp_pin()
    LOG_INFO("Test", "SPI Bus: %d", _board.get_spi_bus());
    LOG_INFO("Test", "Flash CS Pin: %d", _board.get_flash_cs_pin());
    LOG_INFO("Test", "Flash Hold Pin: %d", _board.get_flash_hold_pin());
    LOG_INFO("Test", "Flash WP Pin: %d", _board.get_flash_wp_pin());


    LOG_INFO("Test", "[STEP 5] Flash memory test");

    if (!_flash)
    {
        _flash = std::make_shared<Flash>();
    }

    const uint32_t t0 = Utils::millis();

    if (!_flash) {
        LOG_ERROR("Test", "Flash pointer is null! Initialization failed in setup.");
        return waitForUserInput("Type PASSED to continue or FAILED to retry");
    }

    if (!_flash->isInitialized())
    {
        if (!_flash->init(_board.get_spi_bus(), _board.get_flash_cs_pin(), _board.get_flash_hold_pin(), _board.get_flash_wp_pin()))
        {
            LOG_ERROR("Init", "Failed to initialize External Flash");
            return waitForUserInput("Type PASSED to continue or FAILED to retry");
        }
        LOG_INFO("Init", "External Flash initialized");
    }
    else
    {
        LOG_INFO("Init", "External Flash already initialized, reusing instance");
    }

    LOG_INFO("Test", "Flash: verifying readiness...");
    if (!_flash->isInitialized())
    {
        LOG_ERROR("Test", "Flash init failed: verify external SPI flash wiring and availability.");
        return waitForUserInput("Type PASSED to continue or FAILED to retry");
    }
    LOG_INFO("Test", "Flash: init OK (%lu ms)", (unsigned long)(Utils::millis() - t0));

    const std::string testFile = "test.txt";
    const std::string missingFile = "ghost.txt";

    LOG_INFO("Test", "Flash: read missing file '%s' (expected empty)", missingFile.c_str());
    std::string readData = _flash->readFile(missingFile.c_str());
    if (!readData.empty())
    {
        LOG_ERROR("Test", "Unexpected data returned for missing file.");
    }
    else
    {
        LOG_INFO("Test", "Flash: missing file check OK");
    }

    LOG_INFO("Test", "Flash: write '%s'", testFile.c_str());
    const char* writeMsg = "Hello, ESP32 Flash Storage!\n";
    if (!_flash->writeFile(testFile.c_str(), 
                       reinterpret_cast<const uint8_t*>(writeMsg), 
                       strlen(writeMsg)))
    {
        LOG_ERROR("Test", "Flash write failed.");
    }
    else
    {
        LOG_INFO("Test", "Flash: write OK");
    }

    LOG_INFO("Test", "Flash: read '%s'", testFile.c_str());
    readData = _flash->readFile(testFile.c_str());
    if (readData.empty())
    {
        LOG_ERROR("Test", "Flash read failed after write.");
    }
    else
    {
        LOG_INFO("Test", "Flash read content: %s", readData.c_str());
    }

    LOG_INFO("Test", "Flash: append to '%s'", testFile.c_str());
    const char* appendMsg = "Appended line.\n";
    if (!_flash->appendFile(testFile.c_str(), 
                        reinterpret_cast<const uint8_t*>(appendMsg), 
                        strlen(appendMsg)))
    {
        LOG_ERROR("Test", "Flash append failed.");
    }
    else
    {
        LOG_INFO("Test", "Flash: append OK");
    }

    LOG_INFO("Test", "Flash: read back after append");
    readData = _flash->readFile(testFile.c_str());
    if (!readData.empty())
    {
        LOG_INFO("Test", "Flash read after append: %s", readData.c_str());
    }
    else
    {
        LOG_ERROR("Test", "Flash read failed after append.");
    }

    if (!_flash->fileExists(testFile.c_str()))
    {
        LOG_ERROR("Test", "Flash file existence check failed.");
    }
    else
    {
        LOG_INFO("Test", "Flash: fileExists('%s') OK", testFile.c_str());
    }

    const uint32_t t_clear = Utils::millis();
    LOG_INFO("Test", "Flash: clear start (this can take several seconds on full-chip erase)...");
    
    if (!_flash->clearFlash())
    {
        LOG_ERROR("Test", "Flash clear failed.");
    }
    else if (_flash->fileExists(testFile.c_str()))
    {
        LOG_ERROR("Test", "Flash clear did not remove test file.");
    }
    else
    {
        LOG_INFO("Test", "Flash: clear OK (%lu ms)", (unsigned long)(Utils::millis() - t_clear));
    }

    LOG_INFO("Test", "Flash: test completed in %lu ms", (unsigned long)(Utils::millis() - t0));

    return waitForUserInput("Check the logs above in the serial console. Type PASSED to continue or FAILED to retry");
}

bool TestRoutine::clearFlashMemory()
{
    LOG_INFO("Test", "\n=== TEST FLASH ERASE ===");
    
    if (!_flash)
    {
        _flash = std::make_shared<Flash>();
    }

    if (!_flash->isInitialized() && !_flash->init(_board.get_spi_bus(), _board.get_flash_cs_pin(), _board.get_flash_hold_pin(), _board.get_flash_wp_pin()))
    {
        LOG_ERROR("Test", "Flash init failed");
        return waitForUserInput("Type PASSED to continue or FAILED to retry");
    }

    bool confirmed = isWebTestContext();

    if (!confirmed) {
        printf("WARNING: This operation will format the entire Flash memory.\n");
        printf("All data will be lost!\n");
        printf("Are you sure you want to continue? (Y/n): \n");

        char buffer[16] = {0};
        Utils::readLine(buffer, sizeof(buffer));
        std::string input(buffer);
        trimString(input);
        confirmed = input == "Y" || input == "y";
    }

    if (confirmed) {
        LOG_INFO("Test", "Formatting in progress... it might take some time.");
        if (_flash->clearMemory()) {
            LOG_INFO("Test", "Formatting completed successfully!");
            return waitForUserInput("Memory cleared. Type PASSED to continue or FAILED to retry");
        } else {
            LOG_ERROR("Test", "Error during formatting!");
        }
    } else {
        LOG_INFO("Test", "Operation cancelled.");
    }
    
    return waitForUserInput("Type PASSED to continue or FAILED to retry");
}

bool TestRoutine::dumpFlashJsonFiles()
{
    LOG_INFO("Test", "[DUMP] Export JSONL telemetry from external flash");

    if (!_flash)
    {
        _flash = std::make_shared<Flash>();
    }

    if (!_flash->isInitialized() && !_flash->init(_board.get_spi_bus(), _board.get_flash_cs_pin(), _board.get_flash_hold_pin(), _board.get_flash_wp_pin()))
    {
        LOG_ERROR("Dump", "Flash init failed");
        return waitForUserInput("Type PASSED to continue or FAILED to retry");
    }

    const char* TELEMETRY_FILENAME = "flight_telemetry.jsonl";

    if (!_flash->fileExists(TELEMETRY_FILENAME))
    {
        LOG_WARNING("Dump", "No telemetry file found (%s) on flash.", TELEMETRY_FILENAME);
        return waitForUserInput("Type PASSED to continue or FAILED to retry");
    }

    printf("\n=== JSONL DUMP START ===\n");
    printf("Capture this serial output to a file on PC.\n");

    // Open file stream directly to read line-by-line
    if (!_flash->openFile(TELEMETRY_FILENAME))
    {
        LOG_ERROR("Dump", "Failed to open telemetry file for reading.");
        return waitForUserInput("Type PASSED to continue or FAILED to retry");
    }

    // Trigger Python script to open the file
    printf("START_FILE:%s\n", TELEMETRY_FILENAME);

    int lineCount = 0;
    std::string line;
    
    // Stream line-by-line to prevent RAM exhaustion
    while (true)
    {
        line = _flash->readLine();
        if (line.empty()) 
        {
            break; // End of file reached
        }

        // Print directly to serial. 
        // Note: readLine() already includes the '\n' at the end.
        printf("%s", line.c_str());
        
        lineCount++;

        // Feed the FreeRTOS watchdog to prevent resets during massive file dumps
        if (lineCount % 50 == 0) 
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    // Ensure we are on a new line before sending the termination string
    printf("\nEND_FILE:%s\n", TELEMETRY_FILENAME);

    _flash->closeFile();

    printf("=== JSONL DUMP END (%d lines extracted) ===\n\n", lineCount);
    LOG_INFO("Dump", "Successfully exported telemetry file.");

    return waitForUserInput("JSONL dump printed on serial. Type PASSED to continue or FAILED to retry");
}

bool TestRoutine::testTelemetry()
{
    LOG_INFO("Test", "[STEP 6] Test telemetria");
    _statusManager.playBlockingPattern(TEST_TELEMETRY, 1000);

    E220LoRaTransmitter lora(Serial2, MANNY_LORA_TX_PIN, MANNY_LORA_RX_PIN,
                             MANNY_LORA_AUX_PIN, MANNY_LORA_M0_PIN, MANNY_LORA_M1_PIN);

    auto initResult = lora.init();
    if (initResult.getCode() != E220_SUCCESS) {
        LOG_ERROR("Test", "LoRa init fallita: %s", initResult.getDescription().c_str());
        return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
    }
    LOG_INFO("Test", "LoRa init OK");

    for (int i = 1; i <= 3; i++) {
        std::string payload = "LORA_TEST_" + std::to_string(i);
        auto result = lora.transmit(payload);
        if (result.getCode() == E220_SUCCESS)
            LOG_INFO("Test", "Pacchetto %d inviato.", i);
        else
            LOG_ERROR("Test", "Invio pacchetto %d fallito: %s", i, result.getDescription().c_str());
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
}

bool TestRoutine::testTelemetryCommand()
{
    LOG_INFO("Test", "[STEP 11] Test ricezione comando LoRa (timeout 30s)");
    _statusManager.playBlockingPattern(TEST_TELEMETRY, 1000);

    E220LoRaTransmitter lora(Serial2, MANNY_LORA_TX_PIN, MANNY_LORA_RX_PIN,
                             MANNY_LORA_AUX_PIN, MANNY_LORA_M0_PIN, MANNY_LORA_M1_PIN);

    auto initResult = lora.init();
    if (initResult.getCode() != E220_SUCCESS) {
        LOG_ERROR("Test", "LoRa init fallita: %s", initResult.getDescription().c_str());
        return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
    }
    LOG_INFO("Test", "LoRa init OK. In attesa di un comando dalla ground station...");

    const uint32_t TIMEOUT_MS = 30000;
    uint32_t start = Utils::millis();
    bool received = false;
    while (Utils::millis() - start < TIMEOUT_MS) {
        CommandPacket cmd;
        if (lora.receive(&cmd)) {
            LOG_INFO("Test", "Comando ricevuto: 0x%02X", cmd.command_id);
            received = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (!received)
        LOG_WARNING("Test", "Timeout: nessun comando ricevuto");

    return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
}

bool TestRoutine::testI2CScan()
{
    LOG_INFO("Test", "Scanning I2C bus...");

    I2CBus* bus = _board.get_i2c_bus(IBoardHardware::Sensor::IMU);
    if (!bus) {
        LOG_ERROR("Test", "I2C bus not available.");
        return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
    }

    struct KnownDevice { const char* name; uint8_t addr; };
    static const KnownDevice known[] = {
        { "BNO055 IMU",   0x28 },
        { "BNO055 IMU 2", 0x29 },
        { "LIS3DHTR",     0x18 },
        { "LIS3DHTR 2",   0x19 },
    };

    bool found = false;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_probe(*bus->get_handle(), addr, 10) == ESP_OK) {
            found = true;
            const char* label = nullptr;
            for (auto& d : known)
                if (d.addr == addr) { label = d.name; break; }
            LOG_INFO("TestRoutine", label ? "Found: 0x%02X %s\n" : "Unknown: 0x%02X\n", addr, label ? label : "");
        }
    } 
    if (!found) LOG_INFO("TestRoutine", "No devices found.\n");

    return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
}

bool TestRoutine::configureE220()
{
    LOG_INFO("LoRa", "Configuring E220...");

    gpio_reset_pin(MANNY_LORA_M0_PIN);
    gpio_reset_pin(MANNY_LORA_M1_PIN);
    gpio_reset_pin(MANNY_LORA_AUX_PIN);
    gpio_reset_pin(MANNY_LORA_TX_PIN);
    gpio_reset_pin(MANNY_LORA_RX_PIN);

    LoRa_E220 e220(MANNY_LORA_RX_PIN, MANNY_LORA_TX_PIN, &Serial2,
                   MANNY_LORA_AUX_PIN, MANNY_LORA_M0_PIN, MANNY_LORA_M1_PIN,
                   UART_BPS_RATE_9600, SERIAL_8N1);
    e220.begin();

    Configuration config = E220LoRaTransmitter::defaultConfiguration();
    auto rs = e220.setConfiguration(config, WRITE_CFG_PWR_DWN_SAVE);
    if (rs.code == E220_SUCCESS)
        LOG_INFO("LoRa", "E220 configurato con successo.");
    else
        LOG_ERROR("LoRa", "setConfiguration fallito: %s", rs.getResponseDescription().c_str());

    Serial2.end();
    return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
}

bool TestRoutine::testE220Connector()
{
    LOG_INFO("Test", "[STEP 7] LoRa Connector Test");

    const gpio_num_t pins[]  = {GPIO_NUM_40, GPIO_NUM_39, GPIO_NUM_38, GPIO_NUM_41, GPIO_NUM_42};
    const char*      names[] = {"GPIO40 AUX", "GPIO39 RX<E220TX", "GPIO38 TX>E220RX", "GPIO41 M1", "GPIO42 M0"};

    for (int i = 0; i < 5; i++) {
        gpio_set_direction(pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(pins[i], 0);
    }

    LOG_INFO("Test", "1: All pins high 3s");
    for (int i = 0; i < 5; i++) gpio_set_level(pins[i], 1);
    vTaskDelay(pdMS_TO_TICKS(3000));
    for (int i = 0; i < 5; i++) gpio_set_level(pins[i], 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    for (int i = 0; i < 5; i++) {
        int bursts = i + 1;
        LOG_INFO("Test", "%s: %d burst(s)", names[i], bursts);
        for (int b = 0; b < bursts; b++) {
            gpio_set_level(pins[i], 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(pins[i], 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    for (int i = 0; i < 5; i++) gpio_reset_pin(pins[i]);
    return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
}

void TestRoutine::testFSMTransitions(RocketFSM& fsm)
{
    LOG_INFO("Test", "\n\n=== STARTING AUTOMATED FSM TEST ===");
    fsm.start();

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);

    RocketState lastLoggedState = RocketState::INACTIVE;
    unsigned long testStartTime = Utils::millis();

    while (true) {
        RocketState currentState = fsm.getCurrentState();

        static unsigned long lastPeriodicLog = 0;
        bool stateChanged = (currentState != lastLoggedState);
        bool periodicLog  = (Utils::millis() - lastPeriodicLog > 10000);

        if (stateChanged || periodicLog) {
            LOG_INFO("Test", "State: %s (runtime: %lu ms, uptime: %.1f sec)",
                     fsm.getStateString(currentState),
                     Utils::millis(),
                     (Utils::millis() - testStartTime) / 1000.0);
            if (periodicLog) lastPeriodicLog = Utils::millis();
            lastLoggedState = currentState;
        }

        if (currentState == RocketState::RECOVERED) {
            LOG_INFO("Test", "=== FSM TEST COMPLETED SUCCESSFULLY ===");
            LOG_INFO("Test", "Total test duration: %.1f seconds", (Utils::millis() - testStartTime) / 1000.0);
            vTaskDelete(NULL);
        }
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

bool TestRoutine::calibrateAndSaveIMU()
{
    LOG_INFO("Test", "\n[STEP 12] IMU Calibration");
    LOG_INFO("Test", "Please perform figure-8 movements and rest the sensor on various axes.");
    
    auto bnoSensor = _model->getBNO055Sensor();
    if (!bnoSensor) {
        LOG_ERROR("Test", "BNO055 Sensor not available!");
        return waitForUserInput("Type FAILED to continue");
    }

    bool calibrated = false;
    LOG_INFO("Test", "Waiting for calibration");

    // Loop until calibration reaches 3 for all sub-sensors
    while (!calibrated)
    {
        _model->updateBNO055();
        IMUData data;
        SensorReadStatus bnoStatus = _model->getBNO055Data(data);
        if (bnoStatus != SensorReadStatus::OK) {
            LOG_WARNING("Test", "Failed to get BNO055 data");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        LOG_INFO("Test", "Calib Status -> SYS: %d, GYRO: %d, ACCEL: %d, MAG: %d",
                 data.calibration_sys, data.calibration_gyro, 
                 data.calibration_accel, data.calibration_mag);

        if (data.calibration_sys == 3 && data.calibration_gyro == 3 && 
            data.calibration_accel == 3 && data.calibration_mag == 3) 
        {
            LOG_INFO("Test", "IMU is FULLY CALIBRATED!");
            calibrated = true;
            break;
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Save directly to the internal ESP32 partition
    if (bnoSensor->saveCalibrationToNVS()) {
        return waitForUserInput("Saved successfully! Type PASSED to continue");
    } else {
        LOG_ERROR("Test", "Failed to save to NVS.");
        return waitForUserInput("Type FAILED to continue");
    }
}

void TestRoutine::run()
{
    LOG_INFO("Test", "=== SYSTEM TEST ROUTINE INITIATED ===");

    while (true)
    {
        _statusManager.setSystemCode(TEST_MENU);

        printf("\n=== MENU TEST ===\n");
        printf("1 - Test alimentazione e LED\n");
        printf("2 - Test sensori\n");
        printf("3 - Test attuatori\n");
        printf("4 - Test SD Card\n");
        printf("5 - I2C scan\n");
        printf("6 - Configura E220 (one-time setup)\n");
        printf("7 - Test telemetria\n");
        printf("8 - Test connettore E220\n");
        printf("9 - Test Flash memory\n");
        printf("10 - Test ricezione comando LoRa\n");
        printf("11 - Esegui tutti i test in sequenza\n");
        printf("12 - Dump JSONL telemetry from Flash\n");
        printf("13 - Format Flash memory\n");
        printf("14 - Calibra IMU e salva in NVS (Internal Flash)\n");
        printf("0 - Esci dal menu test\n");
        printf("Inserisci il numero del test da eseguire:\n");

        char buffer[32] = {0};
        Utils::readLine(buffer, sizeof(buffer));
        std::string input(buffer);
        trimString(input);
        int choice = std::atoi(input.c_str());
        bool testPassed = false;

        showTestPattern(choice);

        switch (choice)
        {
        case 1:  do { testPassed = testPowerAndLEDs();  } while (!testPassed); break;
        case 2:  do { testPassed = testSensors();       } while (!testPassed); break;
        case 3:  do { testPassed = testActuators();     } while (!testPassed); break;
        case 4:  do { testPassed = testSDCard();        } while (!testPassed); break;
        case 5:  do { testPassed = testI2CScan();       } while (!testPassed); break;
        case 6:  do { testPassed = configureE220();     } while (!testPassed); break;
        case 7:  do { testPassed = testTelemetry();     } while (!testPassed); break;
        case 8:  do { testPassed = testE220Connector(); } while (!testPassed); break;
        case 9:  do { testPassed = testFlashMemory();      } while (!testPassed); break;
        case 10: do { testPassed = testTelemetryCommand(); } while (!testPassed); break;
        case 11:
            _statusManager.playBlockingPattern(TEST_ALL, 2000);
            do { testPassed = testPowerAndLEDs();  } while (!testPassed);
            do { testPassed = testSensors();       } while (!testPassed);
            do { testPassed = testActuators();     } while (!testPassed);
            do { testPassed = testSDCard();        } while (!testPassed);
            do { testPassed = testTelemetry();     } while (!testPassed);
            do { testPassed = testFlashMemory();   } while (!testPassed);
            do { testPassed = testTelemetryCommand(); } while (!testPassed);
            _statusManager.playBlockingPattern(TEST_SUCCESS, 2000);
            LOG_INFO("Test", "\n=== TUTTI I TEST COMPLETATI CON SUCCESSO ===");
            break;
        case 12:
                do { testPassed = dumpFlashJsonFiles(); } while (!testPassed); break;
        case 13:
                do { testPassed = clearFlashMemory(); } while (!testPassed); break;
        case 14:
                do { testPassed = calibrateAndSaveIMU(); } while (!testPassed); break;
        case 0:
            _statusManager.playBlockingPattern(TEST_SUCCESS, 2000);
            LOG_INFO("Test", "\n=== USCITA DAL MENU TEST ===");
            _statusManager.setSystemCode(SYSTEM_OK);
            return;
        default:
            printf("Scelta non valida. Riprova.\n");
            continue;
        }

        if (choice >= 1 && choice <= 13) {
            LOG_INFO("Test", "Test completato con successo!");
            _statusManager.playBlockingPattern(TEST_SUCCESS, 1000);
        }
    }
}

void TestRoutine::printSystemInfo()
{
    printf("--- System Information ---\n");
    printf("ESP32 Chip: %s\n",         ESP.getChipModel());
    printf("CPU Frequency: %lu MHz\n", (unsigned long)ESP.getCpuFreqMHz());
    printf("Total Heap: %lu bytes\n",  (unsigned long)ESP.getHeapSize());
    printf("Free Heap: %lu bytes\n",   (unsigned long)ESP.getFreeHeap());
    printf("PSRAM Total: %lu bytes\n", (unsigned long)ESP.getPsramSize());
    printf("PSRAM Free: %lu bytes\n",  (unsigned long)ESP.getFreePsram());
    printf("Flash Size: %lu bytes\n",  (unsigned long)ESP.getFlashChipSize());
    printf("SDK Version: %s\n",        ESP.getSdkVersion());
    printf("FreeRTOS running on %d cores\n", portNUM_PROCESSORS);
    printf("Tick rate: %d Hz\n",       configTICK_RATE_HZ);
    printf("--- End System Information ---\n");
}
