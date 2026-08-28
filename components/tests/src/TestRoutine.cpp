#include "TestRoutine.hpp"
#include "esp_flash.h"
#include "esp_heap_caps.h"
#include "soc/rtc.h"

#include <array>
#include <cstring>
#include <functional>
#include <string>
#include <algorithm>
#include <cctype>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <esp_system.h>

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
      _buzzerController(buzzerController)
{}

bool TestRoutine::waitForUserInput(const char* message)
{
    printf("%s\n(Shortcuts: 'P' = Passed, 'F' = Failed, 'R' = Reboot)\n", message);
    _statusManager.setSystemCode(WAITING_INPUT);

    while (true)
    {
        char buffer[64] = {0};
        Utils::readLine(buffer, sizeof(buffer));
        std::string input(buffer);
        Utils::trimString(input);
        Utils::toUpperString(input);

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
            Utils::trimString(confirm);
            Utils::toUpperString(confirm);

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
    // 11-13 (flash utilities, IMU calibration) have no pattern
    case 14: _statusManager.playBlockingPattern(TEST_ALL,       2000); break;
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

    _model->updateBNO055();

    IMUData imuData;
    SensorReadStatus imuStatus = _model->getBNO055Data(imuData);
    if (imuStatus != SensorReadStatus::OK) {
        _statusManager.playBlockingPattern(IMU_FAIL, 2000);
        LOG_ERROR("Test", "Errore: IMU non inizializzata o errore nella lettura. Read status: %d", static_cast<int>(imuStatus));
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
        LOG_ERROR("Test", "Errore: Barometro 1 non inizializzato. Read status: %d", static_cast<int>(baro1Status));
    } else {
        LOG_INFO("Test", "Barometer 1 Pressure: %.2f Pa",
                 (double)baro1Data.pressure);
    }


    PressureSensorData baro2Data;
    SensorReadStatus baro2Status = _model->getMS561101BA03Data_2(baro2Data);
    if (baro2Status != SensorReadStatus::OK) {
        _statusManager.playBlockingPattern(BARO2_FAIL, 2000);
        LOG_ERROR("Test", "Errore: Barometro 2 non inizializzato. Read status: %d", static_cast<int>(baro2Status));
    } else {
        LOG_INFO("Test", "Barometer 2 Pressure: %.2f Pa",
                 (double)baro2Data.pressure);
    }

    _model->updateLIS3DHTR();

    AccelerometerSensorData acclData;
    SensorReadStatus acclStatus  = _model->getLIS3DHTRData(acclData);
    if (acclStatus != SensorReadStatus::OK) {
        _statusManager.playBlockingPattern(IMU_FAIL, 2000);
        LOG_ERROR("Test", "Errore: Accelerometro non inizializzato. Read status: %d", static_cast<int>(acclStatus));
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

    printf("WARNING: This operation will format the entire Flash memory.\n");
    printf("All data will be lost!\n");
    printf("Are you sure you want to continue? (Y/n): \n");
    
    char buffer[16] = {0};
    Utils::readLine(buffer, sizeof(buffer));
    std::string input(buffer);
    Utils::trimString(input);
    
    if (input == "Y" || input == "y") {
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
        {
            SerialLogger::Lock lineLock;
            printf("%s", line.c_str());
        }

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
            printf(label ? "Found: 0x%02X %s\n" : "Unknown: 0x%02X\n", addr, label ? label : "");
        }
    }
    if (!found) printf("No devices found.\n");

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

    // ── Tests Definition ────────────────
    std::array<TestOption, 13> tests = {{
        {"Alimentation and LED Test", [this]() { return testPowerAndLEDs(); }, true},
        {"Sensors Test", [this]() { return testSensors(); }, true},
        {"Actuators Test", [this]() { return testActuators(); }, true},
        {"SD Card Test", [this]() { return testSDCard(); }, true},
        {"I2C Scan", [this]() { return testI2CScan(); }, false},
        {"E220 Configuration (one-time setup)", [this]() { return configureE220(); }, false},
        {"Telemetry Test", [this]() { return testTelemetry(); }, true},
        {"E220 connector Test", [this]() { return testE220Connector(); }, false},
        {"Flash memory Test", [this]() { return testFlashMemory(); }, true},
        {"LoRa command reception Test", [this]() { return testTelemetryCommand(); }, true},
        {"Dump JSONL telemetry from Flash", [this]() { return dumpFlashJsonFiles(); }, false},
        {"Format Flash memory", [this]() { return clearFlashMemory(); }, false},
        {"IMU calibration and save to NVS (Internal Flash)", [this]() { return calibrateAndSaveIMU(); }, false},
    }};

    // ── Run Tests ────────────────
    bool run = true;
    while (run)
    {
        _statusManager.setSystemCode(TEST_MENU);

        // ── Show Menu ────────────────
        printf("\n=== MENU TEST ===\n");
        const int numTests = static_cast<int>(tests.size());
        for (int i = 0; i < numTests; i++) {
            printf("%d - %s\n", i + 1, tests.at(i).name);
        }
        printf("%d - Execute all Tests in sequence\n", numTests + 1);
        printf("0 - Exit Menu\n");
        printf("Insert the desired action number: (0-%d)\n", numTests + 1);

        // ── Get Choice ────────────────
        char buffer[32] = {0};
        Utils::readLine(buffer, sizeof(buffer));
        std::string input(buffer);
        Utils::trimString(input);

        // reject non-numeric input (atoi would turn it into 0 = exit)
        if (input.empty() || input.find_first_not_of("0123456789") != std::string::npos) {
            printf("Invalid choice. Try again.\n");
            continue;
        }
        int choice = std::atoi(input.c_str());

        showTestPattern(choice);

        if (choice >= 0 && choice <= numTests + 1) {
            // ── Exit ──────────────────────────
            if (choice == 0) {
                _statusManager.playBlockingPattern(TEST_SUCCESS, 2000);
                LOG_INFO("Test", "\n=== EXITING TEST MENU ===");
                _statusManager.setSystemCode(SYSTEM_OK);

                run = false;
            }
            // ── Run Single Test ────────────────
            else if (choice <= numTests)
            {
                while (!tests.at(choice - 1).func());

                LOG_INFO("Test", "Test successfully completed!");
                _statusManager.playBlockingPattern(TEST_SUCCESS, 1000);
            }
            // ── Run All Tests ───────────────────
            // excluding I2C scan, E220 configuration/connector and flash utilities
            else
            {
                _statusManager.playBlockingPattern(TEST_ALL, 2000);

                for (const auto& test : tests) {
                    // check if the test should run
                    if (test.run_all_flag)
                        while (!test.func());
                }

                _statusManager.playBlockingPattern(TEST_SUCCESS, 2000);
                LOG_INFO("Test", "\n=== ALL TESTS SUCCESSFULLY COMPLETED ===");
            }
        } else {
            printf("Invalid choice. Try again.\n");
        }
    }
}

void TestRoutine::printSystemInfo()
{
    rtc_cpu_freq_config_t cpuConf;
    rtc_clk_cpu_freq_get_config(&cpuConf);

    uint32_t flashSize = 0;
    if (esp_flash_get_size(NULL, &flashSize) != ESP_OK) {
        flashSize = 0;
    }

    printf("--- System Information ---\n");
    printf("ESP32 Chip: %s\n",         CONFIG_IDF_TARGET);
    printf("CPU Frequency: %lu MHz\n", (unsigned long)cpuConf.freq_mhz);
    printf("Total Heap: %lu bytes\n",  (unsigned long)heap_caps_get_total_size(MALLOC_CAP_INTERNAL));
    printf("Free Heap: %lu bytes\n",   (unsigned long)esp_get_free_heap_size());
    printf("PSRAM Total: %lu bytes\n", (unsigned long)heap_caps_get_total_size(MALLOC_CAP_SPIRAM));
    printf("PSRAM Free: %lu bytes\n",  (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    printf("Flash Size: %lu bytes\n",  (unsigned long)flashSize);
    printf("SDK Version: %s\n",        esp_get_idf_version());
    printf("FreeRTOS running on %d cores\n", portNUM_PROCESSORS);
    printf("Tick rate: %d Hz\n",       configTICK_RATE_HZ);
    printf("--- End System Information ---\n");
}
