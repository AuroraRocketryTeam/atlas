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

#include <config.h>
#include <board.h>
#include <utils.h>
#include <Logger.hpp>
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
    bool imu_ok   = _model->updateBNO055();
    bool baro1_ok = _model->updateMS561101BA03_1();
    bool baro2_ok = _model->updateMS561101BA03_2();
    bool accl_ok  = _model->updateLIS3DHTR();

    _board.init_sensor_test_pins();

    if (!imu_ok) {
        _statusManager.playBlockingPattern(IMU_FAIL, 2000);
        LOG_ERROR("Test", "Errore: IMU non inizializzata.");
    } else {
        auto bnoData = _model->getBNO055Data();
        LOG_INFO("Test", "IMU Accelerometer: x=%.2f, y=%.2f, z=%.2f m/s^2",
                 (double)bnoData->acceleration_x,
                 (double)bnoData->acceleration_y,
                 (double)bnoData->acceleration_z);
    }
    if (!baro1_ok) {
        _statusManager.playBlockingPattern(BARO1_FAIL, 2000);
        LOG_ERROR("Test", "Errore: Barometro 1 non inizializzato.");
    } else {
        LOG_INFO("Test", "Barometer 1 Pressure: %.2f Pa",
                 (double)_model->getMS561101BA03Data_1()->pressure);
    }
    if (!baro2_ok) {
        _statusManager.playBlockingPattern(BARO2_FAIL, 2000);
        LOG_ERROR("Test", "Errore: Barometro 2 non inizializzato.");
    } else {
        LOG_INFO("Test", "Barometer 2 Pressure: %.2f Pa",
                 (double)_model->getMS561101BA03Data_2()->pressure);
    }
    if (!accl_ok) {
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
        if (_sdCard->writeFile(TEST_FILE, content.c_str())) {
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
    if (!_flash->writeFile(testFile.c_str(), "Hello, ESP32 Flash Storage!\n"))
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
    if (!_flash->appendFile(testFile.c_str(), "Appended line.\n"))
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
    printf("Are you sure you want to continue? (Y/n): ");
    
    char buffer[16] = {0};
    Utils::readLine(buffer, sizeof(buffer));
    std::string input(buffer);
    trimString(input);
    
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
    LOG_INFO("Test", "[DUMP] Export JSON files from external flash");

    if (!_flash)
    {
        _flash = std::make_shared<Flash>();
    }

    if (!_flash)
    {
        LOG_ERROR("Dump", "Failed to initialize External Flash");
        return waitForUserInput("Type PASSED to continue or FAILED to retry");
    }

    if (!_flash->isInitialized() && !_flash->init(_board.get_spi_bus(), _board.get_flash_cs_pin(), _board.get_flash_hold_pin(), _board.get_flash_wp_pin()))
    {
        LOG_ERROR("Dump", "Flash init failed");
        return waitForUserInput("Type PASSED to continue or FAILED to retry");
    }

    printf("\n=== JSON DUMP START ===\n");
    printf("Capture this serial output to a file on PC.\n");

    int dumpedCount = 0;
    int consecutiveMisses = 0;
    const int MAX_CONSECUTIVE_MISSES = 20;
    const int MAX_FILES_TO_CHECK = 10000;

    for (int i = 0; i < MAX_FILES_TO_CHECK; ++i)
    {
        // Periodic watchdog update
        if (i % 25 == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        char filename[64] = {0};
        std::snprintf(filename, sizeof(filename), "JSON_data_%d.json", i);

        std::string content = _flash->readFile(filename);
        if (content.empty())
        {
            consecutiveMisses++;
            if (consecutiveMisses >= MAX_CONSECUTIVE_MISSES) {
                LOG_INFO("Dump", "Reached end of file sequence (stopped scanning at index %d).", i);
                break;
            }
            continue;
        }

        consecutiveMisses = 0;

        printf("START_FILE:%s\n", filename);
        printf("%s", content.c_str());
        
        if (!content.empty() && content.back() != '\n')
        {
            printf("\n");
        }
        
        printf("END_FILE:%s\n", filename); 

        dumpedCount++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    printf("=== JSON DUMP END (%d file%s) ===\n\n", dumpedCount, dumpedCount == 1 ? "" : "s");

    if (dumpedCount == 0)
    {
        LOG_WARNING("Dump", "No JSON_data_*.json files found on flash");
    }
    else
    {
        LOG_INFO("Dump", "Successfully exported %d JSON files", dumpedCount);
    }

    return waitForUserInput("JSON dump printed on serial. Type PASSED to continue or FAILED to retry");
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

    auto csc = e220.getConfiguration();
    if (csc.status.code != E220_SUCCESS) {
        LOG_ERROR("LoRa", "getConfiguration failed: %s", csc.status.getResponseDescription().c_str());
        csc.close();
        Serial2.end();
        return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
    }
    auto config = *(Configuration*)csc.data;
    csc.close();

    config.ADDL = 0x03;
    config.ADDH = 0x00;
    config.CHAN  = 23;
    config.SPED.uartBaudRate  = UART_BPS_115200;
    config.SPED.airDataRate   = AIR_DATA_RATE_100_96;
    config.SPED.uartParity    = MODE_00_8N1;
    config.OPTION.subPacketSetting  = SPS_200_00;
    config.OPTION.RSSIAmbientNoise  = RSSI_AMBIENT_NOISE_DISABLED;
    config.OPTION.transmissionPower = POWER_17;
    config.TRANSMISSION_MODE.enableRSSI        = RSSI_ENABLED;
    config.TRANSMISSION_MODE.fixedTransmission = FT_FIXED_TRANSMISSION;
    config.TRANSMISSION_MODE.enableLBT         = LBT_DISABLED;
    config.TRANSMISSION_MODE.WORPeriod         = WOR_2000_011;

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
        case 0:
            _statusManager.playBlockingPattern(TEST_SUCCESS, 2000);
            LOG_INFO("Test", "\n=== USCITA DAL MENU TEST ===");
            _statusManager.setSystemCode(SYSTEM_OK);
            return;
        default:
            printf("Scelta non valida. Riprova.\n");
            continue;
        }

        if (choice >= 1 && choice <= 11) {
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
