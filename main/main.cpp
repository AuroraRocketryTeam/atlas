// Standard libraries
#include <Arduino.h>
#include <variant>
#include <string>
#include <cstring>

// FreeRTOS
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// WiFi and communication
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_netif.h>
#include <esp_err.h>
#include <nvs_flash.h>

// Configuration and pins
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include <config.h>
#include <pins.h>
#include <board.h>
#include <utils.h>

// Interfaces
#include <ISensor.hpp>
#include <ILogger.hpp>
#include <ITransmitter.hpp>

// System model
#include <RocketModel.hpp>

// Sensors used in the system model
#include <BNO055Sensor.hpp>
#include <MS561101BA03.hpp>
#include <LIS3DHTRSensor.hpp>
#include <GPS.hpp>

// Storage and logging
#include <SD-master.hpp>
#include <Flash.hpp>
#include <RocketLogger.hpp>
#include <Logger.hpp>

// Controllers and filters
#include <LEDController.hpp>
#include <BuzzerController.hpp>
#include <StatusManager.hpp>

// Main system
#include <RocketFSM.hpp>
#include <E220LoRaTransmitter.hpp>

/**
 * @brief Uncomment to enable sensor calibration routine at startup.
 * Useful for fast testing without needing precise sensor reads.
 *
 */
#define CALIBRATE_SENSORS
#define ENABLE_TEST_ROUTINE
#define TEST_FILE "/test.txt"

// Board hardware instance
Board board;

// Create controller instances
LEDController ledController(board.get_rgb_red_pin(), board.get_rgb_green_pin(), board.get_rgb_blue_pin());
BuzzerController buzzerController(board.get_buzzer_pin());
StatusManager statusManager(ledController, buzzerController);

// Define the system model
std::shared_ptr<RocketModel> rocketModel = nullptr;

std::shared_ptr<SD> sdCard = nullptr;
std::shared_ptr<Flash> flash = nullptr;

// Define the RocketLogger
std::shared_ptr<RocketLogger> logger = nullptr;

// FSM instance
std::unique_ptr<RocketFSM> rocketFSM;

// Utility functions
void testFSMTransitions(RocketFSM &fsm);
void initializeComponents(std::shared_ptr<BNO055Sensor>& bno055,
                          std::shared_ptr<LIS3DHTRSensor>& accl,
                          std::shared_ptr<MS561101BA03>& baro1,
                          std::shared_ptr<MS561101BA03>& baro2,
                          std::shared_ptr<GPS>& gps);
void GPSfix(std::shared_ptr<GPS> gps);
void printSystemInfo();
void testRoutine();

void setup()
{
    // Initialize actuator pins
    gpio_config(&actuators_gpio_config);
    
    gpio_set_level(MAIN_ACTUATOR_PIN, LOW);
    gpio_set_level(DROGUE_ACTUATOR_PIN, LOW);

    // Initialize LED pins (only those not handled by controllers)
    gpio_config(&led_gpio_config);

    gpio_set_level(LED_BUILT_IN, LOW);
    gpio_set_level(LED_RED_PIN, HIGH);
    
    // Install driver for blocking reads of Utils::readLine
    // Regular console output already works via the vfs bound by CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    usb_serial_jtag_driver_config_t usb_cfg = { .tx_buffer_size = 1024, .rx_buffer_size = 1024 };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));

    // Signal initialization start
    board.init();
    gpio_set_level(LED_RED_PIN, HIGH);

    // Initialize controllers
    ledController.init();
    buzzerController.init();

    // Initialize status patterns
    statusManager.init();

    // Set initial status with PRE_FLIGHT_MODE
    statusManager.setSystemCode(PRE_FLIGHT_MODE);

    LOG_INFO("Main", "\n=== Aurora Rocketry Flight Software ===");
    LOG_INFO("Main", "Firmware Board: %s", Board::BOARD_NAME);
    LOG_INFO("Main", "Initializing system...");

    // Initialize components
    LOG_INFO("Main", "Initializing sensors...");
    std::shared_ptr<BNO055Sensor> bno055 = nullptr;
    std::shared_ptr<LIS3DHTRSensor> accl = nullptr;
    std::shared_ptr<MS561101BA03> baro1 = nullptr;
    std::shared_ptr<MS561101BA03> baro2 = nullptr;
    std::shared_ptr<GPS> gps = nullptr;
    initializeComponents(bno055, accl, baro1, baro2, gps);
    LOG_INFO("Main", "All components initialized");

    // Initialize logger
    LOG_INFO("Init", "Initializing rocket logger...");
    logger = std::make_shared<RocketLogger>();
    LOG_INFO("Init", "Rocket logger initialized");

    // Create Nemesis instance (storage backends are optional)
    rocketModel = std::make_shared<RocketModel>(bno055, accl, baro1, baro2, gps, sdCard, flash);
    LOG_INFO("Main", "RocketModel system model created");

#ifdef ENABLE_TEST_ROUTINE
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    // Start test routine if in test mode
    LOG_INFO("Main", "=== TEST MODE ENABLED ===");
    testRoutine();
#endif

#ifdef CALIBRATE_SENSORS
    // Checking sensors calibration
    statusManager.setSystemCode(CALIBRATING);
    GPSfix(gps);
    statusManager.setSystemCode(SYSTEM_OK);
#endif
    // Print system information
    printSystemInfo();

    // Initialize and start FSM
    LOG_INFO("Main", "=== System initialization complete ===");
    LOG_INFO("Main", "\n=== Initializing Flight State Machine ===");
    rocketFSM = std::make_unique<RocketFSM>(rocketModel, sdCard, logger);
    rocketFSM->init();
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Wait for arming pin to be enabled before starting FSM
    statusManager.setSystemCode(PRE_FLIGHT_MODE);
    while (!board.is_armed())
    {
        LOG_WARNING("Main", "System not armed! Waiting for arming signal...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Start FSM tasks
    LOG_INFO("Main", "Starting Flight State Machine...");
    statusManager.setSystemCode(FSM_STARTED);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    rocketFSM->start();
    statusManager.setSystemCode(FLIGHT_MODE);

    // Signal successful initialization
    gpio_set_level(LED_RED_PIN, LOW);
    gpio_set_level(LED_GREEN_PIN, HIGH);
    LOG_INFO("Main", "SETUP COMPLETE - SYSTEM IN FLIGHT MODE");
}

void loop()
{
    auto currentState = rocketFSM->getCurrentState();
    LOG_INFO("Main", "Current FSM State: %s", rocketFSM->getStateString(currentState));
    LOG_INFO("Main", "Free heap: %u bytes", ESP.getFreeHeap());

    unsigned long lastHeartbeat = 0;
    bool ledState = false;

    // Heartbeat every 2 seconds
    if (Utils::millis() - lastHeartbeat > 2000)
    {
        LOG_INFO("Main", "Last heartbeat at %lu ms - System running", Utils::millis());
        lastHeartbeat = Utils::millis();
        ledState = !ledState;
        gpio_set_level(LED_BUILT_IN, ledState);

        // Monitor RocketLogger memory usage
        if (logger)
        {
            int logCount = logger->getLogCount();
            LOG_INFO("Main", "RocketLogger entries: %d", logCount);

            // If log count is high, warn about memory usage
            if (logCount > 800)
            {
                LOG_WARNING("Main", "RocketLogger approaching memory limit (%d entries)", logCount);
            }
        }

        // Optional: Print current state periodically
        RocketState lastLoggedState = RocketState::INACTIVE;
        RocketState currentState = rocketFSM->getCurrentState();

        if (currentState != lastLoggedState)
        {
            LOG_INFO("Main", "Current FSM State: %s",
                     rocketFSM->getStateString(currentState));
            lastLoggedState = currentState;
        }
    }

    // Small delay to prevent watchdog issues
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

void testFSMTransitions(RocketFSM &fsm)
{
    LOG_INFO("Test", "\n\n=== STARTING AUTOMATED FSM TEST ===");
    LOG_INFO("Test", "Testing all state transitions with automatic timeouts");
    LOG_INFO("Test", "Will cycle through all states, observing task execution\n");

    // Il test inizia automaticamente quando fsm.start() viene chiamato
    // Le transizioni sono tutte temporizzate nei metodi di transizione definiti nella classe RocketFSM

    fsm.start();
    // Use FreeRTOS timing for more reliable 1-second intervals
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1 second

    RocketState lastLoggedState = RocketState::INACTIVE;
    unsigned long testStartTime = Utils::millis();
    while (true)
    {
        RocketState currentState = fsm.getCurrentState();

        // Only log when state changes or every 10 seconds
        static unsigned long lastPeriodicLog = 0;
        bool stateChanged = (currentState != lastLoggedState);
        bool periodicLog = (Utils::millis() - lastPeriodicLog > 10000);

        if (stateChanged || periodicLog)
        {
            LOG_INFO("Test", "State: %s (runtime: %lu ms, uptime: %.1f sec)",
                     fsm.getStateString(currentState),
                     Utils::millis(),
                     (Utils::millis() - testStartTime) / 1000.0);

            if (periodicLog)
                lastPeriodicLog = Utils::millis();

            lastLoggedState = currentState;
        }

        // Termina il test quando raggiungiamo lo stato RECOVERED
        if (currentState == RocketState::RECOVERED)
        {
            LOG_INFO("Test", "=== FSM TEST COMPLETED SUCCESSFULLY ===");
            LOG_INFO("Test", "All states were visited in the correct order!");
            LOG_INFO("Test", "Total test duration: %.1f seconds", (Utils::millis() - testStartTime) / 1000.0);
            vTaskDelete(NULL);
        }
        // Use FreeRTOS delay for precise timing
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

static bool initializeWifiStaForEspNow()
{
    static bool initialized = false;
    if (initialized) {
        return true;
    }

    // NVS init (required by Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ret = nvs_flash_erase();
        if (ret != ESP_OK) {
            LOG_ERROR("WiFi", "nvs_flash_erase failed: %s", esp_err_to_name(ret));
            return false;
        }
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        LOG_ERROR("WiFi", "nvs_flash_init failed: %s", esp_err_to_name(ret));
        return false;
    }

    // init network stack and event loop 
    ret = esp_netif_init();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        LOG_ERROR("WiFi", "esp_netif_init failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_event_loop_create_default();
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        LOG_ERROR("WiFi", "esp_event_loop_create_default failed: %s", esp_err_to_name(ret));
        return false;
    }

    (void)esp_netif_create_default_wifi_sta();

    // init wifi drivers 
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_INIT_STATE) {
        LOG_ERROR("WiFi", "esp_wifi_init failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        LOG_ERROR("WiFi", "esp_wifi_set_mode failed: %s", esp_err_to_name(ret));
        return false;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_STOPPED) {
        LOG_ERROR("WiFi", "esp_wifi_start failed: %s", esp_err_to_name(ret));
        return false;
    }

    // disconnect wifi
    ret = esp_wifi_disconnect();
    if (ret != ESP_OK && ret != ESP_ERR_WIFI_NOT_INIT && ret != ESP_ERR_WIFI_CONN) {
        LOG_WARNING("WiFi", "esp_wifi_disconnect returned: %s", esp_err_to_name(ret));
    }

    initialized = true;
    return true;
}

void initializeComponents(std::shared_ptr<BNO055Sensor>& bno055,
                          std::shared_ptr<LIS3DHTRSensor>& accl,
                          std::shared_ptr<MS561101BA03>& baro1,
                          std::shared_ptr<MS561101BA03>& baro2,
                          std::shared_ptr<GPS>& gps)
{
    LOG_INFO("Init", "\n--- Initializing Components ---");

    // Initialize sensors
    LOG_INFO("Init", "Initializing sensors...");

    // Initialize BNO055 (IMU)
    bno055 = std::make_shared<BNO055Sensor>(board.get_i2c_bus(IBoardHardware::Sensor::IMU), board.get_bno055_i2c_address());
    if (bno055 && bno055->init())
    {
        LOG_INFO("Init", "BNO055 (IMU) initialized");
    }
    else
    {
        LOG_ERROR("Init", "Failed to initialize BNO055");
    }

    // Initialize barometers
    baro1 = std::make_shared<MS561101BA03>(board.get_spi_bus(), MANNY_BAROMETER_CS_PIN);
    if (baro1 && baro1->init())
    {
        LOG_INFO("Init", "Barometer 1 initialized");
    }
    else
    {
        LOG_ERROR("Init", "Failed to initialize Barometer 1");
    }

    baro2 = std::make_shared<MS561101BA03>(board.get_spi_bus(), board.get_barometer2_cs_pin());
    if (baro2 && baro2->init())
    {
        LOG_INFO("Init", "Barometer 2 initialized");
    }
    else
    {
        LOG_ERROR("Init", "Failed to initialize Barometer 2");
    }

    // Initialize accelerometer
    accl = std::make_shared<LIS3DHTRSensor>(board.get_i2c_bus(IBoardHardware::Sensor::ACC));
    if (accl && accl->init())
    {
        LOG_INFO("Init", "LIS3DHTR (Accelerometer) initialized");
    }
    else
    {
        LOG_ERROR("Init", "Failed to initialize LIS3DHTR");
    }

    // Initialize GPS
    gps = std::make_shared<GPS>(GPS_TX_PIN, GPS_RX_PIN);

    if (gps && gps->init())
    {
        LOG_INFO("Init", "GPS initialized");
    }
    else
    {
        LOG_ERROR("Init", "Failed to initialize GPS");
    }

    // Inizializza la scheda SD
    LOG_INFO("Init", "Initializing SD card for logging...");
    sdCard = std::make_shared<SD>();
    if (sdCard && sdCard->init(board.get_spi_bus()))
    {
        LOG_INFO("Init", "SD card initialized");
        sdCard->openFile("test.txt");
        LOG_INFO("Init", "Testing SD card write...");
        std::string content = "SD card write test successful! Timestamp: " + std::to_string(Utils::millis()) + " ms";
        if (sdCard->writeFile("test.txt", content.c_str()))
        {
            LOG_INFO("Init", "SD card write test successful");
            char *readContent = sdCard->readFile("test.txt");
            if (readContent)
            {
                LOG_INFO("Init", "Read from SD card: %s", readContent);
            }
            else
            {
                LOG_ERROR("Init", "Failed to read back from SD card");
            }
        }
        else
        {
            LOG_ERROR("Init", "SD card write test failed");
        }
        sdCard->closeFile();
    }
    else
    {
        LOG_ERROR("Init", "Failed to initialize SD card");
    }

    LOG_INFO("Init", "Initializing external flash for mirrored logging...");
    flash = std::make_shared<Flash>();
    if (flash && flash->init(board.get_spi_bus(), board.get_flash_cs_pin(), board.get_flash_hold_pin(), board.get_flash_wp_pin()))
    {
        LOG_INFO("Init", "External flash initialized");
    }
    else
    {
        LOG_ERROR("Init", "Failed to initialize external flash");
    }

    // Initializa ESP-NOW connection for telemetry
    LOG_INFO("Init", "Initializing ESP-NOW for telemetry...");
    if (initializeWifiStaForEspNow())
    {
        if (esp_now_init() == ESP_OK)
        {
            LOG_INFO("Init", "ESP-NOW initialized");
            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, RECEIVER_MAC_ADDRESS, 6);
            peerInfo.channel = 0;
            peerInfo.encrypt = false;

            if (esp_now_add_peer(&peerInfo) == ESP_OK)
            {
                LOG_INFO("Init", "ESP-NOW peer added");
            }
            else
            {
                LOG_ERROR("Init", "Failed to add ESP-NOW peer");
            }
        }
        else
        {
            LOG_ERROR("Init", "Failed to initialize ESP-NOW");
        }
    }
    else
    {
        LOG_ERROR("Init", "Failed to initialize Wi-Fi");
    }

    // We don't need to initialize LEDManager anymore - StatusManager handles it
    LOG_INFO("Init", "Status indicators initialized");
}

// Function to check GPS fix
void GPSfix(std::shared_ptr<GPS> gps)
{
    if (gps)
    {
        LOG_INFO("GPS", "Checking GPS lock...");

        bool gpsLocked = false;
        unsigned long startTime = Utils::millis();

        while (!gpsLocked && (Utils::millis() - startTime < GPS_FIX_TIMEOUT_MS))
        {
            auto gpsDataUpdate = gps->updateData();
            if (gpsDataUpdate)
            {
                LOG_INFO("GPS", "Getting GPS data...");
                auto gpsData = gps->getData();
                auto fixType = gpsData->fixType;
                auto satellites = gpsData->satellites;

                LOG_INFO("GPS", "Fix value: %d", fixType);
                if (fixType >= GPS_MIN_FIX)
                {
                    gpsLocked = true;
                    LOG_INFO("GPS", "GPS lock acquired. Satellites: %d", satellites);
                }
            }
            vTaskDelay(GPS_FIX_LOOKUP_INTERVAL_MS / portTICK_PERIOD_MS);
        }

        if (!gpsLocked)
        {
            LOG_ERROR("GPS", "GPS lock not acquired within timeout period.");
        }
    }

    LOG_INFO("Calibration", "Sensor calibration complete.");
    statusManager.setSystemCode(SYSTEM_OK);
}

// Utility function to calculate mean sensor readings
Eigen::Vector3f calculateMean(const std::vector<Eigen::Vector3f> &readings)
{
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const auto &reading : readings)
    {
        mean += reading;
    }
    mean /= readings.size();
    return mean;
}

// Utility function to calculate standard deviation of sensor readings
Eigen::Vector3f calculateStandardDeviation(const std::vector<Eigen::Vector3f> &readings)
{
    Eigen::Vector3f mean = calculateMean(readings);
    Eigen::Vector3f variance = Eigen::Vector3f::Zero();
    for (const auto &reading : readings)
    {
        Eigen::Vector3f diff = reading - mean;
        variance += diff.cwiseProduct(diff);
    }
    variance /= static_cast<float>(readings.size() - 1);
    return variance.cwiseSqrt();
}

// Helper function to show a pattern for a specific test
void showTestPattern(int testNumber, StatusManager &statusManager)
{
    switch (testNumber)
    {
    case 1:
        statusManager.playBlockingPattern(TEST_POWER, 1000);
        break;
    case 2:
        statusManager.playBlockingPattern(TEST_SENSORS, 1000);
        break;
    case 3:
        statusManager.playBlockingPattern(TEST_ACTUATORS, 1000);
        break;
    case 4:
        statusManager.playBlockingPattern(TEST_SD, 1000);
        break;
    case 5:
        statusManager.playBlockingPattern(TEST_SENSORS, 1000);
        break;
    case 6:
        statusManager.playBlockingPattern(TEST_TELEMETRY, 1000);
        break;
    case 7:
        statusManager.playBlockingPattern(TEST_TELEMETRY, 1000);
        break;
    case 8:
        statusManager.playBlockingPattern(TEST_TELEMETRY, 1000);
        break;
    case 9:
        statusManager.playBlockingPattern(TEST_SD, 1000);
        break;
    case 10:
        statusManager.playBlockingPattern(TEST_ALL, 2000);
        break;
    default:
        break;
    }
}

// helper functions for std::string manipulation
static void trimString(std::string &s)
{
    auto notSpace = [](unsigned char c) { return !std::isspace(c); };
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), notSpace));
    s.erase(std::find_if(s.rbegin(), s.rend(), notSpace).base(), s.end());
}

static void toUpperString(std::string &s)
{
    for (char &c : s)
    {
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    }
}

// Modified waitForUserInput with buzzer patterns
bool waitForUserInput(const char *message)
{
    printf("%s\n(Shortcuts: 'P' = Passed, 'F' = Failed, 'R' = Reboot)\n", message);
    statusManager.setSystemCode(WAITING_INPUT);

    while (true)
    {
        char buffer[64] = {0};
        Utils::readLine(buffer, sizeof(buffer));
        {
            std::string input(buffer);
            trimString(input);
            toUpperString(input);

            if (input == "PASSED" || input == "P")
            {
                statusManager.playBlockingPattern(TEST_SUCCESS, 1000);
                return true;
            }
            if (input == "FAILED" || input == "F")
            {
                statusManager.playBlockingPattern(TEST_FAILURE, 1000);
                return false;
            }
            if (input == "REBOOT" || input == "R")
            {
                LOG_WARNING("Test", "System is going to reboot, are you sure?");
                LOG_WARNING("Test", "Type REBOOT (or 'R') to confirm or anything else to cancel.");

                char confirmBuffer[64] = {0};
                Utils::readLine(confirmBuffer, sizeof(confirmBuffer));
                std::string confirm(confirmBuffer);

                if (confirm.empty())
                {
                    LOG_INFO("Test", "No confirmation - continuing normal operation.");
                    continue;
                }
                trimString(confirm);
                toUpperString(confirm);
                
                if (confirm == "REBOOT" || confirm == "R")
                {
                    LOG_WARNING("Test", "Rebooting system...");
                    ESP.restart();
                }
                else
                {
                    LOG_INFO("Test", "Reboot cancelled.");
                }
            }
            else if (!input.empty())
            {
                LOG_WARNING("Test", "Unrecognized input. Please type P, F, or R.");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Test routine subroutines with proper pattern handling
bool testPowerAndLEDs()
{
    LOG_INFO("Test", "\n[STEP 1] Verifica alimentazione e LED di stato");
    LOG_INFO("Test", "3 lampeggi rossi...");

    for (int i = 0; i < 3; i++) {
        ledController.setColor(ART_LED_RED);
        vTaskDelay(pdMS_TO_TICKS(500));
        ledController.setOff();
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
}

bool testSensors()
{
    LOG_INFO("Test", "\n[STEP 2] Test sensori");
    bool imu_ok = rocketModel->updateBNO055();
    bool baro1_ok = rocketModel->updateMS561101BA03_1();
    bool baro2_ok = rocketModel->updateMS561101BA03_2();
    bool accl_ok = rocketModel->updateLIS3DHTR();

    board.init_sensor_test_pins();
    if (!imu_ok)
    {
        statusManager.playBlockingPattern(IMU_FAIL, 2000);
        LOG_ERROR("Test", "Errore: IMU non inizializzata.");
    } else {
        // Testing IMU accelerometer
        auto bnoData = rocketModel->getBNO055Data();
        auto accelImuX = bnoData->acceleration_x;
        auto accelImuY = bnoData->acceleration_y;
        auto accelImuZ = bnoData->acceleration_z;
        LOG_INFO("Test", "IMU Accelerometer: x=%.2f, y=%.2f, z=%.2f m/s^2",
                    (double)accelImuX,
                    (double)accelImuY,
                    (double)accelImuZ);
        // board.signal_sensor_ok(IBoardHardware::Sensor::IMU);
    }
    if (!baro1_ok)
    {
        statusManager.playBlockingPattern(BARO1_FAIL, 2000);
        LOG_ERROR("Test", "Errore: Barometro 1 non inizializzato.");
    } else {
        auto baro1Data = rocketModel->getMS561101BA03Data_1();
        auto pressureBaro1 = baro1Data->pressure;
        LOG_INFO("Test", "Barometer 1 Pressure: %.2f hPa", (double)pressureBaro1);
        // board.signal_sensor_ok(IBoardHardware::Sensor::BARO1);
    }
    if (!baro2_ok)
    {
        statusManager.playBlockingPattern(BARO2_FAIL, 2000);
        LOG_ERROR("Test", "Errore: Barometro 2 non inizializzato.");
    } else {
        auto baro2Data = rocketModel->getMS561101BA03Data_2();
        auto pressureBaro2 = baro2Data->pressure;
        LOG_INFO("Test", "Barometer 2 Pressure: %.2f hPa", (double)pressureBaro2);
        // board.signal_sensor_ok(IBoardHardware::Sensor::BARO2);
    }
    if (!accl_ok)
    {
        statusManager.playBlockingPattern(IMU_FAIL, 2000);
        LOG_ERROR("Test", "Errore: Accelerometro non inizializzato.");
    } else {
        // Testing LIS3DHTR accelerometer
        LOG_INFO("Test", "LIS3DHTR Accelerometer: Data logged internally");
        board.signal_sensor_ok(IBoardHardware::Sensor::ACC);
    }

    // After all tests, go to user input
    return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
}

bool testActuators()
{
    LOG_INFO("Test", "[STEP 3] Test attuatori");

    LOG_INFO("Test", "Accensione attuatori uno per volta...");

    // DROGUE test
    statusManager.playBlockingPattern(TEST_ACTUATORS, 500); // Show test pattern first
    gpio_set_level(DROGUE_ACTUATOR_PIN, HIGH);
    buzzerController.playTone(TONE_MID, 1000);
    gpio_set_level(DROGUE_ACTUATOR_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(1000));

    // MAIN test
    statusManager.playBlockingPattern(TEST_ACTUATORS, 500); // Show test pattern first
    gpio_set_level(MAIN_ACTUATOR_PIN, HIGH);
    buzzerController.playTone(TONE_MID, 1000);
    gpio_set_level(MAIN_ACTUATOR_PIN, LOW);

    return waitForUserInput("Verificare accensione LED e tensione in uscita da DROGUE e MAIN, verificare funzionamento Buzzer. Scrivi PASSED o FAILED");
}

bool testSDCard()
{
    LOG_INFO("Test", "[STEP 4] Test SD Card");
    LOG_INFO("Test", "Inizializzazione scheda SD e verifica scrittura/lettura...");

    if (sdCard->openFile(TEST_FILE))
    {
        std::string content = "SD card write test successful! Timestamp: " + std::to_string(Utils::millis()) + " ms\n";
        if (sdCard->writeFile(TEST_FILE, content.c_str()))
        {
            LOG_INFO("Test", "SD card write test successful");
            char *readContent = sdCard->readFile(TEST_FILE);
            if (readContent)
            {
                LOG_INFO("Test", "Read from SD card: %s", readContent);
            }
            else
            {
                statusManager.playBlockingPattern(SD_READ_FAIL, 2000);
                LOG_ERROR("Test", "Failed to read back from SD card");
            }
        }
        else
        {
            statusManager.playBlockingPattern(SD_WRITE_FAIL, 2000);
            LOG_ERROR("Test", "SD card write test failed");
        }
        sdCard->closeFile();
    }
    else
    {
        statusManager.playBlockingPattern(SD_MOUNT_FAIL, 2000);
        LOG_ERROR("Test", "Failed to open test file on SD card");
    }

    return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
}

bool testFlashMemory()
{
    LOG_INFO("Test", "[STEP 5] Test Flash memory");

    flash = std::make_shared<Flash>();
    
    const uint32_t t0 = Utils::millis();

    if (flash && flash->init(board.get_spi_bus(), board.get_flash_cs_pin(), board.get_flash_hold_pin(), board.get_flash_wp_pin()))
    {
        LOG_INFO("Init", "External Flash initialized");
    }
    else
    {
        LOG_ERROR("Init", "Failed to initialize External Flash");
    }

    if (!flash) {
        LOG_ERROR("Test", "Flash pointer is null! Initialization failed in setup.");
        return waitForUserInput("Type PASSED to continue or FAILED to retry");
    }

    LOG_INFO("Test", "Flash: verifying readiness...");
    // Calling init() again is safe, it returns true if already initialized
    if (!flash->init())
    {
        LOG_ERROR("Test", "Flash init failed: verify external SPI flash wiring and availability.");
        return waitForUserInput("Type PASSED to continue or FAILED to retry");
    }
    LOG_INFO("Test", "Flash: init OK (%lu ms)", (unsigned long)(Utils::millis() - t0));

    const std::string testFile = "test.txt";
    const std::string missingFile = "ghost.txt";

    // Missing file read should return nullptr.
    LOG_INFO("Test", "Flash: read missing file '%s' (expected nullptr)", missingFile.c_str());
    char *readData = flash->readFile(missingFile.c_str());
    if (readData != nullptr)
    {
        LOG_ERROR("Test", "Unexpected data returned for missing file.");
        delete[] readData;
    }
    else
    {
        LOG_INFO("Test", "Flash: missing file check OK");
    }

    LOG_INFO("Test", "Flash: write '%s'", testFile.c_str());
    if (!flash->writeFile(testFile.c_str(), "Hello, ESP32 Flash Storage!\n"))
    {
        LOG_ERROR("Test", "Flash write failed.");
    }
    else
    {
        LOG_INFO("Test", "Flash: write OK");
    }

    LOG_INFO("Test", "Flash: read '%s'", testFile.c_str());
    readData = flash->readFile(testFile.c_str());
    if (readData == nullptr)
    {
        LOG_ERROR("Test", "Flash read failed after write.");
    }
    else
    {
        LOG_INFO("Test", "Flash read content: %s", readData);
        delete[] readData;
    }

    LOG_INFO("Test", "Flash: append to '%s'", testFile.c_str());
    if (!flash->appendFile(testFile.c_str(), "Appended line.\n"))
    {
        LOG_ERROR("Test", "Flash append failed.");
    }
    else
    {
        LOG_INFO("Test", "Flash: append OK");
    }

    LOG_INFO("Test", "Flash: read back after append");
    readData = flash->readFile(testFile.c_str());
    if (readData != nullptr)
    {
        LOG_INFO("Test", "Flash read after append: %s", readData);
        delete[] readData;
    }
    else
    {
        LOG_ERROR("Test", "Flash read failed after append.");
    }

    if (!flash->fileExists(testFile.c_str()))
    {
        LOG_ERROR("Test", "Flash file existence check failed.");
    }
    else
    {
        LOG_INFO("Test", "Flash: fileExists('%s') OK", testFile.c_str());
    }

    const uint32_t t_clear = Utils::millis();
    LOG_INFO("Test", "Flash: clear start (this can take several seconds on full-chip erase)...");
    
    if (!flash->clearFlash())
    {
        LOG_ERROR("Test", "Flash clear failed.");
    }
    else if (flash->fileExists(testFile.c_str()))
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

bool testTelemetry()
{
    LOG_INFO("Test", "[STEP 6] Test telemetria");
    statusManager.playBlockingPattern(TEST_TELEMETRY, 1000);

    // Serial1 is used by GPS
    E220LoRaTransmitter lora(Serial2, MANNY_LORA_TX_PIN, MANNY_LORA_RX_PIN, MANNY_LORA_AUX_PIN, MANNY_LORA_M0_PIN, MANNY_LORA_M1_PIN);

    LOG_INFO("Test", "Inizializzazione E220...");
    auto initResult = lora.init();
    if (initResult.getCode() != E220_SUCCESS)
    {
        LOG_ERROR("Test", "LoRa init fallita: %s", initResult.getDescription().c_str());
        // I know it's an obvious fail, but returning false would just end up in a loop
        return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
    }
    LOG_INFO("Test", "LoRa init OK");

    for (int i = 1; i <= 3; i++)
    {
        std::string payload = "LORA_TEST_" + std::to_string(i);
        LOG_INFO("Test", "Invio pacchetto %d: %s", i, payload.c_str());
        auto result = lora.transmit(payload);
        if (result.getCode() == E220_SUCCESS)
        {
            LOG_INFO("Test", "Pacchetto %d inviato.", i);
        }
        else
        {
            LOG_ERROR("Test", "Invio pacchetto %d fallito: %s", i, result.getDescription().c_str());
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    LOG_INFO("Test", "Verificare ricezione 3 pacchetti LORA_TEST_1/2/3 sulla ground station.");
    return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
}

bool testI2CScan()
{
    LOG_INFO("Test", "Scanning I2C bus...");

    // TODO: implement second bus
    I2CBus* bus = board.get_i2c_bus(IBoardHardware::Sensor::IMU);
    if (!bus) {
        LOG_ERROR("Test", "I2C bus not available.");
        return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
    }

    struct KnownDevice { const char* name; uint8_t addr; };
    // Device possible addresses from docs
    static const KnownDevice known[] = {
        { "BNO055 IMU", 0x28 },
        { "BNO055 IMU 2", 0x29 },
        { "LIS3DHTR", 0x18 },
        { "LIS3DHTR 2", 0x19 },
    };

    bool found = false;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        if (i2c_master_probe(*bus->get_handle(), addr, 10) == ESP_OK) {
            found = true;
            const char* label = nullptr;
            for (auto& d : known)
                if (d.addr == addr) { label = d.name; break; }
            if (label)
                printf("Found: 0x%02X %s\n", addr, label);
            else
                printf("Unknown: 0x%02X\n", addr);
        }
    }
    if (!found) printf("No devices found.\n");

    return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
}

bool configureE220()
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
    auto config = *(Configuration *)csc.data;
    csc.close();

    config.ADDL = 0x03;
    config.ADDH = 0x00;
    config.CHAN = 23;

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

// E220 connector test to identify the pins
//
// Pattern: All high, then N bursts of (0,5s H/L)
bool testE220Connector()
{
    LOG_INFO("Test", "[STEP 7] LoRa Connector Test");

    const gpio_num_t pins[]   = {GPIO_NUM_40, GPIO_NUM_39, GPIO_NUM_38, GPIO_NUM_41, GPIO_NUM_42};
    const char*      names[]  = {"GPIO40 AUX", "GPIO39 RX<E220TX", "GPIO38 TX>E220RX", "GPIO41 M1", "GPIO42 M0"};

    for (int i = 0; i < 5; i++) {
        // Release from JTAG function
        gpio_set_direction(pins[i], GPIO_MODE_OUTPUT);
        gpio_set_level(pins[i], 0);
    }

    LOG_INFO("Test", "1: All pins high 3s");
    for (int i = 0; i < 5; i++) gpio_set_level(pins[i], 1);
    vTaskDelay(pdMS_TO_TICKS(3000));
    for (int i = 0; i < 5; i++) gpio_set_level(pins[i], 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    int bursts = 1, b;
    for (int i = 0; i < 5; i++, bursts = i + 1) {
        LOG_INFO("Test", "%s: %d burst(s)", names[i], bursts);
        for (b = 0; b < bursts; b++) {
            gpio_set_level(pins[i], 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_set_level(pins[i], 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }

    // Reset pins for safety
    for (int i = 0; i < 5; i++) {
        gpio_reset_pin(pins[i]);
    }

    return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
}

// Main test routine
void testRoutine()
{
    LOG_INFO("Test", "=== SYSTEM TEST ROUTINE INITIATED ===");

    // Menu di selezione test
    while (true)
    {
        // Menu is BLUE with no buzzer
        statusManager.setSystemCode(TEST_MENU);

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
        printf("10 - Esegui tutti i test in sequenza\n");
        printf("0 - Esci dal menu test\n");
        printf("Inserisci il numero del test da eseguire:\n");

        char buffer[32] = {0};
        Utils::readLine(buffer, sizeof(buffer));
        std::string input(buffer);
        trimString(input);
        int choice = std::atoi(input.c_str());
        bool testPassed = false;

        // Show specific pattern for this test
        showTestPattern(choice, statusManager);

        switch (choice)
        {
        case 1:
            do
            {
                testPassed = testPowerAndLEDs();
            } while (!testPassed);
            break;
        case 2:
            do
            {
                testPassed = testSensors();
            } while (!testPassed);
            break;
        case 3:
            do
            {
                testPassed = testActuators();
            } while (!testPassed);
            break;
        case 4:
            do
            {
                testPassed = testSDCard();
            } while (!testPassed);
            break;
        case 5:
            do
            {
                testPassed = testI2CScan();
            } while (!testPassed);
            break;
        case 6:
            do
            {
                testPassed = configureE220();
            } while (!testPassed);
            break;
        case 7:
            do
            {
                testPassed = testTelemetry();
            } while (!testPassed);
            break;
        case 8:
            do
            {
                testPassed = testE220Connector();
            } while (!testPassed);
            break;
        case 9:
            do
            {
                testPassed = testFlashMemory();
            } while (!testPassed);
            break;
        case 10:
            // Show all tests pattern
            statusManager.playBlockingPattern(TEST_ALL, 2000);

            // Execute all tests in sequence
            do
            {
                testPassed = testPowerAndLEDs();
            } while (!testPassed);
            do
            {
                testPassed = testSensors();
            } while (!testPassed);
            do
            {
                testPassed = testActuators();
            } while (!testPassed);
            do
            {
                testPassed = testSDCard();
            } while (!testPassed);
            do
            {
                testPassed = testTelemetry();
            } while (!testPassed);
            do
            {
                testPassed = testFlashMemory();
            } while (!testPassed);

            // All tests successful
            statusManager.playBlockingPattern(TEST_SUCCESS, 2000);
            LOG_INFO("Test", "\n=== TUTTI I TEST COMPLETATI CON SUCCESSO ===");
            break;
        case 0:
            // Exit test mode with success pattern
            statusManager.playBlockingPattern(TEST_SUCCESS, 2000);
            LOG_INFO("Test", "\n=== USCITA DAL MENU TEST ===");
            statusManager.setSystemCode(SYSTEM_OK);
            return;
        default:
            printf("Scelta non valida. Riprova.\n");
            continue;
        }

        if (choice >= 1 && choice <= 10)
        {
            LOG_INFO("Test", "Test completato con successo!");
            // Show success pattern before returning to menu
            statusManager.playBlockingPattern(TEST_SUCCESS, 1000);
        }
    }
}

void printSystemInfo()
{
    printf("--- System Information ---\n");
    printf("ESP32 Chip: %s\n", ESP.getChipModel());
    printf("CPU Frequency: %lu MHz\n", (unsigned long)ESP.getCpuFreqMHz());
    printf("Total Heap: %lu bytes\n", (unsigned long)ESP.getHeapSize());
    printf("Free Heap: %lu bytes\n", (unsigned long)ESP.getFreeHeap());
    printf("PSRAM Total: %lu bytes\n", (unsigned long)ESP.getPsramSize());
    printf("PSRAM Free: %lu bytes\n", (unsigned long)ESP.getFreePsram());
    printf("Flash Size: %lu bytes\n", (unsigned long)ESP.getFlashChipSize());
    printf("SDK Version: %s\n", ESP.getSdkVersion());

    // FreeRTOS information
    printf("FreeRTOS running on %d cores\n", portNUM_PROCESSORS);
    printf("Tick rate: %d Hz\n", configTICK_RATE_HZ);
    printf("--- End System Information ---\n");
}

// The ESP-IDF entry point, which must be C-linkage
extern "C" void app_main() {
    // Initialize the Arduino core background tasks
    initArduino();
    
    setup();
    
    while (1) {
        loop();
    }
}
