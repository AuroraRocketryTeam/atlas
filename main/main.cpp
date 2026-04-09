// Standard libraries
#include "driver/gpio.h"
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
#include <RocketLogger.hpp>
#include <Logger.hpp>

// Controllers and filters
#include <LEDController.hpp>
#include <BuzzerController.hpp>
#include <StatusManager.hpp>

// Main system
#include <RocketFSM.hpp>

/**
 * @brief Uncomment to enable sensor calibration routine at startup.
 * Useful for fast testing without needing precise sensor reads.
 *
 */
#define CALIBRATE_SENSORS
#define ENABLE_TEST_ROUTINE
#define TEST_FILE "/test.txt"

// Create controller instances
LEDController ledController(LED_RED_PIN, LED_GREEN_PIN, LED_BLUE_PIN);
BuzzerController buzzerController(BUZZER_PIN);
StatusManager statusManager(ledController, buzzerController);

// Board hardware instance
Board board;

// Define the system model
std::shared_ptr<RocketModel> rocketModel = nullptr;

// Type definitions
using TransmitDataType = std::variant<char *, std::string, nlohmann::json>;

std::shared_ptr<SD> sdCard = nullptr;

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

    // uart(PORT_NO, RX_BUF_SIZE, TX_BUF_SIZE, QUEUE_SIZE, &uart_queue, INTERRUPT_FLAGS);
    ESP_ERROR_CHECK(uart_driver_install((uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));

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

    // Create Nemesis instance (constructor expects: logger, bno, lis3dh, ms56_1, ms56_2, gps)
    rocketModel = std::make_shared<RocketModel>(logger, bno055, accl, baro1, baro2, gps);
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
    bno055 = std::make_shared<BNO055Sensor>();
    if (bno055 && bno055->init())
    {
        LOG_INFO("Init", "BNO055 (IMU) initialized");
    }
    else
    {
        LOG_ERROR("Init", "Failed to initialize BNO055");
    }

    // Initialize barometers
    baro1 = std::make_shared<MS561101BA03>(board.get_i2c_bus(IBoardHardware::Sensor::BARO1), MS56_I2C_ADDR_1);
    if (baro1 && baro1->init())
    {
        LOG_INFO("Init", "Barometer 1 initialized");
    }
    else
    {
        LOG_ERROR("Init", "Failed to initialize Barometer 1");
    }

    baro2 = std::make_shared<MS561101BA03>(board.get_i2c_bus(IBoardHardware::Sensor::BARO2), MS56_I2C_ADDR_2);
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
    if (sdCard && sdCard->init())
    {
        LOG_INFO("Init", "SD card initialized");
        sdCard->openFile("test.txt");
        LOG_INFO("Init", "Testing SD card write...");
        std::string content = "SD card write test successful! Timestamp: " + std::to_string(Utils::millis()) + " ms";
        if (sdCard->writeFile("test.txt", content))
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
        statusManager.playBlockingPattern(TEST_TELEMETRY, 1000);
        break;
    case 6:
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
    printf("%s\n Or type REBOOT to restart the system.\n", message);
    statusManager.setSystemCode(WAITING_INPUT);

    while (true)
    {
        char buffer[64] = {0};
        if (fgets(buffer, sizeof(buffer), stdin) != nullptr)
        {
            std::string input(buffer);
            trimString(input);
            toUpperString(input);

            if (input == "PASSED")
            {
                statusManager.playBlockingPattern(TEST_SUCCESS, 1000);
                return true;
            }
            if (input == "FAILED")
            {
                statusManager.playBlockingPattern(TEST_FAILURE, 1000);
                return false;
            }
            if (input == "REBOOT")
            {
                LOG_WARNING("Test", "System is going to reboot, are you sure?");
                LOG_WARNING("Test", "Type REBOOT to confirm or anything else to cancel.");

                char confirmBuffer[64] = {0};
                std::string confirm;
                if (fgets(confirmBuffer, sizeof(confirmBuffer), stdin) != nullptr)
                {
                    confirm.assign(confirmBuffer);
                }

                if (confirm.empty())
                {
                    LOG_INFO("Test", "No confirmation - continuing normal operation.");
                    continue;
                }
                trimString(confirm);
                toUpperString(confirm);
                if (confirm == "REBOOT")
                {
                    LOG_WARNING("Test", "Rebooting system...");
                    ESP.restart();
                }
            }
            else
            {
                LOG_INFO("Test", "Reboot cancelled.");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// Test routine subroutines with proper pattern handling
bool testPowerAndLEDs()
{
    LOG_INFO("Test", "\n[STEP 1] Verifica alimentazione e LED di stato");
    LOG_INFO("Test", "Controllare manualmente:");
    LOG_INFO("Test", " - LED di alimentazione componenti accesi");
    LOG_INFO("Test", " - LED presenza SD acceso");
    LOG_INFO("Test", " - LED attuatori visibili");
    return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
}

bool testSensors()
{
    LOG_INFO("Test", "\n[STEP 2] Test sensori");
    bool testFailed = false;
    bool imu_ok = rocketModel->updateBNO055();
    bool baro1_ok = rocketModel->updateMS561101BA03_1();
    bool baro2_ok = rocketModel->updateMS561101BA03_2();
    bool accl_ok = rocketModel->updateLIS3DHTR();

    if (!imu_ok)
    {
        testFailed = true;
        statusManager.playBlockingPattern(IMU_FAIL, 2000);
        LOG_ERROR("Test", "Errore: IMU non inizializzata.");
    }
    if (!baro1_ok)
    {
        statusManager.playBlockingPattern(BARO1_FAIL, 2000);
        LOG_ERROR("Test", "Errore: Barometro 1 non inizializzato.");
    }
    if (!baro2_ok)
    {
        statusManager.playBlockingPattern(BARO2_FAIL, 2000);
        LOG_ERROR("Test", "Errore: Barometro 2 non inizializzato.");
    }
    if (!accl_ok)
    {
        statusManager.playBlockingPattern(IMU_FAIL, 2000);
        LOG_ERROR("Test", "Errore: Accelerometro non inizializzato.");
    }

    if (!testFailed)
    {
        LOG_INFO("Test", "Verifica output dei sensori...");

        board.init_sensor_test_pins();

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

        // Testing barometers
        auto baro1Data = rocketModel->getMS561101BA03Data_1();
        auto pressureBaro1 = baro1Data->pressure;
        LOG_INFO("Test", "Barometer 1 Pressure: %.2f hPa", (double)pressureBaro1);
        // board.signal_sensor_ok(IBoardHardware::Sensor::BARO1);

        auto baro2Data = rocketModel->getMS561101BA03Data_2();
        auto pressureBaro2 = baro2Data->pressure;
        LOG_INFO("Test", "Barometer 2 Pressure: %.2f hPa", (double)pressureBaro2);
        // board.signal_sensor_ok(IBoardHardware::Sensor::BARO2);

        // Testing LIS3DHTR accelerometer
        LOG_INFO("Test", "LIS3DHTR Accelerometer: Data logged internally");
        board.signal_sensor_ok(IBoardHardware::Sensor::ACC);
    }

    // After all tests, go to user input
    return waitForUserInput("Scrivi PASSED per continuare o FAILED per ripetere");
}

bool testActuators()
{
    LOG_INFO("Test", "\n[STEP 3] Test attuatori");

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
        if (sdCard->writeFile(TEST_FILE, content))
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

bool testTelemetry()
{
    LOG_INFO("Test", "\n[STEP 5] Test telemetria");
    LOG_INFO("Test", "Inizializzazione antenne e verifica collegamento...");
    LOG_INFO("Test", "Verificare sul monitor ricezione pacchetti LORA.");

    // Test LORA patterns
    statusManager.playBlockingPattern(TEST_TELEMETRY, 1000);

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
        printf("5 - Test telemetria\n");
        printf("6 - Esegui tutti i test in sequenza\n");
        printf("7 - Esci dal menu test\n");
        printf("Inserisci il numero del test da eseguire:\n");

        char buffer[32] = {0};
        fgets(buffer, sizeof(buffer), stdin);
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
                testPassed = testTelemetry();
            } while (!testPassed);
            break;
        case 6:
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

            // All tests successful
            statusManager.playBlockingPattern(TEST_SUCCESS, 2000);
            LOG_INFO("Test", "\n=== TUTTI I TEST COMPLETATI CON SUCCESSO ===");
            break;
        case 7:
            // Exit test mode with success pattern
            statusManager.playBlockingPattern(TEST_SUCCESS, 2000);
            LOG_INFO("Test", "\n=== USCITA DAL MENU TEST ===");
            statusManager.setSystemCode(SYSTEM_OK);
            return;
        default:
            printf("Scelta non valida. Riprova.\n");
            continue;
        }

        if (choice >= 1 && choice <= 6)
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
