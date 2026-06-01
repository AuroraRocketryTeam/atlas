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
#include <TestRoutine.hpp>

/**
 * @brief Uncomment to enable sensor calibration routine at startup.
 * Useful for fast testing without needing precise sensor reads.
 *
 */
#define CALIBRATE_SENSORS
#define ENABLE_TEST_ROUTINE

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
void initializeComponents(std::shared_ptr<BNO055Sensor>& bno055,
                          std::shared_ptr<LIS3DHTRSensor>& accl,
                          std::shared_ptr<MS561101BA03>& baro1,
                          std::shared_ptr<MS561101BA03>& baro2,
                          std::shared_ptr<GPS>& gps);
void GPSfix(std::shared_ptr<GPS> gps);

void setup()
{
    // Actuator pins are configured in board.init()

    // Initialize LED pins (only those not handled by controllers)
    gpio_config(&led_gpio_config);

    gpio_set_level(LED_BUILT_IN, LOW);
    gpio_set_level(LED_RED_PIN, HIGH);

#ifdef CONFIG_INSTALL_USB_JTAG_DRIVER
    // I/O becomes blocking and buffer is limited
    usb_serial_jtag_driver_config_t usb_cfg = { .tx_buffer_size = 1024, .rx_buffer_size = 1024 };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));
#endif

    // Signal initialization start
    board.init();
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
    LOG_INFO("Main", "=== TEST MODE ENABLED ===");
    TestRoutine tests(board, rocketModel, sdCard, flash, statusManager, ledController, buzzerController);
    tests.run();
#endif

#ifdef CALIBRATE_SENSORS
    // Checking sensors calibration
    statusManager.setSystemCode(CALIBRATING);
    GPSfix(gps);
    statusManager.setSystemCode(SYSTEM_OK);
#endif
    // Print system information
    TestRoutine::printSystemInfo();

    // Initialize and start FSM
    LOG_INFO("Main", "=== System initialization complete ===");
    LOG_INFO("Main", "\n=== Initializing Flight State Machine ===");
    rocketFSM = std::make_unique<RocketFSM>(rocketModel, sdCard, logger, &board);
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
    if (board.get_gps_tx_pin() == GPIO_NUM_NC || board.get_gps_rx_pin() == GPIO_NUM_NC)
    {
        LOG_WARNING("Init", "GPS not configured");
    }
    else
    {
        gps = std::make_shared<GPS>(board.get_gps_tx_pin(), board.get_gps_rx_pin());
        if (gps->init())
        {
            LOG_INFO("Init", "GPS initialized");
        }
        else
        {
            LOG_ERROR("Init", "Failed to initialize GPS");
        }
    }

    // Initialize SD card
    if (board.get_sd_cs_pin() == GPIO_NUM_NC)
    {
        LOG_WARNING("Init", "SD card not available on this board (CS pin not connected)");
    }
    else
    {
        LOG_INFO("Init", "Initializing SD card for logging...");
        sdCard = std::make_shared<SD>();
        if (sdCard->init(board.get_spi_bus(), board.get_sd_cs_pin()))
        {
            LOG_ERROR("Init", "SD card write test failed");
        }
        sdCard->closeFile();
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
                auto fixType = gpsData.fixType;
                auto satellites = gpsData.satellites;

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

// The ESP-IDF entry point, which must be C-linkage
extern "C" void app_main() {
    // Initialize the Arduino core background tasks
    initArduino();
    
    setup();
    
    while (1) {
        loop();
    }
}