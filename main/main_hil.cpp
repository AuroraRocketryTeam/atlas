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
#include <esp_err.h>
#include <esp_heap_caps.h>

// Configuration and pins
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include <config.h>
#include <pins.h>
#include <board.h>
#include <utils.h>

// Interfaces
#include <ISensor.hpp>
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
#include <SerialLogger.hpp>
#include <PayloadSerializer.hpp>

// Controllers and filters
#include <LEDController.hpp>
#include <BuzzerController.hpp>
#include <StatusManager.hpp>

// Main system
#include <RocketFSM.hpp>
#include <RuntimeConfig.hpp>
#include <E220LoRaTransmitter.hpp>
#include <TestRoutine.hpp>

// Board hardware instance
static Board board;

// Create controller instances
static LEDController ledController(board.get_rgb_red_pin(), board.get_rgb_green_pin(), board.get_rgb_blue_pin());
static BuzzerController buzzerController(board.get_buzzer_pin());
static StatusManager statusManager(ledController, buzzerController);

// Define the system model
static std::shared_ptr<RocketModel> rocketModel = nullptr;

static std::shared_ptr<SD> sdCard = nullptr;
static std::shared_ptr<Flash> flash = nullptr;

// Define the RocketLogger
static std::shared_ptr<RocketLogger> logger = nullptr;

// FSM instance
static std::unique_ptr<RocketFSM> rocketFSM;
static std::shared_ptr<TestRoutine> testRoutine = nullptr;

// Utility functions
void printSystemInfo();

// HIL lifecycle helpers
static void createAndStartFSM();
static void resetHilSimulationIfRequested();
static void resetHilSimulation();

void setupHil()
{
    // Install driver for blocking reads of Utils::readLine
    // Regular console output already works via the vfs bound by CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    usb_serial_jtag_driver_config_t usb_cfg = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 1024,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));

    // Signal initialization start
    board.init();

    gpio_set_level(board.get_rgb_blue_pin(), LOW);
    gpio_set_level(board.get_rgb_red_pin(), HIGH);

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

    // Initialize NVS
    ESP_ERROR_CHECK(board.initNvs() ? ESP_OK : ESP_FAIL);
    ESP_ERROR_CHECK(runtime_config_init(&board));

    // Initialize components
    // LOG_INFO("Main", "Initializing sensors...");
    std::shared_ptr<BNO055Sensor> bno055 = nullptr;
    std::shared_ptr<LIS3DHTRSensor> accl = nullptr;
    std::shared_ptr<MS561101BA03> baro1 = nullptr;
    std::shared_ptr<MS561101BA03> baro2 = nullptr;
    std::shared_ptr<GPS> gps = nullptr;
    // initializeComponents(bno055, accl, baro1, baro2, gps);
    LOG_INFO("Main", "Initialize components skipped");

    // Initialize logger
    LOG_INFO("Init", "Initializing rocket logger...");
    logger = std::make_shared<RocketLogger>();
    logger->setSerializer(PayloadSerializers::toJson);
    LOG_INFO("Init", "Rocket logger initialized");

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

    // Create Nemesis instance (constructor expects: bno, lis3dh, ms56_1, ms56_2, gps, sdCard, flash)
    rocketModel = std::make_shared<RocketModel>(bno055, accl, baro1, baro2, gps, sdCard, flash);
    LOG_INFO("Main", "RocketModel system model created");

    testRoutine = std::make_shared<TestRoutine>(
        board,
        rocketModel,
        sdCard,
        flash,
        statusManager,
        ledController,
        buzzerController);

    // Print system information
    printSystemInfo();

    // Switch to simulation time.
    // Utils::setTimeSource(TimeSource::SIMULATION);
    Utils::setSimMillis(0);

    // Initialize and start FSM
    LOG_INFO("Main", "=== System initialization complete ===");

    // Wait for arming pin to be enabled before starting FSM
    statusManager.setSystemCode(PRE_FLIGHT_MODE);
    while (!board.is_armed())
    {
        LOG_WARNING("Main", "System not armed! Waiting for arming signal...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    createAndStartFSM();

    statusManager.setSystemCode(FLIGHT_MODE);

    // Signal successful initialization
    gpio_set_level(board.get_rgb_red_pin(), LOW);
    gpio_set_level(board.get_rgb_green_pin(), HIGH);
    LOG_INFO("Main", "SETUP COMPLETE - SYSTEM IN FLIGHT MODE");
}

static void createAndStartFSM()
{
    LOG_INFO("Main", "\n=== Initializing Flight State Machine ===");

    rocketFSM = std::make_unique<RocketFSM>(rocketModel, sdCard, logger, &board, testRoutine);
    rocketFSM->init();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Start FSM tasks
    LOG_INFO("Main", "Starting Flight State Machine...");
    rocketFSM->start();
    statusManager.setSystemCode(FSM_STARTED);
}

static void resetHilSimulationIfRequested()
{
    if (!rocketModel)
    {
        return;
    }

    if (!rocketModel->getResetSimulationFlag())
    {
        return;
    }

    resetHilSimulation();
}

static void resetHilSimulation()
{
    LOG_WARNING("Main", "HIL reset requested. Reinitializing FSM runtime...");

    statusManager.setSystemCode(PRE_FLIGHT_MODE);

    // Destroy the current FSM.
    //
    // Important:
    // RocketFSM / TaskManager destructors must stop all running FreeRTOS tasks
    // before task objects are destroyed.
    if (rocketFSM)
    {
        LOG_INFO("Main", "Destroying current RocketFSM instance");
        rocketFSM.reset();
        vTaskDelay(250 / portTICK_PERIOD_MS);
    }

    // Reset simulation time before starting the next run.
    Utils::setSimMillis(0);

    // Reset logical model state.
    //
    // RocketModel::reset() must clear the reset flag, otherwise this function
    // will be called again on every loop iteration.
    if (rocketModel)
    {
        LOG_INFO("Main", "Resetting RocketModel");
        rocketModel->reset();
    }

    // Recreate a fresh FSM/TaskManager/task object graph.
    createAndStartFSM();

    statusManager.setSystemCode(FLIGHT_MODE);

    gpio_set_level(board.get_rgb_red_pin(), LOW);
    gpio_set_level(board.get_rgb_green_pin(), HIGH);

    LOG_INFO("Main", "HIL simulation reset complete");
}

void loopHil()
{
    resetHilSimulationIfRequested();

    static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 5000;
    static unsigned long lastHeartbeat = 0;
    static unsigned long lastStateLog = 0;
    static bool ledState = false;
    static RocketState lastLoggedState = RocketState::INACTIVE;
    static bool stateLogged = false;

    // Heartbeat every HEARTBEAT_INTERVAL_MS
    if (Utils::realMillis() - lastHeartbeat > HEARTBEAT_INTERVAL_MS)
    {
        LOG_INFO("Main", "Last heartbeat at %lu ms - System running", Utils::realMillis());
        lastHeartbeat = Utils::realMillis();
        ledState = !ledState;
        gpio_set_level(board.get_rgb_blue_pin(), ledState);

        LOG_INFO("Main", "Free heap: %u bytes", ESP.getFreeHeap());
        const uint32_t internalCaps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
        LOG_INFO("Main", "Health: internal free=%u min=%u largest=%u tasks=%u",
                 static_cast<unsigned>(heap_caps_get_free_size(internalCaps)),
                 static_cast<unsigned>(heap_caps_get_minimum_free_size(internalCaps)),
                 static_cast<unsigned>(heap_caps_get_largest_free_block(internalCaps)),
                 static_cast<unsigned>(uxTaskGetNumberOfTasks()));
        if (rocketFSM) rocketFSM->logActiveTaskStackHealth();

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

    }

    const unsigned long now = Utils::realMillis();
    const RocketState currentState = rocketFSM->getCurrentState();
    if (!stateLogged || currentState != lastLoggedState || now - lastStateLog >= 30000)
    {
        LOG_INFO("Main", "Current FSM State: %s", rocketFSM->getStateString(currentState));
        lastLoggedState = currentState;
        lastStateLog = now;
        stateLogged = true;
    }

    // Small delay to prevent watchdog issues
    vTaskDelay(100 / portTICK_PERIOD_MS);
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
