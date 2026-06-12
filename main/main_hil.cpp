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
#include "lwip/sockets.h"
#include "lwip/ip4_addr.h"
#include "lwip/inet.h"

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
#include <E220LoRaTransmitter.hpp>

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
void printSystemInfo();
void wifi_softap_init(void);
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);

// HIL lifecycle helpers
static void createAndStartFSM();
static void resetHilSimulationIfRequested();
static void resetHilSimulation();

#if CONFIG_AURORA_HIL_SIMULATION

static constexpr const char *HIL_WIFI_SSID = CONFIG_AURORA_HIL_WIFI_SSID;
static constexpr const char *HIL_WIFI_PASSWORD = CONFIG_AURORA_HIL_WIFI_PASSWORD;
static constexpr int HIL_WIFI_CHANNEL = CONFIG_AURORA_HIL_WIFI_CHANNEL;
static constexpr int HIL_MAX_STA_CONN = CONFIG_AURORA_HIL_MAX_STA_CONN;

static constexpr const char *HIL_AP_IP_ADDR = CONFIG_AURORA_HIL_AP_IP_ADDR;
static constexpr const char *HIL_AP_NETMASK = CONFIG_AURORA_HIL_AP_NETMASK;

static_assert(sizeof(CONFIG_AURORA_HIL_WIFI_SSID) > 1,
              "CONFIG_AURORA_HIL_WIFI_SSID must not be empty");

static_assert(
    sizeof(CONFIG_AURORA_HIL_WIFI_PASSWORD) == 1 ||
    sizeof(CONFIG_AURORA_HIL_WIFI_PASSWORD) >= 9,
    "CONFIG_AURORA_HIL_WIFI_PASSWORD must be empty or at least 8 characters"
);

#endif

void setup()
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

    wifi_softap_init();
    LOG_INFO("Main", "WiFi soft AP ready...");

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

    rocketFSM = std::make_unique<RocketFSM>(rocketModel, sdCard, logger, &board);
    rocketFSM->init();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Start FSM tasks
    statusManager.setSystemCode(FSM_STARTED);
    
    LOG_INFO("Main", "Force transition - READY_FOR_LAUNCH");
    rocketFSM->forceTransition(RocketState::READY_FOR_LAUNCH);
    
    LOG_INFO("Main", "Starting Flight State Machine...");
    rocketFSM->start();
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

void loop()
{
    resetHilSimulationIfRequested();

    static unsigned long lastHeartbeat = 0;
    static bool ledState = false;
    static RocketState lastLoggedState = RocketState::INACTIVE;

    if (!rocketFSM)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        return;
    }

    // Heartbeat every 2 seconds
    if (Utils::realMillis() - lastHeartbeat > 2000)
    {
        lastHeartbeat = Utils::realMillis();

        LOG_INFO("Main", "Last heartbeat at %lu ms - System running", Utils::realMillis());

        ledState = !ledState;
        gpio_set_level(board.get_rgb_blue_pin(), ledState);

        LOG_INFO("Main", "Free heap: %u bytes", ESP.getFreeHeap());

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
        RocketState currentState = rocketFSM->getCurrentState();

        // if (currentState != lastLoggedState)
        // {
        LOG_INFO("Main", "Current FSM State: %s", rocketFSM->getStateString(currentState));
            // lastLoggedState = currentState;
        // }
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

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED)
    {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        LOG_INFO("wifi_softap", "station connected, aid=%d", event->aid);
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED)
    {
        wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
        LOG_INFO("wifi_softap", "station disconnected, aid=%d", event->aid);
    }
}

void wifi_softap_init(void)
{
    // esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     ret = nvs_flash_init();
    // }
    // ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *ap_netif = esp_netif_create_default_wifi_ap();
    if (ap_netif == nullptr)
    {
        LOG_ERROR("wifi_softap", "Failed to create default WiFi AP netif");
        return;
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register WiFi events
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    // Register IP event
    // ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &ip_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {};
    wifi_config.ap.ssid_len = strlen(HIL_WIFI_SSID);
    wifi_config.ap.channel = HIL_WIFI_CHANNEL;
    wifi_config.ap.max_connection = HIL_MAX_STA_CONN;

    std::strncpy(reinterpret_cast<char *>(wifi_config.ap.ssid),
            HIL_WIFI_SSID,
            sizeof(wifi_config.ap.ssid) - 1);

    std::strncpy(reinterpret_cast<char *>(wifi_config.ap.password),
            HIL_WIFI_PASSWORD,
            sizeof(wifi_config.ap.password) - 1);

    if (strlen(HIL_WIFI_PASSWORD) == 0)
    {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    else
    {
        wifi_config.ap.authmode = WIFI_AUTH_WPA2_WPA3_PSK;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));

    esp_netif_ip_info_t ip_info = {};

    if (esp_netif_str_to_ip4(HIL_AP_IP_ADDR, &ip_info.ip) != ESP_OK)
    {
        LOG_ERROR("wifi_softap", "Invalid HIL AP IP address: %s", HIL_AP_IP_ADDR);
        return;
    }

    // In SoftAP mode, the ESP32 itself is also the gateway.
    if (esp_netif_str_to_ip4(HIL_AP_IP_ADDR, &ip_info.gw) != ESP_OK)
    {
        LOG_ERROR("wifi_softap", "Invalid HIL AP gateway address: %s", HIL_AP_IP_ADDR);
        return;
    }

    if (esp_netif_str_to_ip4(HIL_AP_NETMASK, &ip_info.netmask) != ESP_OK)
    {
        LOG_ERROR("wifi_softap", "Invalid HIL AP netmask: %s", HIL_AP_NETMASK);
        return;
    }

    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    LOG_INFO("wifi_softap",
            "SoftAP started. SSID:%s IP:%s NETMASK:%s CHANNEL:%d MAX_STA:%d",
            HIL_WIFI_SSID,HIL_AP_IP_ADDR,HIL_AP_NETMASK,HIL_WIFI_CHANNEL,HIL_MAX_STA_CONN);
}

// The ESP-IDF entry point, which must be C-linkage
extern "C" void app_main()
{
    // Initialize the Arduino core background tasks
    initArduino();

    setup();

    while (1)
    {
        loop();
    }
}