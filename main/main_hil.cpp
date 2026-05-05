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

// Define the RocketLogger
std::shared_ptr<RocketLogger> logger = nullptr;

// FSM instance
std::unique_ptr<RocketFSM> rocketFSM;

// Utility functions
void printSystemInfo();    
void wifi_softap_init(void);
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

#define ESP_WIFI_SSID "myssid"
#define ESP_WIFI_CHANNEL 1
#define ESP_WIFI_PASSWORD "mypassword" 
#define ESP_MAX_STA_CONN 1


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
    LOG_INFO("Init", "Rocket logger initialized");

    // Create Nemesis instance (constructor expects: logger, bno, lis3dh, ms56_1, ms56_2, gps)
    rocketModel = std::make_shared<RocketModel>(logger, bno055, accl, baro1, baro2, gps);
    LOG_INFO("Main", "RocketModel system model created");

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
    
    LOG_INFO("Main", "Force transition - READY_FOR_LAUNCH");
    rocketFSM->forceTransition(RocketState::READY_FOR_LAUNCH);
    
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

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
        LOG_INFO("wifi_softap", "station connected, aid=%d", event->aid);
    }

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
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

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register WiFi events
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    // Register IP event
    // ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &ip_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_WIFI_SSID,
            .password = ESP_WIFI_PASSWORD,
            .ssid_len = strlen(ESP_WIFI_SSID),
            .channel = ESP_WIFI_CHANNEL,
            .authmode = WIFI_AUTH_WPA2_WPA3_PSK,
            .max_connection = ESP_MAX_STA_CONN,
        },
    };

    if (strlen(ESP_WIFI_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    ESP_ERROR_CHECK(esp_netif_dhcps_stop(ap_netif));

    esp_netif_ip_info_t ip_info;
    IP4_ADDR(&ip_info.ip, 192, 168, 42, 1);
    IP4_ADDR(&ip_info.gw, 192, 168, 42, 1);
    IP4_ADDR(&ip_info.netmask, 255, 255, 255, 0);

    ESP_ERROR_CHECK(esp_netif_set_ip_info(ap_netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(ap_netif));

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    LOG_INFO("wifi_softap", "SoftAP started. SSID:%s", ESP_WIFI_SSID);
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
