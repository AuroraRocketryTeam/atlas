#pragma once

#include "boards_hardware/IBoardHardware.hpp"
#include "esp_netif.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstdint>

// Manny board hardware map
static constexpr gpio_num_t MANNY_I2C_SDA_PIN = GPIO_NUM_11;
static constexpr gpio_num_t MANNY_I2C_SCL_PIN = GPIO_NUM_12;
static constexpr i2c_port_t MANNY_I2C_PORT = I2C_NUM_0;

static constexpr gpio_num_t MANNY_DROGUE_ACTUATOR_PIN = GPIO_NUM_5;
static constexpr gpio_num_t MANNY_MAIN_ACTUATOR_PIN = GPIO_NUM_4;

static constexpr gpio_num_t MANNY_BUZZER_PIN = GPIO_NUM_21;

static constexpr gpio_num_t MANNY_LED_RED_PIN = GPIO_NUM_18;
static constexpr gpio_num_t MANNY_LED_GREEN_PIN = GPIO_NUM_8;
static constexpr gpio_num_t MANNY_LED_BLUE_PIN = GPIO_NUM_7;
static constexpr gpio_num_t MANNY_LED_BUILTIN_PIN = MANNY_LED_BLUE_PIN;

static constexpr gpio_num_t MANNY_FLASH_HOLD_PIN = GPIO_NUM_9;
static constexpr gpio_num_t MANNY_FLASH_CS_PIN = GPIO_NUM_10;
static constexpr gpio_num_t MANNY_FLASH_WP_PIN = GPIO_NUM_14;

static constexpr gpio_num_t MANNY_SPI_MOSI_PIN = GPIO_NUM_37;
static constexpr gpio_num_t MANNY_SPI_MISO_PIN = GPIO_NUM_13;
static constexpr gpio_num_t MANNY_SPI_CLK_PIN = GPIO_NUM_17;

static constexpr gpio_num_t MANNY_BAROMETER_CS_PIN = GPIO_NUM_6;
static constexpr gpio_num_t MANNY_BAROMETER2_CS_PIN = GPIO_NUM_NC;

static constexpr gpio_num_t MANNY_BATTERY_DIVIDER_ADC_PIN = GPIO_NUM_1;
static constexpr int MANNY_BNO055_I2C_ADDRESS = 0x29;

static constexpr gpio_num_t MANNY_SD_CS_PIN = GPIO_NUM_NC;
static constexpr gpio_num_t MANNY_ARMING_PIN = GPIO_NUM_NC;

static constexpr gpio_num_t MANNY_GPS_TX_PIN = GPIO_NUM_NC;
static constexpr gpio_num_t MANNY_GPS_RX_PIN = GPIO_NUM_NC;

static constexpr gpio_num_t MANNY_LORA_AUX_PIN = GPIO_NUM_40;
static constexpr gpio_num_t MANNY_LORA_RX_PIN  = GPIO_NUM_39;
static constexpr gpio_num_t MANNY_LORA_TX_PIN  = GPIO_NUM_38;
static constexpr gpio_num_t MANNY_LORA_M1_PIN  = GPIO_NUM_41;
static constexpr gpio_num_t MANNY_LORA_M0_PIN  = GPIO_NUM_42;

// Hardware note: main and drogue outputs are valid only when board supply is 6.4V.

/**
 * @brief Board hardware implementation for the Manny flight computer.
 *
 * MannyBoard owns the board-specific pin map, shared I2C/SPI buses, and common
 * ESP service initialization. Callers should use this class instead of calling
 * ESP-IDF WiFi, NVS, or networking setup directly so dependencies are prepared
 * once and in the expected order.
 */
class MannyBoard : public IBoardHardware {
public:
    static constexpr const char* BOARD_NAME = "Manny";

private:
    bool is_initialized;

    // Shared-service state used to make initialization calls idempotent.
    //
    // In HIL builds GroundServicesTask and HilSimulationTask both acquire the
    // SoftAP from their own FreeRTOS task, on different cores, at the same
    // time. Every function that reads or writes the fields below takes this
    // recursive mutex so the reference count and the underlying ESP-IDF WiFi
    // setup stay consistent. It is recursive because startWifiSoftAp() calls
    // initNetworking(), which calls initNvs().
    SemaphoreHandle_t shared_service_mutex;
    bool nvs_initialized;
    bool networking_initialized;
    bool wifi_started;
    uint32_t wifi_softap_users;
    esp_netif_t* sta_netif;
    esp_netif_t* ap_netif;
    char wifi_ip_address[16];
    char wifi_mac_address[18];
    I2CBus _i2c_bus;
    SPIBus _spi_bus;
    ISPIHandler* spi_handler;

public:
    MannyBoard();
    ~MannyBoard() override;

    void init() override;

    const char* get_board_name() const override;
    gpio_num_t get_i2c_sda_pin() const override;
    gpio_num_t get_i2c_scl_pin() const override;

    gpio_num_t get_gps_tx_pin() const override;
    gpio_num_t get_gps_rx_pin() const override;
    gpio_num_t get_arming_pin() const override;

    gpio_num_t get_flash_cs_pin() const override;
    gpio_num_t get_flash_hold_pin() const override;
    gpio_num_t get_flash_wp_pin() const override;
    gpio_num_t get_spi_miso_pin() const override;
    gpio_num_t get_spi_mosi_pin() const override;
    gpio_num_t get_spi_clk_pin() const override;
    gpio_num_t get_barometer_cs_pin() const override;

    gpio_num_t get_buzzer_pin() const override;
    gpio_num_t get_rgb_red_pin() const override;
    gpio_num_t get_rgb_green_pin() const override;
    gpio_num_t get_rgb_blue_pin() const override;
    gpio_num_t get_main_actuator_pin() const override;
    gpio_num_t get_drogue_actuator_pin() const override;

    gpio_num_t get_battery_divider_adc_pin() const override;
    int get_bno055_i2c_address() const override;

    ISPIHandler* get_spi_handler() const override;
    void set_spi_handler(ISPIHandler* handler) override;

    gpio_num_t get_lora_cs_pin() const override;
    gpio_num_t get_lora_aux_pin() const override;
    gpio_num_t get_lora_m0_pin() const override;
    gpio_num_t get_lora_m1_pin() const override;
    gpio_num_t get_lora_tx_pin() const override;
    gpio_num_t get_lora_rx_pin() const override;

    gpio_num_t get_barometer2_cs_pin() const override;
    gpio_num_t get_sd_cs_pin() const override;

    bool is_armed() const override;

    /**
     * @brief Initialize and cache readiness for the default NVS partition.
     */
    bool initNvs() override;

    /**
     * @brief Report whether NVS was successfully initialized.
     */
    bool isNvsReady() const override;

    /**
     * @brief Initialize NVS, esp-netif, the default event loop, and WiFi driver.
     */
    bool initNetworking() override;

    /**
     * @brief Start Manny WiFi in station mode.
     */
    bool startWifiSta() override;

    /**
     * @brief Acquire Manny WiFi SoftAP using sdkconfig values.
     *
     * SoftAP users are reference counted so HIL and Ground Services can overlap
     * without one task stopping WiFi while the other still needs it.
     */
    bool startWifiSoftAp() override;

    /**
     * @brief Release one Manny WiFi user and stop WiFi when unused.
     */
    bool stopWifi() override;

    /**
     * @brief Return true while Manny WiFi is started.
     */
    bool isWifiConnected() const override;

    /**
     * @brief Return the current board-managed WiFi IP address.
     */
    const char* getWifiIpAddress() const override;

    /**
     * @brief Return the current board-managed WiFi MAC address.
     */
    const char* getWifiMacAddress() const override;

    /**
     * @brief Initialize Bluetooth services when Manny needs them.
     */
    bool initBluetooth() override;

    /**
     * @brief Stop Bluetooth services when Manny needs them.
     */
    bool stopBluetooth() override;

    I2CBus* get_i2c_bus(Sensor sensor) override;
    SPIBus* get_spi_bus() override;


    void init_sensor_test_pins() override;
    void signal_sensor_ok(Sensor sensor) override;
};

using Board = MannyBoard;
