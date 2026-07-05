#pragma once

#include "driver/gpio.h"
#include <I2CBus.hpp>
#include <SPIBus.hpp>

class ISPIHandler;

/**
 * @brief Hardware abstraction for board-specific pins, buses, and shared services.
 *
 * Consumers should use this interface as the single entry point for hardware
 * resources that require board-specific setup. This keeps common procedures
 * such as NVS, networking, WiFi, and bus initialization centralized in the
 * concrete board implementation.
 */
class IBoardHardware {
public:
    virtual ~IBoardHardware() = default;

    /**
     * @brief Initialize base board peripherals such as GPIOs and shared buses.
     */
    virtual void init() = 0;

    virtual const char* get_board_name() const = 0;
    virtual gpio_num_t get_i2c_sda_pin() const = 0;
    virtual gpio_num_t get_i2c_scl_pin() const = 0;

    virtual gpio_num_t get_gps_tx_pin() const = 0;
    virtual gpio_num_t get_gps_rx_pin() const = 0;
    virtual gpio_num_t get_arming_pin() const = 0;

    virtual gpio_num_t get_flash_cs_pin() const = 0;
    virtual gpio_num_t get_flash_hold_pin() const = 0;
    virtual gpio_num_t get_flash_wp_pin() const = 0;
    virtual gpio_num_t get_spi_miso_pin() const = 0;
    virtual gpio_num_t get_spi_mosi_pin() const = 0;
    virtual gpio_num_t get_spi_clk_pin() const = 0;
    virtual gpio_num_t get_barometer_cs_pin() const = 0;

    virtual gpio_num_t get_buzzer_pin() const = 0;
    virtual gpio_num_t get_rgb_red_pin() const = 0;
    virtual gpio_num_t get_rgb_green_pin() const = 0;
    virtual gpio_num_t get_rgb_blue_pin() const = 0;
    virtual gpio_num_t get_main_actuator_pin() const = 0;
    virtual gpio_num_t get_drogue_actuator_pin() const = 0;

    virtual gpio_num_t get_battery_divider_adc_pin() const = 0;
    virtual int get_bno055_i2c_address() const = 0;

    virtual ISPIHandler* get_spi_handler() const = 0;
    virtual void set_spi_handler(ISPIHandler* handler) = 0;

    virtual gpio_num_t get_lora_cs_pin() const = 0;
    virtual gpio_num_t get_lora_aux_pin() const = 0;
    virtual gpio_num_t get_lora_m0_pin() const = 0;
    virtual gpio_num_t get_lora_m1_pin() const = 0;
    virtual gpio_num_t get_lora_tx_pin() const = 0;
    virtual gpio_num_t get_lora_rx_pin() const = 0;

    virtual gpio_num_t get_barometer2_cs_pin() const = 0;
    virtual gpio_num_t get_sd_cs_pin() const = 0;

    virtual bool is_armed() const = 0;

    /**
     * @brief Initialize the default NVS partition required by ESP services.
     */
    virtual bool initNvs() = 0;

    /**
     * @brief Return true after NVS has been initialized by the board.
     */
    virtual bool isNvsReady() const = 0;

    /**
     * @brief Initialize networking dependencies shared by WiFi modes.
     *
     * Implementations are responsible for calling the required lower-level
     * setup in the right order, including NVS, esp-netif, the default event
     * loop, and WiFi driver initialization.
     */
    virtual bool initNetworking() = 0;

    /**
     * @brief Start WiFi in station mode after ensuring dependencies are ready.
     */
    virtual bool startWifiSta() = 0;

    /**
     * @brief Acquire WiFi SoftAP service using the board's configured defaults.
     *
     * Calls are reference counted by the board implementation. Multiple tasks
     * may acquire the same SoftAP lifetime; each successful call must be paired
     * with stopWifi(). The physical WiFi driver is stopped only after the last
     * SoftAP user releases it.
     */
    virtual bool startWifiSoftAp() = 0;

    /**
     * @brief Release one board-managed WiFi user and stop WiFi when unused.
     */
    virtual bool stopWifi() = 0;

    /**
     * @brief Return true while the board has WiFi started.
     */
    virtual bool isWifiConnected() const = 0;

    /**
     * @brief Return the last board-managed WiFi IP address, or an empty string.
     */
    virtual const char* getWifiIpAddress() const = 0;

    /**
     * @brief Return the last board-managed WiFi MAC address, or an empty string.
     */
    virtual const char* getWifiMacAddress() const = 0;

    /**
     * @brief Initialize board-managed Bluetooth services, when supported.
     */
    virtual bool initBluetooth() = 0;

    /**
     * @brief Stop board-managed Bluetooth services, when supported.
     */
    virtual bool stopBluetooth() = 0;

    enum class Sensor { IMU, BARO1, BARO2, ACC };

    virtual I2CBus* get_i2c_bus(Sensor sensor) = 0;
    virtual SPIBus* get_spi_bus() = 0;


    virtual void init_sensor_test_pins() = 0;
    virtual void signal_sensor_ok(Sensor sensor) = 0;
};
