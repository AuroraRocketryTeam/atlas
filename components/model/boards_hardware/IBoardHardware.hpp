#pragma once

#include "driver/gpio.h"
#include <I2CBus.hpp>
#include <SPIBus.hpp>

class ISPIHandler;

class IBoardHardware {
public:
    virtual ~IBoardHardware() = default;
    virtual void init() = 0;

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

    virtual bool is_armed() const = 0;

    enum class Sensor { IMU, BARO1, BARO2, ACC };

    virtual I2CBus* get_i2c_bus(Sensor sensor) = 0;
    virtual SPIBus* get_spi_bus() = 0;


    virtual void init_sensor_test_pins() = 0;
    virtual void signal_sensor_ok(Sensor sensor) = 0;
};
