#pragma once

#include "driver/gpio.h"

// TO DELETE!!! TEMPORARY HANDLER CLASS TO AVOID ERRORS!!!
class I2CHandler;
class ISPIHandler;

class IBoardHardware {
public:
    virtual ~IBoardHardware() = default;
    virtual void init() = 0;

    virtual int get_i2c_port() const = 0;
    virtual int get_i2c_sda_pin() const = 0;
    virtual int get_i2c_scl_pin() const = 0;
    virtual int get_i2c_frequency_hz() const = 0;

    virtual int get_gps_tx_pin() const = 0;
    virtual int get_gps_rx_pin() const = 0;
    virtual int get_arming_pin() const = 0;

    virtual int get_flash_cs_pin() const = 0;
    virtual int get_flash_hold_pin() const = 0;
    virtual int get_flash_wp_pin() const = 0;
    virtual int get_spi_miso_pin() const = 0;
    virtual int get_spi_mosi_pin() const = 0;
    virtual int get_spi_clk_pin() const = 0;
    virtual int get_barometer_cs_pin() const = 0;

    virtual int get_buzzer_pin() const = 0;
    virtual int get_rgb_red_pin() const = 0;
    virtual int get_rgb_green_pin() const = 0;
    virtual int get_rgb_blue_pin() const = 0;
    virtual int get_main_actuator_pin() const = 0;
    virtual int get_drogue_actuator_pin() const = 0;

    virtual int get_battery_divider_adc_pin() const = 0;
    virtual int get_bno055_i2c_address() const = 0;

    virtual I2CHandler* get_i2c_handler() const = 0;
    virtual ISPIHandler* get_spi_handler() const = 0;
    virtual void set_i2c_handler(I2CHandler* handler) = 0;
    virtual void set_spi_handler(ISPIHandler* handler) = 0;

    virtual int get_lora_cs_pin() const = 0;

    virtual bool is_armed() const = 0;
};