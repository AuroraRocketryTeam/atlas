#pragma once

#include "boards_hardware/IBoardHardware.hpp"
#include "driver/i2c.h"

// Manny board hardware map
static constexpr gpio_num_t MANNY_I2C_SDA_PIN = GPIO_NUM_11;
static constexpr gpio_num_t MANNY_I2C_SCL_PIN = GPIO_NUM_12;
static constexpr i2c_port_t MANNY_I2C_PORT = I2C_NUM_0;
static constexpr int MANNY_I2C_FREQUENCY_HZ = 100000;

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

static constexpr gpio_num_t MANNY_BATTERY_DIVIDER_ADC_PIN = GPIO_NUM_1;
static constexpr int MANNY_BNO055_I2C_ADDRESS = 0x29;

static constexpr int MANNY_ARMING_PIN = -1;

static constexpr int MANNY_GPS_TX_PIN = 17;
static constexpr int MANNY_GPS_RX_PIN = 16;

// Hardware note: main and drogue outputs are valid only when board supply is 6.4V.

static const char* TAG = "Manny";

class MannyBoard : public IBoardHardware {
private:
    bool is_initialized;
    I2CHandler* i2c_handler;
    ISPIHandler* spi_handler;

public:
    MannyBoard();
    ~MannyBoard() override;

    void init() override;

    int get_i2c_port() const override;
    int get_i2c_sda_pin() const override;
    int get_i2c_scl_pin() const override;
    int get_i2c_frequency_hz() const override;

    int get_gps_tx_pin() const override;
    int get_gps_rx_pin() const override;
    int get_arming_pin() const override;

    int get_flash_cs_pin() const override;
    int get_flash_hold_pin() const override;
    int get_flash_wp_pin() const override;
    int get_spi_miso_pin() const override;
    int get_spi_mosi_pin() const override;
    int get_spi_clk_pin() const override;
    int get_barometer_cs_pin() const override;

    int get_buzzer_pin() const override;
    int get_rgb_red_pin() const override;
    int get_rgb_green_pin() const override;
    int get_rgb_blue_pin() const override;
    int get_main_actuator_pin() const override;
    int get_drogue_actuator_pin() const override;

    int get_battery_divider_adc_pin() const override;
    int get_bno055_i2c_address() const override;

    I2CHandler* get_i2c_handler() const override;
    ISPIHandler* get_spi_handler() const override;
    void set_i2c_handler(I2CHandler* handler) override;
    void set_spi_handler(ISPIHandler* handler) override;

    int get_lora_cs_pin() const override;

    bool is_armed() const override;
};