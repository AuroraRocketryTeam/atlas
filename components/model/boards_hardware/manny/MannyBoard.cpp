#include "boards_hardware/manny/MannyBoard.hpp"

MannyBoard::MannyBoard() : is_initialized(false), i2c_handler(nullptr), spi_handler(nullptr) {}

MannyBoard::~MannyBoard() = default;

void MannyBoard::init() {
    const gpio_config_t led_gpio_config = {
        .pin_bit_mask = (1ULL << MANNY_LED_RED_PIN) | (1ULL << MANNY_LED_GREEN_PIN) | (1ULL << MANNY_LED_BUILTIN_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };

    const gpio_config_t actuators_gpio_config = {
        .pin_bit_mask = (1ULL << MANNY_DROGUE_ACTUATOR_PIN) | (1ULL << MANNY_MAIN_ACTUATOR_PIN) | (1ULL << MANNY_BUZZER_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };

    gpio_config(&actuators_gpio_config);
    gpio_config(&led_gpio_config);    

    is_initialized = true;
}

int MannyBoard::get_i2c_port() const {
    return static_cast<int>(MANNY_I2C_PORT);
}

int MannyBoard::get_i2c_sda_pin() const {
    return static_cast<int>(MANNY_I2C_SDA_PIN);
}

int MannyBoard::get_i2c_scl_pin() const {
    return static_cast<int>(MANNY_I2C_SCL_PIN);
}

int MannyBoard::get_i2c_frequency_hz() const {
    return MANNY_I2C_FREQUENCY_HZ;
}

int MannyBoard::get_gps_tx_pin() const {
    return MANNY_GPS_TX_PIN;
}

int MannyBoard::get_gps_rx_pin() const {
    return MANNY_GPS_RX_PIN;
}

int MannyBoard::get_arming_pin() const {
    return MANNY_ARMING_PIN;
}

int MannyBoard::get_flash_cs_pin() const {
    return static_cast<int>(MANNY_FLASH_CS_PIN);
}

int MannyBoard::get_flash_hold_pin() const {
    return static_cast<int>(MANNY_FLASH_HOLD_PIN);
}

int MannyBoard::get_flash_wp_pin() const {
    return static_cast<int>(MANNY_FLASH_WP_PIN);
}

int MannyBoard::get_spi_miso_pin() const {
    return static_cast<int>(MANNY_SPI_MISO_PIN);
}

int MannyBoard::get_spi_mosi_pin() const {
    return static_cast<int>(MANNY_SPI_MOSI_PIN);
}

int MannyBoard::get_spi_clk_pin() const {
    return static_cast<int>(MANNY_SPI_CLK_PIN);
}

int MannyBoard::get_barometer_cs_pin() const {
    return static_cast<int>(MANNY_BAROMETER_CS_PIN);
}

int MannyBoard::get_buzzer_pin() const {
    return static_cast<int>(MANNY_BUZZER_PIN);
}

int MannyBoard::get_rgb_red_pin() const {
    return static_cast<int>(MANNY_LED_RED_PIN);
}

int MannyBoard::get_rgb_green_pin() const {
    return static_cast<int>(MANNY_LED_GREEN_PIN);
}

int MannyBoard::get_rgb_blue_pin() const {
    return static_cast<int>(MANNY_LED_BLUE_PIN);
}

int MannyBoard::get_main_actuator_pin() const {
    return static_cast<int>(MANNY_MAIN_ACTUATOR_PIN);
}

int MannyBoard::get_drogue_actuator_pin() const {
    return static_cast<int>(MANNY_DROGUE_ACTUATOR_PIN);
}

int MannyBoard::get_battery_divider_adc_pin() const {
    return static_cast<int>(MANNY_BATTERY_DIVIDER_ADC_PIN);
}

int MannyBoard::get_bno055_i2c_address() const {
    return MANNY_BNO055_I2C_ADDRESS;
}

I2CHandler* MannyBoard::get_i2c_handler() const {
    return i2c_handler;
}

ISPIHandler* MannyBoard::get_spi_handler() const {
    return spi_handler;
}

void MannyBoard::set_i2c_handler(I2CHandler* handler) {
    i2c_handler = handler;
}

void MannyBoard::set_spi_handler(ISPIHandler* handler) {
    spi_handler = handler;
}

int MannyBoard::get_lora_cs_pin() const {
    return -1;
}

bool MannyBoard::is_armed() const {
    return true;
}