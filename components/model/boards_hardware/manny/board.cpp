#include "board.h"

MannyBoard::MannyBoard() : is_initialized(false), spi_handler(nullptr) {}

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

    _i2c_bus.init(MANNY_I2C_PORT, MANNY_I2C_SDA_PIN, MANNY_I2C_SCL_PIN);
    _spi_bus.init(SPI2_HOST, MANNY_SPI_MOSI_PIN, MANNY_SPI_MISO_PIN, MANNY_SPI_CLK_PIN);

    is_initialized = true;
}

I2CBus* MannyBoard::get_i2c_bus(Sensor sensor) {
    return &_i2c_bus;
}

SPIBus* MannyBoard::get_spi_bus() {
    return &_spi_bus;
}

gpio_num_t MannyBoard::get_barometer2_cs_pin() const {
    return MANNY_BAROMETER2_CS_PIN;
}

gpio_num_t MannyBoard::get_gps_tx_pin() const {
    return MANNY_GPS_TX_PIN;
}

gpio_num_t MannyBoard::get_gps_rx_pin() const {
    return MANNY_GPS_RX_PIN;
}

gpio_num_t MannyBoard::get_arming_pin() const {
    return MANNY_ARMING_PIN;
}

gpio_num_t MannyBoard::get_flash_cs_pin() const {
    return MANNY_FLASH_CS_PIN;
}

gpio_num_t MannyBoard::get_flash_hold_pin() const {
    return MANNY_FLASH_HOLD_PIN;
}

gpio_num_t MannyBoard::get_flash_wp_pin() const {
    return MANNY_FLASH_WP_PIN;
}

gpio_num_t MannyBoard::get_spi_miso_pin() const {
    return MANNY_SPI_MISO_PIN;
}

gpio_num_t MannyBoard::get_spi_mosi_pin() const {
    return MANNY_SPI_MOSI_PIN;
}

gpio_num_t MannyBoard::get_spi_clk_pin() const {
    return MANNY_SPI_CLK_PIN;
}

gpio_num_t MannyBoard::get_barometer_cs_pin() const {
    return MANNY_BAROMETER_CS_PIN;
}

gpio_num_t MannyBoard::get_buzzer_pin() const {
    return MANNY_BUZZER_PIN;
}

gpio_num_t MannyBoard::get_rgb_red_pin() const {
    return MANNY_LED_RED_PIN;
}

gpio_num_t MannyBoard::get_rgb_green_pin() const {
    return MANNY_LED_GREEN_PIN;
}

gpio_num_t MannyBoard::get_rgb_blue_pin() const {
    return MANNY_LED_BLUE_PIN;
}

gpio_num_t MannyBoard::get_main_actuator_pin() const {
    return MANNY_MAIN_ACTUATOR_PIN;
}

gpio_num_t MannyBoard::get_drogue_actuator_pin() const {
    return MANNY_DROGUE_ACTUATOR_PIN;
}

gpio_num_t MannyBoard::get_battery_divider_adc_pin() const {
    return MANNY_BATTERY_DIVIDER_ADC_PIN;
}

int MannyBoard::get_bno055_i2c_address() const {
    return MANNY_BNO055_I2C_ADDRESS;
}

ISPIHandler* MannyBoard::get_spi_handler() const {
    return spi_handler;
}

void MannyBoard::set_spi_handler(ISPIHandler* handler) {
    spi_handler = handler;
}

gpio_num_t MannyBoard::get_lora_cs_pin() const {
    return GPIO_NUM_NC;
}

gpio_num_t MannyBoard::get_lora_aux_pin() const {
    return MANNY_LORA_AUX_PIN;
}

gpio_num_t MannyBoard::get_lora_m0_pin() const {
    return MANNY_LORA_M0_PIN;
}

gpio_num_t MannyBoard::get_lora_m1_pin() const {
    return MANNY_LORA_M1_PIN;
}

gpio_num_t MannyBoard::get_lora_tx_pin() const {
    return MANNY_LORA_TX_PIN;
}

gpio_num_t MannyBoard::get_lora_rx_pin() const {
    return MANNY_LORA_RX_PIN;
}

bool MannyBoard::is_armed() const {
    return true;
}

void MannyBoard::init_sensor_test_pins() {}

void MannyBoard::signal_sensor_ok(IBoardHardware::Sensor sensor) {}
