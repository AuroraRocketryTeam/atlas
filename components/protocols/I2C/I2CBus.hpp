#pragma once

#include "driver/i2c_master.h"

/**
 * @brief I2C master bus handler class
 * 
 */
class I2CBus
{
    i2c_master_bus_handle_t _i2c_master_handle;

public:
    /**
     * @brief Initialize the I2C master bus with minimal configuration.
     * 
     * @param port   I2C Controller (I2C_NUM_0 or I2C_NUM_1)
     * @param sda_io SDA GPIO number 
     * @param scl_io SCL GPIO number
     * @return 
        - ESP_OK: I2C master bus initialized successfully.
        - ESP_ERR_INVALID_ARG: I2C bus initialization failed because of invalid argument.
        - ESP_ERR_NO_MEM: Create I2C bus failed because of out of memory.
        - ESP_ERR_NOT_FOUND: No more free bus.
     */
    esp_err_t init(i2c_port_t port, gpio_num_t sda_io, gpio_num_t scl_io);

    /**
     * @brief Initialize the I2C master bus with the specified configuration.
     * 
     * @param master_cfg I2C master bus configuration
     * @return 
        - ESP_OK: I2C master bus initialized successfully.
        - ESP_ERR_INVALID_ARG: I2C bus initialization failed because of invalid argument.
        - ESP_ERR_NO_MEM: Create I2C bus failed because of out of memory.
        - ESP_ERR_NOT_FOUND: No more free bus.
     */
    esp_err_t init(i2c_master_bus_config_t master_cfg);

    /**
     * @brief Get the bus' master handle.
     * 
     * @return pointer to the bus' master handler 
     */
    i2c_master_bus_handle_t *get_handle();
};
