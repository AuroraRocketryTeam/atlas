#pragma once

#include "driver/spi_master.h"
#include "driver/gpio.h"

/**
 * @brief SPI master bus 
 */
class SPIBus
{
    spi_host_device_t _host = SPI2_HOST;
    bool _initialized = false;

public:
    /**
     * @brief Initialize the SPI master bus.
     *
     * @param host     SPI peripheral (SPI2_HOST or SPI3_HOST)
     * @param mosi     MOSI GPIO
     * @param miso     MISO GPIO
     * @param clk      CLK GPIO
     * @param max_transfer_sz  Max DMA transfer size in bytes (0 = default 4096)
     */
    esp_err_t init(spi_host_device_t host, gpio_num_t mosi, gpio_num_t miso, gpio_num_t clk,
                   int max_transfer_sz = 0);

    spi_host_device_t get_host();
    bool is_initialized();
};
