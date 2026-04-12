#include "SPIBus.hpp"

esp_err_t SPIBus::init(spi_host_device_t host, gpio_num_t mosi, gpio_num_t miso, gpio_num_t clk,
                       int max_transfer_sz)
{
    _host = host;

    spi_bus_config_t cfg = {
        .mosi_io_num     = mosi,
        .miso_io_num     = miso,
        .sclk_io_num     = clk,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1, // disable quad
        .max_transfer_sz = max_transfer_sz,
    };

    // Auto DMA channlel selection, recommended by docs
    esp_err_t err = spi_bus_initialize(host, &cfg, SPI_DMA_CH_AUTO);
    if (err == ESP_OK) {
        _initialized = true;
    }
    return err;
}

spi_host_device_t SPIBus::get_host() { 
    return _host;
}
bool SPIBus::is_initialized() {
    return _initialized; 
}