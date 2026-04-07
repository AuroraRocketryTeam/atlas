#include "I2CBus.hpp"

esp_err_t I2CBus::init(i2c_port_t port, gpio_num_t sda_io, gpio_num_t scl_io) {
    i2c_master_bus_config_t _master_cfg = {};

    _master_cfg.i2c_port   = port;
    _master_cfg.sda_io_num = sda_io;
    _master_cfg.scl_io_num = scl_io;
    _master_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
    _master_cfg.glitch_ignore_cnt = 7;

    esp_err_t err = i2c_new_master_bus(&_master_cfg, &_i2c_master_handle);
    
    return err;
}

esp_err_t I2CBus::init(i2c_master_bus_config_t master_cfg) {
    i2c_master_bus_config_t _master_cfg = master_cfg;

    esp_err_t err = i2c_new_master_bus(&_master_cfg, &_i2c_master_handle);

    return err;
}

i2c_master_bus_handle_t *I2CBus::get_handle() {
    return &_i2c_master_handle;
}
