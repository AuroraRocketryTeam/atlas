#include "bme680_sensor.h"
#include "driver/i2c.h"
#include "const/pins.h"
#include <stdio.h>

/**
 * @brief I2C master initialization.
 * 
 * @param i2c_port 
 * @return esp_err_t 
 */
static esp_err_t i2c_master_init(i2c_port_t i2c_port) {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = SDA_PIN;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = SCL_PIN;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    return i2c_param_config(i2c_port, &conf) || i2c_driver_install(i2c_port, conf.mode, 
            I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

/**
 * @brief BME680 initialization.
 * 
 * @param self 
 * @return true 
 * @return false 
 */
bool bme680_init(Sensor* self) {
    BME680Sensor* sensor = (BME680Sensor*) self;

    if (i2c_master_init(sensor->i2c_port) != ESP_OK) {
        printf("Errore inizializzazione I2C\n");
        return false;
    }

    printf("BME680 inizializzato correttamente.\n");
    return true;
}

/**
 * @brief BME680 data read.
 * 
 * @param self 
 * @param data 
 * @return true 
 * @return false 
 */
bool bme680_read_data(Sensor* self, bme_sensor_data_t* data) {
    BME680Sensor* sensor = (BME680Sensor*) self;

    /**
     * @brief STUB. 
     * 
     */
    data->temperature = 2500;  // 25.00°C
    data->pressure = 100000;   // 1000 hPa
    data->humidity = 50000;    // 50% RH
    data->gas_resistance = 100000; // ohm measuremet.

    printf("Dati letti: T=%d°C, P=%d hPa, H=%d%%, R=%d Ohm\n",
           data->temperature / 100, data->pressure / 100, 
           data->humidity / 1000, data->gas_resistance);
    return true;
}

/**
 * @brief BME680 sensor creation.
 * 
 * @param sensor 
 */
void bme680_sensor_create(BME680Sensor* sensor) {
    sensor->base.init = bme680_init;
    sensor->base.read_data = bme680_read_data;
    sensor->i2c_port = I2C_MASTER_NUM;
    sensor->i2c_address = BME680_I2C_ADDR;
}
