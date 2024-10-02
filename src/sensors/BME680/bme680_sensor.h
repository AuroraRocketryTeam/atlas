#ifndef BME680_SENSOR_H
#define BME680_SENSOR_H

#include "sensor.h"
#include "driver/i2c.h"

#define BME680_I2C_ADDR 0x76
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define SDA_PIN 21
#define SCL_PIN 22

typedef struct {
    Sensor base;
    i2c_port_t i2c_port;
    uint8_t i2c_address;
} BME680Sensor;

bool bme680_init(Sensor* self);
bool bme680_read_data(Sensor* self, bme_sensor_data_t* data);

void bme680_sensor_create(BME680Sensor* sensor);

#endif // BME680_SENSOR_H
