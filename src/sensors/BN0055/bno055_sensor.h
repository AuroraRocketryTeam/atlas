#ifndef BNO055_SENSOR_H
#define BNO055_SENSOR_H

#include "sensors/BN0055/sensor.h"
#include "driver/i2c.h"

// I2C BNO055
#define BNO055_I2C_ADDR 0x28
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
} BNO055Sensor;

bool bno055_init(Sensor* self);
bool bno055_read_data(Sensor* self, bno_sensor_data_t* data);

void bno055_sensor_create(BNO055Sensor* sensor);

#endif // BNO055_SENSOR_H
