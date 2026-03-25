#pragma once

#include <ISensor.hpp>
#include <AccelerometerSensorData.hpp>
#include <config.h>
#include <memory>
#include "driver/i2c.h"
#include "lis3dh_reg.h"

// Custom handle to pass ESP-IDF specific info to the ST driver
typedef struct {
    i2c_port_t i2c_port;
    uint8_t i2c_address;
} esp_sensor_handle_t;

class LIS3DHTRSensor : public ISensor
{
public:
    LIS3DHTRSensor(i2c_port_t port = I2C_NUM_0, uint8_t address = 0x18);
    
    bool init() override;
    bool updateData() override;
    std::shared_ptr<AccelerometerSensorData> getData();

private:
    // Holds port and address
    esp_sensor_handle_t _handle; 
    // The ST driver context
    stmdev_ctx_t _dev_ctx;       
    
    std::shared_ptr<AccelerometerSensorData> _data;

    // Static wrappers required by ST driver
    static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
    static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
};