#pragma once

#include <ISensor.hpp>
#include <AccelerometerSensorData.hpp>
#include <config.h>
#include <memory>
#include <I2CBus.hpp>
#include "lis3dh_reg.h"

class LIS3DHTRSensor : public ISensor
{
public:
    LIS3DHTRSensor(I2CBus* bus, uint8_t address = 0x18);
    ~LIS3DHTRSensor();

    bool init() override;
    bool updateData() override;
    std::shared_ptr<AccelerometerSensorData> getData();

private:
    i2c_master_dev_handle_t _dev_handle;
    // The ST driver context
    stmdev_ctx_t _dev_ctx;

    std::shared_ptr<AccelerometerSensorData> _data;

    // Static wrappers required by ST driver
    static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
    static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
};
