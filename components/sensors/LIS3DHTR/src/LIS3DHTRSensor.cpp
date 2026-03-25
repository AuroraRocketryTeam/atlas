#include "LIS3DHTRSensor.hpp"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

static const char* TAG = "LIS3DHTRSensor";

int32_t LIS3DHTRSensor::platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    esp_sensor_handle_t *sensor = (esp_sensor_handle_t *)handle;
    
    // LIS3DH requires the MSB of the register address to be set to 1 for multiple byte writes
    if (len > 1) reg |= 0x80;

    // ESP-IDF standard write: construct a buffer with the register address followed by the data
    uint8_t* write_buf = (uint8_t*)malloc(len + 1);
    write_buf[0] = reg;
    memcpy(&write_buf[1], bufp, len);

    esp_err_t err = i2c_master_write_to_device(sensor->i2c_port, sensor->i2c_address, write_buf, len + 1, pdMS_TO_TICKS(100));
    free(write_buf);

    return (err == ESP_OK) ? 0 : -1;
}

int32_t LIS3DHTRSensor::platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    esp_sensor_handle_t *sensor = (esp_sensor_handle_t *)handle;
    
    // LIS3DH requires the MSB of the register address to be set to 1 for multiple byte reads
    if (len > 1) reg |= 0x80;

    esp_err_t err = i2c_master_write_read_device(sensor->i2c_port, sensor->i2c_address, &reg, 1, bufp, len, pdMS_TO_TICKS(100));
    
    return (err == ESP_OK) ? 0 : -1;
}

LIS3DHTRSensor::LIS3DHTRSensor(i2c_port_t port, uint8_t address) 
{
    _handle.i2c_port = port;
    _handle.i2c_address = address;

    // Link the ST context to our platform functions
    _dev_ctx.write_reg = platform_write;
    _dev_ctx.read_reg = platform_read;
    _dev_ctx.handle = (void *)&_handle;
}

bool LIS3DHTRSensor::init()
{
    uint8_t whoamI;

    // Check device ID using ST's API
    lis3dh_device_id_get(&_dev_ctx, &whoamI);
    if (whoamI != LIS3DH_ID)
    {
        ESP_LOGE(TAG, "Sensor not found! Expected 0x%02X, got 0x%02X", LIS3DH_ID, whoamI);
        setInitialized(false);
        return false;
    }

    // Initialize sensor using ST's semantic APIs
    lis3dh_block_data_update_set(&_dev_ctx, PROPERTY_ENABLE);    // Enable BDU
    lis3dh_data_rate_set(&_dev_ctx, LIS3DH_ODR_50Hz);            // 50Hz Output Data Rate
    lis3dh_full_scale_set(&_dev_ctx, LIS3DH_2g);                 // 2G range
    lis3dh_operating_mode_set(&_dev_ctx, LIS3DH_HR_12bit);       // High resolution (12-bit)

    setInitialized(true);
    return true;
}

bool LIS3DHTRSensor::updateData()
{
    if (!isInitialized()) return false;

    lis3dh_reg_t reg;
    lis3dh_xl_data_ready_get(&_dev_ctx, &reg.byte);
    
    // Check if new data is available
    if (reg.byte)
    {
        int16_t data_raw_acceleration[3];
        
        // ST's API reads all 3 axes at once
        lis3dh_acceleration_raw_get(&_dev_ctx, data_raw_acceleration);

        _data = std::make_shared<AccelerometerSensorData>("LIS3DHTR");

        // ST provides macro functions to convert raw integers to float mg based on the scale/resolution
        _data->acceleration_x = lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[0]) * 0.001f * GRAVITY;
        _data->acceleration_y = lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[1]) * 0.001f * GRAVITY;
        _data->acceleration_z = lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[2]) * 0.001f * GRAVITY;
        
        _data->timestamp = (uint32_t)(esp_timer_get_time() / 1000ULL);
        
        return true;
    }
    return false;
}

std::shared_ptr<AccelerometerSensorData> LIS3DHTRSensor::getData()
{
    return _data;
}