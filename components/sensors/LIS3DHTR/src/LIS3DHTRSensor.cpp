#include "LIS3DHTRSensor.hpp"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "LIS3DHTRSensor";

LIS3DHTRSensor::LIS3DHTRSensor(const char *sensorName, I2CBus* bus, uint8_t address)
    : ISensor(sensorName), _dev_handle(nullptr)
{
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = address;
    dev_cfg.scl_speed_hz = 400000;
    i2c_master_bus_add_device(*bus->get_handle(), &dev_cfg, &_dev_handle);

    _dev_ctx.write_reg = platform_write;
    _dev_ctx.read_reg = platform_read;
    _dev_ctx.handle = (void *)&_dev_handle;

    _data.setSensorName(this->getSensorName());
}

LIS3DHTRSensor::~LIS3DHTRSensor()
{
    if (_dev_handle) {
        i2c_master_bus_rm_device(_dev_handle);
    }
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
    if (!isInitialized()) {
        return false;
    } 

    lis3dh_reg_t reg;
    lis3dh_xl_data_ready_get(&_dev_ctx, &reg.byte);

    // Check if new data is available
    if (reg.byte)
    {
        int16_t data_raw_acceleration[3];
        
        // ST's API reads all 3 axes at once
        lis3dh_acceleration_raw_get(&_dev_ctx, data_raw_acceleration);

        // ST provides macro functions to convert raw integers to float mg based on the scale/resolution
        _data.acceleration_x = lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[0]) * 0.001f * GRAVITY;
        _data.acceleration_y = lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[1]) * 0.001f * GRAVITY;
        _data.acceleration_z = lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[2]) * 0.001f * GRAVITY;

        _data.timestamp = (uint32_t)(esp_timer_get_time() / 1000ULL);

        return true;
    }
    return false;
}

AccelerometerSensorData LIS3DHTRSensor::getData()
{
    return _data;
}

int32_t LIS3DHTRSensor::platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
    i2c_master_dev_handle_t dev = *(i2c_master_dev_handle_t *)handle;

    // LIS3DH requires the MSB of the register address to be set to 1 for multiple byte writes
    if (len > 1) reg |= 0x80;

    // ESP-IDF standard write: construct a buffer with the register address followed by the data
    uint8_t* write_buf = (uint8_t*)malloc(len + 1);
    write_buf[0] = reg;
    memcpy(&write_buf[1], bufp, len);

    esp_err_t err = i2c_master_transmit(dev, write_buf, len + 1, 100);
    free(write_buf);

    return (err == ESP_OK) ? 0 : -1;
}

int32_t LIS3DHTRSensor::platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{
    i2c_master_dev_handle_t dev = *(i2c_master_dev_handle_t *)handle;

    // LIS3DH requires the MSB of the register address to be set to 1 for multiple byte reads
    if (len > 1) reg |= 0x80;

    esp_err_t err = i2c_master_transmit_receive(dev, &reg, 1, bufp, len, 100);

    return (err == ESP_OK) ? 0 : -1;
}