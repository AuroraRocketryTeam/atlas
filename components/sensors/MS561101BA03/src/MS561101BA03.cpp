#include "MS561101BA03.hpp"
#include "freertos/FreeRTOS.h"

#include <utils.h>

MS561101BA03::MS561101BA03(I2CBus* bus, uint8_t address) : _address(address), _dev_handle(nullptr)
{
    memset(_calibrationData, 0, sizeof(_calibrationData));

    i2c_device_config_t dev_cfg = {
        // from the MS5611 datasheet, 7 bit address
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = address,
        // TODO: check this. I found 400khz on the datasheet as maximum freq
        .scl_speed_hz    = 400000,
    };
    i2c_master_bus_add_device(*bus->get_handle(), &dev_cfg, &_dev_handle);
}

MS561101BA03::~MS561101BA03()
{
    if (_dev_handle) {
        i2c_master_bus_rm_device(_dev_handle);
    }
}

bool MS561101BA03::init()
{
    reset();
    vTaskDelay(pdMS_TO_TICKS(10));

    // Read calibration data
    if (!readCalibrationData()) {
        return false;
    }

    return true;
}

bool MS561101BA03::updateData()
{
    // Read raw pressure and temperature
    uint32_t D1 = readRawPressure();
    uint32_t D2 = readRawTemperature();

    if (D1 == 0 || D2 == 0) {
        return false;
    }

    _data = std::make_shared<PressureSensorData>("MS561101BA03");

    // Calculate compensated pressure and temperature
    calculatePressureAndTemperature(D1, D2, _data->pressure, _data->temperature);

    _data->timestamp = Utils::millis();

    return true;
}

void MS561101BA03::reset()
{
    writeCommand(MS5611_CMD_RESET);
}

bool MS561101BA03::readCalibrationData()
{
    for (uint8_t i = 0; i < 8; i++) {
        _calibrationData[i] = readPROM(MS5611_CMD_PROM_READ + (i * 2));
        if (i > 0 && _calibrationData[i] == 0) {
            return false; // Invalid calibration data
        }
    }
    return true;
}

uint16_t MS561101BA03::readPROM(uint8_t address)
{
    uint8_t cmd = address;
    uint8_t buf[2] = {};

    esp_err_t err = i2c_master_transmit_receive(_dev_handle, &cmd, 1, buf, 2, 100);
    if (err != ESP_OK) {
        return 0;
    }

    return ((uint16_t)buf[0] << 8) | buf[1];
}

void MS561101BA03::writeCommand(uint8_t command)
{
    i2c_master_transmit(_dev_handle, &command, 1, 100);
}

uint32_t MS561101BA03::readADC()
{
    uint8_t cmd = MS5611_CMD_ADC_READ;
    uint8_t buf[3] = {};

    // read 3 bytes
    esp_err_t err = i2c_master_transmit_receive(_dev_handle, &cmd, 1, buf, 3, 100);
    if (err != ESP_OK) {
        return 0;
    }

    return ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
}

uint32_t MS561101BA03::readRawPressure()
{
    writeCommand(MS5611_CMD_CONV_D1_2048);
    vTaskDelay(pdMS_TO_TICKS(10)); // Wait for conversion
    return readADC();
}

uint32_t MS561101BA03::readRawTemperature()
{
    writeCommand(MS5611_CMD_CONV_D2_4096);
    vTaskDelay(pdMS_TO_TICKS(10)); // Wait for conversion
    return readADC();
}

void MS561101BA03::calculatePressureAndTemperature(uint32_t D1, uint32_t D2, float& pressure, float& temperature)
{
    // Extract calibration coefficients
    uint16_t C1 = _calibrationData[1];
    uint16_t C2 = _calibrationData[2];
    uint16_t C3 = _calibrationData[3];
    uint16_t C4 = _calibrationData[4];
    uint16_t C5 = _calibrationData[5];
    uint16_t C6 = _calibrationData[6];

    // Calculate temperature
    int32_t dT = D2 - ((uint32_t)C5 << 8);
    int32_t TEMP = 2000 + (((int64_t)dT * C6) >> 23);

    // Calculate pressure
    int64_t OFF = ((int64_t)C2 << 16) + (((int64_t)C4 * dT) >> 7);
    int64_t SENS = ((int64_t)C1 << 15) + (((int64_t)C3 * dT) >> 8);

    // Second order temperature compensation
    int32_t T2 = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;

    if (TEMP < 2000) {
        T2 = (dT * dT) >> 31;
        OFF2 = (5 * (TEMP - 2000) * (TEMP - 2000)) >> 1;
        SENS2 = OFF2 >> 1;

        if (TEMP < -1500) {
            OFF2 = OFF2 + 7 * (TEMP + 1500) * (TEMP + 1500);
            SENS2 = SENS2 + ((11 * (TEMP + 1500) * (TEMP + 1500)) >> 1);
        }
    }

    TEMP = TEMP - T2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;

    int32_t P = (((D1 * SENS) >> 21) - OFF) >> 15;

    temperature = TEMP / 100.0f;
    pressure = P; // Convert to hPa/mbar
}

std::shared_ptr<PressureSensorData> MS561101BA03::getData()
{
    return _data;
}
