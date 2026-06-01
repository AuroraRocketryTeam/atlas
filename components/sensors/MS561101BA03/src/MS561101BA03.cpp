#include "MS561101BA03.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <utils.h>

MS561101BA03::MS561101BA03(SPIBus* bus, gpio_num_t cs_pin)
{
    memset(_calibrationData, 0, sizeof(_calibrationData));

    spi_device_interface_config_t dev_cfg = {
        .mode           = 0,           // SPI mode 0 (CPOL=0, CPHA=0)
        .clock_speed_hz = 20'000'000,    // 20 MHz (MS5611 max)
        .spics_io_num   = cs_pin,
        .queue_size     = 1,
    };
    spi_bus_add_device(bus->get_host(), &dev_cfg, &_dev_handle);
}

MS561101BA03::~MS561101BA03()
{
    if (_dev_handle) {
        spi_bus_remove_device(_dev_handle);
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
    uint32_t now = Utils::millis();

    switch (_state) {
        // Conversion started, but data not ready
        case BaroState::IDLE:
            writeCommand(MS5611_CMD_CONV_D1_2048);
            _conv_start_time = now;
            _state = BaroState::WAIT_D1;
            return false; 

        // Finished conversion of D1 and starting D2
        case BaroState::WAIT_D1:
            // Check if 10ms has passed without blocking the task
            if (now - _conv_start_time >= CONV_TIME_NEEDED) {
                _d1 = readADC();
                writeCommand(MS5611_CMD_CONV_D2_4096);
                _conv_start_time = now;
                _state = BaroState::WAIT_D2;
            }
            return false; 

        // Prepare final data
        case BaroState::WAIT_D2:
            if (now - _conv_start_time >= CONV_TIME_NEEDED) {
                uint32_t D2 = readADC();
                
                calculatePressureAndTemperature(_d1, D2, _data.pressure, _data.temperature);
                _data.timestamp = now;
                
                _state = BaroState::IDLE;
                return true;
            }
            return false;
    }
    return false;
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
    // The sensor does not send data until it has received and
    // clocked the address byte, so we read starting from rx[1]
    spi_transaction_t t = {};
    t.length    = 24; // 3 bytes
    t.rxlength  = 24;
    t.flags     = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.tx_data[0] = address;
    t.tx_data[1] = 0x00;
    t.tx_data[2] = 0x00;

    esp_err_t err = spi_device_polling_transmit(_dev_handle, &t);
    if (err != ESP_OK) return 0;

    return ((uint16_t)t.rx_data[1] << 8) | t.rx_data[2];
}
void MS561101BA03::writeCommand(uint8_t command)
{
    spi_transaction_t t = {};
    t.length    = 8; // one byte
    t.flags     = SPI_TRANS_USE_TXDATA;  // only write
    t.tx_data[0] = command;
    spi_device_polling_transmit(_dev_handle, &t);
}

uint32_t MS561101BA03::readADC()
{
    // The sensor does not send data until it has received and
    // clocked the command byte, so we read starting from rx[1]
    spi_transaction_t t = {};
    t.length    = 32;   // 4 bytes
    t.rxlength  = 32;
    t.flags     = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
    t.tx_data[0] = MS5611_CMD_ADC_READ;
    t.tx_data[1] = 0x00;
    t.tx_data[2] = 0x00;
    t.tx_data[3] = 0x00;

    esp_err_t err = spi_device_polling_transmit(_dev_handle, &t);
    if (err != ESP_OK) return 0;

    return ((uint32_t)t.rx_data[1] << 16) | ((uint32_t)t.rx_data[2] << 8) | t.rx_data[3];
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
        T2 = ((int64_t)dT * dT) >> 31;
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

PressureSensorData MS561101BA03::getData()
{
    return _data;
}
