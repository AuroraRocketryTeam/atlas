#pragma once
#include <ISensor.hpp>
#include <SPIBus.hpp>
#include <PressureSensorData.hpp>

// MS5611 Commands
#define MS5611_CMD_RESET        0x1E
#define MS5611_CMD_CONV_D1_256  0x40
#define MS5611_CMD_CONV_D1_512  0x42
#define MS5611_CMD_CONV_D1_1024 0x44
#define MS5611_CMD_CONV_D1_2048 0x46
#define MS5611_CMD_CONV_D1_4096 0x48
#define MS5611_CMD_CONV_D2_256  0x50
#define MS5611_CMD_CONV_D2_512  0x52
#define MS5611_CMD_CONV_D2_1024 0x54
#define MS5611_CMD_CONV_D2_2048 0x56
#define MS5611_CMD_CONV_D2_4096 0x58
#define MS5611_CMD_ADC_READ     0x00
#define MS5611_CMD_PROM_READ    0xA0

/**
 * @brief MS561101BA03 sensor class
 */
class MS561101BA03 : public ISensor
{
public:
    MS561101BA03(SPIBus* bus, gpio_num_t cs_pin);
    ~MS561101BA03();

    bool init() override;

    /**
     * @brief Update the sensor data. 
     * NOTE: as we need to wait for the conversion of D1 and D2, the task is 
     * divided in three stages, the actual values are updated at the third
     * 
     * @return true 
     * @return false 
     */
    bool updateData() override;

    PressureSensorData getData();

private:
    spi_device_handle_t _dev_handle = nullptr;
    uint16_t _calibrationData[8];

    bool readCalibrationData();
    void reset();
    uint32_t readADC();
    uint16_t readPROM(uint8_t address);
    void writeCommand(uint8_t command);
    uint32_t readRawPressure();
    uint32_t readRawTemperature();
    void calculatePressureAndTemperature(uint32_t D1, uint32_t D2, float& pressure, float& temperature);

    PressureSensorData _data{"MS561101BA03"};
    
    enum class BaroState { IDLE, WAIT_D1, WAIT_D2 };
    BaroState _state = BaroState::IDLE;
    uint32_t _conv_start_time = 0;
    uint32_t CONV_TIME_NEEDED = 10; // 10ms
    uint32_t _d1 = 0;
};
