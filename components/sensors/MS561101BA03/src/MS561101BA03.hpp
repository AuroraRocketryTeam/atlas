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
    MS561101BA03(const char* sensorName, SPIBus* bus, gpio_num_t cs_pin);
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
    bool reset();
    bool readADC(uint32_t& value, const char* conversion, uint32_t conversionElapsedMs);
    bool readPROM(uint8_t address, uint16_t& value);
    bool writeCommand(uint8_t command);
    bool startPressureConversion();
    bool calculatePressureAndTemperature(uint32_t D1, uint32_t D2, float& pressure, float& temperature);
    bool isPlausibleMeasurement(float pressure, float temperature, uint32_t timestamp) const;
    uint8_t calculatePromCrc4() const;
    bool shouldLogFailure();
    void reportReadFailure(const char* operation, esp_err_t error = ESP_OK);
    void reportInvalidAdcValue(const char* conversion, uint32_t value, uint32_t conversionElapsedMs);

    PressureSensorData _data;
    
    enum class BaroState { IDLE, WAIT_D1, WAIT_D2 };
    BaroState _state = BaroState::IDLE;
    uint32_t _conv_start_time = 0;
    uint32_t _d1_measurement_timestamp = 0;
    static constexpr uint32_t CONV_TIME_NEEDED = 10; // 10ms
    static constexpr uint32_t ERROR_LOG_INTERVAL_MS = 5000;
    static constexpr uint32_t SLOW_SPI_TRANSACTION_MS = 2;
    static constexpr uint32_t ADC_MAX = 0xFFFFFF;
    static constexpr float MIN_PRESSURE_PA = 1000.0f;
    static constexpr float MAX_PRESSURE_PA = 120000.0f;
    static constexpr float MIN_TEMPERATURE_C = -40.0f;
    static constexpr float MAX_TEMPERATURE_C = 85.0f;
    // Diagnostic only: high-dynamic flight data must not be rejected by this guard.
    static constexpr float MAX_PRESSURE_RATE_PA_PER_SECOND = 30000.0f;
    static constexpr float MAX_TEMPERATURE_RATE_C_PER_SECOND = 100.0f;
    uint32_t _d1 = 0;
    uint32_t _lastErrorLogMs = 0;
};
