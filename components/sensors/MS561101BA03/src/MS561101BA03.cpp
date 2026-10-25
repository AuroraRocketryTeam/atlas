#include "MS561101BA03.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <SerialLogger.hpp>
#include <array>
#include <cmath>
#include <utils.h>

namespace {
constexpr uint8_t calculatePromCrc4(const std::array<uint16_t, 8>& coefficients)
{
    auto prom = coefficients;
    prom[7] &= 0xFF00; // CRC is stored in the low nibble of PROM word 7.

    uint16_t remainder = 0;
    for (uint8_t byte = 0; byte < 16; ++byte) {
        remainder ^= (byte & 1) ? (prom[byte >> 1] & 0x00FF) : (prom[byte >> 1] >> 8);
        for (uint8_t bit = 0; bit < 8; ++bit) {
            remainder = (remainder & 0x8000) ? ((remainder << 1) ^ 0x3000) : (remainder << 1);
        }
    }
    return (remainder >> 12) & 0x0F;
}

// AN520's published test vector must produce CRC-4 0xB.
static_assert(calculatePromCrc4({0x3132, 0x3334, 0x3536, 0x3738,
                                 0x3940, 0x4142, 0x4344, 0x4500}) == 0x0B);
} // namespace

MS561101BA03::MS561101BA03(const char *sensorName, SPIBus *bus, gpio_num_t cs_pin) : ISensor(sensorName)
{
    memset(_calibrationData, 0, sizeof(_calibrationData));

    spi_device_interface_config_t dev_cfg = {};
    dev_cfg.mode = 0;                    // SPI mode 0 (CPOL=0, CPHA=0)
    dev_cfg.clock_speed_hz = 10'000'000;
    dev_cfg.spics_io_num = cs_pin;
    dev_cfg.queue_size = 7;
    
    const esp_err_t err = spi_bus_add_device(bus->get_host(), &dev_cfg, &_dev_handle);
    if (err != ESP_OK) {
        LOG_ERROR(getSensorName(), "Failed to attach to SPI bus: %s", esp_err_to_name(err));
    }

    _data.setSensorName(this->getSensorName());
}

MS561101BA03::~MS561101BA03()
{
    if (_dev_handle) {
        spi_bus_remove_device(_dev_handle);
    }
}

bool MS561101BA03::init()
{
    if (!_dev_handle || !reset()) return false;
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
            // SPI2 is shared with storage, but ESP-IDF serializes distinct device handles.
            // The datasheet prefers idle SPI traffic during conversion for lowest noise; availability wins here.
            // Do not lock the bus over the sensor's internal 10 ms conversion.
            if (!startPressureConversion()) return false;
            return false; 

        // Finished conversion of D1 and starting D2
        case BaroState::WAIT_D1:
            // Check if 10ms has passed without blocking the task
            if (now - _conv_start_time >= CONV_TIME_NEEDED) {
                const uint32_t conversionElapsedMs = Utils::millis() - _conv_start_time;
                if (!readADC(_d1, "D1", conversionElapsedMs)) {
                    _state = BaroState::IDLE;
                    return false;
                }
                if (!writeCommand(MS5611_CMD_CONV_D2_4096)) {
                    _state = BaroState::IDLE;
                    return false;
                }
                // As for D1, do not count SPI-bus wait time as sensor conversion time.
                _conv_start_time = Utils::millis();
                _state = BaroState::WAIT_D2;
            }
            return false; 

        // Prepare final data
        case BaroState::WAIT_D2:
            if (now - _conv_start_time >= CONV_TIME_NEEDED) {
                uint32_t D2 = 0;
                float pressure = 0.0f;
                float temperature = 0.0f;

                const uint32_t conversionElapsedMs = Utils::millis() - _conv_start_time;
                if (!readADC(D2, "D2", conversionElapsedMs)) {
                    _state = BaroState::IDLE;
                    return false;
                }
                if (!calculatePressureAndTemperature(_d1, D2, pressure, temperature)) {
                    _state = BaroState::IDLE;
                    return false;
                }
                if (!isPlausibleMeasurement(pressure, temperature, _d1_measurement_timestamp)) {
                    reportReadFailure("unusual pressure/temperature rate");
                }

                _data.pressure = pressure;
                _data.temperature = temperature;
                _data.timestamp = _d1_measurement_timestamp;

                // Pipeline the next D1 conversion now instead of waiting for the
                // next 50 Hz SensorTask call to re-enter IDLE. D1+D2 still use
                // separate non-blocking 10 ms conversions, while the usual
                // published-sample cadence drops from about 60 ms to 40 ms.
                // This completed sample remains valid if the next command fails.
                if (!startPressureConversion()) _state = BaroState::IDLE;
                return true;
            }
            return false;
    }
    return false;
}

bool MS561101BA03::reset()
{
    return writeCommand(MS5611_CMD_RESET);
}

bool MS561101BA03::readCalibrationData()
{
    for (uint8_t i = 0; i < 8; i++) {
        if (!readPROM(MS5611_CMD_PROM_READ + (i * 2), _calibrationData[i])) return false;
    }

    bool allZero = true;
    bool allOnes = true;
    for (const uint16_t word : _calibrationData) {
        allZero &= word == 0x0000;
        allOnes &= word == 0xFFFF;
    }
    if (allZero || allOnes) {
        LOG_ERROR(getSensorName(), "PROM read is all %s", allZero ? "0x0000" : "0xFFFF");
        return false;
    }

    const uint8_t storedCrc = _calibrationData[7] & 0x0F;
    const uint8_t calculatedCrc = calculatePromCrc4();
    if (calculatedCrc != storedCrc) {
        LOG_ERROR(getSensorName(), "PROM CRC-4 mismatch: stored=0x%X calculated=0x%X",
                  storedCrc, calculatedCrc);
        return false;
    }
    return true;
}

bool MS561101BA03::readPROM(uint8_t address, uint16_t& value)
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
    if (err != ESP_OK) {
        reportReadFailure("PROM read", err);
        return false;
    }

    value = ((uint16_t)t.rx_data[1] << 8) | t.rx_data[2];
    return true;
}

bool MS561101BA03::writeCommand(uint8_t command)
{
    spi_transaction_t t = {};
    t.length    = 8; // one byte
    t.flags     = SPI_TRANS_USE_TXDATA;  // only write
    t.tx_data[0] = command;
    const uint32_t transactionStartMs = Utils::millis();
    const esp_err_t err = spi_device_polling_transmit(_dev_handle, &t);
    const uint32_t transactionElapsedMs = Utils::millis() - transactionStartMs;
    if (err != ESP_OK) {
        if (shouldLogFailure()) {
            LOG_WARNING(getSensorName(), "SPI command 0x%02X failed after %lu ms: %s",
                        command, static_cast<unsigned long>(transactionElapsedMs), esp_err_to_name(err));
        }
        return false;
    }
    if (transactionElapsedMs >= SLOW_SPI_TRANSACTION_MS && shouldLogFailure()) {
        LOG_WARNING(getSensorName(), "SPI command 0x%02X blocked for %lu ms",
                    command, static_cast<unsigned long>(transactionElapsedMs));
    }
    return true;
}

bool MS561101BA03::startPressureConversion()
{
    if (!writeCommand(MS5611_CMD_CONV_D1_4096)) return false;

    // The command can wait behind storage traffic; time conversion from its completion.
    _conv_start_time = Utils::millis();
    // D1 is the pressure measurement; D2 only supplies temperature compensation.
    _d1_measurement_timestamp = _conv_start_time + CONV_TIME_NEEDED;
    _state = BaroState::WAIT_D1;
    return true;
}

bool MS561101BA03::readADC(uint32_t& value, const char* conversion, uint32_t conversionElapsedMs)
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

    const uint32_t transactionStartMs = Utils::millis();
    const esp_err_t err = spi_device_polling_transmit(_dev_handle, &t);
    const uint32_t transactionElapsedMs = Utils::millis() - transactionStartMs;
    if (err != ESP_OK) {
        if (shouldLogFailure()) {
            LOG_WARNING(getSensorName(), "%s ADC SPI read failed after wait=%lu ms, transaction=%lu ms: %s",
                        conversion,
                        static_cast<unsigned long>(conversionElapsedMs),
                        static_cast<unsigned long>(transactionElapsedMs),
                        esp_err_to_name(err));
        }
        return false;
    }

    value = ((uint32_t)t.rx_data[1] << 16) | ((uint32_t)t.rx_data[2] << 8) | t.rx_data[3];
    if (value == 0 || value == ADC_MAX) {
        reportInvalidAdcValue(conversion, value, conversionElapsedMs);
        return false;
    }
    if (transactionElapsedMs >= SLOW_SPI_TRANSACTION_MS && shouldLogFailure()) {
        LOG_WARNING(getSensorName(), "%s ADC SPI transaction blocked for %lu ms after wait=%lu ms",
                    conversion,
                    static_cast<unsigned long>(transactionElapsedMs),
                    static_cast<unsigned long>(conversionElapsedMs));
    }
    return true;
}

bool MS561101BA03::calculatePressureAndTemperature(uint32_t D1, uint32_t D2, float& pressure, float& temperature)
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
    pressure = P; // Datasheet result is in 0.01 mbar, numerically equal to Pa.

    if (!std::isfinite(pressure) || pressure < MIN_PRESSURE_PA || pressure > MAX_PRESSURE_PA ||
        !std::isfinite(temperature) || temperature < MIN_TEMPERATURE_C || temperature > MAX_TEMPERATURE_C) {
        reportReadFailure("out-of-range conversion");
        return false;
    }
    return true;
}

bool MS561101BA03::isPlausibleMeasurement(float pressure, float temperature, uint32_t timestamp) const
{
    if (_data.timestamp == 0) return true;

    const float elapsedSeconds = static_cast<float>(timestamp - _data.timestamp) / 1000.0f;
    return std::fabs(pressure - _data.pressure) <= MAX_PRESSURE_RATE_PA_PER_SECOND * elapsedSeconds &&
           std::fabs(temperature - _data.temperature) <= MAX_TEMPERATURE_RATE_C_PER_SECOND * elapsedSeconds;
}

uint8_t MS561101BA03::calculatePromCrc4() const
{
    std::array<uint16_t, 8> prom{};
    memcpy(prom.data(), _calibrationData, sizeof(_calibrationData));
    return ::calculatePromCrc4(prom);
}

bool MS561101BA03::shouldLogFailure()
{
    const uint32_t now = Utils::millis();
    if (_lastErrorLogMs != 0 && now - _lastErrorLogMs < ERROR_LOG_INTERVAL_MS) return false;
    _lastErrorLogMs = now;
    return true;
}

void MS561101BA03::reportReadFailure(const char* operation, esp_err_t error)
{
    if (!shouldLogFailure()) return;
    if (error == ESP_OK) LOG_WARNING(getSensorName(), "%s", operation);
    else LOG_WARNING(getSensorName(), "%s failed: %s", operation, esp_err_to_name(error));
}

void MS561101BA03::reportInvalidAdcValue(const char* conversion, uint32_t value, uint32_t conversionElapsedMs)
{
    if (!shouldLogFailure()) return;
    LOG_WARNING(getSensorName(), "%s ADC=0x%06lX after %lu ms (%s)",
                conversion,
                static_cast<unsigned long>(value),
                static_cast<unsigned long>(conversionElapsedMs),
                value == 0 ? "conversion not ready or bus read failure" : "all bits high");
}

PressureSensorData MS561101BA03::getData()
{
    return _data;
}
