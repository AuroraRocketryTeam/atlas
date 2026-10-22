#pragma once
#include <SensorData.hpp>
#include <cstdint>
#include <cstdio>

/**
 * @brief Data structure for Pressure sensor readings
 * 
 */
class PressureSensorData : public SensorData
{
public:
    PressureSensorData() = default;
    // Pressure (hPa)
    float pressure = 0.0f;

    // Temperature (°C)
    float temperature = 0.0f;

    // Metadata
    uint32_t timestamp = 0;

    size_t serializeJson(char* buffer, size_t maxLength) const {
        return std::snprintf(buffer, maxLength,
            "{\"source\":\"%s\",\"t\":%lu,\"p_pa\":%.8g,\"temp_c\":%.4g}\n",
            getSensorName(), static_cast<unsigned long>(timestamp), pressure, temperature);
    }
};
