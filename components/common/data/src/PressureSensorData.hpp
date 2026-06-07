#pragma once
#include <SensorData.hpp>
#include <string.h>

/**
 * @brief Data structure for Pressure sensor readings
 * 
 */
class PressureSensorData : public SensorData
{
public:
    PressureSensorData(const char* sensorName) : SensorData(sensorName) {}
    // Pressure (hPa)
    float pressure = 0.0f;

    // Temperature (°C)
    float temperature = 0.0f;

    // Metadata
    uint32_t timestamp = 0;

    size_t serializeJson(char* buffer, size_t maxLength) const override {
        return snprintf(buffer, maxLength, 
            "{\"source\":\"%s\",\"sensorData\":{\"pressure\":%f,\"temperature\":%f,\"timestamp\":%lu}}\n",
            getSensorName().c_str(), pressure, temperature, (unsigned long)timestamp);
    }
};