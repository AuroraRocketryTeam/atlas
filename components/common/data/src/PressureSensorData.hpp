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

    json toJSON() const {
        json j;
        j["source"] = getSensorName();

        json sensorDataJson;
        sensorDataJson["pressure"] = pressure;
        sensorDataJson["temperature"] = temperature;
        sensorDataJson["timestamp"] = timestamp;

        j["sensorData"] = sensorDataJson;

        return j;
    }
};