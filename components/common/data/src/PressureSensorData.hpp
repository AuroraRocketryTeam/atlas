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
    PressureSensorData(std::string sensorName) : SensorData(sensorName) {}

    // Pressure (hPa)
    float pressure;

    // Temperature (°C)
    float temperature;

    // Metadata
    uint32_t timestamp;

    json toJSON() const override {
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