#pragma once
#include <SensorData.hpp>
#include <string.h>

/**
 * @brief Data structure for a general accelerometer sensor readings
 * 
 */
class AccelerometerSensorData : public SensorData
{
public:
    AccelerometerSensorData(std::string sensorName) : SensorData(sensorName) {}

    // Acceleration (m/s^2)
    float acceleration_x;
    float acceleration_y;
    float acceleration_z;
    
    // Metadata
    uint32_t timestamp;

    json toJSON() const override {
        json j;
        j["source"] = getSensorName();

        json sensorDataJson;
        sensorDataJson["acceleration_x"] = acceleration_x;
        sensorDataJson["acceleration_y"] = acceleration_y;
        sensorDataJson["acceleration_z"] = acceleration_z;
        sensorDataJson["timestamp"] = timestamp;

        j["sensorData"] = sensorDataJson;

        return j;
    }
};