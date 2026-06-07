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
    AccelerometerSensorData() = default;
    // Acceleration (m/s^2)
    float acceleration_x = 0.0f;
    float acceleration_y = 0.0f;
    float acceleration_z = 0.0f;
    
    // Metadata
    uint32_t timestamp = 0;

    size_t serializeJson(char* buffer, size_t maxLength) const override {
        return snprintf(buffer, maxLength, 
            "{\"source\":\"%s\",\"sensorData\":{\"acceleration_x\":%f,\"acceleration_y\":%f,\"acceleration_z\":%f,\"timestamp\":%lu}}\n",
            getSensorName().c_str(), acceleration_x, acceleration_y, acceleration_z, (unsigned long)timestamp);
    }
};