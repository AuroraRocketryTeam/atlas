#pragma once
#include <SensorData.hpp>
#include <cstdint>
#include <cstdio>

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

    size_t serializeJson(char* buffer, size_t maxLength) const {
        return std::snprintf(buffer, maxLength,
            "{\"src\":\"%s\",\"t\":%lu,\"accx\":%.6g,\"accy\":%.6g,\"accz\":%.6g}\n",
            getSensorName(), static_cast<unsigned long>(timestamp),
            acceleration_x, acceleration_y, acceleration_z);
    }
};
