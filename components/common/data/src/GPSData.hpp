#pragma once
#include <SensorData.hpp>
#include <string.h>

/**
 * @brief Data structure for GPS sensor readings
 * 
 */
class GPSData : public SensorData
{
public:
    GPSData() = default;
    uint8_t satellites = 0;
    
    // Fix type (0=no fix, 1=dead reckoning, 2=2D fix, 3=3D fix, 4=GNSS+dead reckoning, 5=time-only fix) 
    uint8_t fixType = 0;
    
    // Latitude in degrees * 10^7
    float latitude = 0;
    
    // Longitude in degrees * 10^7
    float longitude = 0;
    
    // Altitude above mean sea level in meters
    float altitude = 0.0;
    
    // Ground speed in meters per second
    float ground_speed = 0.0;
    
    // Horizontal dilution of precision
    float hdop = 0.0;
    
    // Metadata
    uint32_t timestamp = 0;
    
    size_t serializeJson(char* buffer, size_t maxLength) const override {
        return snprintf(buffer, maxLength, 
            "{\"source\":\"%s\",\"sensorData\":{\"satellites\":%u,\"fixType\":%u,\"latitude\":%f,\"longitude\":%f,\"altitude\":%f,\"ground_speed\":%f,\"hdop\":%f,\"timestamp\":%lu}}\n",
            getSensorName().c_str(), satellites, fixType, latitude, longitude, altitude, ground_speed, hdop, (unsigned long)timestamp);
    }
};
