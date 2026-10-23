#pragma once
#include <SensorData.hpp>
#include <cstdint>
#include <cstdio>

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
    
    size_t serializeJson(char* buffer, size_t maxLength) const {
        return std::snprintf(buffer, maxLength,
            "{\"src\":\"%s\",\"t\":%lu,\"sats\":%u,\"fix\":%u,\"lat\":%.8g,\"lon\":%.8g,"
            "\"alt_m\":%.6g,\"speed\":%.6g,\"hdop\":%.4g}\n",
            getSensorName(), static_cast<unsigned long>(timestamp), satellites, fixType,
            latitude, longitude, altitude, ground_speed, hdop);
    }
};
