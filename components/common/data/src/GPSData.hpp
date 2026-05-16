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
    GPSData(std::string sensorName="GPS") : SensorData(sensorName) {}

    // Number of satellites in view
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
    float timestamp = 0;
    json toJSON() const override {
        json j;
        j["source"] = getSensorName();

        json sensorDataJson;
        sensorDataJson["satellites"] = satellites;
        sensorDataJson["fixType"] = fixType;
        sensorDataJson["latitude"] = latitude;
        sensorDataJson["longitude"] = longitude;
        sensorDataJson["altitude"] = altitude;
        sensorDataJson["ground_speed"] = ground_speed;
        sensorDataJson["hdop"] = hdop;
        sensorDataJson["timestamp"] = timestamp;

        j["sensorData"] = sensorDataJson;

        return j;
    }
};
