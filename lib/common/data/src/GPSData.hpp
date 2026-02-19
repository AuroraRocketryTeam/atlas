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
    uint8_t satellites;
    
    // Fix type (0=no fix, 1=dead reckoning, 2=2D fix, 3=3D fix, 4=GNSS+dead reckoning, 5=time-only fix)
    uint8_t fixType;
    
    // Latitude in degrees * 10^7
    int32_t latitude;
    
    // Longitude in degrees * 10^7
    int32_t longitude;
    
    // Altitude above mean sea level in meters
    double altitude;
    
    // Ground speed in meters per second
    double ground_speed;
    
    // Horizontal dilution of precision
    double hdop;
    
    // Metadata
    uint32_t timestamp;

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
