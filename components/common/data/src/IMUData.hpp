#pragma once
#include <SensorData.hpp>
#include <string.h>

/**
 * @brief Data structure for a general IMU sensor
 * 
 */
class IMUData : public SensorData
{
public:
    IMUData(std::string sensorName) : SensorData(sensorName) {}

    // Calibration status
    uint8_t calibration_sys = 0;
    uint8_t calibration_gyro = 0;
    uint8_t calibration_accel = 0;
    uint8_t calibration_mag = 0;
    
    // Orientation (Euler angles in degrees)
    float orientation_x = 0.0f;
    float orientation_y = 0.0f;
    float orientation_z = 0.0f;
    
    // Angular velocity (rad/s)
    float angular_velocity_x = 0.0f;
    float angular_velocity_y = 0.0f;
    float angular_velocity_z = 0.0f;
    
    // Linear acceleration (m/s^2)
    float linear_acceleration_x = 0.0f;
    float linear_acceleration_y = 0.0f;
    float linear_acceleration_z = 0.0f;
    
    // Total acceleration (m/s^2)
    float acceleration_x = 0.0f;
    float acceleration_y = 0.0f;
    float acceleration_z = 0.0f;

    // Gravity vector (m/s^2)
    float gravity_x = 0.0f;
    float gravity_y = 0.0f;
    float gravity_z = 0.0f;
    
    // Magnetometer (microtesla)
    float magnetometer_x = 0.0f;
    float magnetometer_y = 0.0f;
    float magnetometer_z = 0.0f;
    
    // Quaternion orientation
    double quaternion_w = 1.0;
    double quaternion_x = 0.0;
    double quaternion_y = 0.0;
    double quaternion_z = 0.0;
    
    // Temperature (°C)
    float temperature = 0.0f;
    
    // Metadata
    float timestamp = 0;
    
    json toJSON() const override {
        json j;
        j["source"] = getSensorName();

        json sensorDataJson;
        sensorDataJson["calibration_sys"] = calibration_sys;
        sensorDataJson["calibration_gyro"] = calibration_gyro;
        sensorDataJson["calibration_accel"] = calibration_accel;
        sensorDataJson["calibration_mag"] = calibration_mag;
        sensorDataJson["orientation_x"] = orientation_x;
        sensorDataJson["orientation_y"] = orientation_y;
        sensorDataJson["orientation_z"] = orientation_z;
        sensorDataJson["angular_velocity_x"] = angular_velocity_x;
        sensorDataJson["angular_velocity_y"] = angular_velocity_y;
        sensorDataJson["angular_velocity_z"] = angular_velocity_z;
        sensorDataJson["linear_acceleration_x"] = linear_acceleration_x;
        sensorDataJson["linear_acceleration_y"] = linear_acceleration_y;
        sensorDataJson["linear_acceleration_z"] = linear_acceleration_z;
        sensorDataJson["acceleration_x"] = acceleration_x;
        sensorDataJson["acceleration_y"] = acceleration_y;
        sensorDataJson["acceleration_z"] = acceleration_z;
        sensorDataJson["gravity_x"] = gravity_x;
        sensorDataJson["gravity_y"] = gravity_y;
        sensorDataJson["gravity_z"] = gravity_z;
        sensorDataJson["magnetometer_x"] = magnetometer_x;
        sensorDataJson["magnetometer_y"] = magnetometer_y;
        sensorDataJson["magnetometer_z"] = magnetometer_z;
        sensorDataJson["quaternion_w"] = quaternion_w;
        sensorDataJson["quaternion_x"] = quaternion_x;
        sensorDataJson["quaternion_y"] = quaternion_y;
        sensorDataJson["quaternion_z"] = quaternion_z;
        sensorDataJson["temperature"] = temperature;
        sensorDataJson["timestamp"] = timestamp;

        j["sensorData"] = sensorDataJson;

        return j;
    }
};