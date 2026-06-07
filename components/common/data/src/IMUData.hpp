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
    IMUData() = default;

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
    uint32_t timestamp = 0;
    
    size_t serializeJson(char* buffer, size_t maxLength) const override {
        return snprintf(buffer, maxLength,
            "{\"source\":\"%s\",\"sensorData\":{"
            "\"csys\":%u,\"cgyro\":%u,\"caccel\":%u,\"cmag\":%u,"
            "\"ox\":%f,\"oy\":%f,\"oz\":%f,"
            "\"avx\":%f,\"avy\":%f,\"avz\":%f,"
            "\"lax\":%f,\"lay\":%f,\"laz\":%f,"
            "\"ax\":%f,\"ay\":%f,\"az\":%f,"
            "\"gx\":%f,\"gy\":%f,\"gz\":%f,"
            "\"mx\":%f,\"my\":%f,\"mz\":%f,"
            "\"qw\":%f,\"qx\":%f,\"qy\":%f,\"qz\":%f,"
            "\"te\":%f,\"t\":%lu}}\n",
            getSensorName().c_str(),
            calibration_sys, calibration_gyro, calibration_accel, calibration_mag,
            orientation_x, orientation_y, orientation_z,
            angular_velocity_x, angular_velocity_y, angular_velocity_z,
            linear_acceleration_x, linear_acceleration_y, linear_acceleration_z,
            acceleration_x, acceleration_y, acceleration_z,
            gravity_x, gravity_y, gravity_z,
            magnetometer_x, magnetometer_y, magnetometer_z,
            quaternion_w, quaternion_x, quaternion_y, quaternion_z,
            temperature, (unsigned long)timestamp);
    }
};