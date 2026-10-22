#pragma once
#include <SensorData.hpp>
#include <cstdarg>
#include <cstdint>
#include <limits>

/**
 * @brief Data structure for a general IMU sensor
 * 
 */
class IMUData : public SensorData
{
public:
    IMUData() = default;

    enum ValidGroup : uint16_t {
        VALIDITY_SENSOR_CALIBRATION = 1u << 0,
        VALIDITY_SYSTEM_CALIBRATION = 1u << 1,
        VALIDITY_ATTITUDE = 1u << 2,
        VALIDITY_ANGULAR_VELOCITY = 1u << 3,
        VALIDITY_LINEAR_ACCELERATION = 1u << 4,
        VALIDITY_ACCELERATION = 1u << 5,
        VALIDITY_GRAVITY = 1u << 6,
        VALIDITY_MAGNETOMETER = 1u << 7,
        VALIDITY_TEMPERATURE = 1u << 8,
    };
    uint16_t validGroups = 0;

    // Calibration status
    uint8_t calibration_sys = 0;
    uint8_t calibration_gyro = 0;
    uint8_t calibration_accel = 0;
    uint8_t calibration_mag = 0;
    
    // Orientation (Euler angles in degrees)
    float orientation_x = 0.0f;
    float orientation_y = 0.0f;
    float orientation_z = 0.0f;
    
    // Angular velocity (deg/s), matching the BNO055 native gyro output.
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
    
    size_t serializeJson(char* buffer, size_t maxLength) const {
        struct Writer {
            char* buffer;
            size_t capacity;
            size_t length = 0;
            bool failed = false;

            void append(const char* format, ...) __attribute__((format(printf, 2, 3))) {
                if (failed) return;
                const size_t available = length < capacity ? capacity - length : 0;
                va_list args;
                va_start(args, format);
                const int written = vsnprintf(available ? buffer + length : nullptr,
                                              available, format, args);
                va_end(args);
                if (written < 0) failed = true;
                else length += static_cast<size_t>(written);
            }
        };
        Writer writer{buffer, maxLength};

        writer.append("{\"source\":\"%s\",\"sensorData\":{\"t\":%lu",
               getSensorName(), static_cast<unsigned long>(timestamp));
        if (validGroups & VALIDITY_SYSTEM_CALIBRATION)
            writer.append(",\"csys\":%u", calibration_sys);
        if (validGroups & VALIDITY_SENSOR_CALIBRATION)
            writer.append(",\"cgyro\":%u,\"caccel\":%u,\"cmag\":%u",
                   calibration_gyro, calibration_accel, calibration_mag);
        if (validGroups & VALIDITY_ATTITUDE)
            writer.append(",\"ox\":%f,\"oy\":%f,\"oz\":%f,\"qw\":%f,\"qx\":%f,\"qy\":%f,\"qz\":%f",
                   orientation_x, orientation_y, orientation_z,
                   quaternion_w, quaternion_x, quaternion_y, quaternion_z);
        if (validGroups & VALIDITY_ANGULAR_VELOCITY)
            writer.append(",\"avx\":%f,\"avy\":%f,\"avz\":%f",
                   angular_velocity_x, angular_velocity_y, angular_velocity_z);
        if (validGroups & VALIDITY_LINEAR_ACCELERATION)
            writer.append(",\"lax\":%f,\"lay\":%f,\"laz\":%f",
                   linear_acceleration_x, linear_acceleration_y, linear_acceleration_z);
        if (validGroups & VALIDITY_ACCELERATION)
            writer.append(",\"ax\":%f,\"ay\":%f,\"az\":%f",
                   acceleration_x, acceleration_y, acceleration_z);
        if (validGroups & VALIDITY_GRAVITY)
            writer.append(",\"gx\":%f,\"gy\":%f,\"gz\":%f",
                   gravity_x, gravity_y, gravity_z);
        if (validGroups & VALIDITY_MAGNETOMETER)
            writer.append(",\"mx\":%f,\"my\":%f,\"mz\":%f",
                   magnetometer_x, magnetometer_y, magnetometer_z);
        if (validGroups & VALIDITY_TEMPERATURE)
            writer.append(",\"te\":%f", temperature);
        writer.append("}}\n");
        return writer.failed ? std::numeric_limits<size_t>::max() : writer.length;
    }
};
