#ifndef BNO055_SENSOR_DATA_H
#define BNO055_SENSOR_DATA_H
#include "sensors/SensorData.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

class BNO055SensorData : public SensorData
{
public:
    BNO055SensorData() : board_temperature(-39), system_calibration(0), gyro_calibration(0), accel_calibration(0), mag_calibration(0) {}

    /**
     * @brief Get the Orientation on x,y,z axes in degrees
     *
     * @return sensors_event_t orientation
     */
    sensors_event_t getOrientation() const;
    /**
     * @brief Get the Angular Velocity on x,y,z axes in rad/s
     *
     * @return sensors_event_t angularVelocity
     */
    sensors_event_t getAngularVelocity() const;
    /**
     * @brief Get the Linear Acceleration on x,y,z axes in m/s^2
     *
     * @return sensors_event_t linearAcceleration
     */
    sensors_event_t getLinearAcceleration() const; // Linear acceleration on x,y,z axes in m/s^2
    /**
     * @brief Get the Magnetometer on x,y,z axes in uT
     *
     * @return sensors_event_t magnetometer
     */
    sensors_event_t getMagnetometer() const;       // Magnetometer on x,y,z axes in uT
    /**
     * @brief Get the Accelerometer on x,y,z axes in m/s^2
     *
     * @return sensors_event_t accelerometer
     */
    sensors_event_t getAccelerometer() const;      // Accelerometer on x,y,z axes in m/s^2
    /**
     * @brief Get the Gravity on x,y,z axes in m/s^2
     *
     * @return sensors_event_t gravity
     */
    sensors_event_t getGravity() const;            // Gravity on x,y,z axes in m/s^2

    /**
     * @brief Get the Board Temperature 
     * @note The temperature is in °C
     * @note The working temperature of the sensor is between -40° and 85°C
     * @return int8_t 
     */
    int8_t getBoardTemperature() const;

    /**
     * @brief Get the System Calibration status
     * @note 0=uncalibrated, 1=partially calibrated, 2=mostly calibrated, 3=fully calibrated
     * @return uint8_t 
     */
    uint8_t getSystemCalibration() const;
    /**
     * @brief Get the Gyroscope Calibration status
     * @note 0=uncalibrated, 1=partially calibrated, 2=mostly calibrated, 3=fully calibrated
     * @return uint8_t 
     */
    uint8_t getGyroCalibration() const;
    /**
     * @brief Get the Accelerometer Calibration status
     * @note 0=uncalibrated, 1=partially calibrated, 2=mostly calibrated, 3=fully calibrated
     * @return uint8_t 
     */
    uint8_t getAccelCalibration() const;
    /**
     * @brief Get the Magnetometer Calibration status
     * @note 0=uncalibrated, 1=partially calibrated, 2=mostly calibrated, 3=fully calibrated
     * @return uint8_t 
     */
    uint8_t getMagCalibration() const;

    String toString() override;

private:
    sensors_event_t orientation;
    sensors_event_t angularVelocity;
    sensors_event_t linearAcceleration;
    sensors_event_t magnetometer;
    sensors_event_t accelerometer;
    sensors_event_t gravity;
    int8_t board_temperature;
    uint8_t system_calibration;
    uint8_t gyro_calibration;
    uint8_t accel_calibration;
    uint8_t mag_calibration;
};
#endif // BNO055_SENSOR_DATA_H