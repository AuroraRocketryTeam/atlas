#pragma once

#include <BNO055SensorInterface.hpp>
#include <SensorData.hpp>
#include <ISensor.hpp>

/**
 * @class BNO055Sensor
 * @brief High-level driver for the Bosch BNO055 IMU using BNO055SensorInterface.
 *
 * Provides initialization, optional calibration helpers, hardware self-test and
 * a consolidated data readout via ISensor::getData(). Typical SensorData keys
 * include accelerometer, angular velocity (gyroscope), orientation and board
 * temperature depending on interface configuration.
 */
class BNO055Sensor : public ISensor
{
public:
    /** @brief Construct a new BNO055 sensor wrapper. */
    BNO055Sensor();

    /**
     * @brief Initialize the BNO055 device and prepare it for measurements.
     * @return true on success, false otherwise.
     */
    bool init() override;

    /**
     * @brief Run the device calibration routine (blocking until complete if implemented).
     * @return true if calibration completed successfully, false otherwise.
     */
    bool calibrate();

    /**
     * @brief Execute a hardware self-test if supported by the interface.
     * @return true if the test passes, false otherwise.
     */
    bool hardwareTest();

    /**
     * @brief Read the latest available IMU data and package it as SensorData.
     * @return SensorData populated with the latest readings; std::nullopt on failure.
     */
    std::optional<SensorData> getData() override;
    
    /**
     * @struct CalibrationStatus
     * @brief Per-subsystem calibration completeness indicators.
     * Fields typically range 0..3 where 3 indicates fully calibrated.
     */
    struct CalibrationStatus {
        uint8_t sys;   /**< System calibration level. */
        uint8_t gyro;  /**< Gyroscope calibration level. */
        uint8_t accel; /**< Accelerometer calibration level. */
        uint8_t mag;   /**< Magnetometer calibration level. */
    };

    /**
     * @brief Get the current calibration status across subsystems.
     * @return CalibrationStatus with subsystem indicators.
     */
    CalibrationStatus getCalibration();

private:
    /** @brief Low-level interface to the BNO055 device. */
    BNO055SensorInterface bno_interface;
};

