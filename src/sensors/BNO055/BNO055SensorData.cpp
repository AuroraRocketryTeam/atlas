#include "BNO055SensorData.h"

sensors_event_t BNO055SensorData::getOrientation() const
{
    return this->orientation;
}

sensors_event_t BNO055SensorData::getAngularVelocity() const
{
    return this->angularVelocity;
}

sensors_event_t BNO055SensorData::getLinearAcceleration() const
{
    return this->linearAcceleration;
}

sensors_event_t BNO055SensorData::getMagnetometer() const
{
    return this->magnetometer;
}

sensors_event_t BNO055SensorData::getAccelerometer() const
{
    return this->accelerometer;
}

sensors_event_t BNO055SensorData::getGravity() const
{
    return this->gravity;
}

int8_t BNO055SensorData::getBoardTemperature() const
{
    return this->board_temperature;
}

uint8_t BNO055SensorData::getSystemCalibration() const
{
    return this->system_calibration;
}

uint8_t BNO055SensorData::getGyroCalibration() const
{
    return this->gyro_calibration;
}

uint8_t BNO055SensorData::getAccelCalibration() const
{
    return this->accel_calibration;
}

uint8_t BNO055SensorData::getMagCalibration() const
{
    return this->mag_calibration;
}

String BNO055SensorData::toString()
{
    String result = "Orientation: [" + String(this->orientation.orientation.x) + ", " + String(this->orientation.orientation.y) + ", " + String(this->orientation.orientation.z) + "]\n";
    result += "Angular Velocity: [" + String(this->angularVelocity.gyro.x) + ", " + String(this->angularVelocity.gyro.y) + ", " + String(this->angularVelocity.gyro.z) + "]\n";
    result += "Linear Acceleration: [" + String(this->linearAcceleration.acceleration.x) + ", " + String(this->linearAcceleration.acceleration.y) + ", " + String(this->linearAcceleration.acceleration.z) + "]\n";
    result += "Magnetometer: [" + String(this->magnetometer.magnetic.x) + ", " + String(this->magnetometer.magnetic.y) + ", " + String(this->magnetometer.magnetic.z) + "]\n";
    result += "Accelerometer: [" + String(this->accelerometer.acceleration.x) + ", " + String(this->accelerometer.acceleration.y) + ", " + String(this->accelerometer.acceleration.z) + "]\n";
    result += "Gravity: [" + String(this->gravity.acceleration.x) + ", " + String(this->gravity.acceleration.y) + ", " + String(this->gravity.acceleration.z) + "]\n";
    result += "Board Temperature: " + String(this->board_temperature) + " °C\n";
    result += "Calibration Levels: Sys=" + String(this->system_calibration) + " Gyro=" + String(this->gyro_calibration) + " Accel=" + String(this->accel_calibration) + " Mag=" + String(this->mag_calibration);
    return result;
}
