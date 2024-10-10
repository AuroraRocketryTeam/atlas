#include "BNO055SensorData.h"

String BNO055SensorData::toString()
{
    sensors_vec_t orient = this->getOrientation(), angVel = this->getAngularVelocity(),
                  linAcc = this->getLinearAcceleration(), mag = this->getMagnetometer(),
                  accel = this->getAccelerometer(), grav = this->getGravity();

    String result = "Orientation: [" + String(orient.x) + ", " + String(orient.y) + ", " + String(orient.z) + "]\n";
    result += "Angular Velocity: [" + String(angVel.x) + ", " + String(angVel.y) + ", " + String(angVel.z) + "]\n";
    result += "Linear Acceleration: [" + String(linAcc.x) + ", " + String(linAcc.y) + ", " + String(linAcc.z) + "]\n";
    result += "Magnetometer: [" + String(mag.x) + ", " + String(mag.y) + ", " + String(mag.z) + "]\n";
    result += "Accelerometer: [" + String(accel.x) + ", " + String(accel.y) + ", " + String(accel.z) + "]\n";
    result += "Gravity: [" + String(grav.x) + ", " + String(grav.y) + ", " + String(grav.z) + "]\n";
    result += "Board Temperature: " + String(this->getBoardTemperature()) + " °C\n";

    result += String("Calibration Levels:") +
              " Sys=" + String(this->getSystemCalibration()) +
              " Gyro=" + String(this->getGyroCalibration()) +
              " Accel=" + String(this->getAccelCalibration()) +
              " Mag=" + String(this->getMagCalibration());
    return result;
}

void BNO055SensorData::log(CborLogger &logger)
{
    json serialized_data;
    sensors_vec_t orient = this->getOrientation(), angVel = this->getAngularVelocity(),
                  linAcc = this->getLinearAcceleration(), mag = this->getMagnetometer(),
                  accel = this->getAccelerometer(), grav = this->getGravity();
    json orientation_data = {orient.x, orient.y, orient.z};
    json angular_velocity = {angVel.x, angVel.y, angVel.z};
    json linear_acceleration = {linAcc.x, linAcc.y, linAcc.z};
    json magnetometer_data = {mag.x, mag.y, mag.z};
    json accelerometer_data = {accel.x, accel.y, accel.z};
    json gravity_data = {grav.x, grav.y, grav.z};
    json calibration_levels = {
        {"System", this->getSystemCalibration()},
        {"Gyro", this->getGyroCalibration()},
        {"Accel", this->getAccelCalibration()},
        {"Mag", this->getMagCalibration()}};

    serialized_data["Orientation"] = orientation_data;
    serialized_data["Angular Velocity"] = angular_velocity;
    serialized_data["Linear Acceleration"] = linear_acceleration;
    serialized_data["Magnetometer"] = magnetometer_data;
    serialized_data["Accelerometer"] = accelerometer_data;
    serialized_data["Gravity"] = gravity_data;
    serialized_data["Board Temperature"] = this->getBoardTemperature();
    serialized_data["Calibration Levels"] = calibration_levels;

    logger.addEntry("BNO055", serialized_data);
}
