/*
// Macros needed for a conflict between similar macro variables names of Arduino.h and Eigen.h
#ifdef B1
#undef B1
#endif
#ifdef B2
#undef B2
#endif
#ifdef B3
#undef B3
#endif
#ifdef B0
#undef B0
#endif

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <string>
#include <stdlib.h>

#include <ArduinoEigen.h>

// Calibration Phase. This function must run when the Launcher is still in the launching position.
// It will also use the magnetometer to align the Z axis with the North. We might not get exact Norht since the readings
// might be modified by the presence of the aluminum frame. It is just to get a rough idea of the North.

int main() {

    // Gravity vector got from the accelerometer
    std::vector<Eigen::Vector3f> gravity_readings;
    for (int i = 0; i < 200; ++i) {
        // Simulate reading from accelerometer
        Eigen::Vector3f reading(0, 0, 9.81); // Replace with actual reading of accelerometer
        gravity_readings.push_back(reading);
        // delay(10); // Simulate delay between readings
    }
    // TO DO: COMPUTE STANDARD DEVIATION OF THE GRAVITY READINGS, IF TOO HIGH, REPEAT THE CALIBRATION

    // Calculate the mean of the gravity readings
    Eigen::Vector3f gravity_sum = Eigen::Vector3f::Zero();
    for (const auto& reading : gravity_readings) {
        gravity_sum += reading;
    }
    Eigen::Vector3f gravity = gravity_sum / gravity_readings.size();
    Eigen::Vector3f expected_gravity(0, 0, 9.80537); // Expected gravity vector for specific location (Forlì - 34 m over sea level)
    // Got from: https://www.sensorsone.com/local-gravity-calculator/

    // Rotation Axis: cross product of gravity in local R.F. and Z R.F.
    Eigen::Vector3f z(0, 0, 1);
    Eigen::Vector3f g = gravity.normalized();
    Eigen::Vector3f axis = g.cross(z);
    axis.normalize();

    // Angle: acos of the dot product
    float angle = acos(g.dot(z));

    // Quaternion
    Eigen::Quaternionf q_rot(Eigen::AngleAxisf(angle, axis));

    // Rotate the quaternion to align with north
    // Reading of the angle relative to North
    Eigen::Vector3f magnetometer(21.87, 27.56, -19.75); // Magnetometer reading
    Eigen::Vector3f magnetometer_z_aligned = q_rot * magnetometer;
    float yaw = atan2f(magnetometer[1], magnetometer[0]); // Yaw angle in radians
    Eigen::Quaternionf q_yaw(Eigen::AngleAxisf(yaw, Eigen::Vector3f(0, 0, 1)));
    Eigen::Quaternionf initial_quaternion = q_yaw * q_rot;
    initial_quaternion.normalize();

    // Bias of the accelerometer
    Eigen::Vector3f initial_gravity = initial_quaternion * gravity;
    Eigen::Vector3f bias_a = initial_gravity - expected_gravity;

    // Bias of the gyroscope
    Eigen::Vector3f initial_omega(0, 0, 0); // Mean of various readings
    Eigen::Vector3f bias_w = initial_omega - Eigen::Vector3f(0, 0, 0); // Assuming no rotation

    return 0;
}
*/