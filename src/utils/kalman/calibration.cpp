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

Eigen::Vector3f standard_deviation(const std::vector<Eigen::Vector3f>& readings) {
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const auto& reading : readings) {
        mean += reading;
    }
    mean /= readings.size();

    Eigen::Vector3f variance = Eigen::Vector3f::Zero();
    for (const auto& reading : readings) {
        Eigen::Vector3f diff = reading - mean;
        variance += diff.cwiseProduct(diff); // componente a componente: (x^2, y^2, z^2)
    }
    variance /= static_cast<float>(readings.size() - 1);

    return variance.cwiseSqrt(); // raíz cuadrada componente a componente
}

int main() {

    Eigen::Vector3f TolSTD(0.1, 0.1, 0.1); // Tolerance for standard deviation
    Eigen::Vector3f std(1, 1, 1); // Standard deviation of the gravity readings

    while ((std.array() > TolSTD.array()).any()) {
        // Gravity vector got from the accelerometer
        std::vector<Eigen::Vector3f> gravity_readings;
        for (int i = 0; i < 200; ++i) {
            // Simulate reading from accelerometer
            Eigen::Vector3f reading(0, 0, 9.81); // Replace with actual reading of accelerometer
            gravity_readings.push_back(reading);
            // delay(10); // Simulate delay between readings
        }
        Eigen::Vector3f std = standard_deviation(gravity_readings);
        if ((std.array() > TolSTD.array()).any()) {
            std::cout << "Standard deviation too high, repeat calibration." << std::endl;
            // delay(1000); // Simulate delay before next calibration attempt
        } else {
            std::cout << "Measuring succesful!" << std::endl;
        }
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
    Eigen::Vector3f north_body = magnetometer.normalized();
    Eigen::Vector3f y_axis_abs(0, 1, 0); // North vector in ENU frame
    Eigen::Vector3f north_abs = q_rot * north_body; // Rotate magntetic north to body frame
    float angle_rad = std::acos(north_abs.dot(y_axis_abs) / north_abs.norm());
    Eigen::Quaternionf q_north(Eigen::AngleAxisf(angle_rad, Eigen::Vector3f(0, 0, 1))); // Quaternion to align with North
    Eigen::Quaternionf initial_quaternion = q_north * q_rot;    // Rotation Abs RF to align with North
    initial_quaternion.normalize();

    // Bias of the accelerometer. gravity is in ENU coordinates, so we need to rotate it to match the sensor's frame of reference.
    Eigen::Quaternionf q_absolute_to_body = initial_quaternion.conjugate();
    Eigen::Vector3f initial_gravity_body = q_absolute_to_body * gravity;
    Eigen::Vector3f expected_gravity_body = q_absolute_to_body * expected_gravity;
    Eigen::Vector3f bias_a = initial_gravity_body - expected_gravity_body;

    // Bias of the gyroscope
    Eigen::Vector3f initial_omega(0, 0, 0); // Mean of various readings
    Eigen::Vector3f bias_w = initial_omega - Eigen::Vector3f(0, 0, 0); // Assuming no rotation

    return 0;
}
*/