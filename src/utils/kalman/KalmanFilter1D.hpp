#ifndef KALMANFILTER1D_HPP
#define KALMANFILTER1D_HPP

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

#define EKF_N 6 // Size of state space [y-position, y-velocity, y-acceleration, w-quaternion, x-quaternion, y-quaternion, z-quaternion] 
#define EKF_M 7 // Size of observation (measurement) space [3-positions, 3-accelerations, 4-quaternion_rot]

#include <tinyekf.h>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <random>
#include <vector>
#include <ArduinoEigen.h>
#include "esp_task_wdt.h"

/**
 * @brief One-dimensional Extended Kalman Filter for sensor fusion.
 * 
 * This class implements a 1D EKF for fusing IMU (accelerometer, gyroscope) and barometer data.
 * The state vector includes position, velocity, and orientation (as quaternion).
 * The filter estimates the vertical position and velocity, as well as orientation, 
 * compensating for sensor biases and gravity.
 */
class KalmanFilter1D {
public:
    /**
     * @brief Construct a new KalmanFilter1D object and perform initial calibration.
     * 
     * @param gravity_value Initial gravity vector (from accelerometer).
     * @param magnometer_value Initial magnetic field vector (from magnetometer).
     */
    KalmanFilter1D(Eigen::Vector3f gravity_value, Eigen::Vector3f magnometer_value);
    
    /**
     * @brief Perform one EKF prediction and update step.
     * 
     * @param dt Time step in seconds.
     * @param omega Gyroscope readings [rad/s].
     * @param accel Accelerometer readings [m/s^2].
     * @param pressure Barometer reading (altitude or pressure).
     * @return Estimated position and velocity vectors.
     */
    std::vector<std::vector<float>> step(float dt, float omega[3], float accel[3], float pressure);
    
    /**
     * @brief Get the current EKF state vector.
     * 
     * @return Pointer to the state vector.
     */
    float* state();

private:
    // TinyEKF structure for the Extended Kalman Filter
    ekf_t ekf;

    // initial covariances of state noise, measurement noise
    // Q matrix (model) // We obtain this value from a comparison between a model and the real data.
    const float P0 = 1e-4; 
    const float V0 = 1e-4;
    const float q_a = 1e-8;
    const float b_a = 1e-8;
    const float b_g = 1e-8;

    // Bias of the sensors (initialized in the constructor functino with calibration data)
    Eigen::Vector3f bias_a; // Bias vector for accelerometer
    Eigen::Vector3f bias_g; // Bias vector for gyroscope

    // R matrix (measurements)
    const float A0 = 1e-3;
    const float G0 = 1e-6;
    const float Z0 = 1;
    const float R0 = 1; // !!! TODO: This is a placeholder for barometer variance, should be estimated based on barometer readings.

    // Process noise covariance
    float Q[EKF_N*EKF_N] = {
        P0, 0, 0, 0, 0, 0,
        0, V0, 0, 0, 0, 0,
        0, 0, q_a, 0, 0, 0,
        0, 0, 0, q_a, 0, 0,
        0, 0, 0, 0, q_a, 0,
        0, 0, 0, 0, 0, q_a
    };

    // Measurement noise covariance
    float R[EKF_M*EKF_M] = {
        A0, 0, 0, 0, 0, 0, 0,
        0, A0, 0, 0, 0, 0, 0,
        0, 0, A0, 0, 0, 0, 0,
        0, 0, 0, G0, 0, 0, 0,
        0, 0, 0, 0, G0, 0, 0,
        0, 0, 0, 0, 0, G0, 0,
        0, 0, 0, 0, 0, 0, R0
    };

    // Initially, the acceleration is constantly zero, so it won't change
    float H[EKF_M*EKF_N] = {
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0,
        1, 0, 0, 0, 0, 0,
    };
    
    // Measurement Jacobian with input/output relations
    float F[EKF_N*EKF_N];

    // Gravity vector in ENU coordinates
    const Eigen::Vector3f gravity{0, 0, -9.803};

    /**
     * @brief Calibrate the filter using gravity and magnetometer readings.
     * 
     * @param gravity_readings Gravity vector sample.
     * @param magnetometer_value Magnetometer vector sample.
     * @return Tuple of initial quaternion, accelerometer bias, gyroscope bias.
     */
    std::tuple<Eigen::Quaternionf, Eigen::Vector3f, Eigen::Vector3f> calibration(Eigen::Vector3f gravity_value, Eigen::Vector3f magnetometer_value);

    /**
     * @brief Rotate a vector from world to body frame using a quaternion.
     * 
     * @param q Quaternion representing rotation.
     * @param vec_world Vector in world frame.
     * @return Rotated vector in body frame.
     */
    Eigen::Vector3f rotateToBody(const Eigen::Quaternionf& q, const Eigen::Vector3f& vec_world);

    /**
     * @brief Compute the Jacobian of the acceleration measurement w.r.t. quaternion numerically.
     * 
     * @param dt Time step in seconds.
     * @param q_nominal Nominal quaternion.
     * @param accel_world Acceleration in world frame.
     * @param omega Angular velocity.
     * @param epsilon Perturbation for numerical differentiation.
     * @return Jacobian matrix (3x4).
     */
    Eigen::Matrix<float, 3, 4> computeHqAccelJacobian(const float dt, const Eigen::Quaternionf& q_nominal, const Eigen::Vector3f& accel_world, const Eigen::Vector3f& omega, float epsilon = 1e-5);

    /**
     * @brief EKF process and measurement model.
     * 
     * @param dt Time step.
     * @param fx Output: predicted state.
     * @param hx Output: predicted measurement.
     * @param omega_z Gyroscope readings.
     * @param accel_z Accelerometer readings.
     * @param pressure Barometer reading.
     */
    void run_model(float dt, float fx[EKF_N], float hx[EKF_M], float omega_z[3], float accel_z[3], float pressure);

    /**
     * @brief Compute the Jacobian of the process model numerically.
     * 
     * @param dt Time step.
     * @param omega_z Gyroscope readings.
     * @param accel_z Accelerometer readings.
     * @param h_pressure_sensor Barometer reading.
     */
    void computeJacobianF_tinyEKF(float dt, float omega_z[3], float accel_z[3], float h_pressure_sensor);

    /**
     * @brief Convert a quaternion to Euler angles (roll, pitch, yaw).
     * 
     * @param q Quaternion.
     * @param roll Output: roll angle.
     * @param pitch Output: pitch angle.
     * @param yaw Output: yaw angle.
     */
    void quaternionToEulerAngles(const Eigen::Quaternionf& q, float& roll, float& pitch, float& yaw);
};

#endif // KALMANFILTER1D_HPP