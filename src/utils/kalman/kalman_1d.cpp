/*
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <stdio.h>
#include <string.h>
#include <string>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <random>  // Include for random number generation

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "../lib/Eigen/Geometry"
#include "../lib/Eigen/Dense"

// Get Data from files
// Acceleration
std::vector<float> getAcc(int lineNumberSought) {
    std::string line, csvItem;
    std::vector<float> acc;
    std::ifstream myfile("../csv_measurements/accFlt.csv");

    int lineNumber = 0;
    if (myfile.is_open()) {
        while (getline(myfile,line)) {
            if(lineNumber == lineNumberSought) {
                std::istringstream myline(line);
                while(getline(myline, csvItem, ',')) {
                    acc.push_back(std::stof(csvItem));
                }
                break;
            }
            
            lineNumber++;
        }

        myfile.close();
    
    } else {
        std::cerr << "Unable to load file!" << std::endl;
    }
    for (auto& val : acc) {
        val *= 9.803f; // Convert to m/s^2
    }
    return acc;
}

// Angular velocity
std::vector<float> getOmega(int lineNumberSought) {
    std::string line, csvItem;
    std::vector<float> omega;
    std::ifstream myfile("../csv_measurements/wVelFlt.csv");

    int lineNumber = 0;
    if (myfile.is_open()) {
        while (getline(myfile,line)) {
            if(lineNumber == lineNumberSought) {
                std::istringstream myline(line);
                while(getline(myline, csvItem, ',')) {
                    omega.push_back(std::stof(csvItem));
                }
                break;
            }
            
            lineNumber++;
        }

        myfile.close();
    
    } else {
        std::cerr << "Unable to load file!" << std::endl;
    }
    return omega;
}

// Altitude from pressure sensor
std::vector<float> getAltitudes(int lineNumberSought) {
    std::string line;
    std::vector<float> altitudes;
    std::ifstream myfile("../csv_measurements/altimeter.csv");

    int lineNumber = 0;
    if (myfile.is_open()) {
        while (getline(myfile, line)) {
            if (lineNumber == lineNumberSought) {
                try {
                    altitudes.push_back(std::stof(line));
                } catch (const std::invalid_argument& e) {
                    std::cerr << "Invalid float in line: " << line << std::endl;
                }
                break;
            }
            lineNumber++;
        }
        myfile.close();
    } else {
        std::cerr << "Unable to load file!" << std::endl;
    }

    return altitudes;
}

// Save values to a CSV file
void saveToCSV(const std::vector<float>& acc, const std::string& filename) {
    // Open in append mode to avoid overwriting previous data
    std::ofstream file(filename, std::ios::app);

    if (file.is_open()) {
        // Make sure the vector has exactly 3 values
        if (acc.size() == 3) {
            file << acc[0] << "," << acc[1] << "," << acc[2] << "\n";
        } else {
            std::cerr << "Error: vector must have exactly 3 elements. SaveToCSV\n";
        }

        file.close();
    } else {
        std::cerr << "Unable to open file for writing.\n";
    }
}

void quaternionToEulerAngles(const Eigen::Quaternionf& q, float& roll, float& pitch, float& yaw) {
    // Extract quaternion components
    float w = q.w();
    float x = q.x();
    float y = q.y();
    float z = q.z();

    // Calculate roll (ϕ)
    roll = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));

    // Calculate pitch (θ)
    pitch = std::asin(2.0f * (w * y - z * x));

    // Calculate yaw (ψ)
    yaw = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

// These must be defined before including TinyEKF.h
#define EKF_N 6 // Size of state space [3-positions, 3-velocities, 3-accelerations, 4-quaternion_rot] 
#define EKF_M 7 // Size of observation (measurement) space [3-positions, 3-accelerations, 4-quaternion_rot]

#include "../lib/TinyEKF-master/src/tinyekf.h"

static const float dt = 0.005; // 200 Hz

// initial covariances of state noise, measurement noise
// Q matrix (model) // We obtain this value from a comparison between a model and the real data.
static const float P0 = 1e-4; 
static const float V0 = 1e-4;
static const float q_a = 1e-8;
static const float b_a = 1e-8;
static const float b_g = 1e-8;

// Bias of the sensors: obtained from the calibration of the sensors
static const float b_a_x = -0.0005f; // Bias in x-axis
static const float b_a_y = -0.0061f; // Bias in y-axis
static const float b_a_z = 0.0009f; // Bias in z-axis
static const float b_g_x = 0.0f; // Bias in x-axis
static const float b_g_y = 0.0f; // Bias in y-axis
static const float b_g_z = 0.0f; // Bias in z-axis
static const Eigen::Vector3f bias_a(b_a_x, b_a_y, b_a_z); // Bias vector for accelerometer
static const Eigen::Vector3f bias_g(b_g_x, b_g_y, b_g_z); // Bias vector for gyroscope

// R matrix (measurements)
static const float A0 = 1e-3;
static const float G0 = 1e-6;
static const float Z0 = 1;

float estimateBaroVar(float v) {
    float std = (std::abs(v) / 300.0f) * 29.0f + 1.0f;
    return std * std;
}

static const Eigen::Vector3f gravity(0, 0, -9.803); // Gravity vector in ENU coordinates

// Set fixed process-noise covariance matrix Q, see [1]  ---------------------

static const float Q[EKF_N*EKF_N] = {

    P0, 0, 0, 0, 0, 0,
    0, V0, 0, 0, 0, 0,
    0, 0, q_a, 0, 0, 0,
    0, 0, 0, q_a, 0, 0,
    0, 0, 0, 0, q_a, 0,
    0, 0, 0, 0, 0, q_a
};

// Set fixed measurement noise covariance matrix R ----------------------------

float R[EKF_M*EKF_M] = {
    A0, 0, 0, 0, 0, 0, 0,
    0, A0, 0, 0, 0, 0, 0,
    0, 0, A0, 0, 0, 0, 0,
    0, 0, 0, G0, 0, 0, 0,
    0, 0, 0, 0, G0, 0, 0,
    0, 0, 0, 0, 0, G0, 0,
    0, 0, 0, 0, 0, 0, estimateBaroVar(0)
};

// Jacobian matrix
float F[EKF_N*EKF_N] = {
    1, dt, 0, 0, 0, 0,
    0, 1, 0, 0, 0, 0,

    0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0,
    0, 0, 0, 0, 1, 0,
    0, 0, 0, 0, 0, 1

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

Eigen::Vector3f rotateToBody(const Eigen::Quaternionf& q, const Eigen::Vector3f& vec_world) {
    return q.conjugate() * vec_world;  // Equivalent to R^T * vec
}

// Compute H_q^{(a)} numerically
Eigen::Matrix<float, 3, 4> computeHqAccelJacobian(
    const Eigen::Quaternionf& q_nominal,
    const Eigen::Vector3f& accel_z,
    const Eigen::Vector3f& omega,
    float epsilon = 1e-5)
{
    // Update quaternion
    // omega = [wx, wy, wz] in rad/s
    Eigen::Vector3f omega_eigen = omega - bias_g; // Subtract gyroscope bias
    Eigen::Vector3f axis = omega_eigen.normalized();
    float theta = omega_eigen.norm() * dt;
    Eigen::Quaternionf delta_q(Eigen::AngleAxisf(theta, axis)); // delta_q = cos(theta/2) + axis*sin(theta/2)

    Eigen::Quaternionf q_rot =  q_nominal * delta_q;
    q_rot.normalize();
    
    Eigen::Vector3f accel_world = q_rot * (accel_z - bias_a) + gravity;  // Equivalent to q * a * q.inverse()

    // std::cout << "Line: " << lineNum << ", Accel: " << accel_abs.transpose() << std::endl; 
    Eigen::Matrix<float, 3, 4> H;
    Eigen::Vector4f q_vec = q_nominal.coeffs();  // (x, y, z, w)

    for (int i = 0; i < 4; ++i) {
        Eigen::Vector4f dq = Eigen::Vector4f::Zero();
        dq(i) = epsilon;

        // Perturb positively
        Eigen::Vector4f q_plus_vec = q_vec + dq;
        Eigen::Quaternionf q_plus(q_plus_vec(3), q_plus_vec(0), q_plus_vec(1), q_plus_vec(2));
        q_plus.normalize();

        // Perturb negatively
        Eigen::Vector4f q_minus_vec = q_vec - dq;
        Eigen::Quaternionf q_minus(q_minus_vec(3), q_minus_vec(0), q_minus_vec(1), q_minus_vec(2));
        q_minus.normalize();

        // Rotate vector under perturbed quaternions
        Eigen::Vector3f a_plus = rotateToBody(q_plus, accel_world);
        Eigen::Vector3f a_minus = rotateToBody(q_minus, accel_world);

        // Central difference
        H.col(i) = (a_plus - a_minus) / (2.0 * epsilon);
    }

    return H;  // size 3x4
}

static void initialize_ekf(ekf_t *ekf) {
    // Position
    ekf -> x[0] = 0;

    // Velocity
    ekf -> x[1] = 0;

    // Quaternion: received from calibration phase
    ekf -> x[2] = 0.9541;
    ekf -> x[3] = -0.0571;
    ekf -> x[4] = 0.2734;
    ekf -> x[5] = -0.1079;

}

static void run_model(ekf_t * ekf, float dt, float fx[EKF_N], float hx[EKF_M], float omega_x, float omega_y, float omega_z, float accel_z[3], float h_pressure_sensor) {

    // Update quaternion; omega = [wx, wy, wz] in rad/s
    Eigen::Vector3f omega_eigen(omega_x - bias_g[0], omega_y - bias_g[1], omega_z - bias_g[2]);
    float theta = omega_eigen.norm() * dt;
    Eigen::Vector3f axis = omega_eigen.normalized();
    Eigen::Quaternionf delta_q(Eigen::AngleAxisf(theta, axis)); // delta_q = cos(theta/2) + axis*sin(theta/2)
    Eigen::Quaternionf q_nominal(ekf->x[2], ekf->x[3], ekf->x[4], ekf->x[5]);
    Eigen::Quaternionf q_rot =  q_nominal*delta_q;
    q_rot.normalize();

    Eigen::Vector3f acc_body(accel_z[0], accel_z[1], accel_z[2]);
    Eigen::Vector3f accel_abs = q_rot * (acc_body - bias_a) + gravity;  // Equivalent to q * a * q.inverse()

    // Position
    fx[0] = (float)(ekf->x[0] + ekf->x[1]*dt);
    
    // Velocities
    fx[1] = (float)(ekf->x[1] + accel_abs[2]*dt);

    // Quaternion
    fx[2] = q_rot.w();
    fx[3] = q_rot.x();
    fx[4] = q_rot.y();
    fx[5] = q_rot.z();

    // Measurements
    // Here we have to put the expected measurements of the acceleration /Review for future updates
    hx[0] = accel_z[0] + bias_a[0];
    hx[1] = accel_z[1] + bias_a[1];
    hx[2] = accel_z[2] + bias_a[2];
    hx[3] = omega_x + bias_g[0];
    hx[4] = omega_y + bias_g[1];
    hx[5] = omega_z + bias_g[2];
    hx[6] = h_pressure_sensor;
}

void computeJacobianF_tinyEKF(ekf_t* ekf, float dt, float omega_x, float omega_y, float omega_z, float accel_z[3], float F_out[EKF_N * EKF_N], float h_pressure_sensor) {
    const float epsilon = 1e-5f;
    float fx_base[EKF_N];
    float hx_dummy[EKF_M]; // Not used
    std::vector<float> original_state(ekf->x, ekf->x + EKF_N);

    run_model(ekf, dt, fx_base, hx_dummy, omega_x, omega_y, omega_z, accel_z, h_pressure_sensor);

    for (int i = 0; i < EKF_N; ++i) {
        // Perturb state
        ekf->x[i] += epsilon;

        float fx_perturbed[EKF_N];
        run_model(ekf, dt, fx_perturbed, hx_dummy, omega_x, omega_y, omega_z, accel_z, h_pressure_sensor);

        for (int j = 0; j < EKF_N; ++j) {
            F_out[j * EKF_N + i] = (fx_perturbed[j] - fx_base[j]) / epsilon;
        }

        ekf->x[i] = original_state[i]; // Restore original state
    }
}

// Returns altitude in meters given pressure in Pascals
float pressureToAltitude(float pressurePa, float seaLevelPressurePa = 101325.0, float T0 = 288.15) {
    return T0/0.0065 * (1.0 - pow(pressurePa / seaLevelPressurePa, 0.1903));
}

int main() {
    std::ofstream file("../csv_measurements/accEkf.csv", std::ios::trunc);
    file.close();
    std::ofstream file2("../csv_measurements/velEkf.csv", std::ios::trunc);
    file2.close();
    std::ofstream file3("../csv_measurements/posEkf.csv", std::ios::trunc);
    file3.close();

    int lineNum = 1;
    int totalLines = 819;

    ekf_t ekf = {0};
    const float pdiag[EKF_N] = {P0, V0, q_a, q_a, q_a, q_a};

    ekf_initialize(&ekf, pdiag);
    initialize_ekf(&ekf);

    float accX;
    float accY;
    float accZ;
    float omega_x;
    float omega_y;
    float omega_z;
    float h_pressure_sensor;
    float bias_pressure_sensor = 0.0f; // Bias for pressure sensor, can be adjusted if needed

    while (lineNum <= totalLines) {
        std::vector<float> acc = getAcc(lineNum);           // Get the accelerometer data
        std::vector<float> omega = getOmega(lineNum);       // Get the Angular velocity
        std::vector<float> h_pressure_sensor_vec = getAltitudes(lineNum);    // Get the pressure sensor data

        accX = acc[0];
        accY = acc[1];
        accZ = acc[2];
        //std::cout << "Line: " << lineNum << ", Acc: " << accX << ", " << accY << ", " << accZ << std::endl;
        Eigen::Vector3f accel_z(accX, accY, accZ);
        omega_x = omega[0]*M_PI/180.0; // Convert to rad/s
        omega_y = omega[1]*M_PI/180.0; // Convert to rad/s
        omega_z = omega[2]*M_PI/180.0; // Convert to rad/s
        h_pressure_sensor = h_pressure_sensor_vec[0] - bias_pressure_sensor; // Altitude from pressure sensor
        // std::cout << "Line: " << lineNum << ", Altitude: " << h_pressure_sensor << std::endl;

        float fx[EKF_N] = {0};
        float hx[EKF_M] = {0};
        
        Eigen::Matrix<float,3,4> Hq = computeHqAccelJacobian(
            Eigen::Quaternionf(ekf.x[2], ekf.x[3], ekf.x[4], ekf.x[5]),
            Eigen::Vector3f(accX, accY, accZ),
            Eigen::Vector3f(omega_x, omega_y, omega_z)
        );     

        float H[EKF_M*EKF_N] = {
            
            0, 0, Hq(0,0), Hq(0,1), Hq(0,2), Hq(0,3),
            0, 0, Hq(1,0), Hq(1,1), Hq(1,2), Hq(1,3),
            0, 0, Hq(2,0), Hq(2,1), Hq(2,2), Hq(2,3),
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0
        };

        R[EKF_M*EKF_M - 1] = estimateBaroVar(ekf.x[1]); // Update the last element of R with the barometer variance
        
        // Set the observation vector z
        float z[EKF_M] = {accX, accY, accZ, omega_x, omega_y, omega_z, h_pressure_sensor};

        run_model(&ekf, dt, fx, hx, omega_x, omega_y, omega_z, acc.data(), h_pressure_sensor);
        
        ekf_predict(&ekf, fx, F, Q);

        ekf_update(&ekf, z, hx, H, R);

        Eigen::Quaternionf q(ekf.x[2], ekf.x[3], ekf.x[4], ekf.x[5]);
        float roll, pitch, yaw;
        quaternionToEulerAngles(q, roll, pitch, yaw);
        

        std::vector<float> posEKF = { ekf.x[0], 0, 0};
        std::vector<float> velEKF = { ekf.x[1], 0, 0};
        saveToCSV(posEKF, "../csv_measurements/posEkf.csv");
        saveToCSV(velEKF, "../csv_measurements/velEkf.csv");
        std::cout << "Line: " << lineNum << ", Position: " << ekf.x[0] << ", Velocity: " << ekf.x[1] << std::endl;

        lineNum++;  // Move to the next line
    }

    std::cout << "EKF completed for " << totalLines << " lines." << std::endl;
    

    return 0;
}
*/