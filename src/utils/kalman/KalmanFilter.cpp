#include "KalmanFilter.hpp"

KalmanFilter::KalmanFilter(std::vector<Eigen::Vector3f> gravity_readings) {
    //calibration phase
    std::tuple<Eigen::Quaternionf, Eigen::Vector3f, Eigen::Vector3f> calibration_data = calibration(gravity_readings);

    ekf_initialize(&ekf, Q_diag);

    // Position
    ekf.x[0] = 0;
    ekf.x[1] = 0;
    ekf.x[2] = 0;

    // Velocity
    ekf.x[3] = 0;
    ekf.x[4] = 0;
    ekf.x[5] = 0;

    // Quaternion: received from calibration phase
    ekf.x[6] = std::get<0>(calibration_data).x();
    ekf.x[7] = std::get<0>(calibration_data).y();
    ekf.x[8] = std::get<0>(calibration_data).z();
    ekf.x[9] = std::get<0>(calibration_data).w();
    
    // Accel Bias
    ekf.x[10] = std::get<1>(calibration_data)[0];
    ekf.x[11] = std::get<1>(calibration_data)[1];
    ekf.x[12] = std::get<1>(calibration_data)[2];
    
    // Gyro Bias
    ekf.x[13] = std::get<2>(calibration_data)[0];
    ekf.x[14] = std::get<2>(calibration_data)[1];
    ekf.x[15] = std::get<2>(calibration_data)[2];
}

float* KalmanFilter::state() {
    return ekf.x;
}

std::tuple<Eigen::Quaternionf, Eigen::Vector3f, Eigen::Vector3f> KalmanFilter::calibration(std::vector<Eigen::Vector3f> gravity_readings) {
    // TO DO: COMPUTE STANDARD DEVIATION OF THE GRAVITY READINGS, IF TOO HIGH, REPEAT THE CALIBRATION

    // Calculate the mean of the gravity readings
    Eigen::Vector3f gravity_sum = Eigen::Vector3f::Zero();
    for (const auto& reading : gravity_readings) {
        gravity_sum += reading;
    }
    Eigen::Vector3f gravity = gravity_sum / gravity_readings.size();
    
    // Expected gravity vector for specific location (Forlì - 34 m over sea level)
    // Got from: https://www.sensorsone.com/local-gravity-calculator/
    Eigen::Vector3f expected_gravity(0, 0, 9.80537);

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
    //!!!Eigen::Vector3f magnetometer_z_aligned = q_rot * magnetometer;
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

    return std::make_tuple(initial_quaternion, bias_a, bias_w);
}

Eigen::Vector3f KalmanFilter::rotateToBody(const Eigen::Quaternionf& q, const Eigen::Vector3f& vec_world) {
    return q.conjugate() * vec_world;  // Equivalent to R^T * vec
}

Eigen::Matrix<float, 3, 4> KalmanFilter::computeHqAccelJacobian(const Eigen::Quaternionf& q_nominal, const Eigen::Vector3f& accel_world, float epsilon)
{
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
        H.col(i) = (a_plus - a_minus) / (2.0f * epsilon);
    }

    return H;  // size 3x4
}

std::vector<std::vector<float>> KalmanFilter::step(float dt, float omega[3], float accel[3]) {
    Eigen::Vector3f accel_z(accel[0], accel[1], accel[2]);
    
    // Conversion to rad/s
    float omega_x = omega[0]*(float)M_PI/180.0f;
    float omega_y = omega[1]*(float)M_PI/180.0f;
    float omega_z = omega[2]*(float)M_PI/180.0f;

    float fx[EKF_N] = {0};
    float hx[EKF_M] = {0};

    // Biases
    Eigen::Vector3f bias_a(ekf.x[10], ekf.x[11], ekf.x[12]);
    Eigen::Vector3f bias_g(ekf.x[13], ekf.x[14], ekf.x[15]);

    // Update quaternion
    // omega = [wx, wy, wz] in rad/s
    Eigen::Vector3f omega_eigen(omega_x - bias_g[0], omega_y - bias_g[1], omega_z - bias_g[2]);
    Eigen::Vector3f axis = omega_eigen.normalized();
    float theta = omega_eigen.norm() * dt;
    Eigen::Quaternionf delta_q(Eigen::AngleAxisf(theta, axis)); // delta_q = cos(theta/2) + axis*sin(theta/2)

    Eigen::Quaternionf q_nominal(ekf.x[6], ekf.x[7], ekf.x[8], ekf.x[9]);
    Eigen::Quaternionf q_rot =  q_nominal * delta_q;
    q_rot.normalize();
    
    Eigen::Vector3f accel_abs = q_rot * (accel_z - bias_a) - gravity;  // Equivalent to q * a * q.inverse()

    Eigen::Matrix<float,3,4> Hq = computeHqAccelJacobian(
        Eigen::Quaternionf(ekf.x[6], ekf.x[7], ekf.x[8], ekf.x[9]),
        Eigen::Vector3f(accel_abs[0], accel_abs[1], accel_abs[2])
    );    

    float H[EKF_M*EKF_N] = {
        0, 0, 0, 0, 0, 0, Hq(0,0), Hq(0,1), Hq(0,2), Hq(0,3), 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, Hq(1,0), Hq(1,1), Hq(1,2), Hq(1,3), 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, Hq(2,0), Hq(2,1), Hq(2,2), Hq(2,3), 0, 0, 1, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
    };

    // F: state transition Jacobian
    float F[EKF_N*EKF_N] = {
        1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    
        0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    
        0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
    
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
        
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1
    
    };

    // Set the observation vector z
    float z[EKF_M] = {accel[0], accel[1], accel[2], omega_x, omega_y, omega_z};

    run_model(dt, fx, hx, omega_x, omega_y, omega_z, accel);
    
    // I have Q_diag, but Q shouldn't be diagonal for the fun
    ekf_predict(&ekf, fx, F, Q_diag);

    ekf_update(&ekf, z, hx, H, R);

    std::vector<float> posEKF = { ekf.x[0], ekf.x[1], ekf.x[2] };
    std::vector<float> velEKF = { ekf.x[3], ekf.x[4], ekf.x[5] };
    std::vector<float> accEKF = { ekf.x[6], ekf.x[7], ekf.x[8] };
    
    std::vector<std::vector<float>> result = {posEKF, velEKF, accEKF};

    return result;
}

void KalmanFilter::run_model(float dt, float fx[EKF_N], float hx[EKF_M], float omega_x, float omega_y, float omega_z, float accel_z[3]) {
    // Biases
    Eigen::Vector3f bias_a(ekf.x[10], ekf.x[11], ekf.x[12]);
    Eigen::Vector3f bias_g(ekf.x[13], ekf.x[14], ekf.x[15]);

    // Update quaternion
    // omega = [wx, wy, wz] in rad/s
    Eigen::Vector3f omega_eigen(omega_x - bias_g[0], omega_y - bias_g[1], omega_z - bias_g[2]);
    float theta = omega_eigen.norm() * dt;
    Eigen::Vector3f axis = omega_eigen.normalized();
    Eigen::Quaternionf delta_q(Eigen::AngleAxisf(theta, axis)); // delta_q = cos(theta/2) + axis*sin(theta/2)
    Eigen::Quaternionf q_nominal(ekf.x[6], ekf.x[7], ekf.x[8], ekf.x[9]);
    Eigen::Quaternionf q_rot =  q_nominal * delta_q;
    q_rot.normalize();

    Eigen::Vector3f acc_body(accel_z[0], accel_z[1], accel_z[2]);
    Eigen::Vector3f accel_abs = q_rot * (acc_body - bias_a) + gravity;  // Equivalent to q * a * q.inverse()

    // Position
    fx[0] = (float)(ekf.x[0] + ekf.x[3]*dt);
    fx[1] = (float)(ekf.x[1] + ekf.x[4]*dt);
    fx[2] = (float)(ekf.x[2] + ekf.x[5]*dt);
    
    // Velocities
    fx[3] = (float)(ekf.x[3] + accel_abs[0]*dt);
    fx[4] = (float)(ekf.x[4] + accel_abs[1]*dt);
    fx[5] = (float)(ekf.x[5] + accel_abs[2]*dt);

    // Quaternion
    fx[6] = q_rot.w();
    fx[7] = q_rot.x();
    fx[8] = q_rot.y();
    fx[9] = q_rot.z();

    // Define the noise standard deviations for each bias component
    // These values would typically be based on the sensor's datasheet or empirically determined
    const float accel_bias_stddev = 0.000001f;  // Acceleration bias noise (e.g., 1 mG)
    const float gyro_bias_stddev = 0.000001f*(float)M_PI/180;   // Gyroscope bias noise (e.g., 0.1 deg/s)

    std::default_random_engine generator;  // Random number generator
    std::normal_distribution<float> accel_noise(0.0, accel_bias_stddev);  // Gaussian noise for accelerometer bias
    std::normal_distribution<float> gyro_noise(0.0, gyro_bias_stddev);   // Gaussian noise for gyro bias

    // Now we apply the noise to the bias components
    // Accel Bias (BNO055 accelerometer)
    fx[10] = (float)(ekf.x[10]) + accel_noise(generator);  // X-axis accel bias with noise
    fx[11] = (float)(ekf.x[11]) + accel_noise(generator);  // Y-axis accel bias with noise
    fx[12] = (float)(ekf.x[12]) + accel_noise(generator);  // Z-axis accel bias with noise

    // Gyro Bias (BNO055 gyroscope)
    fx[13] = (float)(ekf.x[13]) + gyro_noise(generator);   // X-axis gyro bias with noise
    fx[14] = (float)(ekf.x[14]) + gyro_noise(generator);   // Y-axis gyro bias with noise
    fx[15] = (float)(ekf.x[15]) + gyro_noise(generator);   // Z-axis gyro bias with noise

    // Measurements
    // Here we have to put the expected measurements of the acceleration /Review for future updates
    hx[0] = accel_z[0] + bias_a[0];
    hx[1] = accel_z[1] + bias_a[1];
    hx[2] = accel_z[2] + bias_a[2];
    hx[3] = omega_x + bias_g[0];
    hx[4] = omega_y + bias_g[1];
    hx[5] = omega_z + bias_g[2];
}

// NEVER USED?? !!!
void KalmanFilter::computeJacobianF_tinyEKF(float dt, float omega_x, float omega_y, float omega_z, float accel_z[3], float F_out[EKF_N * EKF_N]) {
    const float epsilon = 1e-5f;
    float fx_base[EKF_N];
    float hx_dummy[EKF_M]; // Not used
    std::vector<float> original_state(ekf.x, ekf.x + EKF_N);

    run_model(dt, fx_base, hx_dummy, omega_x, omega_y, omega_z, accel_z);

    for (int i = 0; i < EKF_N; ++i) {
        // Perturb state
        ekf.x[i] += epsilon;

        float fx_perturbed[EKF_N];
        run_model(dt, fx_perturbed, hx_dummy, omega_x, omega_y, omega_z, accel_z);

        for (int j = 0; j < EKF_N; ++j) {
            F_out[j * EKF_N + i] = (fx_perturbed[j] - fx_base[j]) / epsilon;
        }

        ekf.x[i] = original_state[i]; // Restore original state
    }
}

