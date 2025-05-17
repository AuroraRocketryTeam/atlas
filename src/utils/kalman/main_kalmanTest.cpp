#include <vector>
#include <tuple>
#include "../../sensors/MPRLS/MPRLSSensor.hpp"
#include "../../sensors/BNO055/BNO055Sensor.hpp"
#include "./KalmanFilter.hpp"
 
 // Elements needed to update the kalman filter
 unsigned long lastTime = 0;
 
 KalmanFilter ekf;
 
 static MPRLSSensor mprls;
 static BNO055Sensor bno;
 
 // Vector to keep track of the position
 std::vector<std::tuple<float, float, float>> positionLog;
 
 void setup()
 {
    mprls.init();
    bno.init();

    Serial.begin(115200);
 }
 
 void loop()
 {    
    // Evaluating time differences
    unsigned long currentTime = millis();
    float delta_t = (lastTime > 0) ? (currentTime - lastTime) / 1000.0 : 0.0;
    lastTime = currentTime;
    
    /* Currently unused measurements
    // Retrieve data from MPRLS sensor    
    auto mprlsDataOpt = mprls.getData();
    if (!mprlsDataOpt.has_value()) {
        Serial.println("MPRLS data not available");
        return;
    }
    auto mprlsData = mprlsDataOpt.value();
    float mprls_pressure = std::get<float>(mprlsData.getData("pressure").value());
    */

    // Retrieve data from BNO055 sensor
    auto bnoDataOpt = bno.getData();
    if (!bnoDataOpt.has_value()) {
        Serial.println("BNO055 data not available");
        return;
    }
    auto bnoData = bnoDataOpt.value();

    float bno_angVelocity_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("angular_velocity").value())["x"]);
    float bno_angVelocity_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("angular_velocity").value())["y"]);
    float bno_angVelocity_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("angular_velocity").value())["z"]);

    float bno_accelerometer_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("accelerometer").value())["x"]);
    float bno_accelerometer_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("accelerometer").value())["y"]);
    float bno_accelerometer_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("accelerometer").value())["z"]);
    
    /* Currently unused measurements
    float bno_orientation_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("orientation").value())["x"]);
    float bno_orientation_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("orientation").value())["y"]);
    float bno_orientation_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("orientation").value())["z"]);

    
    float bno_linearAccel_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("linear_acceleration").value())["x"]);
    float bno_linearAccel_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("linear_acceleration").value())["y"]);
    float bno_linearAccel_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("linear_acceleration").value())["z"]);
    
    float bno_magnetometer_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("magnetometer").value())["x"]);
    float bno_magnetometer_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("magnetometer").value())["y"]);
    float bno_magnetometer_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("magnetometer").value())["z"]);
    
    float bno_gravity_x = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["x"]);
    float bno_gravity_y = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["y"]);
    float bno_gravity_z = static_cast<float>(std::get<std::map<std::string, float>>(bnoData.getData("gravity").value())["z"]);
    
    float bno_temperature = static_cast<float>(std::get<int8_t>(bnoData.getData("board_temperature").value()));
    
    float bno_quaternion_w = static_cast<float>(std::get<std::map<std::string, double>>(bnoData.getData("quaternion").value())["w"]);
    float bno_quaternion_x = static_cast<float>(std::get<std::map<std::string, double>>(bnoData.getData("quaternion").value())["x"]);
    float bno_quaternion_y = static_cast<float>(std::get<std::map<std::string, double>>(bnoData.getData("quaternion").value())["y"]);
    float bno_quaternion_z = static_cast<float>(std::get<std::map<std::string, double>>(bnoData.getData("quaternion").value())["z"]);
    */

    float omega[3] = {bno_angVelocity_x, bno_angVelocity_y, bno_angVelocity_z};
    float acc[3] = {bno_accelerometer_x, bno_accelerometer_y, bno_accelerometer_z};

    std::vector<std::vector<float>> measurements = ekf.step(delta_t, omega, acc);
    
    std::vector<float> posEKF = measurements[0];
    std::vector<float> velEKF = measurements[1];
    std::vector<float> accEKF = measurements[2];

    Serial.println("Predicted pos: " + String(posEKF[0]) + ", " + String(posEKF[1]) + ", " + String(posEKF[2]));
    delay(10);
 }