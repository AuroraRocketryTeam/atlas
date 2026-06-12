#include "BNO055Printer.hpp"
#include "../utils/PrintUtils.hpp"

BNO055Printer::BNO055Printer(ISensor* sensor) : bno055(sensor) { }

void BNO055Printer::displayCalibrationStatus() {
    auto data = bno055->getData();

    PrintUtils::printHeader("CALIBRATION STATUS: 0=not calibrated, 3=fully calibrated");
    Serial.println("Sys: " + String(data.calibration_sys));
    Serial.println("Gyro: " + String(data.calibration_gyro));
    Serial.println("Accel: " + String(data.calibration_accel));
    Serial.println("Mag: " + String(data.calibration_mag));
    Serial.println("");
}

void BNO055Printer::displayMagnetometer() {
    printXYZMap("magnetometer");
    Serial.println(" uT");
    Serial.println("");
}

void BNO055Printer::displayOrientation() {
    printXYZMap("orientation");
    Serial.println("");
}

void BNO055Printer::displayAccelleration() {
    printXYZMap("accelerometer");
    Serial.println(" m/s²");
    Serial.println("");
}

void BNO055Printer::displayGyroscope() {
    printXYZMap("angular_velocity");
    Serial.println(" rad/s");
    Serial.println("");
}

void BNO055Printer::displayLinearAccelleration() {
    printXYZMap("linear_acceleration");
    Serial.println("");
}

void BNO055Printer::displayGravity() {
    printXYZMap("gravity");
    Serial.println("");
}

void BNO055Printer::printXYZMap(const char* key) {
    auto data = bno055->getData();
    PrintUtils::printHeader(key);

    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;

    if (strcmp(key, "magnetometer") == 0) {
        x = data.magnetometer_x;
        y = data.magnetometer_y;
        z = data.magnetometer_z;
    } else if (strcmp(key, "orientation") == 0) {
        x = data.orientation_x;
        y = data.orientation_y;
        z = data.orientation_z;
    } else if (strcmp(key, "accelerometer") == 0) {
        x = data.acceleration_x;
        y = data.acceleration_y;
        z = data.acceleration_z;
    } else if (strcmp(key, "angular_velocity") == 0) {
        x = data.angular_velocity_x;
        y = data.angular_velocity_y;
        z = data.angular_velocity_z;
    } else if (strcmp(key, "linear_acceleration") == 0) {
        x = data.linear_acceleration_x;
        y = data.linear_acceleration_y;
        z = data.linear_acceleration_z;
    } else if (strcmp(key, "gravity") == 0) {
        x = data.gravity_x;
        y = data.gravity_y;
        z = data.gravity_z;
    } else {
        Serial.println(String("ERROR: Could not read ") + key + " values");
        return;
    }

    Serial.print("X: " + String(x, DECIMAL_PLACES));
    Serial.print(" Y: " + String(y, DECIMAL_PLACES));
    Serial.print(" Z: " + String(z, DECIMAL_PLACES));    
}
