#include <BNO055Sensor.hpp>
#include <utils.h>

BNO055Sensor::BNO055Sensor(I2CBus* bus, uint8_t address)
    : _bno_interface(bus, address)
{
}

bool BNO055Sensor::init()
{
    int attempts = 0;
    uint start = Utils::millis();
    bool initialized = false;
    
    while (attempts++ < SENSOR_LOOKUP_MAX_ATTEMPTS) {
        if (_bno_interface.init()) {
            // Auto-load calibration from NVS
            loadCalibrationFromNVS();

            if (_bno_interface.set_operation_mode(BNO055_OPERATION_MODE_AMG)) {
                initialized = true;
                break;
            } else {
                return false;
            }
        } else {
            return false;
        }
        
        uint end = Utils::millis();
        while (end - start < SENSOR_LOOKUP_TIMEOUT) {
            end = Utils::millis();
        }
        start = Utils::millis();
    }
    
    if (!initialized) {
        return false;
    }
    
    this->setInitialized(true);
    return true;
}


bool BNO055Sensor::updateData()
{
    if (!this->isInitialized())
    {
        return false;
    }

    _data = IMUData("BNO055");

    _data.calibration_sys = _bno_interface.check_calibration_sys();
    _data.calibration_gyro = _bno_interface.check_calibration_gyro();
    _data.calibration_accel = _bno_interface.check_calibration_accel();
    _data.calibration_mag = _bno_interface.check_calibration_mag();

    // Get orientation (Euler angles)
    std::vector<float> orientation = _bno_interface.get_euler_deg();
    _data.orientation_x = orientation[0];
    _data.orientation_y = orientation[1];
    _data.orientation_z = orientation[2];

    // Get angular velocity (gyroscope)
    std::vector<float> angular_velocity = _bno_interface.get_gyro_dps();
    _data.angular_velocity_x = angular_velocity[0];
    _data.angular_velocity_y = angular_velocity[1];
    _data.angular_velocity_z = angular_velocity[2];

    // Get linear acceleration
    std::vector<float> linear_accel = _bno_interface.get_linear_accel();
    _data.linear_acceleration_x = linear_accel[0];
    _data.linear_acceleration_y = linear_accel[1];
    _data.linear_acceleration_z = linear_accel[2];

    // Get magnetometer data
    std::vector<float> mag = _bno_interface.get_mag();
    _data.magnetometer_x = mag[0];
    _data.magnetometer_y = mag[1];
    _data.magnetometer_z = mag[2];

    // Get acceleration data
    std::vector<float> accel = _bno_interface.get_accel();
    _data.acceleration_x = accel[0];
    _data.acceleration_y = accel[1];
    _data.acceleration_z = accel[2];

    // Get gravity vector
    std::vector<float> gravity = _bno_interface.get_gravity();
    _data.gravity_x = gravity[0];
    _data.gravity_y = gravity[1];
    _data.gravity_z = gravity[2];

    // Get temperature
    _data.temperature = _bno_interface.get_temperature();

    // Get quaternion data
    std::vector<float> quaternion = _bno_interface.get_quaternion();
    _data.quaternion_w = quaternion[0];
    _data.quaternion_x = quaternion[1];
    _data.quaternion_y = quaternion[2];
    _data.quaternion_z = quaternion[3];

    // Update timestamp
    _data.timestamp = Utils::millis();

    return true;
}

bool BNO055Sensor::hardwareTest() {
    bool accel_status = _bno_interface.selftest_accel();
    bool mag_status = _bno_interface.selftest_mag();
    bool gyro_status = _bno_interface.selftest_gyro();
    bool mcu_status = _bno_interface.selftest_mcu();

    // !!! These are not strictly hardware stuff, should we add another test function?
    // bool system_status = _bno_interface.check_system_error();
    // bool clock_status = _bno_interface.check_clock_status();

    return accel_status && mag_status && gyro_status && mcu_status;
}

IMUData BNO055Sensor::getData() {
    return _data;
}

bool BNO055Sensor::isFullyCalibrated() {
    if (!this->isInitialized()) {
        return false;
    }

    // If the calibration was restored from NVS, we can assume it's fully calibrated
    // As stated in the datasheet, the calibration values gets reset to 0 on startup,
    // and increase only with movements calibration (which not happen on file loading)
    //References (Section 3.11/3.11.4): https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf
    if (_calibration_restored) {
        return true; 
    }
    
    return _bno_interface.check_calibration() == 3;
}

bool BNO055Sensor::saveCalibrationToNVS() {
    uint8_t calib_data[22];
    if (!_bno_interface.get_calibration_profile(calib_data)) {
        LOG_ERROR("BNO055", "Failed to read calibration from sensor.");
        return false;
    }

    nvs_handle_t my_handle;
    // "bno055" is the namespace inside the nvs partition
    if (nvs_open("bno055", NVS_READWRITE, &my_handle) != ESP_OK) return false;

    esp_err_t err = nvs_set_blob(my_handle, "calib_profile", calib_data, sizeof(calib_data));
    if (err == ESP_OK) {
        nvs_commit(my_handle);
        LOG_INFO("BNO055", "Calibration saved to internal NVS successfully!");
    }
    nvs_close(my_handle);
    return (err == ESP_OK);
}

bool BNO055Sensor::loadCalibrationFromNVS() {
    nvs_handle_t my_handle;
    if (nvs_open("bno055", NVS_READONLY, &my_handle) != ESP_OK) {
        LOG_WARNING("BNO055", "No calibration found in NVS (Normal on first boot).");
        return false;
    }

    size_t required_size = 22;
    uint8_t calib_data[22];
    esp_err_t err = nvs_get_blob(my_handle, "calib_profile", calib_data, &required_size);
    nvs_close(my_handle);

    if (err == ESP_OK && required_size == 22) {
        if (_bno_interface.set_calibration_profile(calib_data)) {
            LOG_INFO("BNO055", "Calibration loaded from NVS successfully! Current calibration status: %d", _bno_interface.check_calibration());
            _calibration_restored = true;
            return true;
        }
    }
    return false;
}