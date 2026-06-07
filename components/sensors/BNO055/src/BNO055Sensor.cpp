#include <BNO055Sensor.hpp>
#include <utils.h>

BNO055Sensor::BNO055Sensor(const char *sensorName, I2CBus* bus, uint8_t address)
    : ISensor(sensorName), _bno_interface(bus, address)
{
    _data.setSensorName(this->getSensorName());
}

bool BNO055Sensor::init()
{
    int attempts = 0;
    uint32_t start = Utils::millis();
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
        
        uint32_t end = Utils::millis();
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

    // Get burst data from the sensor
    if (!_bno_interface.get_burst_data(_burst_data))
    {
        return false;
    }

    // Map calibration statuses
    _data.calibration_sys = _burst_data.calib_sys;
    _data.calibration_gyro = _burst_data.calib_gyro;
    _data.calibration_accel = _burst_data.calib_accel;
    _data.calibration_mag = _burst_data.calib_mag;

    // Map orientation (Euler angles)
    _data.orientation_x = _burst_data.euler[0];
    _data.orientation_y = _burst_data.euler[1];
    _data.orientation_z = _burst_data.euler[2];

    // Map angular velocity (gyroscope)
    _data.angular_velocity_x = _burst_data.gyro[0];
    _data.angular_velocity_y = _burst_data.gyro[1];
    _data.angular_velocity_z = _burst_data.gyro[2];

    // Map linear acceleration
    _data.linear_acceleration_x = _burst_data.lin_accel[0];
    _data.linear_acceleration_y = _burst_data.lin_accel[1];
    _data.linear_acceleration_z = _burst_data.lin_accel[2];

    // Map magnetometer data
    _data.magnetometer_x = _burst_data.mag[0];
    _data.magnetometer_y = _burst_data.mag[1];
    _data.magnetometer_z = _burst_data.mag[2];

    // Map acceleration data
    _data.acceleration_x = _burst_data.accel[0];
    _data.acceleration_y = _burst_data.accel[1];
    _data.acceleration_z = _burst_data.accel[2];

    // Map gravity vector
    _data.gravity_x = _burst_data.gravity[0];
    _data.gravity_y = _burst_data.gravity[1];
    _data.gravity_z = _burst_data.gravity[2];

    // Map temperature
    _data.temperature = _burst_data.temp;

    // Map quaternion data (order from get_burst_data parsing is W, X, Y, Z)
    _data.quaternion_w = _burst_data.quaternion[0];
    _data.quaternion_x = _burst_data.quaternion[1];
    _data.quaternion_y = _burst_data.quaternion[2];
    _data.quaternion_z = _burst_data.quaternion[3];

    // Update timestamp and validation flag
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