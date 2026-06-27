#pragma once

#include "esp_private/adc_private.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Arduino.h>
#include <memory>
#include <BNO055Sensor.hpp>
#include <LIS3DHTRSensor.hpp>
#include <MS561101BA03.hpp>
#include <GPS.hpp>
#include <IMUData.hpp>
#include <AccelerometerSensorData.hpp>
#include <PressureSensorData.hpp>
#include <GPSData.hpp>
#include <Command.hpp>
#include <SD-master.hpp>
#include <Flash.hpp>
#include <IStorage.hpp>
#include <config.h>

enum class SensorReadStatus {
    OK,
    MUTEX_TIMEOUT,
    SENSOR_ERROR,   // Sensor is present but the last update() failed
    NOT_PRESENT     // Sensor pointer is null (and we aren't injecting HIL data)
};

/**
 * @brief Class representing the Nemesis rocket model and sensor/state storage.
 *
 */
class RocketModel
{
public:
    ~RocketModel();

    /**
     * @brief Construct a new Nemesis object
     *
     * @param bno Shared pointer to the BNO055Sensor instance.
     * @param lis3dh Shared pointer to the LIS3DHTRSensor instance.
     * @param ms56_1 Shared pointer to the first MS561101BA03 instance.
     * @param ms56_2 Shared pointer to the second MS561101BA03 instance.
     * @param gps Shared pointer to the GPS instance.
     * @param sd Shared pointer to SD storage backend (optional).
     * @param flash Shared pointer to external flash backend (optional).
     */
    RocketModel(std::shared_ptr<BNO055Sensor> bno,
            std::shared_ptr<LIS3DHTRSensor> lis3dh,
            std::shared_ptr<MS561101BA03> ms56_1,
            std::shared_ptr<MS561101BA03> ms56_2,
            std::shared_ptr<GPS> gps,
            std::shared_ptr<SD> sd = nullptr,
            std::shared_ptr<Flash> flash = nullptr);

    /**
     * @brief Update the BNO055 sensor data
     *
     * @return true if the update was successful
     * @return false if the update failed
     */
    bool updateBNO055();

    /**
     * @brief Update the LIS3DHTR sensor data
     *
     * @return true if the update was successful
     * @return false if the update failed
     */
    bool updateLIS3DHTR();

    /**
     * @brief Update the first MS561101BA03 sensor data
     *
     * @return true if the update was successful
     * @return false if the update failed
     */
    bool updateMS561101BA03_1();

    /**
     * @brief Update the second MS561101BA03 sensor data
     *
     * @return true if the update was successful
     * @return false if the update failed
     */
    bool updateMS561101BA03_2();

    /**
     * @brief Update the GPS sensor data
     *
     * @return true if the update was successful
     * @return false if the update failed
     */
    bool updateGPS();

    /**
     * @brief Read the battery status
     *
     */
    void readBattery();

    /**
     * @brief Get the BNO055 sensor data
     *
     * @param data Reference to the IMUData object to be filled
     * @return a SensorReadStatus containing the status result
     */
    SensorReadStatus getBNO055Data(IMUData& data);

    /**
     * @brief Get the LIS3DHTR sensor data
     *
     * @param data Reference to the AccelerometerSensorData object to be filled
     * @return a SensorReadStatus containing the status result
     */
    SensorReadStatus getLIS3DHTRData(AccelerometerSensorData& data);

    /**
     * @brief Get the first MS561101BA03 sensor data
     *
     * @param data Reference to the PressureSensorData object to be filled
     * @return a SensorReadStatus containing the status result
     */
    SensorReadStatus getMS561101BA03Data_1(PressureSensorData& data);

    /**
     * @brief Get the second MS561101BA03 sensor data
     *
     * @param data Reference to the PressureSensorData object to be filled
     * @return a SensorReadStatus containing the status result
     */
    SensorReadStatus getMS561101BA03Data_2(PressureSensorData& data);

    /**
     * @brief Get the GPS sensor data
     *
     * @param data Reference to the GPSData object to be filled
     * @return a SensorReadStatus containing the status result
     */
    SensorReadStatus getGPSData(GPSData& data);

    /**
     * @brief Check GPS availability
     */
    bool hasGPS() { return _gps != nullptr; }

    
    
    /**
     * @brief Thread-safe helper to check storage availability.
     */
    bool isStorageInitialized(uint32_t timeoutMs = 100) const;

    /**
     * @brief Thread-safe helper to check if a file exists in storage.
     */
    bool storageFileExists(const char* filename, uint32_t timeoutMs = 200);

    /**
     * @brief Thread-safe helper to reset the read cursor of a file in storage.
     */
    bool storageResetReadCursor(const char* filename, uint32_t timeoutMs = 200);

    /**
     * @brief Thread-safe helper to read a line from storage.
     */
    std::string storageReadLine(uint32_t timeoutMs = 200);

    /**
     * @brief Thread-safe helper to read the entire content of a file from storage.
     */
    std::string storageReadFile(const char* filename, uint32_t timeoutMs = 200);
    
    /**
     * @brief Thread-safe helper to write content to a file in storage.
     */
    bool storageWriteFile(const char* filename, const uint8_t* data, size_t length, uint32_t timeoutMs = 200);
    
    /**
     * @brief Thread-safe helper to append content to a file in storage.
     */
    bool storageAppendFile(const char* filename, const uint8_t* data, size_t length, uint32_t timeoutMs = 200);

#if CONFIG_AURORA_HIL_SIMULATION
    // Simulation
     /**
     * @brief Setter of simulation reset flag.
     *
     * @param bool the reset flag value.
     */
    void setResetSimulationFlag(bool value);

     /**
     * @brief Getter of simulation reset flag.
     *
     * @return bool the reset flag value.
     */
    bool getResetSimulationFlag();
#endif

    /**
     * @brief Set the simulated BNO055 sensor data
     *
     * @param data The IMU data to simulate
     * @return true if the data was set successfully, false otherwise
     */
    bool setSimulatedBNO055Data(IMUData data);

    /**
     * @brief Set the simulated LIS3DHTR sensor data
     *
     * @param data The Accelerometer sensor data to simulate
     * @return true if the data was set successfully, false otherwise
     */
    bool setSimulatedLIS3DHTRData(AccelerometerSensorData data);

    /**
     * @brief Set the simulated MS561101BA03 sensor data
     *
     * @param data The pressure data of the first MS561101BA03 sensor to simulate
     * @return true if the data was set successfully, false otherwise
     */
    bool setSimulatedMS561101BA03Data_1(PressureSensorData data);

    /**
     * @brief Set the simulated MS561101BA03 sensor data
     *
     * @param data The pressure data of the second MS561101BA03 sensor to simulate
     * @return true if the data was set successfully, false otherwise
     */
    bool setSimulatedMS561101BA03Data_2(PressureSensorData data);

    /**
     * @brief Set the simulated GPS sensor data
     *
     * @param data The GPS data to simulate
     * @return true if the data was set successfully, false otherwise
     */
    bool setSimulatedGPSData(GPSData data);

    /**
     * @brief Get the flight state variables
     *
     * @return True if the rocket is rising, false otherwise
     */
    bool getIsRising();

    /**
     * @brief Set the flight state variable indicating if the rocket is rising
     *
     * @param isRising True if the rocket is rising, false otherwise
     */
    void setIsRising(bool isRising);

    /**
     * @brief Get the estimated Height Gain Speed
     *
     * @return height gain speed
     */
    float getHeightGainSpeed();

    /**
     * @brief Set the current estimated velocity
     *
     * @param heightGainSpeed The current estimated velocity
     */
    void setHeightGainSpeed(float heightGainSpeed);

    /**
     * @brief Get the Current Estimated Height
     *
     * @return current estimated height
     */
    float getCurrentHeight();

    /**
     * @brief Set the current estimated height
     *
     * @param height The current estimated height
     */
    void setCurrentHeight(float height);

    /**
     * @brief Set the command to open the main parachute.
     *
     * @return true if the command was set successfully
     * @return false otherwise
     */
    bool setOpenMainCommand();

    /**
     * @brief Set the command to open the drogue parachute.
     *
     * @return true if the command was set successfully
     * @return false otherwise
     */
    bool setOpenDrogueCommand();

    /**
     * @brief Set the airbrakes command level.
     *
     * @param lvl Airbrakes actuation level to command
     * @return true if the command was set successfully
     * @return false otherwise
     */
    bool setAirbrakesCommand(float lvl);

    /**
     * @brief Get the current command.
     *
     * @return Command currently stored by the model
     */
    Command getCommand();

    /**
     * @brief Reset the current command to its default state.
     *
     */
    void resetCommand();

    /**
     * @brief Reset the rocket model state.
     *
     */
    void reset();

    /**
     * @brief Get the BNO055 sensor instance.
     */
    std::shared_ptr<BNO055Sensor> getBNO055Sensor();

    /**
     * @brief Check if the IMU is fully calibrated (Status == 3)
     */
    bool isSensorSystemCalibrated();

    /**
     * @brief Feed a pressure reading into the zeroing algorithm
     * @param pressure The current pressure reading
     */
    void addBarometerSample(float pressure);

    /**
     * @brief Get the number of barometer samples collected for zeroing
     */
    int getBarometerSampleCount();

    /**
     * @brief Check if the barometer has finished accumulating baseline samples
     */
    bool isBarometerZeroed() const;

    /**
     * @brief Check if the temperature has finished accumulating baseline samples
     */
    bool isTemperatureZeroed() const;

    /**
     * @brief Get the calculated launchpad baseline pressure
     */
    float getLaunchpadBasePressure() const;

    /**
     * @brief Add a temperature sample for zeroing
     * @param temperature The current temperature reading
     */
    void addTemperatureSample(float temperature);

    /**
     * @brief Get the calculated launchpad baseline temperature
     */
    float getLaunchpadBaseTemperature() const;
    
private:
    // Sensor instances
    std::shared_ptr<BNO055Sensor> _bno;
    std::shared_ptr<LIS3DHTRSensor> _lis3dh;
    std::shared_ptr<MS561101BA03> _ms56_1;
    std::shared_ptr<MS561101BA03> _ms56_2;
    std::shared_ptr<GPS> _gps;

    // Critical mutexes
    SemaphoreHandle_t _imuMutex;
    SemaphoreHandle_t _baro1Mutex;
    SemaphoreHandle_t _baro2Mutex;
    SemaphoreHandle_t _gpsMutex;
    SemaphoreHandle_t _stateMutex;

    adc_oneshot_unit_handle_t _adc1_handle;
    int _batteryAdc;
    float _batteryVoltage, _batteryPercentage;

    // Flight state variables
    bool _isRising;
    float _heightGainSpeed;
    float _currentHeight;

    IMUData _bnoData;
    AccelerometerSensorData _lis3dhData;
    PressureSensorData _ms561101ba03Data_1;
    PressureSensorData _ms561101ba03Data_2;
    GPSData _gpsData;

    // Payload health tracking
    bool _bnoDataValid = false;
    bool _lis3dhDataValid = false;
    bool _ms561101ba03Data_1_Valid = false;
    bool _ms561101ba03Data_2_Valid = false;
    bool _gpsDataValid = false;

#if CONFIG_AURORA_HIL_SIMULATION
    bool _reset_simulation;
#endif

    SemaphoreHandle_t _storageMutex;
    std::shared_ptr<IStorage> _storage;
    Command _cmd;

    // Calibration and Zeroing variables
    std::vector<float> _barometerSamples;
    float _launchpadBasePressure = 0.0f;
    bool _barometerZeroed = false;
    static constexpr size_t REQUIRED_BARO_SAMPLES = 100;

    std::vector<float> _temperatureSamples;
    float _launchpadBaseTemperature = 0.0f;
    bool _temperatureZeroed = false;
    static constexpr size_t REQUIRED_TEMPERATURE_SAMPLES = 100;
};
