#pragma once

#include "esp_private/adc_private.h"
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Arduino.h>
#include <BNO055Sensor.hpp>
#include <MPRLSSensor.hpp>
#include <LIS3DHTRSensor.hpp>
#include <MS561101BA03.hpp>
#include <GPS.hpp>
#include <IMUData.hpp>
#include <AccelerometerSensorData.hpp>
#include <PressureSensorData.hpp>
#include <GPSData.hpp>
#include <Command.hpp>
#include <Termoresistenze.hpp>
#include <SD-master.hpp>
#include <Flash.hpp>
#include <IStorage.hpp>
#include <config.h>

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
     * @return A shared pointer to the BNO055Data
     */
    std::shared_ptr<IMUData> getBNO055Data();

    /**
     * @brief Get the LIS3DHTR sensor data
     *
     * @return A shared pointer to the accelerometer data
     */
    std::shared_ptr<AccelerometerSensorData> getLIS3DHTRData();

    /**
     * @brief Get the first MS561101BA03 sensor data
     *
     * @return A shared pointer to the pressure data of the first MS561101BA03 sensor
     */
    std::shared_ptr<PressureSensorData> getMS561101BA03Data_1();

    /**
     * @brief Get the second MS561101BA03 sensor data
     *
     * @return A shared pointer to the pressure data of the second MS561101BA03 sensor
     */
    std::shared_ptr<PressureSensorData> getMS561101BA03Data_2();

    /**
     * @brief Get the GPS sensor data
     *
     * @return A shared pointer to the GPSData
     */
    std::shared_ptr<GPSData> getGPSData();

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
    char* storageReadLine(uint32_t timeoutMs = 200);

    /**
     * @brief Thread-safe helper to read the entire content of a file from storage.
     */
    char* storageReadFile(const char* filename, uint32_t timeoutMs = 200);
    
    /**
     * @brief Thread-safe helper to write content to a file in storage.
     */
    bool storageWriteFile(const char* filename, const char* content, uint32_t timeoutMs = 200);


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
     * @param data A shared pointer to the IMU data
     */
    void setSimulatedBNO055Data(std::shared_ptr<IMUData> data);

    /**
     * @brief Set the simulated LIS3DHTR sensor data
     *
     * @param data A shared pointer to the AccelerometerSensorData
     */
    void setSimulatedLIS3DHTRData(std::shared_ptr<AccelerometerSensorData> data);

    /**
     * @brief Set the simulated MS561101BA03 sensor data
     *
     * @param data A shared pointer to the pressure data of the first MS561101BA03 sensor
     */
    void setSimulatedMS561101BA03Data_1(std::shared_ptr<PressureSensorData> data);

    /**
     * @brief Set the simulated MS561101BA03 sensor data
     *
     * @param data A shared pointer to the pressure data of the second MS561101BA03 sensor
     */
    void setSimulatedMS561101BA03Data_2(std::shared_ptr<PressureSensorData> data);

    /**
     * @brief Set the simulated GPS sensor data
     *
     * @param data A shared pointer to the GPSData
     */
    void setSimulatedGPSData(std::shared_ptr<GPSData> data);

    /**
     * @brief Get the flight state variables
     *
     * @return A shared pointer to the flight state variables which is true if the rocket is rising, false otherwise
     */
    std::shared_ptr<bool> getIsRising();

    /**
     * @brief Get the estimated Height Gain Speed
     *
     * @return std::shared_ptr<float> representing the height gain speed
     */
    std::shared_ptr<float> getHeightGainSpeed();

    /**
     * @brief Get the Current Estimated Height
     *
     * @return std::shared_ptr<float> representing the current estimated height
     */
    std::shared_ptr<float> getCurrentHeight();

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

private:
    // Sensor instances
    std::shared_ptr<BNO055Sensor> _bno;
    std::shared_ptr<LIS3DHTRSensor> _lis3dh;
    std::shared_ptr<MS561101BA03> _ms56_1;
    std::shared_ptr<MS561101BA03> _ms56_2;
    std::shared_ptr<GPS> _gps;

    // Data related to the rocket state
    std::shared_ptr<IMUData> _bnoData;
    std::shared_ptr<AccelerometerSensorData> _lis3dhData;
    std::shared_ptr<PressureSensorData> _ms561101ba03Data_1;
    std::shared_ptr<PressureSensorData> _ms561101ba03Data_2;
    std::shared_ptr<GPSData> _gpsData;

    adc_oneshot_unit_handle_t _adc1_handle;
    int _batteryAdc;
    float _batteryVoltage, _batteryPercentage;

    // Flight state variables
    std::shared_ptr<bool> _isRising;
    std::shared_ptr<float> _heightGainSpeed;
    std::shared_ptr<float> _currentHeight;
};
