 #pragma once

 #include <Arduino.h>
 #include <BNO055Sensor.hpp>
 #include <MPRLSSensor.hpp>
 #include <LIS3DHTRSensor.hpp>
 #include <MS561101BA03.hpp>
 #include <GPS.hpp>
 #include <Termoresistenze.hpp>
 #include <RocketLogger.hpp>
 #include <config.h>

 class Nemesis {
 public:
    Nemesis(std::shared_ptr<RocketLogger> rocketLogger,
            std::shared_ptr<BNO055Sensor> bno,
            std::shared_ptr<LIS3DHTRSensor> lis3dh,
            std::shared_ptr<MS561101BA03> ms56_1,
            std::shared_ptr<MS561101BA03> ms56_2,
            std::shared_ptr<GPS> gps);

    // Update sensor ready function for each sensor
    bool updateBNO055();
    bool updateLIS3DHTR();
    bool updateMS561101BA03_1();
    bool updateMS561101BA03_2();
    bool updateGPS();
    void readBattery();

    // Getter and setters for each value
    std::shared_ptr<BNO055Data> getBNO055Data();
    std::shared_ptr<LIS3DHTRData> getLIS3DHTRData();
    std::shared_ptr<MS561101BA03Data> getMS561101BA03Data_1();
    std::shared_ptr<MS561101BA03Data> getMS561101BA03Data_2();
    std::shared_ptr<GPSData> getGPSData();

    // Setters for simulated data
    void setSimulatedBNO055Data(std::shared_ptr<BNO055Data> data);
    void setSimulatedLIS3DHTRData(std::shared_ptr<LIS3DHTRData> data);
    void setSimulatedMS561101BA03Data_1(std::shared_ptr<MS561101BA03Data> data);
    void setSimulatedMS561101BA03Data_2(std::shared_ptr<MS561101BA03Data> data);
    void setSimulatedGPSData(std::shared_ptr<GPSData> data);

    // Flight state getters
    std::shared_ptr<bool> getIsRising();
    std::shared_ptr<float> getHeightGainSpeed();
    std::shared_ptr<float> getCurrentHeight();
    
 private:
    // Logger instance
    std::shared_ptr<RocketLogger> _rocketLogger;
    
    // Sensor instances
    std::shared_ptr<BNO055Sensor> _bno;
    std::shared_ptr<LIS3DHTRSensor> _lis3dh;
    std::shared_ptr<MS561101BA03> _ms56_1;
    std::shared_ptr<MS561101BA03> _ms56_2;
    std::shared_ptr<GPS> _gps;

    // Data related to the rocket state
    std::shared_ptr<BNO055Data> _bnoData;
    std::shared_ptr<LIS3DHTRData> _lis3dhData;
    std::shared_ptr<MS561101BA03Data> _ms561101ba03Data_1;
    std::shared_ptr<MS561101BA03Data> _ms561101ba03Data_2;
    std::shared_ptr<GPSData> _gpsData;

    float _batteryAdc, _batteryVoltage, _batteryPercentage;
    
    // Flight state variables
    std::shared_ptr<bool> _isRising;
    std::shared_ptr<float> _heightGainSpeed;
    std::shared_ptr<float> _currentHeight;
 };