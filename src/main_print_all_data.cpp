#include <Arduino.h>
#include <BNO055Sensor.hpp>
#include <MPRLSSensor.hpp>
#include <LIS3DHTRSensor.hpp>
#include <MS561101BA03.hpp>
#include <GPS.hpp>
#include <config.h>
#include "utils/logger/rocket_logger/RocketLogger.hpp"
#include "utils/logger/data/LogSensorData.hpp"
#include "utils/logger/LogData.hpp"

// Sensor objects
BNO055Sensor bno;
//MPRLSSensor mprls;
LIS3DHTRSensor lis3dh;
MS561101BA03 ms56_1(0x77);
MS561101BA03 ms56_2(0x76);
GPS gps;

// Logger object
RocketLogger rocketLogger;

// Timing variables for actuators
unsigned long startTime = 0;
bool toggleState = false;

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.setRxBufferSize(2048);
    
    bno.init();
    //mprls.init();
    lis3dh.init();
    ms56_1.init();
    ms56_2.init();
    gps.init();

    // Testing the buzzer or actuators (just change the pins)
    startTime = millis();
    pinMode(D2, OUTPUT);
    //pinMode(D1, OUTPUT);
}

void loop() {
    // Retrieve data from all sensors
    //auto mprlsDataOpt = mprls.getData();
    auto bnoDataOpt = bno.getData();
    auto lis3dhDataOpt = lis3dh.getData();
    auto ms56DataOpt_1 = ms56_1.getData();
    auto ms56DataOpt_2 = ms56_2.getData();

    auto gpsDataOpt = gps.getData();

    // Create SensorData objects for each sensor
    /*if (mprlsDataOpt.has_value()) {
        rocketLogger.logSensorData(mprlsDataOpt.value());
    }*/

    if (bnoDataOpt.has_value()) {
        rocketLogger.logSensorData(bnoDataOpt.value());
    } else {
        rocketLogger.logError("BNO055 data not available");
    }

    if (lis3dhDataOpt.has_value()) {
        rocketLogger.logSensorData(lis3dhDataOpt.value());
    } else {
        rocketLogger.logError("LIS3DHTR data not available");
    }

    if (ms56DataOpt_1.has_value()) {
        rocketLogger.logSensorData("BAR1", ms56DataOpt_1.value());
    } else {
        rocketLogger.logError("BAR1 data not available");
    }

    if (ms56DataOpt_2.has_value()) {
        rocketLogger.logSensorData("BAR2", ms56DataOpt_2.value());
    } else {
        rocketLogger.logError("BAR2 data not available");
    }

    if (gpsDataOpt.has_value()) {
        rocketLogger.logSensorData("GPS", gpsDataOpt.value());
    } else {
        rocketLogger.logError("GPS data not available");
    }
    
    Serial.println(rocketLogger.getJSONAll().dump().c_str());
    Serial.flush();
    
    rocketLogger.clearData();

    // Actuator control logic
    unsigned long currentTime = millis();

    if (currentTime - startTime >= 3000) {
        if (toggleState) {
            digitalWrite(D2, HIGH);
        } else {
            digitalWrite(D2, LOW);
        }
        toggleState = !toggleState;
        startTime = currentTime;
    }

    delay(100);
}