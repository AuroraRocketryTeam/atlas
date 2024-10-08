#include "BME680Sensor.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include "sensors/BME680/BME680Sensor.h"

bool initialized = false;

BME680Sensor::BME680Sensor()
{
    bme = Adafruit_BME680(&Wire);
}

bool BME680Sensor::init(uint8_t addr)
{
    int attempts = 0;
    while (!bme.begin(addr) && attempts++ < SENSOR_LOOKUP_MAX_ATTEMPTS)
    {
        Serial.println("Could not find a valid BME680 sensor, check wiring!");
        delay(1000);
    }
    if(attempts >= SENSOR_LOOKUP_MAX_ATTEMPTS)
    {
        return initialized;
    }
    // Set up oversampling and filter initialization (NOTE: need to study the optimal values)
    
    Serial.println("BME680 Sensor initialized successfully");
    initialized = true;
    return initialized;
}

bool BME680Sensor::init()
{
    return this->init(0x77);
}

bool BME680Sensor::readData()
{
    if (!initialized)
    {
        Serial.println("BME680 Sensor not initialized");
        return false;
    }
    if (!bme.performReading())
    {
        Serial.println("BME680 Failed to perform reading :(");
        return false;
    } 
    Serial.println("BME680 Reading performed successfully");
    return true;
}

BME680SensorData BME680Sensor::getData()
{
    if (!initialized)
    {
        Serial.println("BME680 Sensor not initialized");
        return BME680SensorData();
    }
    return BME680SensorData(bme.temperature, bme.pressure, bme.humidity, bme.gas_resistance);
}