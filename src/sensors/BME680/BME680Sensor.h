#ifndef BME680_SENSOR_H
#define BME680_SENSOR_H
/**
 * @class BME680Sensor
 * @brief A sensor class for the BME680 sensor, inheriting from the Sensor class template.
 *
 * This class provides methods to initialize the sensor, read data synchronously and asynchronously,
 * and retrieve the sensor data.
 *
 * @note The BME680 sensor is used for measuring temperature, humidity, pressure, and gas.
 *
 * @file BME680Sensor.h
 *
 * @see Sensor
 * @see BME680SensorData
 *
 * @author alessandr.monticell4@studio.unibo.it
 * @author luca.pulga@studio.unibo.it
 */
#include "sensors/Sensor.h"
#include "const/pins.h"
#include "const/config.h"
#include "BME680SensorData.h"

class BME680Sensor : public Sensor<BME680SensorData>
{
public:
    BME680Sensor();
    bool init() override;
    bool readData() override;
    bool init(uint8_t addr);
    BME680SensorData getData() override;
private:
    Adafruit_BME680 bme;
};
#endif // BME680_SENSOR_H