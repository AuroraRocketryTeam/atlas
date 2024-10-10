// BME680SensorData.h
#ifndef BME680_SENSOR_DATA_H
#define BME680_SENSOR_DATA_H

#include "sensors/SensorData.h"
#include <Adafruit_BME680.h>
#include <cstdint>

class BME680SensorData : public SensorData
{
    friend class BME680Sensor;

public:
    float getTemperature() const { return this->temperature; };
    uint32_t getPressure() const { return this->pressure; };
    float getHumidity() const { return this->humidity; };
    uint32_t getGasResistance() const { return this->gas_resistance; };
    String toString() override;
    void log(CborLogger &logger) override;

private:
    BME680SensorData() : temperature(-39), pressure(-1), humidity(-1), gas_resistance(-1) {}
    BME680SensorData(float temp, uint32_t pres, float hum, uint32_t gas_res) : temperature(temp), pressure(pres), humidity(hum), gas_resistance(gas_res) {}

    const float temperature;
    const uint32_t pressure;
    const float humidity;
    const uint32_t gas_resistance;
};

#endif // BME680_SENSOR_DATA_H
