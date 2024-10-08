#include "sensors/BME680/BME680SensorData.h"

float BME680SensorData::getTemperature() const
{
    return temperature;
}

uint32_t BME680SensorData::getPressure() const
{
    return pressure;
}

float BME680SensorData::getHumidity() const
{
    return humidity;
}

uint32_t BME680SensorData::getGasResistance() const
{
    return gas_resistance;
}

void BME680SensorData::toString()
{
    Serial.print("Temperature = ");
    Serial.print(temperature);
    Serial.println(" °C");

    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println(" Pa");

    Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial.print("Gas = ");
    Serial.print(gas_resistance);
    Serial.println(" Ohms");
}