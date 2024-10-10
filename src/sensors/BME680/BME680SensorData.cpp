#include "sensors/BME680/BME680SensorData.h"

String BME680SensorData::toString()
{
    return "Temperature: " + String(this->getTemperature()) + " °C\n" +
           "Pressure: " + String(this->getPressure()) + " Pa\n" +
           "Humidity: " + String(this->getHumidity()) + " %";
           // "Gas Resistance: " + String(this->getGasResistance()) + " Ohms\n";    // Not used.
}

void BME680SensorData::log(CborLogger &logger)
{
    json serialized_data;
    serialized_data["Temperature"] = this->getTemperature();
    serialized_data["Pressure"] = this->getPressure();
    serialized_data["Humidity"] = this->getHumidity();
    // serialized_data["Gas Resistance"] = this->getGasResistance();    // Not used.
    logger.addEntry("BME680", serialized_data);
}