#include "MPRLSSensor.hpp"
#include <config.h>
#include <utils.h>

MPRLSSensor::MPRLSSensor()
{
    _mprls = Adafruit_MPRLS();
}

bool MPRLSSensor::init()
{
    int attempts = 0;
    uint start = Utils::millis();
    while (!_mprls.begin() && attempts++ < SENSOR_LOOKUP_MAX_ATTEMPTS)
    {
        uint end = Utils::millis();
        if (end - start > SENSOR_LOOKUP_TIMEOUT)
        {
            start = Utils::millis();
        }
    }
    if (attempts >= SENSOR_LOOKUP_MAX_ATTEMPTS)
    {
        return this->isInitialized();
    }
    this->setInitialized(true);
    return this->isInitialized();
}

bool MPRLSSensor::updateData()
{
    _data = PressureSensorData("MPRLS");

    _data.pressure = _mprls.readPressure();

    _data.timestamp = Utils::millis();

    if (isnan(_data.pressure)) {
        return false;
    }

    return true;
}

PressureSensorData MPRLSSensor::getData() {
    return _data;
}