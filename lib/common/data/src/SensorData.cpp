#include "SensorData.hpp"

json SensorData::toJSON() const
{
    json j;
    j["sensor"] = sensorName;
    return j;
}
