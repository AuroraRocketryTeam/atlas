#ifndef MPRLS_SENSOR_DATA_H
#define MPRLS_SENSOR_DATA_H
#include "sensors/SensorData.h"

class MPRLSSensorData : public SensorData
{
public:
    MPRLSSensorData() : pressure(-1) {}

    MPRLSSensorData(float _pressure) : pressure(_pressure) {}

    float getPressure() const { return this->pressure; };
    String toString() override;

private:
    float pressure;
};
#endif // MPRLS_SENSOR_DATA_H