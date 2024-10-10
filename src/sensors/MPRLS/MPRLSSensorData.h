#ifndef MPRLS_SENSOR_DATA_H
#define MPRLS_SENSOR_DATA_H
#include "sensors/SensorData.h"

class MPRLSSensorData : public SensorData
{
    friend class MPRLSSensor;

public:
    float getPressure() const { return this->pressure; }
    String toString() override;
    void log(CborLogger &logger) override;

private:
    MPRLSSensorData() : pressure(-10000) {}
    MPRLSSensorData(float _pressure) : pressure(_pressure) {}
    const float pressure;
};
#endif // MPRLS_SENSOR_DATA_H