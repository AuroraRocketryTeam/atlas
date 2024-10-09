#ifndef MPRLS_SENSOR_H
#define MPRLS_SENSOR_H
#include "sensors/Sensor.h"
#include "sensors/MPRLS/MPRLSSensorData.h"
#include <Adafruit_MPRLS.h>

class MPRLSSensor : public Sensor<MPRLSSensorData>
{
public:
    MPRLSSensor();
    bool init() override;
    bool readData() override;
    MPRLSSensorData getData() override;
private:
    Adafruit_MPRLS mprls;
    float pressure;
};
#endif // MPRLS_SENSOR_H