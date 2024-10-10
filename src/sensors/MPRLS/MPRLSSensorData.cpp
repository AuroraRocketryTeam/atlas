#include "MPRLSSensorData.h"

String MPRLSSensorData::toString()
{
    return String("Pressure: " + String(this->getPressure()) + " hPa");
}

void MPRLSSensorData::log(CborLogger &logger)
{
    json serialized_data;
    serialized_data["Pressure"] = this->getPressure();
    logger.addEntry("MPRLS", serialized_data);
}