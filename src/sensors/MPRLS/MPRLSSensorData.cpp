#include "MPRLSSensorData.h"

String MPRLSSensorData::toString()
{
    return String("Pressure: " + String(this->getPressure()) + " hPa");
}