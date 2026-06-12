#include "MPRLSPrinter.hpp"

MPRLSPrinter::MPRLSPrinter(ISensor* sensor) : mprls(sensor) {}

void MPRLSPrinter::displayPressure() {
    auto data = mprls->getData();

    Serial.println("Pressure:" + String(data.pressure, 4));
}