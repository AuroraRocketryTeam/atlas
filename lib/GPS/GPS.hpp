#pragma once

#include <ISensor.hpp>
#include <TinyGPSPlus.h>
#include <config.h>

class GPSSensor : public ISensor
{
public:
    GPSSensor();
    bool init() override;
    std::optional<SensorData> getData() override;

private:
    TinyGPSPlus gps;
};

