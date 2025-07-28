#pragma once

#include <BNO055SensorInterface.hpp>
#include <ISensor.hpp>

class BNO055Sensor : public ISensor
{
public:
    BNO055Sensor();
    bool init() override;
    bool calibrate();
    bool hardwareTest();
    std::optional<SensorData> getData() override;    

private:
    BNO055SensorInterface bno_interface;
};

