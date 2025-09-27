#pragma once

#include "BaseTask.hpp"
#include "SharedData.hpp"
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <GPS.hpp>

class GpsTask : public BaseTask {
public:
    GpsTask(std::shared_ptr<SharedSensorData> sensorData, 
        SemaphoreHandle_t sensorDataMutex) : 
        BaseTask("GpsTask"),
        sensorData(sensorData),
        sensorDataMutex(sensorDataMutex) {        
            gps.init();
        }
    ~GpsTask() override;

protected:
    void taskFunction() override;

private:
    std::shared_ptr<SharedSensorData> sensorData;
    SemaphoreHandle_t sensorDataMutex;
    GPS gps;
};