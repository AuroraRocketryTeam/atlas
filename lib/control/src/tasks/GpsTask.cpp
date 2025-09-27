#include "GpsTask.hpp"

GpsTask::~GpsTask() {
    stop();
}

void GpsTask::taskFunction() {
    while (true) {
        auto gpsData = gps.getData();

        // Write to shared data
        if (gpsData) {
            if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY) == pdTRUE) {
                sensorData->gpsData = *gpsData;

                xSemaphoreGive(sensorDataMutex);
            }
        }
        taskYIELD();
    }
}