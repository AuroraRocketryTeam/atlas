#include "GpsTask.hpp"

void GpsTask::taskFunction()
{
    const TickType_t mutexTimeout = pdMS_TO_TICKS(10);
    unsigned long loopCounter = 0;
    while (running)
    {
        esp_task_wdt_reset();
        if (gps)
        {
            auto gpsData = gps->getData();
            // Write to shared data
            if (gpsData.has_value())
            {
                if (dataMutex && xSemaphoreTake(dataMutex, mutexTimeout) == pdTRUE)
                {
                    sensorData->gpsData = gpsData.value();
                    LOG_INFO("GpsTask", "Got GPS data");
                    xSemaphoreGive(dataMutex);
                    if ((loopCounter & 0x0F) == 0)
                        LOG_INFO("GpsTask", "GPS update stored");
                }
                else
                {
                    if ((loopCounter & 0x0F) == 0)
                        LOG_WARNING("GpsTask", "Failed to take data mutex");
                }
                
                if (xSemaphoreTake(loggerMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    auto timestampData = SensorData("Timestamp");
                    timestampData.setData("timestamp", static_cast<int>(millis()));
                    rocketLogger->logSensorData(timestampData);
                    
                    rocketLogger->logSensorData("GPS", gpsData.value());
                    xSemaphoreGive(loggerMutex);
                } else {
                    LOG_WARNING("GpsTask", "Failed to take logger mutex");
                }

            }
            else
            {
                // GPS fix or data might be temporarily unavailable; log sparsely
                if ((loopCounter & 0x3F) == 0)
                    LOG_WARNING("GpsTask", "No GPS data available");
            }
        }
        else
        {
            // Sensor missing - this is likely a configuration issue; log less frequently
            if ((loopCounter & 0x3F) == 0)
                LOG_WARNING("GpsTask", "No GPS sensor available");
        }

        loopCounter++;
        vTaskDelay(pdMS_TO_TICKS(200)); // 5 Hz
    }
    LOG_INFO("GpsTask", "Task exiting");
}

void GpsTask::onTaskStart()
{
    LOG_INFO("GpsTask", "Task started with stack: %u bytes", config.stackSize);
    LOG_INFO("GpsTask", "GPS: %s", gps ? "OK" : "NULL");
}

void GpsTask::onTaskStop()
{
    LOG_INFO("GpsTask", "Task stopped");
}