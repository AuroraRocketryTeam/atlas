#include "GpsTask.hpp"

void GpsTask::taskFunction()
{
    while (running)
    {
        esp_task_wdt_reset(); // Reset watchdog created in BaseTask
        if (gps)
        {
            auto gpsData = gps->getData();
            // Write to shared data
            if (gpsData)
            {
                if (dataMutex && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(10)) == pdTRUE)
                {
                    sensorData->gpsData = *gpsData;
                    LOG_INFO("GpsTask", "Got GPS data");
                    xSemaphoreGive(dataMutex);
                }
                else
                {
                    LOG_WARNING("GpsTask", "Failed to take data mutex");
                }
            }
            else
            {
                LOG_WARNING("GpsTask", "No GPS data available");
            }
        }
        else
        {
            LOG_WARNING("GpsTask", "No GPS sensor available");
        }
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