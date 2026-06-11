#include "GpsTask.hpp"

GpsTask::GpsTask(std::shared_ptr<RocketModel> rocketModel,
                        std::shared_ptr<RocketLogger> logger
        )
        : BaseTask("GpsTask"),
          _rocketModel(rocketModel),
                    _logger(logger)
    {
        LOG_INFO("GpsTask", "Initialized GPS task");
    }

void GpsTask::taskFunction()
{
    unsigned long loopCounter = 0;
    while (running)
    {
        esp_task_wdt_reset();

        _rocketModel->updateGPS();
        GPSData gpsData;
        SensorReadStatus gpsStatus = _rocketModel->getGPSData(gpsData);
        if (gpsStatus != SensorReadStatus::OK) {
            LOG_WARNING("GpsTask", "Failed to get GPS data");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        LOG_INFO("GpsTask", "Got GPS data");
        
        if (_logger) {
            // Only log GPS data every 10 loops (every ~2 seconds) to reduce memory pressure
            if ((loopCounter % 10) == 0) {

                _logger->logSensorData(gpsData);

                // Log current RocketLogger memory usage for monitoring
                if ((loopCounter % 50) == 0) {
                    LOG_INFO("GpsTask", "RocketLogger entries: %d", _logger->getLogCount());
                }
            }
        }

        loopCounter++;
        
        // Split 200ms delay into 20x10ms chunks for faster exit (still 5 Hz)
        for (int i = 0; i < 20 && running; i++)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    LOG_INFO("GpsTask", "Task exiting");
}

void GpsTask::onTaskStart()
{
    LOG_INFO("GpsTask", "Task started with stack: %u bytes", config.stackSize);
}

void GpsTask::onTaskStop()
{
    LOG_INFO("GpsTask", "Task stopped");
}