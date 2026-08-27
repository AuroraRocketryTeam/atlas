#include "SensorTask.hpp"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_task_wdt.h"

SensorTask::SensorTask(std::shared_ptr<RocketModel> rocketModel,
                        std::shared_ptr<RocketLogger> logger)
    : BaseTask("SensorTask"), 
      rocketModel(rocketModel),
      logger(logger)
{
    LOG_INFO("Sensor", "SensorTask constructor initialized");
}

void SensorTask::onTaskStart()
{
    LOG_INFO("Sensor", "Task started with stack: %u bytes", config.stackSize);
    LOG_INFO("Sensor", "Model pointer: %s", rocketModel ? "OK" : "NULL");
}

void SensorTask::onTaskStop()
{
    LOG_INFO("Sensor", "Task stopped");
}

void SensorTask::taskFunction()
{
    uint32_t loopCount = 0;

    while (running)
    {
        // CRITICAL: Reset the watchdog every loop (watchdog created in BaseTask)
        esp_task_wdt_reset();
        LOG_INFO("SensorTask", "READING SENSORS");
        
        // Check running flag early to exit quickly during shutdown
        if (!running || !rocketModel) break;
        
        // Update sensors through the model with mutex protection
        rocketModel->updateBNO055();
        rocketModel->updateMS561101BA03_1();
        rocketModel->updateMS561101BA03_2();
        rocketModel->updateLIS3DHTR();

        LOG_DEBUG("Sensor", "Updated all sensors");
        
        if (!running) break;

        // Log memory usage every 10 loops
        if (loopCount % 10 == 0)
        {
            uint32_t freeHeap = esp_get_free_heap_size();
            LOG_INFO("Sensor", "L%lu: Stack HwM:%u, Heap=%u, Memory=%u",
                          loopCount, uxTaskGetStackHighWaterMark(NULL), freeHeap,
                          heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
                          
            // Check for low memory condition
            if (freeHeap < 50000) { // Warning threshold
                LOG_WARNING("Sensor", "LOW MEMORY WARNING: Only %u bytes free heap remaining!", freeHeap);
            }
        }

#ifndef CONFIG_AURORA_HIL_SIMULATION
        // Log sensor data every 3 loops if logger is available
        if (logger)
        {
            // Log sensor data through the model
            if (rocketModel)
            {
                IMUData outBnoData;
                SensorReadStatus bnoStatus = rocketModel->getBNO055Data(outBnoData);
                if (bnoStatus == SensorReadStatus::OK) {
                    logger->logSensorData(outBnoData);
                }

                PressureSensorData outMs56Data1;
                SensorReadStatus baro1Status = rocketModel->getMS561101BA03Data_1(outMs56Data1);
                if (baro1Status == SensorReadStatus::OK) {
                    logger->logSensorData(outMs56Data1);
                }

                PressureSensorData outMs56Data2;
                SensorReadStatus baro2Status = rocketModel->getMS561101BA03Data_2(outMs56Data2);
                if (baro2Status == SensorReadStatus::OK) {
                    logger->logSensorData(outMs56Data2);
                }

                AccelerometerSensorData outLis3dhData;
                SensorReadStatus accelStatus = rocketModel->getLIS3DHTRData(outLis3dhData);
                if (accelStatus == SensorReadStatus::OK) {
                    logger->logSensorData(outLis3dhData);
                }
            }

            // Log current RocketLogger memory usage for monitoring
            LOG_INFO("Sensor", "RocketLogger entries logged");
        }
#endif

        loopCount++;
        
        // Shorter delay to exit faster (split into smaller chunks)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}