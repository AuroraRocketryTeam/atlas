#include "SensorTask.hpp"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_task_wdt.h"

// Loop and logging rates live in SensorTask.hpp so HIL can mirror them.

// health check thresholds and reporting period
static constexpr uint32_t HEALTH_CHECK_PERIOD_MS = 5000;
static constexpr uint32_t MIN_STACK_REMAINING_BYTES = 512;
static constexpr uint32_t LOW_HEAP_THRESHOLD_BYTES = 5 * 1024;

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
#ifndef CONFIG_AURORA_HIL_SIMULATION
    uint32_t sensorLogCounter = 0;
#endif
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (running)
    {
        // CRITICAL: Reset the watchdog every loop (watchdog created in BaseTask)
        esp_task_wdt_reset();

        // Check running flag early to exit quickly during shutdown
        if (!running || !rocketModel) break;
        
        // Update sensors through the model with mutex protection
        rocketModel->updateBNO055();
        rocketModel->updateMS561101BA03_1();
        rocketModel->updateMS561101BA03_2();
        rocketModel->updateLIS3DHTR();

        if (!running) break;

        // FreeRTOS reports the minimum-ever remaining stack, not stack usage.
        const uint32_t stackRemaining = uxTaskGetStackHighWaterMark(nullptr);
        const uint32_t freeHeap = esp_get_free_heap_size();

        LOG_EVERY_MS(HEALTH_CHECK_PERIOD_MS, INFO, "Sensor", "Stack HwM:%u, Heap=%u, Memory=%u",
                     static_cast<unsigned>(stackRemaining), static_cast<unsigned>(freeHeap),
                     static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));

        if (stackRemaining < MIN_STACK_REMAINING_BYTES || freeHeap < LOW_HEAP_THRESHOLD_BYTES) {
            LOG_EVERY_MS(HEALTH_CHECK_PERIOD_MS, WARNING, "Sensor",
                         "Resource warning: stack remaining=%u bytes, heap=%u bytes, largest block=%u bytes",
                         static_cast<unsigned>(stackRemaining),
                         static_cast<unsigned>(freeHeap),
                         static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
        }

#ifndef CONFIG_AURORA_HIL_SIMULATION
        // Record every second acquisition cycle: 25 Hz from the 50 Hz outer loop.
        if (++sensorLogCounter >= SENSOR_LOG_INTERVAL_LOOPS)
        {
            sensorLogCounter = 0;
            // Log sensor data through the model
            if (logger && rocketModel)
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
        }
#endif

        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SENSOR_LOOP_PERIOD_MS));
    }
}