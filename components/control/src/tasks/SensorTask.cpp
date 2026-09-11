#include "SensorTask.hpp"
#include "esp_task_wdt.h"

// constants for sensor task loop and logging rates
static constexpr uint32_t SENSOR_LOOP_RATE_HZ = 50;
static constexpr uint32_t SENSOR_LOG_RATE_HZ = 25;
static constexpr uint32_t SENSOR_LOOP_PERIOD_MS = 1000 / SENSOR_LOOP_RATE_HZ;
static constexpr uint32_t SENSOR_LOG_INTERVAL_LOOPS = SENSOR_LOOP_RATE_HZ / SENSOR_LOG_RATE_HZ;

// logging and health check thresholds
static constexpr uint32_t HEALTH_CHECK_INTERVAL_LOOPS = SENSOR_LOOP_RATE_HZ * 5;
static constexpr uint32_t MIN_STACK_REMAINING_BYTES = 512;
static constexpr uint32_t LOW_HEAP_THRESHOLD_BYTES = 5 * 1024;

static_assert(SENSOR_LOOP_RATE_HZ % SENSOR_LOG_RATE_HZ == 0,
              "Sensor log rate must divide the sensor loop rate");

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
    uint32_t healthCheckCounter = 0;
#if !AURORA_HIL_ENABLED
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
        if (++healthCheckCounter >= HEALTH_CHECK_INTERVAL_LOOPS)
        {
            healthCheckCounter = 0;
            const uint32_t stackRemaining = uxTaskGetStackHighWaterMark(nullptr);
            const uint32_t freeHeap = ESP.getFreeHeap();
            if (stackRemaining < MIN_STACK_REMAINING_BYTES || freeHeap < LOW_HEAP_THRESHOLD_BYTES) {
                LOG_WARNING("Sensor", "Resource warning: stack remaining=%u bytes, heap=%u bytes, largest block=%u bytes",
                            static_cast<unsigned>(stackRemaining),
                            static_cast<unsigned>(freeHeap),
                            static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)));
            }
        }

#if !AURORA_HIL_ENABLED
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

            // Log current RocketLogger memory usage for monitoring
            // LOG_INFO("Sensor", "RocketLogger entries logged");
        }
#endif
        
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SENSOR_LOOP_PERIOD_MS));
    }
}
