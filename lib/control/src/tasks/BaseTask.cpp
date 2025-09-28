#include "BaseTask.hpp"
#include "esp_heap_caps.h"
#include <Logger.hpp>

BaseTask::BaseTask(const char *name)
    : taskHandle(nullptr), running(false), taskName(name)
{
    // Constructor
}

BaseTask::~BaseTask()
{
    stop();
}

bool BaseTask::start(const TaskConfig &config)
{
    if (running)
        return false;

    this->config = config;
    BaseType_t result;
    if (config.coreId == TaskCore::ANY_CORE)
    {
        result = xTaskCreate(
            taskWrapper,
            config.name,
            config.stackSize,
            this,
            static_cast<UBaseType_t>(config.priority),
            &taskHandle);
    }
    else
    {
        result = xTaskCreatePinnedToCore(
            taskWrapper,
            config.name,
            config.stackSize,
            this,
            static_cast<UBaseType_t>(config.priority),
            &taskHandle,
            static_cast<BaseType_t>(config.coreId));
    }

    if (result == pdPASS)
    {
        running = true;
        LOG_INFO("BaseTask", "%s started on core %d", taskName, static_cast<int>(config.coreId));
        return true;
    }

    LOG_ERROR("BaseTask", "Failed to create task %s", taskName);
    return false;
}

void BaseTask::stop()
{
    if (!running)
        return;

    // Set flag first
    running = false;

    // Give task time to see the flag change
    vTaskDelay(pdMS_TO_TICKS(100));

    // Verify task handle is still valid
    if (taskHandle != nullptr)
    {
        // Check if task actually exists
        eTaskState taskState = eTaskGetState(taskHandle);
        if (taskState != eDeleted)
        {
            vTaskSuspend(taskHandle);
            vTaskDelay(pdMS_TO_TICKS(50));
            vTaskDelete(taskHandle);
        }
        taskHandle = nullptr;
    }

    LOG_INFO("BaseTask", "Task %s stopped safely", taskName);
}

void BaseTask::taskWrapper(void *parameter)
{
    BaseTask *task = static_cast<BaseTask *>(parameter);
    if (task)
    {
        task->internalTaskFunction();
    }
    vTaskDelete(NULL);
}

void BaseTask::internalTaskFunction()
{
    // Register with watchdog
    esp_task_wdt_add(NULL);

    LOG_INFO("BaseTask", "[TASK] %s starting on core %d", taskName, xPortGetCoreID());

    onTaskStart();

    try
    {
        taskFunction();
    }
    catch (...)
    {
        LOG_ERROR("BaseTask", "Exception in task %s", taskName);
    }

    onTaskStop();

    // Cleanup watchdog
    esp_task_wdt_delete(NULL);

    LOG_INFO("BaseTask", "[TASK] %s ended", taskName);
}

uint32_t BaseTask::getStackHighWaterMark() const
{
    return taskHandle ? uxTaskGetStackHighWaterMark(taskHandle) : 0;
}