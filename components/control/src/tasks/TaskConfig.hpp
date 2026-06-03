#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Arduino.h>

/**
 * @brief Enumeration for different task types.
 * 
 */
enum class TaskType
{
    SENSOR,
    SIMULATION,
    HIL_SIMULATION,
    EKF,
    RECOVERY,
    DATA_COLLECTION,
    STORAGE,
    TELEMETRY,
    GPS,
    ALTITUDE,
    LOGGING,
    AIRBRAKES
};

/**
 * @brief Enumeration for task priority levels.
 * 
 */
enum class TaskPriority
{
    TASK_LOW = 1,
    TASK_MEDIUM = 2,
    TASK_HIGH = 3,
    TASK_CRITICAL = 4,
    TASK_REAL_TIME = 5
};

/**
 * @brief Enumeration for task core assignment.
 * 
 */
enum class TaskCore
{
    CORE_0 = 0, // Critical tasks
    CORE_1 = 1, // Non-critical tasks
    ANY_CORE = tskNO_AFFINITY
};

/**
 * @brief Structure to hold task configuration parameters.
 * 
 */
struct TaskConfig
{
    TaskType type;
    const char *name;
    uint32_t stackSize;
    TaskPriority priority;
    TaskCore coreId;
    bool shouldRun;

    /**
     * @brief Construct a new Task Config object
     *
     * @param t TaskType: The type of task @see enum class TaskType
     * @param n const char*: The name of the task
     * @param stack uint32_t: The stack size for the task
     * @param prio TaskPriority: The priority of the task @see enum class TaskPriority
     * @param core TaskCore: The core ID for the task @see enum class TaskCore
     * @param run bool: Flag indicating if the task should run
     */
    TaskConfig(TaskType t = TaskType::SENSOR, const char *n = "Task", uint32_t stack = 2048,
               TaskPriority prio = TaskPriority::TASK_MEDIUM,
               TaskCore core = TaskCore::ANY_CORE, bool run = true)
        : type(t), name(n), stackSize(stack), priority(prio), coreId(core), shouldRun(run) {}
};

/**
 * @brief Convert TaskType enum to string representation
 * * Uses switch statement to ensure compile-time checking - if a new task
 * is added to the enum, the compiler will warn about a missing case.
 * * @param type The task type to convert
 * @return const char* String representation of the task type
 */
inline const char* taskTypeToString(TaskType type) {
    switch (type) {
        case TaskType::SENSOR:          return "SENSOR";
        case TaskType::SIMULATION:      return "SIMULATION";
        case TaskType::HIL_SIMULATION:  return "HIL_SIMULATION";
        case TaskType::EKF:             return "EKF";
        case TaskType::RECOVERY:        return "RECOVERY";
        case TaskType::DATA_COLLECTION: return "DATA_COLLECTION";
        case TaskType::STORAGE:         return "STORAGE";
        case TaskType::TELEMETRY:       return "TELEMETRY";
        case TaskType::GPS:             return "GPS";
        case TaskType::ALTITUDE:        return "ALTITUDE";
        case TaskType::LOGGING:         return "LOGGING";
        case TaskType::AIRBRAKES:       return "AIRBRAKES";
    }
    return "UNKNOWN_TASK";
}