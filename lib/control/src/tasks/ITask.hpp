#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Arduino.h>
#include "TaskConfig.hpp"

/**
 * @brief Interface for defining tasks in the FSM.
 *
 */
class ITask
{
public:
    virtual ~ITask() = default;

    /**
     * @brief Start the task with the given configuration.
     *
     * @param config TaskConfig: Configuration for the task
     * @return true if the task started successfully
     * @return false otherwise
     */
    virtual bool start(const TaskConfig &config) = 0;
    /**
     * @brief Stop the task.
     *
     */
    virtual void stop() = 0;
    /**
     * @brief Check if the task is currently running.
     *
     * @return true if running
     * @return false otherwise
     */
    virtual bool isRunning() const = 0;
    /**
     * @brief Get the name of the task.
     *
     * @return const char* Name of the task
     */
    virtual const char *getName() const = 0;
    /**
     * @brief Get the stack high water mark for the task.
     *
     * @return uint32_t High water mark in bytes
     */
    virtual uint32_t getStackHighWaterMark() const = 0;

protected:
    virtual void taskFunction() = 0;
};