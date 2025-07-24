#pragma once

#include <functional>
#include <vector>
#include <Arduino.h>

/**
 * @brief Task structure for the scheduler
 */
struct ScheduledTask
{
    std::function<void()> task;
    unsigned long interval;
    unsigned long lastExecution;
    bool enabled;
    String name;

    ScheduledTask(std::function<void()> t, unsigned long i, String n)
        : task(t), interval(i), lastExecution(0), enabled(true), name(n) {}
};

/**
 * @brief Simple cooperative scheduler for the rocket system
 */
class Scheduler
{
public:
    Scheduler();

    /**
     * @brief Add a task to the scheduler
     */
    void addTask(std::function<void()> task, unsigned long intervalMs, String name);

    /**
     * @brief Run the scheduler - should be called in main loop
     */
    void run();

    /**
     * @brief Enable/disable a task
     */
    void enableTask(const String &name, bool enable);

    /**
     * @brief Get number of tasks
     */
    size_t getTaskCount() const { return tasks.size(); }

private:
    std::vector<ScheduledTask> tasks;
};