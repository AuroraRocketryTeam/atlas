#include "SerialLogger.hpp"
#include "esp_heap_caps.h"
#include <Arduino.h>
#include <inttypes.h>
#include <utils.h>

// Serial mutex for thread-safe printing
static SemaphoreHandle_t serialMutex = nullptr;

namespace SerialLogger
{

    void init()
    {
        if (serialMutex == nullptr)
        {
            serialMutex = xSemaphoreCreateMutex();
        }
    }

    void log(LogLevel level, const char *tag, const char *format, ...)
    {
        if (serialMutex == nullptr)
        {
            init();
        }

        // Use shorter timeout to detect deadlocks faster
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            char buffer[256];
            va_list args;
            va_start(args, format);

            // Add timestamp, log level, and tag
            unsigned long timestamp = Utils::millis();

#if CONFIG_AURORA_HIL_SIMULATION
            unsigned long real_timestamp = Utils::realMillis();
#endif

            const char *levelStr = "";
            switch (level)
            {
            case LogLevel::ERROR:
                levelStr = "ERROR";
                break;
            case LogLevel::WARNING:
                levelStr = "WARN ";
                break;
            case LogLevel::INFO:
                levelStr = "INFO ";
                break;
            case LogLevel::DEBUG:
                levelStr = "DEBUG";
                break;
            case LogLevel::TRACE:
                levelStr = "TRACE";
                break;
            }

            vsnprintf(buffer, sizeof(buffer), format, args);
#if CONFIG_AURORA_HIL_SIMULATION            
            printf("[%8" PRIu32 "][%8" PRIu32 "][%s][%s] %s\n", real_timestamp, timestamp, levelStr, tag, buffer);
#else
            printf("[%8" PRIu32 "][%s][%s] %s\n", timestamp, levelStr, tag, buffer);
#endif
            va_end(args);
            xSemaphoreGive(serialMutex);
        }
        else
        {
            printf("[DEADLOCK] Failed to acquire log mutex for: %s\n", tag);
        }
    }

    void debugMemory(const char *location)
    {
        size_t maxAlloc = heap_caps_get_largest_free_block(MALLOC_CAP_DEFAULT);
        
        // Log the memory stat so the variable isn't wasted
        log(LogLevel::INFO, location, "Max Allocatable Block: %zu bytes", maxAlloc);
    }

    SemaphoreHandle_t getSerialMutex()
    {
        if (serialMutex == nullptr)
        {
            init();
        }
        return serialMutex;
    }

    Lock::Lock()
        : _held(xSemaphoreTake(getSerialMutex(), pdMS_TO_TICKS(100)) == pdTRUE)
    {
    }

    Lock::~Lock()
    {
        if (_held)
        {
            xSemaphoreGive(serialMutex);
        }
    }
}
