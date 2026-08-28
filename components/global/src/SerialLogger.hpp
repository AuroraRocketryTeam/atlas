#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include "config.h"

namespace SerialLogger
{
    // Log levels for structured logging
    enum class LogLevel
    {
        ERROR,
        WARNING,
        INFO,
        DEBUG,
        TRACE
    };

    /// @brief Initialize the logger (creates the serial mutex if not already created).
    void init();

    /// @brief Thread-safe log function with printf-style formatting.
    /// @param level Log severity level
    /// @param tag Identifier for the log source
    /// @param format printf-style format string
    /// @param ... Arguments for format string
    void log(LogLevel level, const char *tag, const char *format, ...);

    /// @brief Prints memory usage statistics with fragmentation info.
    /// @param location Identifier string for the debug location
    void debugMemory(const char *location);

    /// @brief Returns the Serial print mutex handle (creates it if not initialized).
    /// @return SemaphoreHandle_t The mutex handle
    SemaphoreHandle_t getSerialMutex();

    /// @brief RAII guard over the Serial print mutex.
    ///
    /// For code that writes to the console without going through log(): the
    /// binary telemetry mirror and the JSONL flash dump.
    /// Keep the scope short, log() only waits 100 ms for the mutex before
    /// giving up and printing a [DEADLOCK] line.
    class Lock
    {
    public:
        Lock();
        ~Lock();

        Lock(const Lock &) = delete;
        Lock &operator=(const Lock &) = delete;

        /// @brief Whether the mutex was actually acquired.
        bool held() const { return _held; }

    private:
        bool _held;
    };
}

#define LOG_ERROR(tag, format, ...) SerialLogger::log(SerialLogger::LogLevel::ERROR, tag, format, ##__VA_ARGS__)
#define LOG_WARNING(tag, format, ...) SerialLogger::log(SerialLogger::LogLevel::WARNING, tag, format, ##__VA_ARGS__)
#define LOG_INFO(tag, format, ...) SerialLogger::log(SerialLogger::LogLevel::INFO, tag, format, ##__VA_ARGS__)
#define LOG_DEBUG(tag, format, ...) SerialLogger::log(SerialLogger::LogLevel::DEBUG, tag, format, ##__VA_ARGS__)
#define LOG_TRACE(tag, format, ...) SerialLogger::log(SerialLogger::LogLevel::TRACE, tag, format, ##__VA_ARGS__)

/// @brief Log at most one line per `ms` milliseconds, per call site.
///
/// @note The counters are function-local statics, so they are shared across
///       instances of a class and are not atomic. Both are fine here: every
///       throttled site lives in a single-instance task and is reached from
///       exactly one thread.
///
/// @param ms Minimum interval between two emissions, in milliseconds
/// @param level Bare level name: ERROR, WARNING, INFO, DEBUG or TRACE
/// @param tag Identifier for the log source
/// @param format printf-style format string
#define LOG_EVERY_MS(ms, level, tag, format, ...)                         \
    do                                                                    \
    {                                                                     \
        static TickType_t _fcLogLast = 0;                                 \
        static bool _fcLogFirst = true;                                   \
        const TickType_t _fcLogNow = xTaskGetTickCount();                 \
        if (_fcLogFirst || (_fcLogNow - _fcLogLast) >= pdMS_TO_TICKS(ms)) \
        {                                                                 \
            _fcLogFirst = false;                                          \
            _fcLogLast = _fcLogNow;                                       \
            LOG_##level(tag, format, ##__VA_ARGS__);                      \
        }                                                                 \
    } while (0)

/// @brief Log on the first call and every `n`th call thereafter.
///
/// @param n Emit once every n calls
/// @param level Bare level name: ERROR, WARNING, INFO, DEBUG or TRACE
#define LOG_EVERY_N(n, level, tag, format, ...)      \
    do                                               \
    {                                                \
        static uint32_t _fcLogCount = 0;             \
        if ((_fcLogCount++ % (n)) == 0)              \
        {                                            \
            LOG_##level(tag, format, ##__VA_ARGS__); \
        }                                            \
    } while (0)

/// @brief Log only when `value` differs from what this call site last saw.
///
/// @param value Value to compare against the previous call (needs operator==)
/// @param level Bare level name: ERROR, WARNING, INFO, DEBUG or TRACE
#define LOG_ON_CHANGE(value, level, tag, format, ...) \
    do                                                \
    {                                                 \
        static decltype(value) _fcLogPrev{};          \
        static bool _fcLogSeen = false;               \
        if (!_fcLogSeen || !(_fcLogPrev == (value)))  \
        {                                             \
            _fcLogSeen = true;                        \
            _fcLogPrev = (value);                     \
            LOG_##level(tag, format, ##__VA_ARGS__);  \
        }                                             \
    } while (0)
