#include "SerialLogger.hpp"
#include "esp_heap_caps.h"
// TODO: replace ESP calls
#include <Arduino.h>
#include <inttypes.h>
#include <cstdio>
#include <cstring>
#include <utils.h>

// TODO: should the SerialLogger be disabled during flight?
// For logs that needs to persist we must use RocketLogger.



// Serial output is shared by many FreeRTOS tasks. Keep printf calls behind one
// mutex so log lines do not interleave and produce unreadable diagnostics.
static SemaphoreHandle_t serialMutex = nullptr;

// Recent logs are kept separately from the serial mutex so Ground Services can
// read the web log buffer without blocking all serial output for long periods.
static SemaphoreHandle_t recentLogMutex = nullptr;

// Fixed-size ring buffer used by the web console. This intentionally stores the
// final formatted log line, not the original format string/arguments.
static constexpr size_t RECENT_LOG_COUNT = 50;
static constexpr size_t RECENT_LOG_LINE_LEN = 256;
static char recentLogs[RECENT_LOG_COUNT][RECENT_LOG_LINE_LEN] = {};
static uint32_t recentLogSequences[RECENT_LOG_COUNT] = {};
static size_t recentLogNext = 0;
static size_t recentLogUsed = 0;

// Monotonic sequence assigned after each successfully buffered line. Clients can
// poll "logs since sequence N" without receiving the whole ring every time.
static uint32_t recentLogSequence = 0;

namespace SerialLogger
{

    /**
     * @brief Lazily create the logger mutexes.
     *
     * Logging can happen before the rest of the application is fully started, so
     * each public function calls init() on demand instead of relying on a global
     * initialization order.
     */
    void init()
    {
        if (serialMutex == nullptr)
        {
            serialMutex = xSemaphoreCreateMutex();
        }
        if (recentLogMutex == nullptr)
        {
            recentLogMutex = xSemaphoreCreateMutex();
        }
    }

    /**
     * @brief Format one log line, print it, and copy it into the recent-log ring.
     *
     * The serial mutex covers formatting and printf so one task owns the whole
     * line. The recent-log mutex only covers ring mutation. If either mutex is
     * busy for too long, the logger drops the buffered copy or emits a deadlock
     * warning instead of blocking flight tasks indefinitely.
     */
    void log(LogLevel level, const char *tag, const char *format, ...)
    {
        if (serialMutex == nullptr)
        {
            init();
        }

        // Use shorter timeout to detect deadlocks faster
        if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
        {
            // The first buffer holds only the caller's formatted payload. The
            // second buffer adds timestamp, level, and tag for final output.
            char buffer[192];
            char line[RECENT_LOG_LINE_LEN];
            va_list args;
            va_start(args, format);

            // Add timestamp, log level, and tag
            unsigned long timestamp = Utils::millis();

#if AURORA_HIL_ENABLED
            // HIL logs include both wall-clock time and simulated mission time.
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
#if AURORA_HIL_ENABLED
            snprintf(line, sizeof(line), "[%8" PRIu32 "][%8" PRIu32 "][%s][%s] %s",
                     real_timestamp, timestamp, levelStr, tag, buffer);
#else
            snprintf(line, sizeof(line), "[%8" PRIu32 "][%s][%s] %s",
                     timestamp, levelStr, tag, buffer);
#endif
            printf("%s\n", line);

            // Best-effort web-console copy. Serial output is the primary sink;
            // if this short lock cannot be acquired, skip buffering this line.
            if (recentLogMutex != nullptr && xSemaphoreTake(recentLogMutex, pdMS_TO_TICKS(10)) == pdTRUE)
            {
                snprintf(recentLogs[recentLogNext], RECENT_LOG_LINE_LEN, "%s", line);
                recentLogSequence++;
                recentLogSequences[recentLogNext] = recentLogSequence;
                recentLogNext = (recentLogNext + 1) % RECENT_LOG_COUNT;
                if (recentLogUsed < RECENT_LOG_COUNT) recentLogUsed++;
                xSemaphoreGive(recentLogMutex);
            }
            va_end(args);
            xSemaphoreGive(serialMutex);
        }
        else
        {
            printf("[DEADLOCK] Failed to acquire log mutex for: %s\n", tag);
        }
    }

    /**
     * @brief Copy the current recent-log ring into a newline-delimited buffer.
     *
     * Lines are returned oldest-to-newest. The function truncates safely when
     * the caller buffer is too small and always leaves output null-terminated.
     */
    size_t getRecentLogs(char *out, size_t out_size)
    {
        if (out == nullptr || out_size == 0) return 0;
        if (recentLogMutex == nullptr) init();

        out[0] = '\0';
        size_t offset = 0;

        if (recentLogMutex != nullptr && xSemaphoreTake(recentLogMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            // recentLogNext points to the next write slot, so the oldest valid
            // line is "used" entries behind it in the ring.
            const size_t first = (recentLogNext + RECENT_LOG_COUNT - recentLogUsed) % RECENT_LOG_COUNT;
            for (size_t i = 0; i < recentLogUsed && offset + 1 < out_size; ++i)
            {
                const size_t idx = (first + i) % RECENT_LOG_COUNT;
                int written = snprintf(out + offset, out_size - offset, "%s\n", recentLogs[idx]);
                if (written < 0) break;
                if (static_cast<size_t>(written) >= out_size - offset)
                {
                    offset = out_size - 1;
                    out[offset] = '\0';
                    break;
                }
                offset += static_cast<size_t>(written);
            }
            xSemaphoreGive(recentLogMutex);
        }

        return offset;
    }

    /**
     * @brief Copy recent logs newer than a client-known sequence number.
     *
     * Ground Services uses this for incremental polling. latest_seq returns the
     * newest sequence currently known even when no new lines fit in the output.
     */
    size_t getRecentLogsSince(uint32_t since_seq, char *out, size_t out_size, uint32_t *latest_seq)
    {
        if (out == nullptr || out_size == 0) return 0;
        if (recentLogMutex == nullptr) init();

        out[0] = '\0';
        size_t offset = 0;

        if (recentLogMutex != nullptr && xSemaphoreTake(recentLogMutex, pdMS_TO_TICKS(50)) == pdTRUE)
        {
            if (latest_seq != nullptr) *latest_seq = recentLogSequence;

            const size_t first = (recentLogNext + RECENT_LOG_COUNT - recentLogUsed) % RECENT_LOG_COUNT;
            for (size_t i = 0; i < recentLogUsed && offset + 1 < out_size; ++i)
            {
                const size_t idx = (first + i) % RECENT_LOG_COUNT;

                // The ring may have wrapped since the client last polled; in
                // that case older sequences are already gone and only retained
                // newer lines can be returned.
                if (recentLogSequences[idx] <= since_seq) continue;

                int written = snprintf(out + offset, out_size - offset, "%s\n", recentLogs[idx]);
                if (written < 0) break;
                if (static_cast<size_t>(written) >= out_size - offset)
                {
                    offset = out_size - 1;
                    out[offset] = '\0';
                    break;
                }
                offset += static_cast<size_t>(written);
            }
            xSemaphoreGive(recentLogMutex);
        }

        return offset;
    }

    /**
     * @brief Log a small heap diagnostic at a named call site.
     */
    void debugMemory(const char *location)
    {
        size_t maxAlloc = ESP.getMaxAllocHeap();
        
        // Log the memory stat so the variable isn't wasted
        log(LogLevel::INFO, location, "Max Allocatable Block: %zu bytes", maxAlloc);
    }

    /**
     * @brief Return the serial mutex for rare callers that need external locking.
     */
    SemaphoreHandle_t getSerialMutex()
    {
        if (serialMutex == nullptr)
        {
            init();
        }
        return serialMutex;
    }
}
