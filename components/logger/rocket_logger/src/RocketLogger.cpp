#include "RocketLogger.hpp"
#include <new>
#include <cstdlib>
#include <cstring>

// Every queue slot stores a full LogPayload variant in internal RAM.
static constexpr size_t MAX_QUEUE_LENGTH = 200;
// A whole flight emits a few dozen events, at most a handful in one FSM tick.
static constexpr size_t MAX_EVENT_QUEUE_LENGTH = 32;

RocketLogger::RocketLogger() {
    _logQueue = xQueueCreate(MAX_QUEUE_LENGTH, sizeof(LogPayload));
    if (_logQueue == nullptr) {
        LOG_ERROR("RocketLogger", "Failed to create log queue! Insufficient heap.");
    }
    _eventQueue = xQueueCreate(MAX_EVENT_QUEUE_LENGTH, sizeof(SystemLog));
    if (_eventQueue == nullptr) {
        LOG_ERROR("RocketLogger", "Failed to create event queue! Insufficient heap.");
    }
}

RocketLogger::~RocketLogger() {
    if (_logQueue != nullptr) {
        vQueueDelete(_logQueue);
        _logQueue = nullptr;
    }
    if (_eventQueue != nullptr) {
        vQueueDelete(_eventQueue);
        _eventQueue = nullptr;
    }
}

void RocketLogger::setSerializer(SerializeFunction serializeFn) {
    _serializeFn = serializeFn;
}

int RocketLogger::getLogCount() const {
    int count = getEventCount();
    if (_logQueue != nullptr) {
        count += uxQueueMessagesWaiting(_logQueue);
    }
    return count;
}

int RocketLogger::getEventCount() const {
    if (_eventQueue == nullptr) return 0;
    return uxQueueMessagesWaiting(_eventQueue);
}

uint32_t RocketLogger::getDroppedCount() const {
    return _dropped.load(std::memory_order_relaxed);
}

void RocketLogger::pushToQueue(const LogPayload& payload) {
    if (const SystemLog* event = std::get_if<SystemLog>(&payload)) {
        pushEvent(*event);
        return;
    }

    if (_logQueue == nullptr) return;

    // Timeout is 0 (pdMS_TO_TICKS(0)) to ensure the caller NEVER blocks.
    // If the queue is full, we drop the oldest item to make room.
    if (xQueueSend(_logQueue, &payload, 0) == errQUEUE_FULL) {
        LogPayload dummy;
        // This counter is shared by multiple producers. Atomic relaxed updates
        // avoid lost increments; it is statistics and does not guard other data.
        if (xQueueReceive(_logQueue, &dummy, 0) == pdTRUE) { // Discard oldest
            _dropped.fetch_add(1, std::memory_order_relaxed);
        }
        if (xQueueSend(_logQueue, &payload, 0) != pdTRUE) { // Insert newest
            _dropped.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

void RocketLogger::pushEvent(const SystemLog& event) {
    if (_eventQueue == nullptr) return;

    // Same never-block, drop-oldest policy as sensor data, but on a queue that
    // only events share, so it fills only if storage stalls for a long time.
    if (xQueueSend(_eventQueue, &event, 0) == errQUEUE_FULL) {
        SystemLog dummy;
        if (xQueueReceive(_eventQueue, &dummy, 0) == pdTRUE) {
            _dropped.fetch_add(1, std::memory_order_relaxed);
        }
        if (xQueueSend(_eventQueue, &event, 0) != pdTRUE) {
            _dropped.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

void RocketLogger::logInfo(const std::string& message) {
    logInfo("RocketLogger", message.c_str());
}

void RocketLogger::logInfo(const char* source, const char* message) {
    pushToQueue(SystemLog{"INFO", source, message});
}

void RocketLogger::logWarning(const std::string& message) {
    logWarning("RocketLogger", message.c_str());
}

void RocketLogger::logWarning(const char* source, const char* message) {
    pushToQueue(SystemLog{"WARNING", source, message});
}

void RocketLogger::logError(const std::string& message) {
    logError("RocketLogger", message.c_str());
}

void RocketLogger::logError(const char* source, const char* message) {
    pushToQueue(SystemLog{"ERROR", source, message});
}

void RocketLogger::logSensorData(const LogPayload& payload) {
    if (!_sensorLoggingEnabled.load()) return;
    pushToQueue(payload);
}

void RocketLogger::clearData() {
    if (_logQueue != nullptr) {
        xQueueReset(_logQueue); 
    }
    if (_eventQueue != nullptr) {
        xQueueReset(_eventQueue);
    }
}

size_t RocketLogger::consumeBatch(uint8_t* outBuffer, size_t bufferCapacity) {
    if (_logQueue == nullptr || _eventQueue == nullptr) {
        LOG_ERROR("RocketLogger", "consumeBatch failed: Log queue is not initialized!\n");
        return 0;
    }
    if (_serializeFn == nullptr) {
        LOG_ERROR("RocketLogger", "consumeBatch failed: Serialize function is not set!\n");
        return 0;
    }
    if (outBuffer == nullptr) {
        LOG_ERROR("RocketLogger", "consumeBatch failed: Output buffer is invalid!\n");
        return 0;
    }

    size_t currentOffset = 0;

    // Serializes queued items into outBuffer. Returns false once the batch is full.
    auto drainQueue = [&](QueueHandle_t queue, auto& item) -> bool {
        while (uxQueueMessagesWaiting(queue) > 0) {
            if (xQueueReceive(queue, &item, 0) != pdTRUE) {
                continue;
            }
            // serializeFn dynamically evaluates size and writes if it fits
            if (_serializeFn(LogPayload{item}, outBuffer, bufferCapacity, currentOffset)) {
                continue;
            }
            if (currentOffset == 0) {
                // Deadlock prevention: It's larger than the entire 4096-byte buffer!
                LOG_ERROR("RocketLogger", "CRITICAL: LogPayload exceeds total buffer capacity! Discarding.\n");
                continue;
            }
            // Normal behavior: Batch is full. Put payload back for the next cycle.
            if (xQueueSendToFront(queue, &item, 0) != pdTRUE) {
                LOG_ERROR("RocketLogger", "CRITICAL: Failed to return payload to front of queue!\n");
            }
            return false;
        }
        return true;
    };

    // Events first: they are rare and must reach storage even when sensor data
    // is backed up.
    SystemLog event;
    LogPayload payload;
    if (drainQueue(_eventQueue, event)) {
        drainQueue(_logQueue, payload);
    }

    return currentOffset;
}

size_t RocketLogger::peekNextEvent(uint8_t* outBuffer, size_t bufferCapacity) const {
    if (!_eventQueue || !_serializeFn || !outBuffer) return 0;
    SystemLog event;
    if (xQueuePeek(_eventQueue, &event, 0) != pdTRUE) return 0;
    size_t length = 0;
    return _serializeFn(LogPayload{event}, outBuffer, bufferCapacity, length) ? length : 0;
}

bool RocketLogger::discardNextEvent() {
    if (!_eventQueue) return false;
    SystemLog event;
    return xQueueReceive(_eventQueue, &event, 0) == pdTRUE;
}
