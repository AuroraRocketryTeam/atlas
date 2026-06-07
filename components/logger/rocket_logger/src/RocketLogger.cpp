#include "RocketLogger.hpp"
#include <new>
#include <cstdlib>
#include <cstring>
// TODO: Find alternative calls for heap
#include <Arduino.h>

RocketLogger::RocketLogger() {
    _logQueue = xQueueCreate(MAX_QUEUE_LENGTH, sizeof(LogPayload));
    if (_logQueue == nullptr) {
        LOG_ERROR("RocketLogger", "Failed to create log queue! Insufficient heap.");
    }
}

RocketLogger::~RocketLogger() {
    if (_logQueue != nullptr) {
        vQueueDelete(_logQueue);
        _logQueue = nullptr;
    }
}

void RocketLogger::setSerializer(SerializeFunction serializeFn) {
    _serializeFn = serializeFn;
}

int RocketLogger::getLogCount() const {
    if (_logQueue == nullptr) return 0;
    return uxQueueMessagesWaiting(_logQueue); 
}

void RocketLogger::pushToQueue(const LogPayload& payload) {
    if (_logQueue == nullptr) return;

    // Timeout is 0 (pdMS_TO_TICKS(0)) to ensure the caller NEVER blocks.
    // If the queue is full, we drop the oldest item to make room.
    if (xQueueSend(_logQueue, &payload, 0) == errQUEUE_FULL) {
        LogPayload dummy;
        xQueueReceive(_logQueue, &dummy, 0); // Discard oldest
        xQueueSend(_logQueue, &payload, 0);  // Insert newest
    }
}

void RocketLogger::logInfo(const std::string& message) {
    pushToQueue(SystemLog{"INFO", "RocketLogger", message.c_str()});
}

void RocketLogger::logWarning(const std::string& message) {
    pushToQueue(SystemLog{"WARNING", "RocketLogger", message.c_str()});
}

void RocketLogger::logError(const std::string& message) {
    pushToQueue(SystemLog{"ERROR", "RocketLogger", message.c_str()});
}

void RocketLogger::logSensorData(const LogPayload& payload) {
    pushToQueue(payload);
}

void RocketLogger::clearData() {
    if (_logQueue != nullptr) {
        xQueueReset(_logQueue); 
    }
}

size_t RocketLogger::consumeBatch(uint8_t* outBuffer, size_t bufferCapacity) {
    // Check each condition
    LOG_DEBUG("RocketLogger", "consumeBatch called. Queue length: %d, SerializeFunction set: %s, outBuffer valid: %s\n",
        getLogCount(), _serializeFn ? "true" : "false", outBuffer ? "true" : "false");
    if (_logQueue == nullptr) {
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
    LogPayload payload;

    LOG_DEBUG("RocketLogger", "Starting batch consumption. Initial queue length: %d\n", getLogCount());
    while (uxQueueMessagesWaiting(_logQueue) > 0) {
        if (xQueueReceive(_logQueue, &payload, 0) == pdTRUE) {
            LOG_DEBUG("RocketLogger", "Attempting to serialize payload. Current batch size: %zu bytes\n", currentOffset);
            // serializeFn dynamically evaluates size and writes if it fits
            bool success = _serializeFn(payload, outBuffer, bufferCapacity, currentOffset);

            if (!success) {
                // It didn't fit in the remaining space
                if (currentOffset == 0) {
                    // Deadlock prevention: It's larger than the entire 4096-byte buffer!
                    LOG_ERROR("RocketLogger", "CRITICAL: LogPayload exceeds total buffer capacity! Discarding.\n");
                    continue; 
                } else {
                    // Normal behavior: Batch is full. Put payload back for the next cycle.
                    if (xQueueSendToFront(_logQueue, &payload, 0) != pdTRUE) {
                        LOG_ERROR("RocketLogger", "CRITICAL: Failed to return payload to front of queue!\n");
                    }
                    break;
                }
            }
        }
    }

    LOG_DEBUG("RocketLogger", "Finished consuming batch. Total batch size: %zu bytes\n", currentOffset);

    return currentOffset;
}