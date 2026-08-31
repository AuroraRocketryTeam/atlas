#pragma once

#include <atomic>
#include <iostream>
#include <string>
#include <vector>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <nlohmann/json.hpp>
#include <SensorData.hpp>
#include "SerialLogger.hpp"
#include "LogPayload.hpp"

// Define the signature that any serializer function must match
using SerializeFunction = bool (*)(const LogPayload& payload, uint8_t* buffer, size_t capacity, size_t& offset);

/**
 * @brief Class to log messages and sensor data.
 * 
 */
class RocketLogger {
public:
    RocketLogger();

    /**
     * @brief Destructor to clean up dynamically allocated memory.
     */
    ~RocketLogger();

    /**
     * @brief Change the serialization format at runtime.
     */
    void setSerializer(SerializeFunction serializeFn);
    
    /**
     * @brief Thread-safe count of buffered log entries.
     */
    int getLogCount() const;

    /** 
     * @brief Total records overwritten or rejected because the queue was full.
     */
    uint32_t getDroppedCount() const;

    /**
     * @brief Log an informational message.
     * 
     * @param message The informational message to log.
     */
    void logInfo(const std::string& message);
    void logInfo(const char* source, const char* message);

    /**
     * @brief Log a warning message.
     * 
     * @param message The warning message to log.
     */
    void logWarning(const std::string& message);

    /**
     * @brief Log an error message.
     * 
     * @param message The error message to log.
     */
    void logError(const std::string& message);

    /**
     * @brief Log sensor data.
     * 
     * @param payload The data to log, wrapped in a LogPayload variant.
     */
    void logSensorData(const LogPayload& payload);

    /**
     * @brief Clear all logged sensor data.
     * 
     */
    void clearData();

/**
     * @brief Consumes logs from the queue until the provided byte buffer is full.
     *
     * @param outBuffer Pointer to the static/pre-allocated array.
     * @param bufferCapacity Maximum bytes the buffer can hold.
     * @return size_t The total number of bytes written to outBuffer.
     */
    size_t consumeBatch(uint8_t* outBuffer, size_t bufferCapacity);
    
    private:
    QueueHandle_t _logQueue;
    std::atomic_uint32_t _dropped{0};

    // Store the function pointer instead of an object pointer
    SerializeFunction _serializeFn = nullptr;
    
    // Internal helper to push to queue
    void pushToQueue(const LogPayload& payload);
};
