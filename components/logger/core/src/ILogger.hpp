// LoggerInterface.h
#pragma once

#include <string>
#include <nlohmann/json.hpp>
#include <SensorData.hpp>
#include <LogPayload.hpp>

/**
 * @brief Interface for loggers.
 * 
 */
class ILogger {
public:
    /**
     * @brief Get the number of logged entries.
     * 
     * @return int The number of logged entries.
     */
    virtual int getLogCount() const = 0;

    /**
     * @brief Log an informational message.
     * 
     * @param message The informational message to log.
     */
    virtual void logInfo(const std::string& message) = 0;

    /**
     * @brief Log a warning message.
     * 
     * @param message The warning message to log.
     */
    virtual void logWarning(const std::string& message) = 0;

    /**
     * @brief Log an error message.
     * 
     * @param message The error message to log.
     */
    virtual void logError(const std::string& message) = 0;

    /**
     * @brief Log sensor data.
     * 
     * @param payload The data to log, wrapped in a LogPayload variant.
     */
    virtual void logSensorData(const LogPayload& payload) = 0;

    /**
     * @brief Get all logged sensor data as a JSON list.
     * 
     * @return A json object.
     */
    virtual json getJSONAll() = 0;

    /**
     * @brief Clear all logged sensor data.
     * 
     */
    virtual void clearData() = 0;

    /**
     * @brief Virtual destructor to ensure proper cleanup in derived classes.
     */
    virtual ~ILogger() = default;
};


