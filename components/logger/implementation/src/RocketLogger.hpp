#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <nlohmann/json.hpp>
#include <ILogger.hpp>
#include <LogMessage.hpp>
#include <SensorData.hpp>
#include "Logger.hpp"

/**
 * @brief Class to log messages and sensor data.
 * 
 */
class RocketLogger : public ILogger {
public:
    RocketLogger();

    /**
     * @brief Destructor to clean up dynamically allocated memory.
     */
    ~RocketLogger();

    /**
     * @brief Thread-safe count of buffered log entries.
     */
    int getLogCount() const;

    /**
     * @brief Log an informational message.
     * 
     * @param message The informational message to log.
     */
    void logInfo(const std::string& message) override;

    /**
     * @brief Log a warning message.
     * 
     * @param message The warning message to log.
     */
    void logWarning(const std::string& message) override;

    /**
     * @brief Log an error message.
     * 
     * @param message The error message to log.
     */
    void logError(const std::string& message) override;

    /**
     * @brief Log sensor data.
     * 
     * @param sensorData The sensor data to log.
     */
    void logSensorData(std::shared_ptr<SensorData> sensorData) override;

    /**
     * @brief Get all logged sensor data as a JSON list.
     * 
     * @return A json object.
     */
    json getJSONAll() const override;

    /**
     * @brief Clear all logged sensor data.
     * 
     */
    void clearData() override;

    /**
     * @brief Serialize all buffered logs into a malloc-allocated JSON C string and clear the buffer.
     *
     * @param outJson Output pointer. Caller must free() on success.
     * @param maxEntries Maximum number of log entries to consume in this call.
     * @return true on success, false if no data or allocation/lock failure.
     */
    bool consumeAllAsJsonChar(char** outJson, size_t maxEntries);

private:
    mutable SemaphoreHandle_t _mutex;
};
