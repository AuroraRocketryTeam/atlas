#include "RocketLogger.hpp"

// Destructor - clean up all dynamically allocated memory
RocketLogger::~RocketLogger() {
    clearData();
}

// Override logInfo to log informational messages
void RocketLogger::logInfo(const std::string& message) {
    ILoggable* logMessage = new LogMessage("RocketLogger", message);
    this->logDataList.push_back(LogData("INFO", logMessage));
}

// Override logWarning to log warning messages
void RocketLogger::logWarning(const std::string& message) {
    ILoggable* logMessage = new LogMessage("RocketLogger", message);
    this->logDataList.push_back(LogData("WARNING", logMessage));
}

// Override logError to log error messages
void RocketLogger::logError(const std::string& message) {
    ILoggable* logMessage = new LogMessage("RocketLogger", message);
    this->logDataList.push_back(LogData("ERROR", logMessage));
}

// Override logData to store sensor data
void RocketLogger::logSensorData(const SensorData sensorData) {
    ILoggable* logSensorData = new LogSensorData(sensorData.getSensorName(), sensorData);    
    this->logDataList.push_back(LogData("SENSOR_DATA", logSensorData));
}

void RocketLogger::logSensorData(const std::string& sensorName, const SensorData sensorData) {
    ILoggable* logSensorData = new LogSensorData(sensorName, sensorData);    
    this->logDataList.push_back(LogData("SENSOR_DATA", logSensorData));
}

// Function to get all logged sensor data as a JSON list
json RocketLogger::getJSONAll() const {
    json jsonDataList = json::array();
    for (const auto& sensorData : this->logDataList) {
        jsonDataList.push_back(sensorData.toJSON());  // Convert each sensor data to JSON
    }
    return jsonDataList;
}

// Clear logged sensor data
void RocketLogger::clearData() {
    size_t initialCount = this->getLogCount();
    
    // Print deleting n elements
    LOG_INFO("RocketLogger", "Clearing %zu log entries...", initialCount);

    // Delete all dynamically allocated LogSensorData objects
    for (auto& logData : this->logDataList) {
        delete logData.getData();
    }

    this->logDataList.clear();
    
    size_t finalCount = this->getLogCount();
    LOG_INFO("RocketLogger", "Clear complete. Before: %zu, After: %zu entries", initialCount, finalCount);
}