#include "RocketLogger.hpp"
#include <new>
#include <cstdlib>
#include <cstring>
// TODO: Find alternative calls for heap
#include <Arduino.h>

RocketLogger::RocketLogger() : _mutex(xSemaphoreCreateMutex()) {
}

// Destructor - clean up all dynamically allocated memory
RocketLogger::~RocketLogger() {
    clearData();
    if (_mutex) {
        vSemaphoreDelete(_mutex);
        _mutex = nullptr;
    }
}

int RocketLogger::getLogCount() const {
    if (_mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        const int count = static_cast<int>(this->logDataList.size());
        xSemaphoreGive(_mutex);
        return count;
    }
    return static_cast<int>(this->logDataList.size());
}

// Override logInfo to log informational messages
void RocketLogger::logInfo(const std::string& message) {
    // Check available memory before allocating
    uint32_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < 5000) {
        return;
    }

    std::shared_ptr<ILoggable> logMessage = std::make_shared<LogMessage>("RocketLogger", message);
    if (logMessage && _mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        this->logDataList.push_back(LogData("INFO", logMessage));
        xSemaphoreGive(_mutex);
    }
}

// Override logWarning to log warning messages
void RocketLogger::logWarning(const std::string& message) {
    // Check available memory before allocating
    uint32_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < 5000) { 
        return;
    }

    std::shared_ptr<ILoggable> logMessage = std::make_shared<LogMessage>("RocketLogger", message);
    if (logMessage && _mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        this->logDataList.push_back(LogData("WARNING", logMessage));
        xSemaphoreGive(_mutex);
    }
}

// Override logError to log error messages
void RocketLogger::logError(const std::string& message) {
    // Check available memory before allocating
    uint32_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < 5000) { 
        return;
    }

    std::shared_ptr<ILoggable> logMessage = std::make_shared<LogMessage>("RocketLogger", message);
    if (logMessage && _mutex && xSemaphoreTake(_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
        this->logDataList.push_back(LogData("ERROR", logMessage));
        xSemaphoreGive(_mutex);
    }
}

// Override logData to store sensor data
void RocketLogger::logSensorData(std::shared_ptr<SensorData> sensorData) {
    if (!_mutex || xSemaphoreTake(_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return;
    }

    // Prevent memory exhaustion by limiting log entries
    // note: this should never happen, but just in case we have a log of it
    const int MAX_LOG_ENTRIES = 500;
    const int currentEntries = static_cast<int>(logDataList.size());

    if (currentEntries >= MAX_LOG_ENTRIES) {
        LOG_WARNING("RocketLogger", "Log buffer full (%d entries), clearing oldest entries", currentEntries);
        
        // Clear half the entries to avoid frequent clears
        int entriesToRemove = MAX_LOG_ENTRIES / 2;
        for (int i = 0; i < entriesToRemove && !logDataList.empty(); i++) {
            // MAKE SURE THE NEW DATA SYSTEM WITH SHARED POINTERS HANDLES MEMORY MANAGEMENT!!!
            // delete logDataList[i].getData();
            logDataList.erase(logDataList.begin());
        }
        LOG_INFO("RocketLogger", "Cleared %d entries, now have %d entries", entriesToRemove, static_cast<int>(logDataList.size()));
    }

    this->logDataList.push_back(LogData("SENSOR_DATA", sensorData));
    xSemaphoreGive(_mutex);
}

// Function to get all logged sensor data as a JSON list
json RocketLogger::getJSONAll() const {
    if (!_mutex || xSemaphoreTake(_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return json::array();
    }

    json jsonDataList = json::array();
    for (const auto& sensorData : this->logDataList) {
        jsonDataList.push_back(sensorData.toJSON());  // Convert each sensor data to JSON
    }
    xSemaphoreGive(_mutex);
    return jsonDataList;
}

// Clear logged sensor data
void RocketLogger::clearData() {
    if (!_mutex || xSemaphoreTake(_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return;
    }

    int initialCount = static_cast<int>(this->logDataList.size());
    
    // Print deleting n elements
    LOG_INFO("RocketLogger", "Clearing %d log entries...", initialCount);

    // MAKE SURE THE NEW DATA SYSTEM WITH SHARED POINTERS HANDLES MEMORY MANAGEMENT!!!
    // Delete all dynamically allocated SensorData objects
    //for (auto& logData : this->logDataList) {
    //    delete logData.getData();
    //}

    this->logDataList.clear();

    int finalCount = static_cast<int>(this->logDataList.size());
    xSemaphoreGive(_mutex);
    LOG_INFO("RocketLogger", "Clear complete. Before: %d, After: %d entries", initialCount, finalCount);
}

bool RocketLogger::consumeAllAsJsonChar(char** outJson, size_t maxEntries) {
    if (outJson == nullptr) {
        return false;
    }
    *outJson = nullptr;

    if (!_mutex || xSemaphoreTake(_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
        return false;
    }

    if (this->logDataList.empty()) {
        xSemaphoreGive(_mutex);
        return false;
    }

    size_t entriesToConsume = std::min(maxEntries, this->logDataList.size());
    if (entriesToConsume == 0) entriesToConsume = this->logDataList.size(); // Fallback if maxEntries=0

    std::string jsonStr;
    try {   
        // Pre-allocate memory to reduce ESP32 heap fragmentation. 
        // Assuming roughly 703 bytes as longest JSON found empirically, 
        // we round up to 1024 bytes per entry for safety.
        jsonStr.reserve(entriesToConsume * 1024);
        
        for (size_t i = 0; i < entriesToConsume; ++i) {
            jsonStr += this->logDataList[i].toJSON().dump();
            jsonStr += "\n";
        }
    } catch (const std::exception&) {
        xSemaphoreGive(_mutex);
        return false;
    }

    char* jsonBuffer = static_cast<char*>(malloc(jsonStr.size() + 1));
    if (jsonBuffer == nullptr) {
        xSemaphoreGive(_mutex);
        return false;
    }

    std::memcpy(jsonBuffer, jsonStr.c_str(), jsonStr.size() + 1);
    
    // Remove only the items we consumed
    this->logDataList.erase(this->logDataList.begin(), this->logDataList.begin() + entriesToConsume);

    xSemaphoreGive(_mutex);
    *outJson = jsonBuffer;
    return true;
}