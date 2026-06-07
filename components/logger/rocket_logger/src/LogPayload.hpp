#pragma once
#include <variant>
#include <nlohmann/json.hpp>
#include "AccelerometerSensorData.hpp"
#include "GPSData.hpp"
#include "IMUData.hpp"
#include "PressureSensorData.hpp"

/**
 * @brief Lightweight struct for internal system logs (INFO, WARN, ERROR)
 */
struct SystemLog {
    char level[10] = {0};
    char source[32] = {0};
    char message[128] = {0};

    SystemLog() = default;

    SystemLog(const char* lvl, const char* src, const char* msg) {
        std::strncpy(level, lvl, sizeof(level) - 1); 
        level[sizeof(level) - 1] = '\0';
        
        std::strncpy(source, src, sizeof(source) - 1); 
        source[sizeof(source) - 1] = '\0';
        
        std::strncpy(message, msg, sizeof(message) - 1); 
        message[sizeof(message) - 1] = '\0';
    }

    // Writes up to maxLength. Returns the true required size (excluding null terminator).
    size_t serializeJson(char* buffer, size_t maxLength) const {
        return snprintf(buffer, maxLength, 
            "{\"type\":\"%s\",\"source\":\"%s\",\"content\":{\"message\":\"%s\"}}\n", 
            level, source, message);
    }
};

// The Variant: The compiler calculates the size of the largest object here
// and reserves exactly that much stack/queue memory.
using LogPayload = std::variant<SystemLog, AccelerometerSensorData, GPSData, IMUData, PressureSensorData>;