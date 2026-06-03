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
    std::string level;
    std::string source;
    std::string message;

    nlohmann::json toJSON() const {
        return {
            {"type", level},
            {"source", source},
            {"content", {{"message", message}}}
        };
    }
};

// The Variant: The compiler calculates the size of the largest object here
// and reserves exactly that much stack/queue memory.
using LogPayload = std::variant<SystemLog, AccelerometerSensorData, GPSData, IMUData, PressureSensorData>;