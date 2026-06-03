#pragma once

#include <string>
#include <nlohmann/json.hpp>
#include <SensorData.hpp>

using json = nlohmann::json;

/**
 * @brief A class to represent a log message.
 * 
 */
class LogMessage {
private:
    // Message to be logged
    char source[16];
    char message[128];

public:
    /**
     * @brief Construct a new Log Message object.
     * 
     * @param source The origin of the log.
     * @param message The message to be logged.
     */
    LogMessage(const char* src, const char* msg) {
        strncpy(source, src, sizeof(source) - 1);
        source[sizeof(source) - 1] = '\0';
        
        strncpy(message, msg, sizeof(message) - 1);
        message[sizeof(message) - 1] = '\0';
    }

    /**
     * @brief Get the message string.
     * 
     * @return A string representing the message.
     */
    std::string getMessage() const {
        return message;
    }

    /**
     * @brief Get the JSON representation of the object.
     * 
     * @return A json object.
     */
json toJSON() const {
        json j;
        j["source"] = source;
        j["message"] = message;
        return j;
    }
};


