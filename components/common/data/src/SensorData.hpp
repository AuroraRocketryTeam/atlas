#pragma once

#include <string>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

/**
 * @brief Class to store sensor data.
 *
 */
class SensorData
{
protected:
    char sensorName[16] = {0};

public:
    SensorData() = default;

    /**
     * @brief Construct a new Sensor Data object.
     *
     * @param sensorName Name of the sensor.
     */
    SensorData(const char* name) {
        strncpy(sensorName, name, sizeof(sensorName) - 1);
        sensorName[sizeof(sensorName) - 1] = '\0';
    }

    /**
     * @brief Stames the data packet with the name of the source sensor.
     * * @param name Name of the sensor.
     */
    void setSensorName(const char* name) {
        strncpy(sensorName, name, sizeof(sensorName) - 1);
        sensorName[sizeof(sensorName) - 1] = '\0';
    }

    /**
     * @brief Get the Sensor Name object
     *
     * @return A string representing the name of the sensor.
     */
    std::string getSensorName() const {
        return std::string(sensorName);
    }

    /**
     * @brief Get the JSON representation of the object.
     *
     * @return A json object.
     */
    virtual size_t serializeJson(char* buffer, size_t maxLength) const = 0;
};


