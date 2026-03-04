#pragma once

#include <string>
#include <map>
#include <variant>
#include <optional>
#include <nlohmann/json.hpp>
#include "ILoggable.hpp"

/**
 * @brief Class to store sensor data.
 *
 */
class SensorData : public ILoggable
{
protected:
    // Name of the sensor
    std::string sensorName;

public:
    /**
     * @brief Construct a new Sensor Data object.
     *
     * @param sensorName Name of the sensor.
     */
    SensorData(std::string sensorName) : sensorName(sensorName) {}

    /**
     * @brief Get the Sensor Name object
     *
     * @return A string representing the name of the sensor.
     */
    std::string getSensorName() const
    {
        return sensorName;
    }

    /**
     * @brief Get the JSON representation of the object.
     *
     * @return A json object.
     */
    virtual json toJSON() const override;
};


