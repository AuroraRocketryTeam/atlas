#pragma once

#include <SensorData.hpp>
#include <optional>

/**
 * @brief Interface for sensors.
 * 
 */
class ISensor
{
public:
    /**
     * @brief Construct a new ISensor object with a specific identity.
     * * @param name Name of the sensor.
     */
    ISensor(const char* name) {
        strncpy(sensorName, name, sizeof(sensorName) - 1);
        sensorName[sizeof(sensorName) - 1] = '\0';
    }

    virtual ~ISensor() = default;

    /**
     * @brief Initialize the sensor.
     *
     * @return true if the sensor was initialized successfully
     * @return false if the sensor failed to initialize
     */
    virtual bool init() = 0;

    /**
     * @brief Update the sensor's class fields with reading values 
     *
     * @return true if the reading was successful, false otherwise.
     */
    virtual bool updateData() = 0;

    /**
     * @brief Check if the sensor is initialized.
     *
     * @return true if the sensor is initialized
     * @return false if the sensor is not initialized
     */
    bool isInitialized() const
    {
        return initialized;
    }

    /**
     * @brief Get the identity of the sensor.
     * @return const char* */
    const char* getSensorName() const 
    {
        return sensorName;
    }

protected:
    void setInitialized(bool initialized)
    {
        this->initialized = initialized;
    }
    
    char sensorName[16] = {0};

private:
    bool initialized = false;
};

