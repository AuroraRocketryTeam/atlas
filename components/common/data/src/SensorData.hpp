#pragma once
#include <string.h>
#include <cstdio>

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
     * @return const char* Name of the sensor.
     */
    const char* getSensorName() const {
        return sensorName;
    }
};