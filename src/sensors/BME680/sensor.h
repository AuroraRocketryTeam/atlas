/**
 * @file sensor.h
 * @author luca.pulga@studio.unibo.it
 * @brief General sensor interface.
 * @version 0.1
 * @date 2024-10-02
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int32_t temperature;
    uint32_t pressure;
    uint32_t humidity;
    uint32_t gas_resistance;
} bme_sensor_data_t;

typedef struct Sensor {
    bool (*init)(struct Sensor* self);
    bool (*read_data)(struct Sensor* self, bme_sensor_data_t* data);
} Sensor;

#endif // SENSOR_H
