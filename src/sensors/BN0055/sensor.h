// sensor.h
#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int32_t temperature;
    float euler_heading;
    float euler_roll;
    float euler_pitch;
} bno_sensor_data_t;

typedef struct Sensor {
    bool (*init)(struct Sensor* self);
    bool (*read_data)(struct Sensor* self, bno_sensor_data_t* data);
} Sensor;

#endif // SENSOR_H
