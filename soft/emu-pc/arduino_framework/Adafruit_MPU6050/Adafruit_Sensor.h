#pragma once

#include <stdint.h>

#pragma once

#include <stdint.h>

// Sensor event types for PC emulation
typedef struct {
    float x;
    float y;
    float z;
} sensors_vec_t;

typedef struct {
    int32_t version;
    int32_t sensor_id;
    int32_t type;
    int32_t reserved0;
    int32_t timestamp;
    union {
        float data[4];
        sensors_vec_t acceleration;
        sensors_vec_t gyro;
        sensors_vec_t temperature;
    };
} sensors_event_t;
