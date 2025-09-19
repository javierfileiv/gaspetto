#pragma once

#include "Adafruit_Sensor.h"
#include "Wire.h"

#include <stdint.h>

// Constants for PC emulation
#define SENSORS_GRAVITY_STANDARD (9.80665f)

// MPU6050 Range constants
typedef enum {
    MPU6050_RANGE_2_G = 0,
    MPU6050_RANGE_4_G,
    MPU6050_RANGE_8_G,
    MPU6050_RANGE_16_G
} mpu6050_accel_range_t;

typedef enum {
    MPU6050_RANGE_250_DEG = 0,
    MPU6050_RANGE_500_DEG,
    MPU6050_RANGE_1000_DEG,
    MPU6050_RANGE_2000_DEG
} mpu6050_gyro_range_t;

typedef enum {
    MPU6050_BAND_260_HZ = 0,
    MPU6050_BAND_184_HZ,
    MPU6050_BAND_94_HZ,
    MPU6050_BAND_44_HZ,
    MPU6050_BAND_21_HZ,
    MPU6050_BAND_10_HZ,
    MPU6050_BAND_5_HZ
} mpu6050_bandwidth_t;

// Forward declarations
typedef struct {
    char name[12];
    int32_t version;
    int32_t sensor_id;
    int32_t type;
    float max_value;
    float min_value;
    float resolution;
    int32_t min_delay;
} sensor_t;

// Stub sensor class for PC emulation
class Adafruit_Sensor {
public:
    virtual ~Adafruit_Sensor() = default;
    virtual bool getEvent(sensors_event_t *)
    {
        return true;
    }
    virtual void getSensor(sensor_t *)
    {
    }
};

// Stub implementation of Adafruit_MPU6050 for PC emulation
class Adafruit_MPU6050 {
public:
    Adafruit_MPU6050() = default;
    ~Adafruit_MPU6050() = default;

    bool begin(uint8_t addr = 0x68, TwoWire *theWire = &Wire)
    {
        // Stub: always succeed
        return true;
    }

    void setAccelerometerRange(mpu6050_accel_range_t range)
    {
        // Stub: do nothing
        accel_range = range;
    }

    void setGyroRange(mpu6050_gyro_range_t range)
    {
        // Stub: do nothing
        gyro_range = range;
    }

    void setFilterBandwidth(mpu6050_bandwidth_t bandwidth)
    {
        // Stub: do nothing
        filter_bandwidth = bandwidth;
    }

    bool getEvent(sensors_event_t *accel, sensors_event_t *gyro, sensors_event_t *temp)
    {
        // Stub: return simulated data
        if (accel) {
            accel->acceleration.x = 0.1f;
            accel->acceleration.y = 0.1f;
            accel->acceleration.z = 9.8f;
        }
        if (gyro) {
            gyro->gyro.x = 0.01f;
            gyro->gyro.y = 0.01f;
            gyro->gyro.z = 0.01f;
        }
        if (temp) {
            temp->data[0] = 25.0f; // Temperature value
        }
        return true;
    }

private:
    mpu6050_accel_range_t accel_range = MPU6050_RANGE_2_G;
    mpu6050_gyro_range_t gyro_range = MPU6050_RANGE_250_DEG;
    mpu6050_bandwidth_t filter_bandwidth = MPU6050_BAND_260_HZ;
};
