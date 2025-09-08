#pragma once
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>

class IMUOrientation {
public:
    struct Offsets {
        float accX{ 0 }, accY{ 0 }, accZ{ 0 };
        float gyroX{ 0 }, gyroY{ 0 }, gyroZ{ 0 };
    };

    bool begin(uint8_t addr = 0x68, TwoWire *theWire = &Wire);
    void calibrate(bool print = true);
    void update();

    float roll() const
    {
        return rollDeg;
    }
    float pitch() const
    {
        return pitchDeg;
    }
    float yaw() const
    {
        return yawDeg;
    }
    float gyroZDeg() const
    {
        return lastGyroZDeg;
    }
    const Offsets &getOffsets() const
    {
        return offsets;
    }

    void zeroYaw()
    {
        yawDeg = 0.0f;
    }

    // Apply gentle yaw decay toward zero while stationary (call from loop when idle)
    void idleYawDampen(float dt);

    // Expose current estimated gyro Z bias (for diagnostics)
    float gyroZBiasValue() const
    {
        return gyroZBias;
    }

private:
    Adafruit_MPU6050 mpu;
    Offsets offsets;

    // Simple complementary filter parameters
    const float alpha = 0.96f; // gyro weight

    // State
    float rollDeg{ 0 }, pitchDeg{ 0 }, yawDeg{ 0 };
    unsigned long prevMicros{ 0 };
    float lastGyroZDeg{ 0 };
    float gyroZBias{ 0 };
    bool lastGyroSpike{ false };
};

// Configuration (override via -D flags)
#ifndef IMU_GYROZ_CLAMP_DEG_S
#define IMU_GYROZ_CLAMP_DEG_S 400.0f
#endif
#ifndef IMU_GYROZ_SPIKE_REJECT_DEG_S
#define IMU_GYROZ_SPIKE_REJECT_DEG_S 800.0f
#endif
#ifndef IMU_GYROZ_BIAS_ALPHA
#define IMU_GYROZ_BIAS_ALPHA 0.0015f
#endif
#ifndef IMU_IDLE_YAW_DAMP_RATE
#define IMU_IDLE_YAW_DAMP_RATE 0.25f /* per second fraction toward zero when idle */
#endif
