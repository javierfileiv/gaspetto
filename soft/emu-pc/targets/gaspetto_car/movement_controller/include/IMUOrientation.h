#pragma once
#include "IMUOrientationInterface.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <Wire.h>

class IMUOrientation : public IMUOrientationInterface {
public:
    bool begin(uint8_t addr = 0x68, TwoWire *theWire = &Wire) override;
    void calibrate(bool print = true) override;
    void update() override;

    float roll() const override
    {
        return rollDeg;
    }
    float pitch() const override
    {
        return pitchDeg;
    }
    float yaw() const override
    {
        return yawDeg;
    }
    float gyroZDeg() const override
    {
        return lastGyroZDeg;
    }
    const Offsets &getOffsets() const override
    {
        return offsets;
    }

    void zeroYaw() override
    {
        yawDeg = 0.0f;
    }

    // Apply gentle yaw decay toward zero while stationary (call from loop when idle)
    void idleYawDampen(float dt) override;

    // Expose current estimated gyro Z bias (for diagnostics)
    float gyroZBiasValue() const override
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
