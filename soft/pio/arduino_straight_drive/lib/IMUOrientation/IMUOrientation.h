#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

class IMUOrientation {
public:
    struct Offsets {
        float accX{0}, accY{0}, accZ{0};
        float gyroX{0}, gyroY{0}, gyroZ{0};
    };

    bool begin(uint8_t addr = 0x68, TwoWire *theWire = &Wire);
    void calibrate(bool print = true);
    void update();

    float roll()  const { return rollDeg; }
    float pitch() const { return pitchDeg; }
    float yaw()   const { return yawDeg; }
    float gyroZDeg() const { return lastGyroZDeg; }
    const Offsets &getOffsets() const { return offsets; }

    void zeroYaw() { yawDeg = 0.0f; }

private:
    Adafruit_MPU6050 mpu;
    Offsets offsets;

    // Simple complementary filter parameters
    const float alpha = 0.96f; // gyro weight

    // State
    float rollDeg{0}, pitchDeg{0}, yawDeg{0};
    unsigned long prevMicros{0};
    float lastGyroZDeg{0};
};
