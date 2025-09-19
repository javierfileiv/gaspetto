#ifndef IMU_ORIENTATION_INTERFACE_H
#define IMU_ORIENTATION_INTERFACE_H

#include <stdint.h>

#ifndef ARDUINO
class TwoWire; // Forward declaration for testing
#else
#include <Wire.h>
#endif

class IMUOrientationInterface {
public:
    struct Offsets {
        float accX{ 0 }, accY{ 0 }, accZ{ 0 };
        float gyroX{ 0 }, gyroY{ 0 }, gyroZ{ 0 };
    };

    virtual ~IMUOrientationInterface() = default;

    virtual bool begin(uint8_t addr = 0x68, TwoWire *theWire = nullptr) = 0;
    virtual void calibrate(bool print = true) = 0;
    virtual void update() = 0;

    virtual float roll() const = 0;
    virtual float pitch() const = 0;
    virtual float yaw() const = 0;
    virtual float gyroZDeg() const = 0;
    virtual const Offsets &getOffsets() const = 0;

    virtual void zeroYaw() = 0;
    virtual void idleYawDampen(float dt) = 0;
    virtual float gyroZBiasValue() const = 0;
};

#endif // IMU_ORIENTATION_INTERFACE_H
