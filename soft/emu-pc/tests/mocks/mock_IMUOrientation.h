#pragma once

#include "mock_base.h"
#ifndef MOCK_IMU_ORIENTATION_H
#define MOCK_IMU_ORIENTATION_H

#include "IMUOrientationInterface.h"

#include <gmock/gmock.h>

class MockIMUOrientation : public MockBase<MockIMUOrientation>, public IMUOrientationInterface {
public:
    MockIMUOrientation();
    ~MockIMUOrientation();

    MOCK_METHOD(bool, _begin, (uint8_t addr, TwoWire *theWire));
    MOCK_METHOD(void, _calibrate, (bool print));
    MOCK_METHOD(void, _update, ());
    MOCK_METHOD(float, _roll, (), (const));
    MOCK_METHOD(float, _pitch, (), (const));
    MOCK_METHOD(float, _yaw, (), (const));
    MOCK_METHOD(float, _gyroZDeg, (), (const));

    virtual bool begin(uint8_t addr = 0x68, TwoWire *theWire = nullptr) override;
    virtual void calibrate(bool print = true) override;
    virtual void update() override;
    virtual float roll() const override;
    virtual float pitch() const override;
    virtual float yaw() const override;
    virtual float gyroZDeg() const override;

private:
    static MockIMUOrientation *instance_;
};

#endif // MOCK_IMU_ORIENTATION_H
