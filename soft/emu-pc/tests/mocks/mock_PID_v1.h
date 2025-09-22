
#pragma once
#include "mock_base.h"

#include <gmock/gmock.h>

// Stub PID class for mocking
class PID {
public:
    PID(double *, double *, double *, double, double, double, int, int)
    {
    }
    PID(double *, double *, double *, double, double, double, int)
    {
    }
    virtual ~PID() = default;
    virtual void SetMode(int)
    {
    }
    virtual bool Compute()
    {
        return true;
    }
    virtual void SetOutputLimits(double, double)
    {
    }
    virtual void SetTunings(double, double, double)
    {
    }
    virtual void SetTunings(double, double, double, int)
    {
    }
    virtual void SetControllerDirection(int)
    {
    }
    virtual void SetSampleTime(int)
    {
    }
    virtual double GetKp()
    {
        return 0.0;
    }
    virtual double GetKi()
    {
        return 0.0;
    }
    virtual double GetKd()
    {
        return 0.0;
    }
    virtual int GetMode()
    {
        return 0;
    }
    virtual int GetDirection()
    {
        return 0;
    }
};

class MockPID : public MockBase<MockPID>, public PID {
public:
    MockPID(double *a, double *b, double *c, double d, double e, double f, int g, int h)
            : PID(a, b, c, d, e, f, g, h)
    {
        set_instance(this);
    }
    MockPID(double *a, double *b, double *c, double d, double e, double f, int g)
            : PID(a, b, c, d, e, f, g)
    {
        set_instance(this);
    }
    ~MockPID() override
    {
        clear_instance(this);
    }

    MOCK_METHOD(void, SetMode, (int), (override));
    MOCK_METHOD(bool, Compute, (), (override));
    MOCK_METHOD(void, SetOutputLimits, (double, double), (override));
    MOCK_METHOD(void, SetTunings, (double, double, double), (override));
    MOCK_METHOD(void, SetTunings, (double, double, double, int), (override));
    MOCK_METHOD(void, SetControllerDirection, (int), (override));
    MOCK_METHOD(void, SetSampleTime, (int), (override));
    MOCK_METHOD(double, GetKp, (), (override));
    MOCK_METHOD(double, GetKi, (), (override));
    MOCK_METHOD(double, GetKd, (), (override));
    MOCK_METHOD(int, GetMode, (), (override));
    MOCK_METHOD(int, GetDirection, (), (override));
};
