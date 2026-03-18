#pragma once

#include <Arduino.h>
/* Minimal PID stub replicating interface used (always computes immediately). */
class PID {
public:
    PID(double *in, double *out, double *sp, double Kp, double Ki, double Kd, int)
    {
        _in = in;
        _out = out;
        _sp = sp;
        _Kp = Kp;
        _Ki = Ki;
        _Kd = Kd;
    }
    void SetSampleTime(int)
    {
    }
    void SetOutputLimits(double, double)
    {
    }
    void SetTunings(double Kp, double Ki, double Kd)
    {
        _Kp = Kp;
        _Ki = Ki;
        _Kd = Kd;
    }
    void SetMode(int)
    {
    }
    bool Compute()
    {
        *_out = (*_sp - *_in) * _Kp;
        return true;
    }

private:
    double *_in, *_out, *_sp;
    double _Kp, _Ki, _Kd;
};
