#include "PID_v1.h"

PID::PID(double *, double *, double *, double, double, double, int, int)
{
}
PID::PID(double *, double *, double *, double, double, double, int)
{
}
void PID::SetMode(int)
{
}
bool PID::Compute()
{
    return true;
}
void PID::SetOutputLimits(double, double)
{
}
void PID::SetTunings(double, double, double)
{
}
void PID::SetTunings(double, double, double, int)
{
}
void PID::SetControllerDirection(int)
{
}
void PID::SetSampleTime(int)
{
}
double PID::GetKp()
{
    return 0.0;
}
double PID::GetKi()
{
    return 0.0;
}
double PID::GetKd()
{
    return 0.0;
}
int PID::GetMode()
{
    return 0;
}
int PID::GetDirection()
{
    return 0;
}
