#ifndef PID_v1_h
#define PID_v1_h

class PID {
public:
#define AUTOMATIC 1
#define MANUAL 0
#define DIRECT 0
#define REVERSE 1
#define P_ON_M 0
#define P_ON_E 1

    PID(double *, double *, double *, double, double, double, int, int);

    PID(double *, double *, double *, double, double, double, int);

    void SetMode(int Mode); // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();

    void SetOutputLimits(double, double);

    void SetTunings(double, double, double);

    void SetTunings(double, double, double, int);

    void SetControllerDirection(int);

    void SetSampleTime(int);

    double GetKp();
    double GetKi();
    double GetKd();
    int GetMode();
    int GetDirection(); //
};
#endif
