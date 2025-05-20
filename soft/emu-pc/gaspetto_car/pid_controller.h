// pid_controller.h
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double minOutput, double maxOutput);
    ~PIDController()
    {
    }

    double compute(double setpoint, double feedback);
    void reset();
    void setGains(double kp, double ki, double kd);
    void setOutputLimits(double minOutput, double maxOutput);

private:
    double kp_;
    double ki_;
    double kd_;
    double integral_;
    double previousError_;
    double minOutput_;
    double maxOutput_;
};

#endif // PID_CONTROLLER_H
