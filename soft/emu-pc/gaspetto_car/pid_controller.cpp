// pid_controller.cpp
#include "pid_controller.h"

PIDController::PIDController(double kp, double ki, double kd, double minOutput, double maxOutput)
        : kp_(kp)
        , ki_(ki)
        , kd_(kd)
        , integral_(0.0)
        , previousError_(0.0)
        , minOutput_(minOutput)
        , maxOutput_(maxOutput)
{
}

double PIDController::compute(double setpoint, double feedback)
{
    double error = setpoint - feedback;
    integral_ += error;
    double derivative = error - previousError_;
    double output = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);

    // Clamp the output to the defined limits
    if (output > maxOutput_)
        output = maxOutput_;
    else if (output < minOutput_)
        output = minOutput_;

    previousError_ = error;
    return output;
}

void PIDController::reset()
{
    integral_ = 0.0;
    previousError_ = 0.0;
}

void PIDController::setGains(double kp, double ki, double kd)
{
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setOutputLimits(double minOutput, double maxOutput)
{
    minOutput_ = minOutput;
    maxOutput_ = maxOutput;
}

// using

#ifndef MY_ACTIVE_OBJECT_H
#define MY_ACTIVE_OBJECT_H

#include "active_object.h"
#include "pid_controller.h" // Include PID controller
#include "state.h"

class MyActiveObject : public ActiveObject {
public:
    MyActiveObject();
    ~MyActiveObject() override;

protected:
    void processEvent(const Event &event) override;

private:
    State *currentState_;

    // ... (motor pin definitions) ...
    // ... (calibration constants for target pulses) ...

    // PID Controllers for each motor
    PIDController pid_motor1_;
    PIDController pid_motor2_;

    long target_pulses_motor1_;
    long target_pulses_motor2_;
    long initial_motor1_pulses_;
    long initial_motor2_pulses_;
    // ... (current_move_direction variables) ...

    // Helper functions
    void setMotorSpeeds(int motor1_speed, int motor2_speed); // Now the output of PID
    void setMotorDirections(int motor1_dir, int motor2_dir);
    bool isTargetReached(); // Still used for final stop condition
    void stopMotors();
    void changeState(State *newState);
    void setTargetPulses(long target1, long target2);
    void setInitialPulseCounts();
    void setCurrentMoveDirection(int dir1, int dir2);
    void resetPID();
    // ...
};

#include "idle_state.h"
#include "my_active_object.h"
#include "pid_controller.h" // Include PID implementation

// External pulse counters
extern volatile long motor1_pulse_count;
extern volatile long motor2_pulse_count;

MyActiveObject::MyActiveObject()
        : ActiveObject(10)
        , currentState_(new IdleState())
        , target_pulses_motor1_(0)
        , target_pulses_motor2_(0)
        , initial_motor1_pulses_(0)
        , initial_motor2_pulses_(0)
        , current_move_direction_motor1_(0)
        , current_move_direction_motor2_(0)
        , pid_motor1_(0.1, 0.01, 0.05, 0, 255)
        , // Example gains and output limits
        pid_motor2_(0.1, 0.01, 0.05, 0, 255)
{
    // ... (pinModeEmulation calls) ...
    stopMotors();
}

// ... (destructor and changeState) ...

void MyActiveObject::resetPID()
{
    pid_motor1_.reset();
    pid_motor2_.reset();
}

void MyActiveObject::processEvent(const Event &event)
{
    if (currentState_) {
        currentState_->processEvent(this, event);
    }
}

void MyActiveObject::setTargetPulses(long target1, long target2)
{
    target_pulses_motor1_ = target1;
    target_pulses_motor2_ = target2;
}

void MyActiveObject::setInitialPulseCounts()
{
    initial_motor1_pulses_ = motor1_pulse_count;
    initial_motor2_pulses_ = motor2_pulse_count;
}

void MyActiveObject::setCurrentMoveDirection(int dir1, int dir2)
{
    current_move_direction_motor1_ = dir1;
    current_move_direction_motor2_ = dir2;
}

bool MyActiveObject::isTargetReached()
{
    long current_pulses_motor1 = abs(motor1_pulse_count - initial_motor1_pulses_);
    long current_pulses_motor2 = abs(motor2_pulse_count - initial_motor1_pulses_);

    return current_pulses_motor1 >= target_pulses_motor1_ &&
           current_pulses_motor2 >= target_pulses_motor2_;
}

void MyActiveObject::setMotorSpeeds(int motor1_speed, int motor2_speed)
{
    analogWriteEmulation(motor1_in2, motor1_speed);
    analogWriteEmulation(motor2_in2, motor2_speed);
}

void MyActiveObject::setMotorDirections(int motor1_dir, int motor2_dir)
{
    digitalWriteEmulation(motor1_in1, (motor1_dir == 1) ? HIGH : LOW);
    digitalWriteEmulation(motor2_in1, (motor2_dir == 1) ? HIGH : LOW);
    digitalWriteEmulation(motor1_in2, (motor1_dir == -1) ?
                                              HIGH :
                                              (current_move_direction_motor1_ == 1 ? LOW : LOW));
    digitalWriteEmulation(motor2_in2, (motor2_dir == -1) ?
                                              HIGH :
                                              (current_move_direction_motor2_ == 1 ? LOW : LOW));
}

void MyActiveObject::stopMotors()
{
    setMotorSpeeds(0, 0);
    setMotorDirections(0, 0);
    current_move_direction_motor1_ = 0;
    current_move_direction_motor2_ = 0;
}

void MovingForwardState::processEvent(MyActiveObject *context, const Event &event)
{
    // Periodically (e.g., based on a timer event)
    if (event.type == EventType::PID_UPDATE) {
        long current_pulses_motor1 = motor1_pulse_count - context->initial_motor1_pulses_;
        long current_pulses_motor2 = motor2_pulse_count - context->initial_motor2_pulses_;

        // Calculate PID output for each motor
        double pid_output_motor1 =
                context->pid_motor1_.compute(context->target_pulses_motor1_, current_pulses_motor1);
        double pid_output_motor2 =
                context->pid_motor2_.compute(context->target_pulses_motor2_, current_pulses_motor2);

        // Apply the PID output as motor speeds (you might need to map the PID output to your motor
        // speed range)
        context->setMotorSpeeds(static_cast<int>(pid_output_motor1),
                                static_cast<int>(pid_output_motor2));

        // Check if the target is reached (you might use a looser tolerance with PID)
        if (abs(current_pulses_motor1 - context->target_pulses_motor1_) < some_tolerance &&
            abs(current_pulses_motor2 - context->target_pulses_motor2_) < some_tolerance) {
            std::cout << "[" << millis() << "] Target reached with PID, transitioning to IdleState."
                      << std::endl;
            context->stopMotors();
            context->resetPID();
            context->changeState(new IdleState());
        }
    }
    // You might still handle a direct "STOP" event here
}

void MovingForwardState::entry(MyActiveObject *context)
{
    std::cout << "[" << millis() << "] Entering MovingForwardState (with PID)." << std::endl;
    context->setCurrentMoveDirection(1, 1);
    context->resetPID(); // Reset PID integrators when starting a new movement
    // Initial motor start might be needed here with a base speed or the first PID output
}

void MovingForwardState::exit(MyActiveObject *context)
{
    std::cout << "[" << millis() << "] Exiting MovingForwardState (with PID)." << std::endl;
    context->setCurrentMoveDirection(0, 0);
    context->stopMotors();
    context->resetPID();
}

    Key Changes for PID Integration:

    PIDController Class: You'll need to implement this as a separate class.
    PID Controller Instances: Add PIDController objects (pid_motor1_, pid_motor2_) as members of MyActiveObject.
    PID Initialization: Initialize the PID controllers in the MyActiveObject constructor with appropriate gains (Kp, Ki, Kd) and output limits (e.g., 0 to 255 for PWM). Tuning these gains is crucial for good performance.
    resetPID(): A helper function to reset the integral term and previous error of the PID controllers when starting a new movement.
    Periodic PID Update: You'll need a mechanism to periodically trigger the PID calculation and motor speed update within the "Moving" states. This could be done using a timer within your active object and sending a PID_UPDATE event to the current state.
    compute() Method: In the processEvent() of the "Moving" state (when a PID_UPDATE event occurs), you'll read the current pulse counts, calculate the PID output for each motor based on the error between the target_pulses_ and the current counts, and then apply these outputs as the motor speeds using context->setMotorSpeeds().
    Stopping Condition with Tolerance: With PID, you might stop when the error (difference between target and actual pulses) is within a small tolerance rather than waiting for the exact target.
    Next Steps for PID Implementation:

    Implement the PIDController class.
    Add PIDController instances to MyActiveObject and initialize them with initial gains and output limits.
    Modify the entry() method of your "Moving" states to reset the PID controllers.
    Implement a timer mechanism (e.g., using millis() in your main loop and enqueuing a PID_UPDATE event periodically to your active object).
    Modify the processEvent() method of your "Moving" states to:
    On receiving a PID_UPDATE event:
    Calculate the PID output for each motor.
    Apply the PID output as motor speeds.
    Check if the target pulse count (or a value within a tolerance) has been reached. If so, stop the motors and transition back to IDLE.
    Tune the PID gains (Kp, Ki, Kd) for your system. This is an iterative process and might require some experimentation to achieve stable and accurate movement.
    Integrating PID control will make your robot's movements much more precise and robust to disturbances compared to the simple open-loop timing we used initially. Remember that proper PID tuning is essential for optimal performance.
    // ... (Helper functions for emulation) ...
#endif // MY_ACTIVE_OBJECT_H
