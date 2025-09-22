#include "MovementController.h"
#ifdef ARDUINO
#include <Arduino.h>
#else
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>

// Arduino compatibility shims (very minimal for linting/off-target build)
static inline unsigned long millis()
{
    static unsigned long fake = 0;
    fake += 50;
    return fake; // advances 50ms each call
}
template <typename T> static inline T constrain(T x, T a, T b)
{
    return x < a ? a : (x > b ? b : x);
}
static inline long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
#endif

MovementController::MovementController(MotorControlInterface &motorControl,
                                       IMUOrientationInterface &imu)
        : _motorControl(motorControl)
        , _imu(imu)
        , g_yawPID(&g_yawInput, &g_yawOutput, &g_yawSetpoint, g_Kp, g_Ki, g_Kd, 0)
{
}

void MovementController::init(uint32_t pwm_freq)
{
    imuOk = _imu.begin();
    _motorControl.init(pwm_freq);

    // Configure PID (20 Hz like test implementation)
    g_yawPID.SetSampleTime(50); // ms
    g_yawPID.SetOutputLimits(-180.0, 180.0); // Degrees-equivalent authority
    g_yawPID.SetTunings(g_Kp, g_Ki, g_Kd);
    g_yawPID.SetMode(1); // AUTOMATIC
}

void MovementController::setMotor(uint32_t motor_left_speed, uint32_t motor_right_speed,
                                  bool forward_motor_left, bool forward_motor_right,
                                  uint32_t timeout_ms)
{
    uint32_t _leftPercent = map(motor_left_speed, 0, 100, 0, 255);
    uint32_t _rightPercent = map(motor_right_speed, 0, 100, 0, 255);

    _motorControl.setMotorSpeeds(_leftPercent, _rightPercent, forward_motor_left,
                                 forward_motor_right);
}

bool MovementController::isTargetReached()
{
    return false;
}

void MovementController::stopBothMotors()
{
    _motorControl.setMotorSpeeds(0, 0, false, false);
}

void MovementController::stopMotorLeft()
{
    _motorControl.stopLeftMotor();
}

void MovementController::stopMotorRight()
{
    _motorControl.stopRightMotor();
}

void MovementController::startStraightDriving(float speed, uint32_t duration_ms)
{
    if (!imuOk) {
        return;
    }

    // Capture current yaw as the target heading
    yawSetpoint = _imu.yaw();
    targetYaw = yawSetpoint;
    baseSpeed = speed;

    // Sync PID variables
    g_yawSetpoint = yawSetpoint;
    g_yawInput = yawSetpoint; // Start at zero error
    g_stableCount = 0;
    g_yawPID.SetMode(1);

    straightDriving = true;
    turningInPlace = false;

    // Initialize timer if set
    if (duration_ms > 0) {
        timedMovement = true;
        movementDurationMs = duration_ms;
        movementStartMs = millis();
    } else {
        timedMovement = false;
    }
}

void MovementController::startTurningInPlace(float target_yaw, float speed, uint32_t duration_ms)
{
    if (!imuOk) {
        return;
    }

    // Normalize target into [-180,180]
    while (target_yaw > 180.0f)
        target_yaw -= 360.0f;
    while (target_yaw < -180.0f)
        target_yaw += 360.0f;

    yawSetpoint = target_yaw;
    targetYaw = target_yaw;
    baseSpeed = speed;

    g_yawSetpoint = yawSetpoint;
    g_yawInput = _imu.yaw();
    g_stableCount = 0;
    g_yawPID.SetMode(1);

    straightDriving = false;
    turningInPlace = true;

    // Initialize timer if set
    if (duration_ms > 0) {
        timedMovement = true;
        movementDurationMs = duration_ms;
        movementStartMs = millis();
    } else {
        timedMovement = false;
    }
}

void MovementController::updateMovement()
{
    if ((!straightDriving && !turningInPlace) || !imuOk) {
        return;
    }

    // Check if timed movement has expired
    if (timedMovement && (millis() - movementStartMs >= movementDurationMs)) {
        stopMovement();
        return;
    }

    // Update current yaw from IMU
    _imu.update();
    currentYaw = _imu.yaw();

    // Wrap handling so that PID sees continuous value near setpoint
    double adjustedYaw = currentYaw;
    double diff = yawSetpoint - currentYaw;
    if (diff > 180.0) {
        adjustedYaw += 360.0; // current below -180 relative to setpoint
    } else if (diff < -180.0) {
        adjustedYaw -= 360.0; // current above +180 relative to setpoint
    }
    g_yawInput = adjustedYaw;
    g_yawSetpoint = yawSetpoint; // keep synchronized

    // Compute PID when library decides interval elapsed
    if (g_yawPID.Compute()) {
        // Smooth output to reduce jitter
        g_prevFilteredOutput = OUTPUT_FILTER_ALPHA * g_yawOutput +
                               (1.0 - OUTPUT_FILTER_ALPHA) * g_prevFilteredOutput;
        motorOffsetOutput = g_prevFilteredOutput;

        // Human-readable error (shortest angular diff)
        pidError = yawDiff(yawSetpoint, currentYaw);
        applyPidOutput();
    }
}

void MovementController::applyPidOutput()
{
    int leftPWM = 0;
    int rightPWM = 0;

    if (turningInPlace) {
        float absErr = std::abs(pidError);

        // Speed deceleration near target
        float speedRatio = 1.0f;
        if (absErr < TURN_DECEL_ANGLE_DEG) {
            speedRatio = std::max(TURN_MIN_SPEED_RATIO, absErr / TURN_DECEL_ANGLE_DEG);
        }
        float commanded = baseSpeed * speedRatio;

        double correctionScale = motorOffsetOutput / 180.0; // map to [-1,1]
        correctionScale = constrain(correctionScale, -1.0, 1.0);

        leftPWM = static_cast<int>(-correctionScale * commanded);
        rightPWM = static_cast<int>(correctionScale * commanded);

        // Stability / deadband check to finalize turn
        if (absErr < TURN_DEADBAND_DEG) {
            g_stableCount++;
            if (g_stableCount >= STABLE_REQUIRED) {
                stopMovement();
                return;
            }
        } else {
            g_stableCount = 0;
        }
    } else {
        // Straight driving heading correction
        float speedScale = baseSpeed / 255.0f;
        double correctionScale = motorOffsetOutput / 180.0; // [-1,1]
        correctionScale = constrain(correctionScale, -0.3, 0.3); // Limit authority

        float left = speedScale - static_cast<float>(correctionScale) * 0.5f;
        float right = speedScale + static_cast<float>(correctionScale) * 0.5f;

        leftPWM = static_cast<int>(left * 255.0f);
        rightPWM = static_cast<int>(right * 255.0f);
    }

    leftPWM = std::clamp(leftPWM, -255, 255);
    rightPWM = std::clamp(rightPWM, -255, 255);

    bool leftForward = leftPWM >= 0;
    bool rightForward = rightPWM >= 0;

    uint32_t leftSpeed = static_cast<uint32_t>(std::abs(leftPWM));
    uint32_t rightSpeed = static_cast<uint32_t>(std::abs(rightPWM));

    _motorControl.setMotorSpeeds(leftSpeed, rightSpeed, leftForward, rightForward);
}

void MovementController::stopMovement()
{
    straightDriving = false;
    turningInPlace = false;
    timedMovement = false;
    _motorControl.stopBothMotors();
}

bool MovementController::isMoving() const
{
    return straightDriving || turningInPlace;
}

float MovementController::getCurrentError() const
{
    return pidError;
}

float MovementController::yawDiff(float target, float current)
{
    float d = target - current;
    while (d > 180.0f)
        d -= 360.0f;
    while (d < -180.0f)
        d += 360.0f;
    return d;
}

void MovementController::checkMovementTimeout()
{
    if (timedMovement && (millis() - movementStartMs >= movementDurationMs)) {
        stopMovement();
    }
}
