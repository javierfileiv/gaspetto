#include <Arduino.h>
#include <MovementController.h>
#include <PID_v1.h>
#include <cstring>

#define safe_clamp(val, low, high)                              \
    ({                                                          \
        __typeof__(val) _val = (val);                           \
        __typeof__(low) _low = (low);                           \
        __typeof__(high) _high = (high);                        \
        (_val < _low) ? _low : ((_val > _high) ? _high : _val); \
    })

#define safe_max(a, b)          \
    ({                          \
        __typeof__(a) _a = (a); \
        __typeof__(b) _b = (b); \
        _a > _b ? _a : _b;      \
    })

#define safe_fabs(x)            \
    ({                          \
        __typeof__(x) _x = (x); \
        (_x < 0.0f) ? -_x : _x; \
    })

/* Anonymous namespace for internal PID state (no header changes needed). */
namespace
{
/* PID library variables (Input, Output, Setpoint). */
double g_yawInput = 0.0;
double g_yawOutput = 0.0;
double g_yawSetpoint = 0.0;

/* Tunable gains (start with same as original manual PID). */
double g_Kp = 2.0;
double g_Ki = 0.01;
double g_Kd = 0.01;

PID g_yawPID(&g_yawInput, &g_yawOutput, &g_yawSetpoint, g_Kp, g_Ki, g_Kd, 0); /* DIRECT mode. */

/* Turn completion / anti-oscillation helpers. */
int g_stableCount = 0;
const int STABLE_REQUIRED = 4; /* Need N consecutive cycles inside deadband. */
const float TURN_DEADBAND_DEG = 3.0f; /* Final acceptance window. */
const float TURN_DECEL_ANGLE_DEG = 35.0f; /* Start reducing speed under this error. */
const float TURN_MIN_SPEED_RATIO = 0.28f; /* Minimum fraction of base speed. */

/* Output smoothing to reduce motor jitter. */
double g_prevFilteredOutput = 0.0;
const double OUTPUT_FILTER_ALPHA = 0.25; /* 0..1 (higher = more weight to new sample). */
}

MovementController::MovementController(MotorControlInterface &motorControl,
                                       IMUOrientationInterface &imu)
        : _motorControl(motorControl)
        , _imu(imu)
{
}

void MovementController::init(uint32_t pwm_freq)
{
    imuOk = _imu.begin();
    _motorControl.init(pwm_freq);

    /* Configure PID (20 Hz like test implementation). */
    g_yawPID.SetSampleTime(50); /* Milliseconds. */
    g_yawPID.SetOutputLimits(-180.0, 180.0); /* Degrees-equivalent authority. */
    g_yawPID.SetTunings(g_Kp, g_Ki, g_Kd);
    g_yawPID.SetMode(1); /* AUTOMATIC. */
    currentState = MovementState::IDLE;
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

void MovementController::stopBothMotors()
{
    _motorControl.setMotorSpeeds(0, 0, false, false);
    currentState = MovementState::IDLE;
}

void MovementController::startStraightDriving(float speed, uint32_t duration_ms)
{
    if (!imuOk) {
        return;
    }

    /* Capture current yaw as the target heading. */
    yawSetpoint = _imu.yaw();
    telemTargetYaw = yawSetpoint;
    baseSpeed = speed;

    /* Sync PID variables. */
    g_yawSetpoint = yawSetpoint;
    g_yawInput = yawSetpoint; /* Start at zero error. */
    g_stableCount = 0;
    g_yawPID.SetMode(1);

    straightStartMs = millis();

    /* Initialize timer if set. */
    if (duration_ms > 0) {
        timedMovement = true;
        movementDurationMs = duration_ms;
        movementStartMs = millis();
    } else {
        timedMovement = false;
    }
    currentState = MovementState::STRAIGHT_DRIVING;
}

void MovementController::startTurningInPlace(float final_yaw_angle, float speed,
                                             uint32_t duration_ms)
{
    if (!imuOk) {
        return;
    }

    /* Normalize target into [-180,180]. */
    while (final_yaw_angle > 180.0f)
        final_yaw_angle -= 360.0f;
    while (final_yaw_angle < -180.0f)
        final_yaw_angle += 360.0f;

    yawSetpoint = final_yaw_angle;
    telemTargetYaw = final_yaw_angle;
    baseSpeed = speed;

    g_yawSetpoint = yawSetpoint;
    g_yawInput = _imu.yaw();
    g_stableCount = 0;
    g_yawPID.SetMode(1);

    straightStartMs = millis();

    /* Initialize timer if set. */
    if (duration_ms > 0) {
        timedMovement = true;
        movementDurationMs = duration_ms;
        movementStartMs = millis();
    } else {
        timedMovement = false;
    }
    currentState = MovementState::TURNING_IN_PLACE;
}

bool MovementController::isMoving() const
{
    return currentState != MovementState::IDLE;
}

void MovementController::updateMovement()
{
    if (currentState == MovementState::IDLE || !imuOk) {
        return;
    }

    /* Check if timed movement has expired. */
    if (timedMovement && (millis() - movementStartMs >= movementDurationMs)) {
        stopMovement();
        return;
    }

    /* Update current yaw from IMU. */
    _imu.update();
    currentYaw = _imu.yaw();

    /* Wrap handling so that PID sees continuous value near setpoint. */
    double adjustedYaw = currentYaw;
    double diff = yawSetpoint - currentYaw;
    if (diff > 180.0) {
        adjustedYaw += 360.0; /* Current below -180 relative to setpoint. */
    } else if (diff < -180.0) {
        adjustedYaw -= 360.0; /* Current above +180 relative to setpoint. */
    }
    g_yawInput = adjustedYaw;
    g_yawSetpoint = yawSetpoint; /* Keep synchronized. */

    /* Compute PID when library decides interval elapsed. */
    if (g_yawPID.Compute()) {
        /* Smooth output to reduce jitter. */
        g_prevFilteredOutput = OUTPUT_FILTER_ALPHA * g_yawOutput +
                               (1.0 - OUTPUT_FILTER_ALPHA) * g_prevFilteredOutput;
        motorOffsetOutput = g_prevFilteredOutput;

        /* Human-readable error (shortest angular diff). */
        pidError = yawDiff(yawSetpoint, currentYaw);
        applyPidOutput();
    }
    /* Send telemetry during PID movement. */
    checkTelemetry();
}

void MovementController::applyPidOutput()
{
    int leftPWM = 0;
    int rightPWM = 0;

    if (currentState == MovementState::TURNING_IN_PLACE) {
        float absErr = safe_fabs(pidError);

        /* Speed deceleration near target. */
        float speedRatio = 1.0f;
        if (absErr < TURN_DECEL_ANGLE_DEG) {
            speedRatio = safe_max(TURN_MIN_SPEED_RATIO, absErr / TURN_DECEL_ANGLE_DEG);
        }
        float commanded = baseSpeed * speedRatio;

        double correctionScale = motorOffsetOutput / 180.0; /* Map to [-1,1]. */
        correctionScale = constrain(correctionScale, -1.0, 1.0);

        leftPWM = static_cast<int>(-correctionScale * commanded);
        rightPWM = static_cast<int>(correctionScale * commanded);

        /* Stability / deadband check to finalize turn. */
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
        /* Straight driving heading correction. */
        float speedScale = baseSpeed / 255.0f;
        double correctionScale = motorOffsetOutput / 180.0; /* [-1,1]. */
        correctionScale = constrain(correctionScale, -0.3, 0.3); /* Limit authority. */

        float left = speedScale - static_cast<float>(correctionScale) * 0.5f;
        float right = speedScale + static_cast<float>(correctionScale) * 0.5f;

        leftPWM = static_cast<int>(left * 255.0f);
        rightPWM = static_cast<int>(right * 255.0f);
    }

    leftPWM = safe_clamp(leftPWM, -255, 255);
    rightPWM = safe_clamp(rightPWM, -255, 255);

    bool leftForward = leftPWM >= 0;
    bool rightForward = rightPWM >= 0;

    uint32_t leftSpeed = static_cast<uint32_t>(abs(leftPWM));
    uint32_t rightSpeed = static_cast<uint32_t>(abs(rightPWM));

    _motorControl.setMotorSpeeds(leftSpeed, rightSpeed, leftForward, rightForward);
}

void MovementController::stopMovement()
{
    timedMovement = false;
    _motorControl.stopBothMotors();
    currentState = MovementState::IDLE;
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

void MovementController::setTelemetryCallback(void (*callback)(const TelemetryPacket &))
{
    telemetryCallback = callback;
}

TelemetryPacket MovementController::buildTelemetryPacket() const
{
    TelemetryPacket telemetry;
    memset(&telemetry, 0, sizeof(telemetry));

    telemetry.targetYaw = telemTargetYaw;
    telemetry.yaw = imuOk ? _imu.yaw() : 0.0f;
    telemetry.err = pidError;
    telemetry.pwmFreq = telemPwmFreq;
    telemetry.kp = static_cast<float>(Kp);
    telemetry.ki = static_cast<float>(Ki);
    telemetry.kd = static_cast<float>(Kd);
    telemetry.imuOk = imuOk ? 1 : 0;

    return telemetry;
}

void MovementController::checkTelemetry()
{
    if (!telemetryCallback) {
        return;
    }

    unsigned long currentTime = millis();
    if (currentTime - lastTelemetryMs >= telemetryIntervalMs) {
        TelemetryPacket telemetry = buildTelemetryPacket();
        telemetryCallback(telemetry);
        lastTelemetryMs = currentTime;
    }
}
