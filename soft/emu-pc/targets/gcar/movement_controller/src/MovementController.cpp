#include "Log.h"
#include "__assert.h"

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
double g_Ki = 0.0;
double g_Kd = 0.0;
PID g_yawPID(&g_yawInput, &g_yawOutput, &g_yawSetpoint, g_Kp, g_Ki, g_Kd, 0); /* DIRECT mode. */

const float TURN_COMPLETION_DEG = 3.0f; /* Stop turn when error within this threshold. */
const unsigned long TURN_TIMEOUT_MS = 7000; /* Force stop turn after this many ms. */
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
    if (imuOk) {
        LOGLN("IMU init OK");
    }
    G_ASSERT_MSG(imuOk, "IMU init FAILED");
    _imu.zeroYaw();
    _imu.calibrate();
    _motorControl.init(pwm_freq);

    /* Configure PID (20 Hz like test implementation). */
    g_yawPID.SetTunings(g_Kp, g_Ki, g_Kd);
    g_yawPID.SetOutputLimits(-180.0, 180.0); /* Degrees-equivalent authority. */
    g_yawPID.SetSampleTime(50); /* Milliseconds. */
    g_yawPID.SetMode(1); /* AUTOMATIC. */
    currentState = MovementState::IDLE;
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
    baseSpeed = constrain(speed, -255.0f, 255.0f);

    /* Sync PID variables. */
    g_yawSetpoint = yawSetpoint;
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
    /* Always update IMU, even when idle — needed for telemetry. */
    _imu.update();

    if (currentState == MovementState::IDLE || !imuOk) {
        yawSetpoint = currentYaw;
        return;
    }

    /* Check if timed movement has expired. */
    if (timedMovement && (millis() - movementStartMs >= movementDurationMs)) {
        stopMovement();
        return;
    }

    /* Force stop turn after timeout. */
    if (currentState == MovementState::TURNING_IN_PLACE &&
        millis() - straightStartMs >= TURN_TIMEOUT_MS) {
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
        motorOffsetOutput = g_yawOutput;

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
        static int stableCount = 0;
        const int STABLE_REQUIRED = 3;

        double correctionScale = motorOffsetOutput / 180.0;
        correctionScale = constrain(correctionScale, -1.0, 1.0);

        int turnPWM = static_cast<int>(safe_fabs(correctionScale) * baseSpeed);
        if (turnPWM < 20)
            turnPWM = 20;

        leftPWM = (correctionScale >= 0) ? -turnPWM : turnPWM;
        rightPWM = -leftPWM;

        if (safe_fabs(pidError) < TURN_COMPLETION_DEG) {
            stableCount++;
            if (stableCount >= STABLE_REQUIRED) {
                stopMovement();
                return;
            }
        } else {
            stableCount = 0;
        }
    } else {
        /* Straight driving heading correction. */
        float speedScale = baseSpeed / 255.0f;
        double correctionScale = motorOffsetOutput / 180.0; /* Map to [-1,1]. */
        correctionScale = constrain(correctionScale, -0.3, 0.3); /* Limit authority. */

        float left = speedScale - static_cast<float>(correctionScale) * 0.5f;
        float right = speedScale + static_cast<float>(correctionScale) * 0.5f;

        leftPWM = static_cast<int>(left * 255.0f);
        rightPWM = static_cast<int>(right * 255.0f);
    }

    leftPWM = safe_clamp(leftPWM, -255, 255);
    rightPWM = safe_clamp(rightPWM, -255, 255);

    if (currentState == MovementState::STRAIGHT_DRIVING && trimOffset != 0.0f) {
        if (trimOffset > 0) {
            leftPWM += static_cast<int>(trimOffset * 2.55f);
        } else {
            rightPWM -= static_cast<int>(-trimOffset * 2.55f);
        }
        leftPWM = safe_clamp(leftPWM, -255, 255);
        rightPWM = safe_clamp(rightPWM, -255, 255);
    }

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

void MovementController::setTelemetryCallback(void (*callback)(const TelemetryPacket &))
{
    telemetryCallback = callback;
}

void MovementController::setTunings(double kp, double ki, double kd)
{
    Kp = kp;
    Ki = ki;
    Kd = kd;
    g_Kp = kp;
    g_Ki = ki;
    g_Kd = kd;
    g_yawPID.SetTunings(kp, ki, kd);
}

void MovementController::setTrim(float trim)
{
    trimOffset = constrain(trim, -100.0f, 100.0f);
}

TelemetryPacket MovementController::buildTelemetryPacket() const
{
    TelemetryPacket telemetry;
    memset(&telemetry, 0, sizeof(telemetry));

    telemetry.targetYaw = telemTargetYaw;
    telemetry.yaw = imuOk ? _imu.yaw() : 0.0f;
    telemetry.err = (currentState == MovementState::IDLE) ? 0.0f : pidError;
    telemetry.pwmFreq = telemPwmFreq;
    telemetry.kp = static_cast<float>(Kp);
    telemetry.ki = static_cast<float>(Ki);
    telemetry.kd = static_cast<float>(Kd);
    telemetry.imuOk = imuOk ? 1 : 0;
    if (currentState == MovementState::STRAIGHT_DRIVING) {
        telemetry.state = baseSpeed > 0 ? 1 : 2;
    } else if (currentState == MovementState::TURNING_IN_PLACE) {
        float diff = yawSetpoint - (imuOk ? _imu.yaw() : 0.0f);
        while (diff > 180.0f)
            diff -= 360.0f;
        while (diff < -180.0f)
            diff += 360.0f;
        telemetry.state = diff > 0 ? 4 : 3;
    } else {
        telemetry.state = 0;
    }
    telemetry.speed = static_cast<uint8_t>(constrain(safe_fabs(baseSpeed), 0.0f, 255.0f));
    telemetry.trim = static_cast<int8_t>(constrain(trimOffset, -100.0f, 100.0f));

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
