// PID-based straight drive using extracted IMUOrientation module
// Commands (serial 115200):
//  f / w : drive forward
//  b / s : drive backward
//  a     : turn left 90 deg
//  d     : turn right 90 deg
//  x     : stop motors
//  z     : zero yaw & set target to 0
//  p     : print PID gains
//  P# I# D# : set gain (example: P1.2 I0.05 D0.02)
//  h     : help

#include "pin_definitions.h"

#include <Arduino.h>
#include "IMUOrientation.h" // resides in lib/IMUOrientation
#include <RadioProtocol.h>
#include "RadioModule.h"
#include "CarState.h"
#include "CommandProcessor.h"
#include <SPI.h>
#include <Wire.h>
#include <cstring>
#include <cstdio>
#include <string.h>
#include <cmath>

// --- Configurable durations (can override via -D MOVEMENT_COMMAND_DURATION_MS=... etc.) ---
#ifndef MOVEMENT_COMMAND_DURATION_MS
#define MOVEMENT_COMMAND_DURATION_MS 3000UL
#endif
#ifndef MANUAL_COMMAND_DURATION_MS
#define MANUAL_COMMAND_DURATION_MS 2000UL
#endif

// If defined as 0 (default) we prevent a straight drive PID correction from reversing a motor.
// Set to 1 (via -DALLOW_REVERSE_FOR_STRAIGHT=1) to allow wheel reversal for aggressive correction.
#ifndef ALLOW_REVERSE_FOR_STRAIGHT
#define ALLOW_REVERSE_FOR_STRAIGHT 0
#endif

// Time window (ms) after a forward/backward command during which we suppress PID differential
// so both motors spin up together (prevents one side starting first due to tiny yaw error).
#ifndef STARTUP_SYNC_MS
#define STARTUP_SYNC_MS 120UL
#endif

static inline float f_abs(float v)
{
    return v < 0 ? -v : v;
}

// Minimum duty (0..1) to overcome motor static friction / driver deadband.
// Can override via -DMIN_EFFECTIVE_DUTY=0.18 (tune experimentally).
#ifndef MIN_EFFECTIVE_DUTY
#define MIN_EFFECTIVE_DUTY 0.18f
#endif

IMUOrientation imu; // IMU instance
// Auto yaw reset behavior (externs declared in CarState.h)
bool autoResetYawOnMove = true;             // If true, always zero yaw on movement/turn command
float autoResetThresholdDeg = 6.0f;         // If autoResetYawOnMove==false, reset only if |current yaw| exceeds this

// ----------------- NRF24 Radio (vehicle) -----------------
#ifndef RADIO_CE_PIN
#define RADIO_CE_PIN PB0
#endif
#ifndef RADIO_CSN_PIN
#define RADIO_CSN_PIN PB1
#endif
// Radio module wrapper usage
unsigned long lastRadioMs = 0;

// Forward declarations
static void onRadioCmd(const String &tok) {
    Serial.print(F("RADIO_CMD> "));
    Serial.println(tok);
    commandSetLastCmd(tok);
    commandProcessToken(tok);
}

// Motor helper: DRV8871 style (two inputs per motor). One side PWM, other LOW.
struct MotorPins
{
    uint8_t in1;
    uint8_t in2;
};
MotorPins motorLeft{PIN_A_MOTOR_LEFT, PIN_B_MOTOR_LEFT};
MotorPins motorRight{PIN_A_MOTOR_RIGHT, PIN_B_MOTOR_RIGHT};

static inline void driveOne(const MotorPins &m, float val)
{
    val     = constrain(val, -1.0f, 1.0f);
    // Enforce a minimum non-zero magnitude so small corrections actually move the motor.
    if (val > 0 && val < MIN_EFFECTIVE_DUTY) val = MIN_EFFECTIVE_DUTY;
    else if (val < 0 && -val < MIN_EFFECTIVE_DUTY) val = -MIN_EFFECTIVE_DUTY;
    int pwm = (int)(f_abs(val) * 255.0f);
    if (val >= 0)
    {
        analogWrite(m.in1, pwm);
        analogWrite(m.in2, 0);
    }
    else
    {
        analogWrite(m.in1, 0);
        analogWrite(m.in2, pwm);
    }
}

inline void drive(float left, float right)
{
    driveOne(motorLeft, left);
    driveOne(motorRight, right);
}

// PID parameters
float Kp                    = 1.2f;
float Ki                    = 0.02f;
float Kd                    = 0.10f;
float integral              = 0.0f;
float prevErr               = 0.0f;
unsigned long prevPidMicros = 0;

// Control state
// Mode enum moved to CarState.h
Mode mode               = Mode::Idle;
float targetYaw         = 0.0f;  // Desired absolute yaw (deg)
float baseSpeed         = 0.35f; // Nominal forward/backward speed (kept <= maxOut so corrections aren't flattened)
float turnSpeed         = 0.50f; // Reduced spin speed for turning
bool turningToAngle     = false; // If performing a 90 deg relative turn
float turnStopThreshold = 4.0f;  // degrees error to finish turn
// Auto-stop timeout for movement commands (ms). Set when a movement starts.
unsigned long movementExpireMs = 0;
int turnDir = 0; // -1 = left (CCW), +1 = right (CW)
unsigned long lastMoveStartMs = 0; // when a forward/back/turn was initiated

// Baseline calibration (first 1s) for zeroing orientation outputs without altering internal filter
// state
bool baselineSet          = false;
unsigned long startMillis = 0;
double sumYaw = 0, sumPitch = 0, sumRoll = 0;
unsigned long sampleCount = 0;
float baselineYaw = 0, baselinePitch = 0, baselineRoll = 0; // subtracted when baselineSet

// Data streaming control
const unsigned long dataIntervalMs = 50; // 20 Hz output suitable for plotting
unsigned long lastDataMs           = 0;

// Store last commanded motor outputs for logging
float lastCmdLeft = 0.0f, lastCmdRight = 0.0f;
bool manualOverride = false;
unsigned long manualExpireMs = 0;

// Unified yaw reset helper (actual implementation)
void resetYawAll(const __FlashStringHelper *reason) {
    float before = imu.yaw();
    imu.zeroYaw();
    targetYaw = 0;
    if (baselineSet) baselineYaw = 0;
    Serial.print(F("(Yaw reset for "));
    Serial.print(reason);
    Serial.print(F(". Before:"));
    Serial.print(before,1);
    Serial.println(')');
}

// Shortest angular difference helper (moved back after refactor)
static float yawDiff(float target, float current) {
    float d = target - current;
    while (d > 180.0f) d -= 360.0f;
    while (d < -180.0f) d += 360.0f;
    return d;
}

void loop()
{
    commandProcessSerial();
    // Radio service (invokes callback for each command)
    radioService();
    imu.update();

    unsigned long now = micros();
    float dt          = (now - prevPidMicros) / 1e6f;
    if (dt <= 0)
        dt = 1e-3f;
    prevPidMicros = now;

    float curYaw = imu.yaw();
    float err    = yawDiff(targetYaw, curYaw);

    if (!manualOverride && (mode == Mode::Forward || mode == Mode::Backward || mode == Mode::Turn))
    {
        // Auto-timeout check (before computing PID). If expired, stop and skip driving.
        if (movementExpireMs != 0 && millis() > movementExpireMs)
        {
            mode = Mode::Idle;
            drive(0, 0);
            lastCmdLeft      = 0;
            lastCmdRight     = 0;
            movementExpireMs = 0;
            Serial.print(F("Auto-stop (")); Serial.print(MOVEMENT_COMMAND_DURATION_MS/1000); Serial.println(F("s timeout)"));
        }
        if (mode == Mode::Idle)
        {
            // Nothing more to do this loop iteration for driving.
            goto after_drive;
        }
        // For pure turning we ignore base direction sign and spin in place
        float directionSign = (mode == Mode::Backward) ? -1.0f : 1.0f;
        if (mode == Mode::Turn)
            directionSign = 1.0f; // spin logic below

        // PID
        integral += err * dt;
        // Anti-windup clamp
        integral         = constrain(integral, -100.0f, 100.0f);
        float derivative = (err - prevErr) / dt;
        prevErr          = err;
        float corr       = Kp * err + Ki * integral + Kd * derivative; // deg-based
        // Scale correction to motor domain
        float maxCorr = 1.0f;                              // limit
        corr = constrain(corr / 90.0f, -maxCorr, maxCorr); // assume 90deg error -> full correction

        float L, R;
        if (mode == Mode::Turn)
        {
            // Use command-specified direction (turnDir) for consistent motor activation
            float spin = turnSpeed * (turnDir == 0 ? (err > 0 ? 1.0f : -1.0f) : (float)turnDir);
            L = spin;
            R = -spin;
            // Stop when close
            if (f_abs(err) < turnStopThreshold)
            {
                mode = Mode::Idle;
                drive(0, 0);
                lastCmdLeft      = 0;
                lastCmdRight     = 0;
                movementExpireMs = 0;
                turnDir = 0;
                Serial.println(F("Turn complete"));
                integral = 0;
                prevErr  = 0;
                goto after_drive;
            }
        }
        else
        {
            float base = baseSpeed * directionSign;
            // Limit correction so we don't instantly reverse a wheel when trying to go straight.
            float maxDiff = baseSpeed * 0.8f; // fraction of base allowed as diff
            float diff    = corr; // already scaled
            if (!ALLOW_REVERSE_FOR_STRAIGHT) diff = constrain(diff, -maxDiff, maxDiff);
            // Startup synchronization: keep diff zero for brief window so both motors begin together
            if ((millis() - lastMoveStartMs) < STARTUP_SYNC_MS) diff = 0.0f;
            L = base + diff;
            R = base - diff;
            // Keep both wheels above minimum effective duty (if moving forward/back) so small diff doesn't stall one side
            if (!ALLOW_REVERSE_FOR_STRAIGHT) {
                float minMag = MIN_EFFECTIVE_DUTY * 1.05f; // slight margin
                if (directionSign > 0) {
                    if (L > 0 && L < minMag) L = minMag;
                    if (R > 0 && R < minMag) R = minMag;
                } else if (directionSign < 0) {
                    if (L < 0 && -L < minMag) L = -minMag;
                    if (R < 0 && -R < minMag) R = -minMag;
                }
            }
            if (!ALLOW_REVERSE_FOR_STRAIGHT)
            {
                // Keep commanded direction (no reversal) while allowing slowing to zero.
                if (directionSign > 0)
                {
                    if (L < 0) L = 0;
                    if (R < 0) R = 0;
                }
                else if (directionSign < 0)
                {
                    if (L > 0) L = 0;
                    if (R > 0) R = 0;
                }
            }
        }
        // Global speed cap to keep robot slow (preserve differential by scaling instead of clamping individually)
        const float maxOut = 0.40f; // overall cap
        float peak = f_abs(L) > f_abs(R) ? f_abs(L) : f_abs(R);
        if (peak > maxOut && peak > 0.0f) {
            float scale = maxOut / peak;
            L *= scale;
            R *= scale;
        }
        drive(L, R);
        lastCmdLeft  = L;
        lastCmdRight = R;
    }

after_drive:
    if (mode == Mode::Idle)
    {
        // Ensure motor command record reflects idle state
        if (!manualOverride && (lastCmdLeft != 0 || lastCmdRight != 0))
        {
            lastCmdLeft  = 0;
            lastCmdRight = 0;
        }
    }

    // Manual per-motor timeout
    if (manualOverride && manualExpireMs && millis() > manualExpireMs)
    {
        drive(0,0);
        lastCmdLeft = lastCmdRight = 0;
        manualOverride = false;
        manualExpireMs = 0;
    Serial.print(F("Manual motor timeout (")); Serial.print(MANUAL_COMMAND_DURATION_MS/1000); Serial.println(F("s)"));
    }

    // Periodic status every 3s
    static unsigned long lastStatusMs = 0;
    static unsigned long lastDiagMs = 0; // radio diag every 5s
    unsigned long ms                  = millis();
    if (ms - lastStatusMs >= 3000)
    {
        lastStatusMs = ms;
        Serial.print(F("Yaw:"));
        Serial.print(curYaw, 2);
        Serial.print(F(" Target:"));
        Serial.print(targetYaw, 2);
        Serial.print(F(" Err:"));
        Serial.print(err, 2);
        Serial.print(F(" GyroZ:"));
        Serial.print(imu.gyroZDeg(), 1);
        Serial.print(F(" Mode:"));
        switch (mode)
        {
        case Mode::Idle:
            Serial.print(F("Idle"));
            break;
        case Mode::Forward:
            Serial.print(F("Fwd"));
            break;
        case Mode::Backward:
            Serial.print(F("Back"));
            break;
        case Mode::Turn:
            Serial.print(F("Turn"));
            break;
        }
        Serial.print(F(" PID(Kp="));
        Serial.print(Kp, 2);
        Serial.print(F(",Ki="));
        Serial.print(Ki, 3);
        Serial.print(F(",Kd="));
        Serial.print(Kd, 2);
        Serial.println(')');
    }

    if (radioIsOk() && ms - lastDiagMs >= 5000)
    {
        lastDiagMs = ms;
        Serial.print(F("RADIO_DIAG tlmOk="));
        Serial.print(radioGetTlmOk());
        Serial.print(F(" tlmFail="));
        Serial.print(radioGetTlmFail());
        Serial.print(F(" cmdRx="));
        Serial.print(radioGetCmdRx());
        Serial.print(F(" mode="));
        Serial.println(radioIsMinimal() ? F("MIN") : F("NORM"));
    }

    // Baseline accumulation first second
    if (!baselineSet)
    {
        if (ms - startMillis < 1000)
        {
            sumYaw += curYaw;
            sumPitch += imu.pitch();
            sumRoll += imu.roll();
            sampleCount++;
        }
        else
        {
            if (sampleCount > 0)
            {
                baselineYaw   = (float)(sumYaw / sampleCount);
                baselinePitch = (float)(sumPitch / sampleCount);
                baselineRoll  = (float)(sumRoll / sampleCount);
            }
            baselineSet = true;
            Serial.print(F("Baseline set YPR: "));
            Serial.print(baselineYaw, 2);
            Serial.print(',');
            Serial.print(baselinePitch, 2);
            Serial.print(',');
            Serial.println(baselineRoll, 2);
        }
    }

    // // Stream parse-friendly data lines at fixed rate
    if (ms - lastDataMs >= dataIntervalMs)
    {
        lastDataMs     = ms;
        float outYaw   = curYaw - (baselineSet ? baselineYaw : 0);
        float outPitch = imu.pitch() - (baselineSet ? baselinePitch : 0);
        float outRoll  = imu.roll() - (baselineSet ? baselineRoll : 0);
        float outErr   = yawDiff(targetYaw, curYaw);

        // Radio telemetry send
        if (radioIsOk() && (ms - lastRadioMs) >= dataIntervalMs)
        {
            lastRadioMs = ms;
            TelemetryPacket tp{};
            tp.ms    = ms;
            tp.yaw   = outYaw;
            tp.pitch = outPitch;
            tp.roll  = outRoll;
            tp.err   = outErr;
            tp.left  = lastCmdLeft;
            tp.right = lastCmdRight;
            tp.mode  = (mode == Mode::Idle ?
                            'I' :
                            (mode == Mode::Forward ? 'F' : (mode == Mode::Backward ? 'B' : 'T')));
            bool ok = radioSendTelemetry(tp);
            if (!ok && (radioGetTlmFail() % 25 == 1))
                Serial.println(F("(Radio telemetry send FAIL)"));
        }
    }
}

// Arduino setup moved here during refactor (was previously at top of old monolithic file)
void setup() {
    Serial.begin(115200);
    while (!Serial && millis() < 3000) {}
    Serial.println(F("Vehicle starting (PID straight drive)"));

    pinMode(PIN_A_MOTOR_LEFT, OUTPUT);
    pinMode(PIN_B_MOTOR_LEFT, OUTPUT);
    pinMode(PIN_A_MOTOR_RIGHT, OUTPUT);
    pinMode(PIN_B_MOTOR_RIGHT, OUTPUT);
    pinMode(PIN_LED, OUTPUT);

    // Start IMU
    if (!imu.begin()) {
        Serial.println(F("IMU init FAILED"));
    } else {
        Serial.println(F("IMU OK"));
    }
    imu.zeroYaw();
    targetYaw = 0;
    startMillis = millis();

    // Radio init
    if (radioInit(onRadioCmd)) {
        Serial.println(F("Radio OK"));
    } else {
        Serial.println(F("Radio init FAILED"));
    }

    // Print help once
    Serial.println(F("Type h for help"));
}
