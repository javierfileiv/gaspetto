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
#include <IMUOrientation.h>
#include <Wire.h>
#include <cmath>

static inline float f_abs(float v)
{
    return v < 0 ? -v : v;
}

IMUOrientation imu;

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

static inline void drive(float left, float right)
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
enum class Mode
{
    Idle,
    Forward,
    Backward,
    Turn
};
Mode mode               = Mode::Idle;
float targetYaw         = 0.0f;  // Desired absolute yaw (deg)
float baseSpeed         = 0.35f; // Reduced nominal forward/backward speed (0..1)
float turnSpeed         = 0.30f; // Reduced spin speed for turning
bool turningToAngle     = false; // If performing a 90 deg relative turn
float turnStopThreshold = 4.0f;  // degrees error to finish turn
// Auto-stop timeout for movement commands (ms). Set when a movement starts.
unsigned long movementExpireMs = 0;

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
bool manualOverride = false; // when true, PID logic skipped and manual commands control motors
// Last received serial command (uppercase token) for inclusion in DATA lines
char lastCmdStr[12] = "-"; // short buffer

static void setLastCmd(const String &tok)
{
    size_t n = tok.length();
    if (n > sizeof(lastCmdStr) - 1)
        n = sizeof(lastCmdStr) - 1;
    for (size_t i = 0; i < n; ++i)
    {
        char c = tok.charAt(i);
        if (c >= 'a' && c <= 'z') c = c - 32;
        lastCmdStr[i] = c;
    }
    lastCmdStr[n] = 0;
}

// Utility shortest angular difference (-180..180)
static float yawDiff(float target, float current)
{
    float d = target - current;
    while (d > 180.0f)
        d -= 360.0f;
    while (d < -180.0f)
        d += 360.0f;
    return d;
}

void printHelp()
{
    Serial.println(F(
        "Commands: f/w forward, b/s backward, a turnL, d turnR, x stop, z zero yaw, P#:Kp I#:Ki D#:Kd, p print gains, h help"));
    Serial.println(F("Note: yaw auto-reset on movement start to reduce accumulated drift."));
    Serial.println(F("Extra: yr toggle auto yaw reset (currently on by default)."));
}
// Configurable auto yaw reset behaviour
bool autoResetYawOnMove = true; // can be toggled via 'yr'
float autoResetThresholdDeg = 15.0f; // if not auto resetting, but drift exceeds this, we still reset

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 4000)
    {
    } // Allow USB to enumerate
    Serial.println(F("Straight Drive PID starting"));

    // Pins
    pinMode(motorLeft.in1, OUTPUT);
    pinMode(motorLeft.in2, OUTPUT);
    pinMode(motorRight.in1, OUTPUT);
    pinMode(motorRight.in2, OUTPUT);
    drive(0, 0);

    Wire.begin();
    if (!imu.begin())
    {
        Serial.println(F("IMU init failed"));
        while (true)
        {
            delay(250);
        }
    }
    Serial.println(F("IMU init OK. Calibrating... keep still"));
    imu.calibrate();
    targetYaw     = imu.yaw();
    prevPidMicros = micros();
    startMillis   = millis();
    Serial.println(
        F("DATA_HEADER,time_ms,yaw_deg,pitch_deg,roll_deg,yaw_error_deg,mode,left,right,cmd"));
    printHelp();
}

void handleSerial()
{
    while (Serial.available())
    {
        String tok = Serial.readStringUntil('\n');
        tok.trim();
        if (tok.length() == 0)
            continue;
    Serial.print(F("CMD> "));
    Serial.println(tok); // echo command
    setLastCmd(tok);
        if (tok.equalsIgnoreCase("lf"))
        {                    // left forward manual
            manualOverride   = true;
            mode             = Mode::Idle;
            movementExpireMs = 0;
            drive(0.5f, lastCmdRight);
            lastCmdLeft = 0.5f;
            Serial.println(F("Manual left forward 0.5"));
        }
        else if (tok.equalsIgnoreCase("lr"))
        { // left reverse
            manualOverride   = true;
            mode             = Mode::Idle;
            movementExpireMs = 0;
            drive(-0.5f, lastCmdRight);
            lastCmdLeft = -0.5f;
            Serial.println(F("Manual left reverse -0.5"));
        }
        else if (tok.equalsIgnoreCase("ls"))
        { // left stop
            manualOverride   = true;
            mode             = Mode::Idle;
            movementExpireMs = 0;
            drive(0.0f, lastCmdRight);
            lastCmdLeft = 0.0f;
            Serial.println(F("Manual left stop"));
        }
        else if (tok.equalsIgnoreCase("rf"))
        { // right forward
            manualOverride   = true;
            mode             = Mode::Idle;
            movementExpireMs = 0;
            drive(lastCmdLeft, 0.5f);
            lastCmdRight = 0.5f;
            Serial.println(F("Manual right forward 0.5"));
        }
        else if (tok.equalsIgnoreCase("rr"))
        { // right reverse
            manualOverride   = true;
            mode             = Mode::Idle;
            movementExpireMs = 0;
            drive(lastCmdLeft, -0.5f);
            lastCmdRight = -0.5f;
            Serial.println(F("Manual right reverse -0.5"));
        }
        else if (tok.equalsIgnoreCase("rs"))
        { // right stop
            manualOverride   = true;
            mode             = Mode::Idle;
            movementExpireMs = 0;
            drive(lastCmdLeft, 0.0f);
            lastCmdRight = 0.0f;
            Serial.println(F("Manual right stop"));
        }
        else if (tok.equalsIgnoreCase("f") || tok.equalsIgnoreCase("w"))
        {
            if (autoResetYawOnMove)
            {
                float before = imu.yaw();
                imu.zeroYaw();
                targetYaw = 0; // full reset
                if (baselineSet) baselineYaw = 0; // keep DATA yaw near 0 after reset
                Serial.print(F("(Yaw reset for forward. Before:")); Serial.print(before,1); Serial.println(')');
            }
            else
            {
                float cur = imu.yaw();
                // If accumulated drift / offset huge, still do a reset to avoid massive correction spin
                if (f_abs(cur) > autoResetThresholdDeg)
                {
                    imu.zeroYaw();
                    targetYaw = 0;
                    Serial.println(F("(Auto override yaw reset due to large drift)"));
                }
                else
                {
                    targetYaw = cur; // hold present heading
                }
            }
            mode             = Mode::Forward;
            turningToAngle   = false;
            integral         = 0;
            prevErr          = 0;
            movementExpireMs = millis() + 3000;
            Serial.println(F("Forward (yaw reset, 3s)"));
            manualOverride = false;
        }
        else if (tok.equalsIgnoreCase("b") || tok.equalsIgnoreCase("s"))
        {
            if (autoResetYawOnMove)
            {
                float before = imu.yaw();
                imu.zeroYaw();
                targetYaw = 0;
                if (baselineSet) baselineYaw = 0;
                Serial.print(F("(Yaw reset for backward. Before:")); Serial.print(before,1); Serial.println(')');
            }
            else
            {
                float cur = imu.yaw();
                if (f_abs(cur) > autoResetThresholdDeg)
                {
                    imu.zeroYaw();
                    targetYaw = 0;
                    Serial.println(F("(Auto override yaw reset due to large drift)"));
                }
                else
                {
                    targetYaw = cur;
                }
            }
            mode             = Mode::Backward;
            turningToAngle   = false;
            integral         = 0;
            prevErr          = 0;
            movementExpireMs = millis() + 3000;
            Serial.println(F("Backward (yaw reset, 3s)"));
            manualOverride = false;
        }
        else if (tok.equalsIgnoreCase("x"))
        {
            mode = Mode::Idle;
            drive(0, 0);
            lastCmdLeft      = 0;
            lastCmdRight     = 0;
            movementExpireMs = 0;
            Serial.println(F("Stop"));
            manualOverride = false;
        }
        else if (tok.equalsIgnoreCase("z"))
        {
            imu.zeroYaw();
            targetYaw = 0;
            Serial.println(F("Yaw zeroed"));
        }
        else if (tok.equalsIgnoreCase("a"))
        {
            float before = imu.yaw();
            imu.zeroYaw(); // forced reset for relative turn
            if (baselineSet) baselineYaw = 0;
            Serial.print(F("(Yaw reset for left turn. Before:")); Serial.print(before,1); Serial.println(')');
            mode             = Mode::Turn;
            turningToAngle   = true;
            targetYaw        = -90.0f;
            movementExpireMs = millis() + 3000;
            Serial.print(F("Turn left to -90 (yaw reset, <=3s)"));
            Serial.println();
            manualOverride = false;
        }
        else if (tok.equalsIgnoreCase("d"))
        {
            float before = imu.yaw();
            imu.zeroYaw(); // forced reset for relative turn
            if (baselineSet) baselineYaw = 0;
            Serial.print(F("(Yaw reset for right turn. Before:")); Serial.print(before,1); Serial.println(')');
            mode             = Mode::Turn;
            turningToAngle   = true;
            targetYaw        = 90.0f;
            movementExpireMs = millis() + 3000;
            Serial.print(F("Turn right to +90 (yaw reset, <=3s)"));
            Serial.println();
            manualOverride = false;
        }
        else if (tok.equalsIgnoreCase("yr"))
        {
            autoResetYawOnMove = !autoResetYawOnMove;
            Serial.print(F("Auto yaw reset on move: "));
            if (baselineSet) baselineYaw = 0;
            Serial.println(autoResetYawOnMove ? F("ON") : F("OFF (adaptive)"));
        }
        else if (tok.equalsIgnoreCase("zg"))
        {
            // Zero target to current yaw WITHOUT resetting sensor (good if drift small and you want new heading)
            targetYaw = imu.yaw();
            integral = 0; prevErr = 0;
            Serial.print(F("Target locked to current yaw: "));
            Serial.println(targetYaw,1);
        }
        else if (tok.equalsIgnoreCase("p"))
        {
            Serial.print(F("Kp="));
            Serial.print(Kp, 3);
            Serial.print(F(" Ki="));
            Serial.print(Ki, 4);
            Serial.print(F(" Kd="));
            Serial.println(Kd, 3);
        }
        else if (tok.equalsIgnoreCase("h"))
        {
            printHelp();
        }
        else if (tok.charAt(0) == 'P' || tok.charAt(0) == 'I' || tok.charAt(0) == 'D')
        {
            char c  = tok.charAt(0);
            float v = tok.substring(1).toFloat();
            if (c == 'P')
                Kp = v;
            else if (c == 'I')
                Ki = v;
            else if (c == 'D')
                Kd = v;
            Serial.print(F("Set "));
            Serial.print(c);
            Serial.print(F(" to "));
            Serial.println(v, 4);
        }
        else if (tok.equalsIgnoreCase("t"))
        {
            Serial.println(F("Motor test start"));
            drive(0.4f, 0.4f);
            delay(800);
            drive(-0.4f, -0.4f);
            delay(800);
            drive(0.4f, -0.4f);
            delay(600); // spin
            drive(-0.4f, 0.4f);
            delay(600); // spin other way
            drive(0, 0);
            lastCmdLeft      = 0;
            lastCmdRight     = 0;
            mode             = Mode::Idle;
            movementExpireMs = 0;
            Serial.println(F("Motor test done"));
        }
        else
        {
            Serial.print(F("Unknown cmd: "));
            Serial.println(tok);
        }
    }
}

void loop()
{
    handleSerial();
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
            Serial.println(F("Auto-stop (3s timeout)"));
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
            // Spin toward target: sign of error determines direction
            float spin = turnSpeed * (err > 0 ? 1.0f : -1.0f);
            L          = spin;
            R          = -spin;
            // Stop when close
            if (f_abs(err) < turnStopThreshold)
            {
                mode = Mode::Idle;
                drive(0, 0);
                lastCmdLeft      = 0;
                lastCmdRight     = 0;
                movementExpireMs = 0;
                Serial.println(F("Turn complete"));
                integral = 0;
                prevErr  = 0;
                goto after_drive;
            }
        }
        else
        {
            float base = baseSpeed * directionSign;
            L          = base + corr;
            R          = base - corr;
        }
        // Global speed cap to keep robot slow
        const float maxOut = 0.40f; // overall cap
        L                  = constrain(L, -maxOut, maxOut);
        R                  = constrain(R, -maxOut, maxOut);
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

    // Periodic status every 3s
    static unsigned long lastStatusMs = 0;
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

    // Stream parse-friendly data lines at fixed rate
    if (ms - lastDataMs >= dataIntervalMs)
    {
        lastDataMs     = ms;
        float outYaw   = curYaw - (baselineSet ? baselineYaw : 0);
        float outPitch = imu.pitch() - (baselineSet ? baselinePitch : 0);
        float outRoll  = imu.roll() - (baselineSet ? baselineRoll : 0);
        float outErr   = yawDiff(targetYaw, curYaw);
        Serial.print(F("DATA,"));
        Serial.print(ms);
        Serial.print(',');
        Serial.print(outYaw, 3);
        Serial.print(',');
        Serial.print(outPitch, 3);
        Serial.print(',');
        Serial.print(outRoll, 3);
        Serial.print(',');
        Serial.print(outErr, 3);
        Serial.print(',');
        switch (mode)
        {
        case Mode::Idle:
            Serial.print('I');
            break;
        case Mode::Forward:
            Serial.print('F');
            break;
        case Mode::Backward:
            Serial.print('B');
            break;
        case Mode::Turn:
            Serial.print('T');
            break;
        }
        Serial.print(',');
    Serial.print(lastCmdLeft, 3);
    Serial.print(',');
    Serial.print(lastCmdRight, 3);
    Serial.print(',');
    Serial.println(lastCmdStr);
    }
}
