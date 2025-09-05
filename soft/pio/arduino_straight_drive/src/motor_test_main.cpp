// Motor PWM threshold test program - receiver side
// Receives PWM commands via nRF24L01+ and applies them to DRV8871 motors
// Commands: "L<value>" for left motor, "R<value>" for right motor, "B<value>" for both
// PWM value range: -255 to +255 (negative = reverse)
// NEW: "F<value>" for straight driving with IMU+PID

#include "IMUOrientation.h"
#include "pin_definitions.h"

#include <Arduino.h>
#include <RF24.h>
#include <RadioProtocol.h>
#include <SPI.h>
#include <Wire.h>
#include <PID_v1.h>

#ifndef RADIO_CE_PIN
#define RADIO_CE_PIN PB0
#endif
#ifndef RADIO_CSN_PIN
#define RADIO_CSN_PIN PB1
#endif

RF24 radio(RADIO_CE_PIN, RADIO_CSN_PIN);
bool radioOk = false;

// IMU for straight driving
IMUOrientation imu;
bool imuOk = false;

// PID variables for PID_v1 library
double yawSetpoint, currentYaw, motorOffsetOutput;
double Kp = 2.0, Ki = 0.01, Kd = 0.01;

// Create PID instance (using 0 for DIRECT mode)
PID myPID(&currentYaw, &motorOffsetOutput, &yawSetpoint, Kp, Ki, Kd, 0);

// Legacy variables (kept for compatibility)
float targetYaw               = 0.0f;
bool straightDriving          = false;
bool turningInPlace           = false; // For 90-degree turns
float baseSpeed               = 0.0f;
unsigned long straightStartMs = 0;
float currentPwmFreq          = 17.0f; // Set to 17Hz for slow movement
char lastCommand              = 'I'; // Track last command for telemetry
float pidError                = 0.0f; // Current PID error for telemetry

// Continuous telemetry during PID movement
unsigned long lastTelemetryMs = 0;
const unsigned long telemetryIntervalMs = 500; // Send telemetry every 1 second during PID

// Movement timer variables
unsigned long movementDurationMs = 5000; // Default 5 seconds
unsigned long movementStartMs = 0;
bool timedMovement = false;

// Forward declarations
void sendTelemetry();


// Motor helper: DRV8871 style (two inputs per motor). One side PWM, other LOW.
struct MotorPins
{
    uint8_t in1;
    uint8_t in2;
};
MotorPins motorLeft{PIN_A_MOTOR_LEFT, PIN_B_MOTOR_LEFT};
MotorPins motorRight{PIN_A_MOTOR_RIGHT, PIN_B_MOTOR_RIGHT};

static inline void driveOne(const MotorPins &m, int pwmValue)
{
    pwmValue = constrain(pwmValue, -255, 255);
    int pwm  = abs(pwmValue);

    if (pwmValue >= 0)
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

// PID straight driving function
static inline float f_abs(float v)
{
    return v < 0 ? -v : v;
}

static float yawDiff(float target, float current)
{
    float d = target - current;
    while (d > 180.0f)
        d -= 360.0f;
    while (d < -180.0f)
        d += 360.0f;
    return d;
}

void updateStraightDriving()
{
    if ((!straightDriving && !turningInPlace) || !imuOk)
        return;

    // Check if timed movement has expired
    if (timedMovement && (millis() - movementStartMs >= movementDurationMs)) {
        straightDriving = false;
        turningInPlace = false;
        timedMovement = false;
        driveOne(motorLeft, 0);
        driveOne(motorRight, 0);
        Serial.println(F("Movement timer expired - stopping"));
        return;
    }

    // Update current yaw from IMU
    currentYaw = imu.yaw();

    // Handle 360-degree wraparound for PID
    double yawDifference = yawSetpoint - currentYaw;
    if (yawDifference > 180.0) {
        currentYaw += 360.0;
    } else if (yawDifference < -180.0) {
        currentYaw -= 360.0;
    }

    // Compute PID output using PID_v1 library
    myPID.Compute();

    // Store error for telemetry (calculate manually for display)
    pidError = yawSetpoint - currentYaw;

    // Apply base speed with PID correction
    int leftPWM, rightPWM;

    if (turningInPlace) {
        // For turning in place, use PID output to control motor speeds
        // PID output range is -180 to +180, normalize to -1.0 to +1.0
        double correctionScale = motorOffsetOutput / 180.0;
        correctionScale = constrain(correctionScale, -1.0, 1.0);

        // INVERTED LOGIC: Try reversing the motor assignments
        // When PID output is positive (need correction), apply opposite to what we had
        leftPWM  = (int)(-correctionScale * baseSpeed);  // Reversed
        rightPWM = (int)(correctionScale * baseSpeed);   // Reversed

        // Check if turn is complete (within 5 degrees of target)
        if (abs(pidError) < 5.0) {
            Serial.println(F("Turn complete - stopping"));
            straightDriving = false;
            turningInPlace = false;
            leftPWM = 0;
            rightPWM = 0;
        }
    } else {
        // Straight driving (forward or backward)
        float speedScale = baseSpeed / 255.0f;

        // Convert PID output to motor correction
        double correctionScale = motorOffsetOutput / 180.0; // Normalize to ±1.0 range
        correctionScale = constrain(correctionScale, -0.3, 0.3); // Limit correction strength

        float left  = speedScale - correctionScale * 0.5; // Apply correction
        float right = speedScale + correctionScale * 0.5;

        // Convert back to PWM range
        leftPWM  = (int)(left * 255.0f);
        rightPWM = (int)(right * 255.0f);
    }

    driveOne(motorLeft, leftPWM);
    driveOne(motorRight, rightPWM);

    // Send continuous telemetry during PID movement
    unsigned long currentTime = millis();
    if (currentTime - lastTelemetryMs >= telemetryIntervalMs) {
        sendTelemetry();
        lastTelemetryMs = currentTime;
    }
}void processCommand(const String &cmd)
{
    if (cmd.length() < 1)
        return;

    char motor = cmd.charAt(0);
    lastCommand = motor; // Track command for telemetry

    int value  = 0;
    if (cmd.length() > 1)
    {
        value = cmd.substring(1).toInt();
    }

    Serial.print(F("CMD: "));
    Serial.print(motor);
    if (cmd.length() > 1)
    {
        Serial.print(F(" = "));
        Serial.println(value);
    }
    else
    {
        Serial.println();
    }

    switch (motor)
    {
    case 'L':
    case 'l':
        driveOne(motorLeft, value);
        Serial.print(F("Left motor: "));
        Serial.println(value);
        break;

    case 'R':
    case 'r':
        driveOne(motorRight, value);
        Serial.print(F("Right motor: "));
        Serial.println(value);
        break;

    case 'B':
    case 'b':
        driveOne(motorLeft, value);
        driveOne(motorRight, value);
        Serial.print(F("Both motors: "));
        Serial.println(value);
        break;

    case 'S':
    case 's':
        // S command now means backward movement with PID
        if (imuOk && value > 0)
        {
            // Capture current yaw as the target heading
            yawSetpoint = imu.yaw();  // Set PID setpoint to current direction
            targetYaw = yawSetpoint;   // Keep for telemetry compatibility
            baseSpeed = -(float)value; // Negative for backward movement

            // Reset and configure PID
            myPID.SetTunings(Kp, Ki, Kd);
            myPID.SetOutputLimits(-180.0, 180.0); // Output in degrees
            myPID.SetSampleTime(50); // 50ms sample time for 20Hz update rate
            myPID.SetMode(1); // AUTOMATIC mode

            straightDriving = true;
            turningInPlace = false;
            straightStartMs = millis();

            // Reset telemetry timer for continuous debug output
            lastTelemetryMs = millis();

            // Initialize timer if set
            if (movementDurationMs > 0) {
                timedMovement = true;
                movementStartMs = millis();
            } else {
                timedMovement = false;
            }

            Serial.print(F("Backward driving at PWM -"));
            Serial.print(value);
            Serial.print(F(", target yaw: "));
            Serial.print(targetYaw);
            if (timedMovement) {
                Serial.print(F(", timer: "));
                Serial.print(movementDurationMs);
                Serial.print(F("ms"));
            }
            Serial.println();
        }
        else if (!imuOk)
        {
            Serial.println(F("IMU not available for backward driving"));
        }
        else
        {
            straightDriving = false;
            turningInPlace = false;
            driveOne(motorLeft, 0);
            driveOne(motorRight, 0);
            Serial.println(F("Backward driving stopped"));
        }
        break;

    case 'F':
    case 'f':
        if (imuOk && value > 0)
        {
            // Capture current yaw as the target heading
            yawSetpoint = imu.yaw();  // Set PID setpoint to current direction
            targetYaw = yawSetpoint;   // Keep for telemetry compatibility
            baseSpeed = (float)value;

            // Reset and configure PID
            myPID.SetTunings(Kp, Ki, Kd);
            myPID.SetOutputLimits(-180.0, 180.0); // Output in degrees
            myPID.SetSampleTime(50); // 50ms sample time for 20Hz update rate
            myPID.SetMode(1); // AUTOMATIC mode (using 1 instead of AUTOMATIC constant)

            straightDriving = true;
            straightStartMs = millis();

            // Reset telemetry timer for continuous debug output
            lastTelemetryMs = millis();

            // Initialize timer if set
            if (movementDurationMs > 0) {
                timedMovement = true;
                movementStartMs = millis();
            } else {
                timedMovement = false;
            }

            Serial.print(F("Straight driving at PWM "));
            Serial.print(value);
            Serial.print(F(", target yaw: "));
            Serial.print(targetYaw);
            if (timedMovement) {
                Serial.print(F(", timer: "));
                Serial.print(movementDurationMs);
                Serial.print(F("ms"));
            }
            Serial.println();
        }
        else if (!imuOk)
        {
            Serial.println(F("IMU not available for straight driving"));
        }
        else
        {
            straightDriving = false;
            driveOne(motorLeft, 0);
            driveOne(motorRight, 0);
            Serial.println(F("Straight driving stopped"));
        }
        break;

    case 'G': // Backward movement with PID
    case 'g':
        if (imuOk && value > 0)
        {
            // Same as F command but with negative speed for backward movement
            yawSetpoint = imu.yaw();
            targetYaw = yawSetpoint;
            baseSpeed = -(float)value; // Negative for backward

            myPID.SetTunings(Kp, Ki, Kd);
            myPID.SetOutputLimits(-180.0, 180.0);
            myPID.SetSampleTime(50);
            myPID.SetMode(1);

            straightDriving = true;
            turningInPlace = false;
            straightStartMs = millis();
            lastTelemetryMs = millis();

            if (movementDurationMs > 0) {
                timedMovement = true;
                movementStartMs = millis();
            } else {
                timedMovement = false;
            }

            Serial.print(F("Backward driving at PWM "));
            Serial.print(value);
            Serial.print(F(", target yaw: "));
            Serial.print(targetYaw);
            if (timedMovement) {
                Serial.print(F(", timer: "));
                Serial.print(movementDurationMs);
                Serial.print(F("ms"));
            }
            Serial.println();
        }
        else if (!imuOk)
        {
            Serial.println(F("IMU not available for backward driving"));
        }
        else
        {
            straightDriving = false;
            turningInPlace = false;
            driveOne(motorLeft, 0);
            driveOne(motorRight, 0);
            Serial.println(F("Backward driving stopped"));
        }
        break;

        break;

    case 'Z':
    case 'z':
        if (cmd.startsWith("TL") || cmd.startsWith("tl")) // Turn left 90 degrees
        {
            if (imuOk)
            {
                float currentYaw = imu.yaw();
                yawSetpoint = currentYaw - 90.0; // Turn left 90 degrees

                // Handle 360-degree wraparound
                if (yawSetpoint < -180.0) {
                    yawSetpoint += 360.0;
                }

                targetYaw = yawSetpoint;
                baseSpeed = 100.0f; // Fixed speed for turning

                myPID.SetTunings(Kp, Ki, Kd);
                myPID.SetOutputLimits(-180.0, 180.0);
                myPID.SetSampleTime(50);
                myPID.SetMode(1);

                straightDriving = false;
                turningInPlace = true;
                straightStartMs = millis();
                lastTelemetryMs = millis();

                Serial.print(F("Turning left 90°: "));
                Serial.print(currentYaw, 1);
                Serial.print(F("° → "));
                Serial.print(yawSetpoint, 1);
                Serial.println(F("°"));
            }
            else
            {
                Serial.println(F("IMU not available for turning"));
            }
            return; // Exit early for two-character command
        }
        else if (cmd.startsWith("TR") || cmd.startsWith("tr")) // Turn right 90 degrees
        {
            if (imuOk)
            {
                float currentYaw = imu.yaw();
                yawSetpoint = currentYaw + 90.0; // Turn right 90 degrees

                // Handle 360-degree wraparound
                if (yawSetpoint > 180.0) {
                    yawSetpoint -= 360.0;
                }

                targetYaw = yawSetpoint;
                baseSpeed = 100.0f; // Fixed speed for turning

                myPID.SetTunings(Kp, Ki, Kd);
                myPID.SetOutputLimits(-180.0, 180.0);
                myPID.SetSampleTime(50);
                myPID.SetMode(1);

                straightDriving = false;
                turningInPlace = true;
                straightStartMs = millis();
                lastTelemetryMs = millis();

                Serial.print(F("Turning right 90°: "));
                Serial.print(currentYaw, 1);
                Serial.print(F("° → "));
                Serial.print(yawSetpoint, 1);
                Serial.println(F("°"));
            }
            else
            {
                Serial.println(F("IMU not available for turning"));
            }
            return; // Exit early for two-character command
        }
        else if (imuOk) // Original Z command
        {
            imu.zeroYaw();
            targetYaw = 0.0f;
            Serial.println(F("Yaw reset to zero"));
        }
        else
        {
            Serial.println(F("IMU not available for yaw reset"));
        }
        break;

    case 'Q':
    case 'q':
        if (value > 0)
        {
#ifdef STM32F4xx
            analogWriteFrequency(value);
            currentPwmFreq = (float)value; // Track the frequency
            Serial.print(F("PWM frequency set to "));
            Serial.print(value);
            Serial.println(F(" Hz for slow movement"));
#else
            Serial.println(F("PWM frequency adjustment not available on this platform"));
#endif
        }
        else
        {
            Serial.println(F("Q command: Set PWM frequency (e.g., Q17 for 17Hz)"));
        }
        break;

    case 'P':
    case 'p':
        Serial.print(F("Current PWM frequency: "));
        Serial.print(currentPwmFreq);
        Serial.println(F(" Hz"));
        Serial.print(F("PID parameters: Kp="));
        Serial.print(Kp);
        Serial.print(F(", Ki="));
        Serial.print(Ki);
        Serial.print(F(", Kd="));
        Serial.println(Kd);
        break;

    case 'T':
    case 't':
        if (value > 0)
        {
            // T command format: T1050 means Kp=1.0, Ki=0.05, Kd=0.0
            // Parse as: first digit = Kp*10, next two digits = Ki*1000, last two = Kd*100
            int kp_int = (value / 1000) % 10;
            int ki_int = (value / 10) % 100;
            int kd_int = value % 10;

            Kp = kp_int / 10.0f;
            Ki = ki_int / 1000.0f;
            Kd = kd_int / 100.0f;

            Serial.print(F("PID updated: Kp="));
            Serial.print(Kp);
            Serial.print(F(", Ki="));
            Serial.print(Ki);
            Serial.print(F(", Kd="));
            Serial.println(Kd);
        }
        else
        {
            Serial.println(F("T command: Tune PID (e.g., T0101 = Kp:0.0, Ki:0.001, Kd:0.01)"));
        }
        break;

    case 'K':
    case 'k':
        if (value >= 0)
        {
            Kp = value / 1000.0; // Value in thousandths (e.g., K50 = 0.05)
            myPID.SetTunings(Kp, Ki, Kd); // Update PID controller
            Serial.print(F("Kp updated to: "));
            Serial.println(Kp, 4);
        }
        else
        {
            Serial.println(F("K command: Set Kp (e.g., K50 = 0.05, K100 = 0.1)"));
        }
        break;

    case 'I':
    case 'i':
        if (value >= 0)
        {
            Ki = value / 1000.0; // Value in thousandths (e.g., I500 = 0.5)
            myPID.SetTunings(Kp, Ki, Kd); // Update PID controller
            Serial.print(F("Ki updated to: "));
            Serial.println(Ki, 4);
        }
        else
        {
            Serial.println(F("I command: Set Ki (e.g., I500 = 0.5, I100 = 0.1)"));
        }
        break;

    case 'V':
    case 'v':
        if (value >= 0)
        {
            Kd = value / 1000.0; // Value in thousandths (e.g., V200 = 0.2)
            myPID.SetTunings(Kp, Ki, Kd); // Update PID controller
            Serial.print(F("Kd updated to: "));
            Serial.println(Kd, 4);
        }
        else
        {
            Serial.println(F("V command: Set Kd (e.g., V200 = 0.2, V100 = 0.1)"));
        }
        break;

    case 'M':
    case 'm':
        if (value > 0)
        {
            movementDurationMs = value * 1000; // Convert seconds to milliseconds
            Serial.print(F("Movement timer set to "));
            Serial.print(movementDurationMs);
            Serial.println(F("ms"));
        }
        else
        {
            // Disable timer by setting to 0
            movementDurationMs = 0;
            timedMovement = false;
            Serial.println(F("Movement timer disabled"));
        }
        break;

    case 'W':
    case 'w':
        // W command: Forward movement with PID (same as F but clearer naming)
        if (imuOk && value > 0)
        {
            // Capture current yaw as the target heading
            yawSetpoint = imu.yaw();  // Set PID setpoint to current direction
            targetYaw = yawSetpoint;   // Keep for telemetry compatibility
            baseSpeed = (float)value;

            // Reset and configure PID
            myPID.SetTunings(Kp, Ki, Kd);
            myPID.SetOutputLimits(-180.0, 180.0); // Output in degrees
            myPID.SetSampleTime(50); // 50ms sample time for 20Hz update rate
            myPID.SetMode(1); // AUTOMATIC mode

            straightDriving = true;
            turningInPlace = false;
            straightStartMs = millis();

            // Reset telemetry timer for continuous debug output
            lastTelemetryMs = millis();

            // Initialize timer if set
            if (movementDurationMs > 0) {
                timedMovement = true;
                movementStartMs = millis();
            } else {
                timedMovement = false;
            }

            Serial.print(F("Forward driving at PWM "));
            Serial.print(value);
            Serial.print(F(", target yaw: "));
            Serial.print(targetYaw);
            if (timedMovement) {
                Serial.print(F(", timer: "));
                Serial.print(movementDurationMs);
                Serial.print(F("ms"));
            }
            Serial.println();
        }
        else if (!imuOk)
        {
            Serial.println(F("IMU not available for forward driving"));
        }
        else
        {
            straightDriving = false;
            turningInPlace = false;
            driveOne(motorLeft, 0);
            driveOne(motorRight, 0);
            Serial.println(F("Forward driving stopped"));
        }
        break;

    case 'A':
    case 'a':
        // A command: Turn left 90 degrees with PID
        if (imuOk && value > 0)
        {
            // Set target yaw to current yaw + 90 degrees (turn left)
            yawSetpoint = imu.yaw() - 90.0;  // Left turn subtracts 90 degrees
            if (yawSetpoint < -180.0) yawSetpoint += 360.0; // Wrap around
            targetYaw = yawSetpoint;
            baseSpeed = (float)value;

            // Reset and configure PID for turning
            myPID.SetTunings(Kp, Ki, Kd);
            myPID.SetOutputLimits(-180.0, 180.0);
            myPID.SetSampleTime(50);
            myPID.SetMode(1);

            straightDriving = false;
            turningInPlace = true;
            straightStartMs = millis();
            lastTelemetryMs = millis();

            // Initialize timer if set
            if (movementDurationMs > 0) {
                timedMovement = true;
                movementStartMs = millis();
            } else {
                timedMovement = false;
            }

            Serial.print(F("Turning left 90° at PWM "));
            Serial.print(value);
            Serial.print(F(", current yaw: "));
            Serial.print(imu.yaw(), 1);
            Serial.print(F(", target yaw: "));
            Serial.print(targetYaw);
            Serial.println();
        }
        else if (!imuOk)
        {
            Serial.println(F("IMU not available for turning"));
        }
        else
        {
            straightDriving = false;
            turningInPlace = false;
            driveOne(motorLeft, 0);
            driveOne(motorRight, 0);
            Serial.println(F("Left turn stopped"));
        }
        break;

    case 'D': // Using D for right turn now
    case 'd':
        // D command: Turn right 90 degrees with PID
        if (imuOk && value > 0)
        {
            // Set target yaw to current yaw + 90 degrees (turn right)
            yawSetpoint = imu.yaw() + 90.0;  // Right turn adds 90 degrees
            if (yawSetpoint > 180.0) yawSetpoint -= 360.0; // Wrap around
            targetYaw = yawSetpoint;
            baseSpeed = (float)value;

            // Reset and configure PID for turning
            myPID.SetTunings(Kp, Ki, Kd);
            myPID.SetOutputLimits(-180.0, 180.0);
            myPID.SetSampleTime(50);
            myPID.SetMode(1);

            straightDriving = false;
            turningInPlace = true;
            straightStartMs = millis();
            lastTelemetryMs = millis();

            // Initialize timer if set
            if (movementDurationMs > 0) {
                timedMovement = true;
                movementStartMs = millis();
            } else {
                timedMovement = false;
            }

            Serial.print(F("Turning right 90° at PWM "));
            Serial.print(value);
            Serial.print(F(", current yaw: "));
            Serial.print(imu.yaw(), 1);
            Serial.print(F(", target yaw: "));
            Serial.print(targetYaw);
            Serial.println();
        }
        else if (!imuOk)
        {
            Serial.println(F("IMU not available for turning"));
        }
        else
        {
            straightDriving = false;
            turningInPlace = false;
            driveOne(motorLeft, 0);
            driveOne(motorRight, 0);
            Serial.println(F("Right turn stopped"));
        }
        break;

    case 'X':
    case 'x':
        // X command: Emergency stop all movement
        driveOne(motorLeft, 0);
        driveOne(motorRight, 0);
        straightDriving = false;
        turningInPlace = false;
        timedMovement = false;
        Serial.println(F("EMERGENCY STOP - All movement stopped"));
        break;

    case 'E':
    case 'e':
        // Send debug telemetry immediately
        sendTelemetry();
        Serial.println(F("Debug telemetry sent"));
        break;

    default:
        Serial.print(F("Unknown motor: "));
        Serial.println(motor);
        break;
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000)
    {
    }
    Serial.println(F("Motor PWM Test Receiver"));
    Serial.println(
        F("Commands: L<val>, R<val>, B<val>, F<val> (forward+PID), Z (reset yaw)"));
    Serial.println(F("         Q<freq> (set PWM frequency), P (show PID), T<val> (tune PID), D (debug telemetry)"));
    Serial.println(F("         M<ms> (set movement timer), K<val> (set Kp), I<val> (set Ki), V<val> (set Kd)"));
    Serial.println(F("PID Movement: W<val> (forward), S<val> (backward), A<val> (left 90°), E<val> (right 90°)"));
    Serial.println(F("         X (emergency stop all movement)"));
    Serial.println(F("PWM range: -255 to +255 (optimized for 17Hz slow movement)"));
    Serial.println(F("WASDX controls: W30 forward, S30 backward, A50 left turn, E50 right turn, X stop"));

    // Setup motor pins
    pinMode(PIN_A_MOTOR_LEFT, OUTPUT);
    pinMode(PIN_B_MOTOR_LEFT, OUTPUT);
    pinMode(PIN_A_MOTOR_RIGHT, OUTPUT);
    pinMode(PIN_B_MOTOR_RIGHT, OUTPUT);
    pinMode(PIN_LED, OUTPUT);

// Set 17Hz PWM frequency for slow movement
#ifdef STM32F4xx
    analogWriteFrequency(17); // 17Hz PWM frequency for slow movement
    currentPwmFreq = 17.0f;   // Initialize frequency tracking
    Serial.println(F("PWM frequency set to 17Hz for slow movement"));
    Serial.println(F("Use Q<freq> to change frequency if needed"));
#else
    currentPwmFreq = 490.0f; // Default Arduino PWM frequency
    Serial.println(F("Using default PWM frequency - try higher frequencies if available"));
#endif

    // Initialize motors to stopped
    driveOne(motorLeft, 0);
    driveOne(motorRight, 0);

    // Initialize PID
    myPID.SetMode(1); // AUTOMATIC mode (using 1 instead of AUTOMATIC constant)
    myPID.SetSampleTime(50); // 50ms sample time
    myPID.SetOutputLimits(-180.0, 180.0); // Output in degrees

    // IMU setup
    if (imu.begin())
    {
        imu.calibrate();
        imu.zeroYaw();
        imuOk = true;
        Serial.println(F("IMU OK - straight driving available"));
    }
    else
    {
        imuOk = false;
        Serial.println(F("IMU FAILED - only direct motor control available"));
        while(1);
    }

    // Radio setup
    if (radio.begin())
    {
        radio.setAddressWidth(5);
        radio.setPALevel(RF24_PA_HIGH);
        radio.setDataRate(RF24_250KBPS);
        radio.setChannel(108);
        radio.setRetries(5, 5);
        radio.enableDynamicPayloads();
        radio.enableAckPayload();
        radio.openReadingPipe(0, RADIO_ADDR_CMD);
        radio.startListening();
        radioOk = true;
        Serial.println(F("Radio OK"));
    }
    else
    {
        Serial.println(F("Radio init FAILED"));
    }

    Serial.println(F("Ready for PWM test commands"));
}

void sendTelemetry() {
    TelemetryPacket telemetry;

    // Initialize all fields to prevent garbage data
    memset(&telemetry, 0, sizeof(telemetry));

    // Fill with current values
    telemetry.targetYaw = targetYaw;
    telemetry.yaw = imuOk ? imu.yaw() : 0.0f;
    telemetry.err = pidError;
    telemetry.pwmFreq = currentPwmFreq;
    telemetry.kp = Kp;
    telemetry.ki = Ki;
    telemetry.kd = Kd;
    telemetry.imuOk = imuOk ? 1 : 0;

    // Update radio address for telemetry and send
    radio.stopListening();
    radio.openWritingPipe(RADIO_ADDR_TLM);

    bool result = radio.write(&telemetry, sizeof(telemetry));

    // Return to command listening
    radio.openWritingPipe(RADIO_ADDR_CMD);
    radio.startListening();

    if (!result) {
        Serial.println(F("Telemetry send failed"));
    }
}

void loop()
{
    // Update IMU if available
    if (imuOk)
    {
        imu.update();
        updateStraightDriving();
    }

    // Handle serial commands for local testing
    if (Serial.available())
    {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd.length() > 0)
        {
            processCommand(cmd);
        }
    }

    // Handle radio commands
    if (radioOk)
    {
        while (radio.available())
        {
            uint8_t len = 0;
            if (radio.isPVariant())
            {
                len = radio.getDynamicPayloadSize();
            }
            if (len == 0 || len > sizeof(CommandPacket))
            {
                // Read & discard to clear FIFO
                uint8_t dump[32];
                radio.read(&dump, 32);
                continue;
            }

            CommandPacket cp{};
            radio.read(&cp, len);
            if (len < sizeof(cp.text))
                cp.text[len] = '\0';
            else
                cp.text[sizeof(cp.text) - 1] = '\0';

            String cmd = String(cp.text);
            cmd.trim();
            if (cmd.length() > 0)
            {
                Serial.print(F("RADIO: "));
                processCommand(cmd);

                // Send ACK
                AckPayload ap{};
                String ack = "OK:" + cmd;
                ack.toCharArray(ap.text, sizeof(ap.text));
                radio.writeAckPayload(0, &ap, sizeof(ap));
            }
        }
    }

    // Blink LED to show activity
    static unsigned long lastBlink = 0;
    if (millis() - lastBlink > 1000)
    {
        lastBlink = millis();
        digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    }
}
