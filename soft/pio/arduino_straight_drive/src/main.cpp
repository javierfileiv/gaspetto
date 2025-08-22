#include <Arduino.h>       // Include Arduino core for math functions and Wire dependency
#include <HardwareTimer.h> // Include the HardwareTimer library
#include <Wire.h>

/* Motor directions. */
#define MOTOR_STOP     0
#define MOTOR_FORWARD  1
#define MOTOR_BACKWARD (-1)

// Simple pow function for squaring (x^2)
float pow2(float x)
{
    return x * x;
}

// Simple square root approximation (Newton's method)
float sqrt_approx(float x)
{
    if (x <= 0)
        return 0;
    float y  = x;
    float y1 = (y + x / y) / 2;
    // Three iterations should be enough for most cases
    y  = y1;
    y1 = (y + x / y) / 2;
    y  = y1;
    y1 = (y + x / y) / 2;
    return y1;
}

// Simple atan approximation
float atan_approx(float x)
{
    // Polynomial approximation of atan
    const float a1  = 0.9998660;
    const float a3  = -0.3302995;
    const float a5  = 0.1801410;
    const float a7  = -0.0851330;
    const float a9  = 0.0208351;
    const float a11 = -0.0023173;

    float x2     = x * x;
    float result = a11;
    result       = result * x2 + a9;
    result       = result * x2 + a7;
    result       = result * x2 + a5;
    result       = result * x2 + a3;
    result       = result * x2 + a1;
    result       = result * x;

    return result;
}

// Control pins for L9110S motor driver
// PWM will be applied directly to one of these pins for speed control.
const int LEFT_MOTOR_IN1  = PB_1;  // Left motor input 1 (e.g., INA) D5 on salaea
const int LEFT_MOTOR_IN2  = PB_2;  // Left motor input 2 (e.g., INB) D4 on salaea
const int RIGHT_MOTOR_IN1 = PB_14; // Right motor input 1 (e.g., INC) D2 on salaea
const int RIGHT_MOTOR_IN2 = PB_15; // Right motor input 2 (e.g., IND) D1 on salaea

// Dead time in microseconds to prevent shoot-through currents
const int DEAD_TIME_US = 20; // Reduced from 100000 to 20μs - still safe but consumes less power

// HardwareTimer objects and channels for each L9110S input pin
HardwareTimer *leftMotorIn1Timer;
uint32_t leftMotorIn1Channel;
HardwareTimer *leftMotorIn2Timer;
uint32_t leftMotorIn2Channel;
HardwareTimer *rightMotorIn1Timer;
uint32_t rightMotorIn1Channel;
HardwareTimer *rightMotorIn2Timer;
uint32_t rightMotorIn2Channel;

// Define a common PWM frequency for motor speed control
const uint32_t MOTOR_PWM_FREQUENCY = 100; // Hz (20 kHz is more efficient and inaudible)

#define MPU6050 0x68                      // MPU6050 I2C address
float AccX, AccY, AccZ;                   // linear acceleration
float GyroX, GyroY, GyroZ;                // angular velocity
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; // used in void loop()
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

// Kalman filter variables for yaw
float kalman_yaw             = 0;                        // Estimated yaw angle
float kalman_bias            = 0;                        // Estimated gyro bias
float kalman_P[2][2]         = {{1.0, 0.0}, {0.0, 1.0}}; // Error covariance matrix
const float kalman_Q_angle   = 0.001;                    // Process noise variance for angle
const float kalman_Q_bias    = 0.003;                    // Process noise variance for bias
const float kalman_R_measure = 0.03;                     // Measurement noise variance

// Power management variables
unsigned long lastSensorReadTime         = 0;
const unsigned long SENSOR_READ_INTERVAL = 20; // Read sensors every 20ms (50Hz)

const int maxSpeed = 120;   // max PWM value (0-255 for analogWrite compatibility)
const int minSpeed = 80;    // min PWM value at which motor moves (0-255)
float angle;                // due to how I orientated my MPU6050 on my car, angle = roll
float targetAngle    = 0;
int equilibriumSpeed = 248; // rough estimate of PWM at the speed pin of the stronger motor, while
                            // driving straight
// and weaker motor at maxSpeed
int leftSpeedVal;
int rightSpeedVal;
bool isDriving     = true;  // it the car driving forward OR rotate/stationary
bool prevIsDriving = true;  // equals isDriving in the previous iteration of void loop()
bool paused        = false; // is the program paused

// Function prototypes
void driving();
void controlSpeed();
void rotate();
int changeSpeed(int motorSpeed, int increment);
void calibrateMPU5060();
void readAcceleration();
void readGyro();
void stopCar();
void clearSerialScreen();
float kalmanFilterYaw(float gyroRate, float dt);
void setLeftMotorSpeed(int direction, uint32_t speed);
void setRightMotorSpeed(int direction, uint32_t speed);
void enterLowPowerMode();
void exitLowPowerMode();

void initializeMotorDriver()
{
    // Initialize the motor driver pins as outputs
    pinMode(LEFT_MOTOR_IN1, OUTPUT);
    pinMode(LEFT_MOTOR_IN2, OUTPUT);
    pinMode(RIGHT_MOTOR_IN1, OUTPUT);
    pinMode(RIGHT_MOTOR_IN2, OUTPUT);
    // Set initial states to LOW to ensure motors are stopped
    digitalWrite(LEFT_MOTOR_IN1, LOW);
    digitalWrite(LEFT_MOTOR_IN2, LOW);
    digitalWrite(RIGHT_MOTOR_IN1, LOW);
    digitalWrite(RIGHT_MOTOR_IN2, LOW);

    // --- HardwareTimer Initialization for L9110S Input Pins (PWM Capable) ---
    // Left Motor IN1 (LEFT_MOTOR_IN1)
    leftMotorIn1Timer = new HardwareTimer(
        (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(LEFT_MOTOR_IN1), PinMap_PWM));
    leftMotorIn1Channel =
        STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(LEFT_MOTOR_IN1), PinMap_PWM));
    if (!leftMotorIn1Timer || leftMotorIn1Channel == (uint32_t)NC)
    {
        Serial.println(
            "Error: Could not get HardwareTimer for LEFT_MOTOR_IN1. Check pin definition.");
        while (1)
            ; // Halt
    }
    leftMotorIn1Timer->pause();
    leftMotorIn1Timer->setPWM(leftMotorIn1Channel, LEFT_MOTOR_IN1, MOTOR_PWM_FREQUENCY, 0); // Init
                                                                                            // with
                                                                                            // 0%
                                                                                            // duty
    leftMotorIn1Timer->setCaptureCompare(leftMotorIn1Channel, 0, PERCENT_COMPARE_FORMAT); // Ensure
                                                                                          // 0% duty
    leftMotorIn1Timer->resume();
    Serial.print("Left Motor IN1 PWM configured on pin ");
    Serial.print(LEFT_MOTOR_IN1);
    Serial.print(" at ");
    Serial.print(MOTOR_PWM_FREQUENCY);
    Serial.println(" Hz.");

    // Left Motor IN2 (LEFT_MOTOR_IN2)
    leftMotorIn2Timer = new HardwareTimer(
        (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(LEFT_MOTOR_IN2), PinMap_PWM));
    leftMotorIn2Channel =
        STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(LEFT_MOTOR_IN2), PinMap_PWM));
    if (!leftMotorIn2Timer || leftMotorIn2Channel == (uint32_t)NC)
    {
        Serial.println(
            "Error: Could not get HardwareTimer for LEFT_MOTOR_IN2. Check pin definition.");
        while (1)
            ; // Halt
    }
    leftMotorIn2Timer->pause();
    leftMotorIn2Timer->setPWM(leftMotorIn2Channel, LEFT_MOTOR_IN2, MOTOR_PWM_FREQUENCY, 0); // Init
                                                                                            // with
                                                                                            // 0%
                                                                                            // duty
    leftMotorIn2Timer->setCaptureCompare(leftMotorIn2Channel, 0, PERCENT_COMPARE_FORMAT); // Ensure
                                                                                          // 0% duty
    leftMotorIn2Timer->resume();
    Serial.print("Left Motor IN2 PWM configured on pin ");
    Serial.print(LEFT_MOTOR_IN2);
    Serial.print(" at ");
    Serial.print(MOTOR_PWM_FREQUENCY);
    Serial.println(" Hz.");

    // Right Motor IN1 (RIGHT_MOTOR_IN1)
    rightMotorIn1Timer = new HardwareTimer(
        (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(RIGHT_MOTOR_IN1), PinMap_PWM));
    rightMotorIn1Channel =
        STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(RIGHT_MOTOR_IN1), PinMap_PWM));
    if (!rightMotorIn1Timer || rightMotorIn1Channel == (uint32_t)NC)
    {
        Serial.println(
            "Error: Could not get HardwareTimer for RIGHT_MOTOR_IN1. Check pin definition.");
        while (1)
            ; // Halt
    }
    rightMotorIn1Timer->pause();
    rightMotorIn1Timer->setPWM(rightMotorIn1Channel, RIGHT_MOTOR_IN1, MOTOR_PWM_FREQUENCY,
                               0);                                 // Init with 0% duty
    rightMotorIn1Timer->setCaptureCompare(rightMotorIn1Channel, 0,
                                          PERCENT_COMPARE_FORMAT); // Ensure
                                                                   // 0%
                                                                   // duty
    rightMotorIn1Timer->resume();
    Serial.print("Right Motor IN1 PWM configured on pin ");
    Serial.print(RIGHT_MOTOR_IN1);
    Serial.print(" at ");
    Serial.print(MOTOR_PWM_FREQUENCY);
    Serial.println(" Hz.");

    // Right Motor IN2 (RIGHT_MOTOR_IN2)
    rightMotorIn2Timer = new HardwareTimer(
        (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(RIGHT_MOTOR_IN2), PinMap_PWM));
    rightMotorIn2Channel =
        STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(RIGHT_MOTOR_IN2), PinMap_PWM));
    if (!rightMotorIn2Timer || rightMotorIn2Channel == (uint32_t)NC)
    {
        Serial.println(
            "Error: Could not get HardwareTimer for RIGHT_MOTOR_IN2. Check pin definition.");
        while (1)
            ; // Halt
    }
    rightMotorIn2Timer->pause();
    rightMotorIn2Timer->setPWM(rightMotorIn2Channel, RIGHT_MOTOR_IN2, MOTOR_PWM_FREQUENCY,
                               0);                                 // Init with 0% duty
    rightMotorIn2Timer->setCaptureCompare(rightMotorIn2Channel, 0,
                                          PERCENT_COMPARE_FORMAT); // Ensure
                                                                   // 0%
                                                                   // duty
    rightMotorIn2Timer->resume();
    Serial.print("Right Motor IN2 PWM configured on pin ");
    Serial.print(RIGHT_MOTOR_IN2);
    Serial.print(" at ");
    Serial.print(MOTOR_PWM_FREQUENCY);
    Serial.println(" Hz.");
    // --- End HardwareTimer Initialization ---
}

void setup()
{
    // Initialize serial communication
    Serial.begin(115200);
    delay(100); // Give serial time to initialize

    Serial.println("\n\n=== Robot Car Controller Starting ===");
    Serial.println("Initializing serial communication...");

    // Initialize I2C communication with MPU6050
    Serial.println("Initializing I2C and MPU6050...");
    Wire.begin();                    // Initialize communication
    Wire.beginTransmission(MPU6050); // Start communication with MPU6050 // MPU=0x68
    Wire.write(0x6B);                // Talk to the register 6B
    Wire.write(0x00);                // Make reset - place a 0 into the 6B register
    Wire.endTransmission(true);      // end the transmission

    // Configure gyroscope range - 0x1B register
    // 0 = +/- 250 degrees/sec (default)
    // 8 = +/- 500 degrees/sec
    // 16 = +/- 1000 degrees/sec
    // 24 = +/- 2000 degrees/sec
    Wire.beginTransmission(MPU6050);
    Wire.write(0x1B);
    Wire.write(0x08); // Set to +/- 500 degrees/sec for better resolution but still filtering
                      // extremes
    Wire.endTransmission(true);

    // Configure accelerometer range - 0x1C register
    // 0 = +/- 2g (default)
    // 8 = +/- 4g
    // 16 = +/- 8g
    // 24 = +/- 16g
    Wire.beginTransmission(MPU6050);
    Wire.write(0x1C);
    Wire.write(0x08); // Set to +/- 4g for better resolution
    Wire.endTransmission(true);

    // Configure power management for lower power consumption
    Wire.beginTransmission(MPU6050);
    Wire.write(0x6B); // Power management register 1
    Wire.write(0x01); // Set to use gyro X axis clock (more stable than internal oscillator)
    Wire.endTransmission(true);

    // Configure sample rate divider to reduce sample rate
    // This increases the time between samples, saving power
    Wire.beginTransmission(MPU6050);
    Wire.write(0x19); // Sample Rate Divider register
    Wire.write(0x13); // Set sample rate to 50Hz (1000/(1+19) = 50Hz)
    Wire.endTransmission(true);

    // Set DLPF (Digital Low Pass Filter)
    Wire.beginTransmission(MPU6050);
    Wire.write(0x1A); // Config register
    Wire.write(0x03); // Set DLPF to 42Hz (helps reduce noise and saves power)
    Wire.endTransmission(true);

    Serial.println("Initializing motor drivers...");
    initializeMotorDriver();
    Serial.println("Calibrating MPU6050 (hold robot still)...");
    calibrateMPU5060();
    delay(20);
    Serial.println("Calibration complete!");

    // Initialize variables
    currentTime   = micros();
    leftSpeedVal  = maxSpeed;
    rightSpeedVal = maxSpeed;

    // Print instructions
    Serial.println("\n=== Setup complete! ===");
    Serial.println("You can now control the car using the serial monitor:");
    Serial.println("'w' - Move forward");
    Serial.println("'a' - Turn left");
    Serial.println("'d' - Turn right");
    Serial.println("'q' - Stop");
    Serial.println("'i' - Print information");
    Serial.println("'p' - Pause/unpause the program");
    Serial.println("'r' - Reset angle to 0");
    Serial.println("Waiting for commands...");
}

void loop()
{
    // Manage time
    previousTime = currentTime;
    currentTime  = micros();
    elapsedTime  = (currentTime - previousTime) / 1000000; // Divide by 1000000 to get seconds

    // Only read sensors at specified intervals to reduce power consumption
    if (millis() - lastSensorReadTime >= SENSOR_READ_INTERVAL)
    {
        lastSensorReadTime = millis();

        // === Read accelerometer (on the MPU6050) data === //
        readAcceleration();
        // Calculating Roll and Pitch from the accelerometer data
        accAngleX = (atan_approx(AccY / sqrt_approx(pow2(AccX) + pow2(AccZ))) * 180 / PI) -
                    AccErrorX; // AccErrorX is calculated in the calculateError() function
        accAngleY =
            (atan_approx(-1 * AccX / sqrt_approx(pow2(AccY) + pow2(AccZ))) * 180 / PI) - AccErrorY;

        // === Read gyroscope (on the MPU6050) data === //
        readGyro();

        // Correct the outputs with the calculated error values
        GyroX -= GyroErrorX; // GyroErrorX is calculated in the calculateError() function
        GyroY -= GyroErrorY;
        GyroZ -= GyroErrorZ;

        // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by
        // seconds (s) to get the angle in degrees
        gyroAngleX += GyroX * elapsedTime; // deg/s * s = deg
        gyroAngleY += GyroY * elapsedTime;

        // Instead of simple integration for yaw, use Kalman filter
        float kalman_filtered_yaw = kalmanFilterYaw(GyroZ, elapsedTime);

        // combine accelerometer- and gyro-estimated angle values. 0.96 and 0.04 values are
        // determined through trial and error by other people
        roll  = 0.96 * gyroAngleX + 0.04 * accAngleX;
        pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
        angle = kalman_filtered_yaw; // Use the Kalman filtered yaw as our angle
                                     // It can roll, pitch, yaw or minus version of the three
        // for me, turning right reduces angle. Turning left increases angle.
    }

    // Process serial input
    if (Serial.available() > 0)
    {
        char c = Serial.read();
        // Print the received character for debugging
        Serial.print("Received character: ");
        Serial.println(c);

        if (c == 'w')
        { // drive forward
            Serial.println("Command: forward");
            isDriving = true;
        }
        else if (c == 'a')
        { // turn left
            Serial.println("Command: left");
            targetAngle += 90;
            if (targetAngle > 180)
            {
                targetAngle -= 360;
            }
            isDriving = false;
        }
        else if (c == 'd')
        { // turn right
            Serial.println("Command: right");
            targetAngle -= 90;
            if (targetAngle <= -180)
            {
                targetAngle += 360;
            }
            isDriving = false;
        }
        else if (c == 'q')
        { // stop or brake
            Serial.println("Command: stop");
            // Don't clear the screen immediately to allow seeing the command
            isDriving = false;
        }
        else if (c == 'i')
        { // print information. When car is stationary, GyroZ should approx. =
          // 1 ( 1 g).
            Serial.println("Current information:");
            Serial.print("angle: ");
            Serial.println(angle);
            Serial.print("targetAngle: ");
            Serial.println(targetAngle);
            Serial.print("GyroZ: ");
            Serial.println(GyroZ);
            Serial.print("elapsedTime (in ms): "); // estimates time to run void loop() once
            Serial.println(elapsedTime * 1000);
            Serial.print("equilibriumSpeed: ");
            Serial.println(equilibriumSpeed);
            Serial.print("leftSpeedVal: ");
            Serial.println(leftSpeedVal);
            Serial.print("rightSpeedVal: ");
            Serial.println(rightSpeedVal);
            Serial.print("isDriving: ");
            Serial.println(isDriving);
            Serial.print("paused: ");
            Serial.println(paused);
        }
        else if (c == 'p')
        { // pause the program
            paused = !paused;
            stopCar();
            isDriving = false;
            Serial.println("Command: pause/unpause");
            Serial.print("Paused state is now: ");
            Serial.println(paused ? "PAUSED" : "RUNNING");

            // Enter or exit low power mode when paused state changes
            if (paused)
            {
                Serial.println("Entering low power mode...");
                enterLowPowerMode();
            }
            else
            {
                Serial.println("Exiting low power mode...");
                exitLowPowerMode();
            }
        }
        else if (c == 'r')
        { // reset angle
            Serial.println("Command: reset angle");
            angle          = 0;
            kalman_yaw     = 0;
            kalman_bias    = 0;
            kalman_P[0][0] = 1.0;
            kalman_P[0][1] = 0.0;
            kalman_P[1][0] = 0.0;
            kalman_P[1][1] = 1.0;
            targetAngle    = 0;
            Serial.println("Angle has been reset to 0");
        }
    }

    static int count;
    static int countStraight;
    if (count < 6)
    {
        count++;
    }
    else
    { // runs once after void loop() runs 7 times. void loop runs about every 2.8ms, so this
      // else condition runs every 19.6ms or 50 times/second
        count = 0;
        if (!paused)
        {
            if (isDriving != prevIsDriving)
            {
                leftSpeedVal  = equilibriumSpeed;
                countStraight = 0;
                Serial.print("mode changed, isDriving: ");
                Serial.println(isDriving);
            }
            if (isDriving)
            {
                /* 3 degress is the threshold for considering the car is driving straight. */
                if (abs(targetAngle - angle) < 3)
                {
                    if (countStraight < 20)
                    {
                        countStraight++;
                    }
                    else
                    {
                        countStraight    = 0;
                        equilibriumSpeed = leftSpeedVal; // to find equilibrium speed, 20
                                                         // consecutive readings need to indicate
                                                         // car is going straight
                        Serial.print("EQUILIBRIUM reached, equilibriumSpeed: ");
                        Serial.println(equilibriumSpeed);
                    }
                }
                else
                {
                    countStraight = 0;
                }
                driving();
            }
            else
            {
                rotate();
            }
            prevIsDriving = isDriving;
        }
    }
}

void driving()
{ // called by void loop(), which isDriving = true
    // Map 0-255 speed value to 0-100% duty cycle for HardwareTimer
    uint32_t mappedLeftSpeed  = map(leftSpeedVal, 0, 255, 0, 100);
    uint32_t mappedRightSpeed = map(rightSpeedVal, 0, 255, 0, 100);

    // Use the safe motor control functions for forward movement
    setLeftMotorSpeed(MOTOR_FORWARD, mappedLeftSpeed);
    setRightMotorSpeed(MOTOR_FORWARD, mappedRightSpeed);

    int deltaAngle = round(targetAngle - angle);
    if (deltaAngle != 0)
        controlSpeed();
}

void controlSpeed()
{ // this function is called by driving ()
    int deltaAngle = round(targetAngle - angle);
    int targetGyroZ;

    // setting up proportional control, see Step 3 on the website
    if (deltaAngle > 30)
    {
        targetGyroZ = 60;
    }
    else if (deltaAngle < -30)
    {
        targetGyroZ = -60;
    }
    else
    {
        targetGyroZ = 2 * deltaAngle;
    }

    // Update both motors based on the correction needed
    if (round(targetGyroZ - GyroX) == 0)
    {
        // No adjustment needed, keep both speeds the same
        rightSpeedVal = leftSpeedVal;
    }
    else if (targetGyroZ > GyroX)
    {
        // Need to turn more to the right
        // Reduce left motor speed
        leftSpeedVal = changeSpeed(leftSpeedVal, -1);
        // Reduce right motor speed for sharper turn
        rightSpeedVal = changeSpeed(rightSpeedVal, +1);
    }
    else
    {
        // Need to turn more to the left
        // Increase left motor speed
        leftSpeedVal = changeSpeed(leftSpeedVal, +1);
        // Reduce right motor speed for sharper turn
        rightSpeedVal = changeSpeed(rightSpeedVal, -1);
    }
}

void rotate()
{ // called by void loop(), which isDriving = false
    int deltaAngle = round(targetAngle - angle);
    int targetGyroZ;

    // Map 0-255 speed value to 0-100% duty cycle for HardwareTimer
    uint32_t mappedLeftSpeed  = map(leftSpeedVal, 0, 255, 0, 100);
    uint32_t mappedRightSpeed = map(rightSpeedVal, 0, 255, 0, 100);

    if (abs(deltaAngle) <= 1)
    {
        stopCar();
    }
    else
    {
        if (angle > targetAngle)
        {
            // Turn left (Left motor forward, Right motor reverse)
            setLeftMotorSpeed(1, mappedLeftSpeed);
            setRightMotorSpeed(-1, mappedRightSpeed);
        }
        else if (angle < targetAngle)
        {
            // Turn right (Left motor reverse, Right motor forward)
            setLeftMotorSpeed(-1, mappedLeftSpeed);
            setRightMotorSpeed(1, mappedRightSpeed);
        }

        // setting up proportional control, see Step 3 on the website
        if (abs(deltaAngle) > 30)
        {
            targetGyroZ = 60;
        }
        else
        {
            targetGyroZ = 2 * abs(deltaAngle);
        }

        if (round(targetGyroZ - abs(GyroX)) == 0)
        {
            ;
        }
        else if (targetGyroZ > abs(GyroX))
        {
            leftSpeedVal = changeSpeed(leftSpeedVal, +1); // would increase abs(GyroX)
        }
        else
        {
            leftSpeedVal = changeSpeed(leftSpeedVal, -1);
        }
        rightSpeedVal = leftSpeedVal;
    }
}

int changeSpeed(int motorSpeed, int increment)
{
    motorSpeed += increment;
    if (motorSpeed > maxSpeed)
    { // to prevent motorSpeed from exceeding 255, which is a problem
      // when using analogWrite
        motorSpeed = maxSpeed;
    }
    else if (motorSpeed < minSpeed)
    {
        motorSpeed = minSpeed;
    }
    return motorSpeed;
}

void calibrateMPU5060()
{
    // When this function is called, ensure the car is stationary. See Step 2 for more info
    Serial.println("Starting MPU6050 calibration... Keep the device still!");
    delay(100); // Make sure the message is sent

    // Read accelerometer values 200 times
    c = 0;
    while (c < 200)
    {
        readAcceleration();
        // Sum all readings
        AccErrorX += (atan_approx((AccY) / sqrt_approx(pow2((AccX)) + pow2((AccZ)))) * 180 / PI);
        AccErrorY +=
            (atan_approx(-1 * (AccX) / sqrt_approx(pow2((AccY)) + pow2((AccZ)))) * 180 / PI);
        c++;
    }
    // Divide the sum by 200 to get the error value, since expected value of reading is zero
    AccErrorX = AccErrorX / 200;
    AccErrorY = AccErrorY / 200;
    Serial.println("Accelerometer calibration complete!");
    delay(100); // Make sure the message is sent
    c = 0;

    // Read gyro values 200 times
    Serial.println("Starting gyroscope calibration...");
    delay(100); // Make sure the message is sent

    while (c < 200)
    {
        readGyro();
        // Sum all readings
        GyroErrorX += GyroX;
        GyroErrorY += GyroY;
        GyroErrorZ += GyroZ;
        c++;
    }
    // Divide the sum by 200 to get the error value
    GyroErrorX = GyroErrorX / 200;
    GyroErrorY = GyroErrorY / 200;
    GyroErrorZ = GyroErrorZ / 200;

    Serial.println("Gyroscope calibration complete!");
    Serial.println("MPU6050 has been fully calibrated");
    Serial.println("Error values:");
    Serial.print("AccErrorX: ");
    Serial.println(AccErrorX);
    Serial.print("AccErrorY: ");
    Serial.println(AccErrorY);
    Serial.print("GyroErrorX: ");
    Serial.println(GyroErrorX);
    Serial.print("GyroErrorY: ");
    Serial.println(GyroErrorY);
    Serial.print("GyroErrorZ: ");
    Serial.println(GyroErrorZ);
    delay(100); // Make sure the message is sent
}

void readAcceleration()
{
    Wire.beginTransmission(MPU6050);
    Wire.write(0x3B);                               // Start with register 0x3B (ACCEL_XOUT_H)
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050, (uint8_t)6); // Read 6 registers total, each axis value is
                                                    // stored in 2 registers

    // We now use 8192.0 instead of 16384.0 because we configured the accelerometer to +/- 4g range
    // For reference:
    // 2g = 16384.0
    // 4g = 8192.0
    // 8g = 4096.0
    // 16g = 2048.0
    AccX = (Wire.read() << 8 | Wire.read()) / 8192.0; // X-axis value
    AccY = (Wire.read() << 8 | Wire.read()) / 8192.0; // Y-axis value
    AccZ = (Wire.read() << 8 | Wire.read()) / 8192.0; // Z-axis value
}

void readGyro()
{
    Wire.beginTransmission(MPU6050);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU6050, (uint8_t)6);

    // We now use 65.5 instead of 131.0 because we configured the gyro to +/- 500 degrees/sec range
    // For reference:
    // 250 deg/s = 131.0
    // 500 deg/s = 65.5
    // 1000 deg/s = 32.8
    // 2000 deg/s = 16.4
    GyroX = (Wire.read() << 8 | Wire.read()) / 65.5;
    GyroY = (Wire.read() << 8 | Wire.read()) / 65.5;
    GyroZ = (Wire.read() << 8 | Wire.read()) / 65.5;

    // Apply limits to raw gyro readings to filter out extreme values
    // Normal human motion rarely exceeds 250 deg/sec
    const float maxGyroRate = 300.0;
    if (GyroX > maxGyroRate)
        GyroX = maxGyroRate;
    if (GyroX < -maxGyroRate)
        GyroX = -maxGyroRate;
    if (GyroY > maxGyroRate)
        GyroY = maxGyroRate;
    if (GyroY < -maxGyroRate)
        GyroY = -maxGyroRate;
    if (GyroZ > maxGyroRate)
        GyroZ = maxGyroRate;
    if (GyroZ < -maxGyroRate)
        GyroZ = -maxGyroRate;
}

void stopCar()
{
    // Use the safe motor control functions to stop both motors
    setLeftMotorSpeed(MOTOR_STOP, 0);
    setRightMotorSpeed(MOTOR_STOP, 0);
}

void clearSerialScreen()
{
    Serial.write(27);    // ASCII Escape character
    Serial.print("[2J"); // ANSI escape sequence to clear screen
    Serial.write(27);    // ASCII Escape character
    Serial.print("[H");  // ANSI escape sequence to move cursor to home position
}

// Kalman filter implementation for yaw estimation
// This takes the gyro rate and time step as inputs
float kalmanFilterYaw(float gyroRate, float dt)
{
    // Step 1: Predict state
    float rate = gyroRate - kalman_bias;
    kalman_yaw += dt * rate;

    // Normalize the angle to keep it within -180 to +180 range
    while (kalman_yaw > 180)
        kalman_yaw -= 360;
    while (kalman_yaw <= -180)
        kalman_yaw += 360;

    // Step 2: Update error covariance matrix
    kalman_P[0][0] += dt * (dt * kalman_P[1][1] - kalman_P[0][1] - kalman_P[1][0] + kalman_Q_angle);
    kalman_P[0][1] -= dt * kalman_P[1][1];
    kalman_P[1][0] -= dt * kalman_P[1][1];
    kalman_P[1][1] += kalman_Q_bias * dt;

    // Step 3: Calculate Kalman gain
    float S = kalman_P[0][0] + kalman_R_measure;
    float K[2];
    K[0] = kalman_P[0][0] / S;
    K[1] = kalman_P[1][0] / S;

    // Step 4: Calculate angle and bias
    // The gyro measurement is treated as the measurement (ideally we'd have a magnetometer here)
    float y = gyroRate - rate;
    kalman_yaw += K[0] * y;
    kalman_bias += K[1] * y;

    // Normalize the angle again after update
    while (kalman_yaw > 180)
        kalman_yaw -= 360;
    while (kalman_yaw <= -180)
        kalman_yaw += 360;

    // Step 5: Calculate new error covariance matrix
    float P00_temp = kalman_P[0][0];
    float P01_temp = kalman_P[0][1];

    kalman_P[0][0] -= K[0] * P00_temp;
    kalman_P[0][1] -= K[0] * P01_temp;
    kalman_P[1][0] -= K[1] * P00_temp;
    kalman_P[1][1] -= K[1] * P01_temp;

    return kalman_yaw;
}

// Safety function to set left motor speed with direction
// direction: 1 for forward, -1 for reverse, 0 for stop
void setLeftMotorSpeed(int direction, uint32_t speed)
{
    // First, stop the motor completely to prevent shoot-through
    leftMotorIn1Timer->setCaptureCompare(leftMotorIn1Channel, 0, PERCENT_COMPARE_FORMAT);
    leftMotorIn2Timer->setCaptureCompare(leftMotorIn2Channel, 0, PERCENT_COMPARE_FORMAT);

    // Wait for the dead time
    delayMicroseconds(DEAD_TIME_US);

    // Now set the new direction and speed
    if (direction > 0)
    {
        // Forward: IN1=PWM, IN2=0
        leftMotorIn1Timer->setCaptureCompare(leftMotorIn1Channel, speed, PERCENT_COMPARE_FORMAT);
    }
    else if (direction < 0)
    {
        // Reverse: IN1=0, IN2=PWM
        leftMotorIn2Timer->setCaptureCompare(leftMotorIn2Channel, speed, PERCENT_COMPARE_FORMAT);
    }
}

// Safety function to set right motor speed with direction
// direction: 1 for forward, -1 for reverse, 0 for stop
void setRightMotorSpeed(int direction, uint32_t speed)
{
    // First, stop the motor completely to prevent shoot-through
    rightMotorIn1Timer->setCaptureCompare(rightMotorIn1Channel, 0, PERCENT_COMPARE_FORMAT);
    rightMotorIn2Timer->setCaptureCompare(rightMotorIn2Channel, 0, PERCENT_COMPARE_FORMAT);

    // Wait for the dead time
    delayMicroseconds(DEAD_TIME_US);

    // Now set the new direction and speed
    if (direction > 0)
    {
        // Forward: IN1=PWM, IN2=0
        rightMotorIn1Timer->setCaptureCompare(rightMotorIn1Channel, speed, PERCENT_COMPARE_FORMAT);
    }
    else if (direction < 0)
    {
        // Reverse: IN1=0, IN2=PWM
        rightMotorIn2Timer->setCaptureCompare(rightMotorIn2Channel, speed, PERCENT_COMPARE_FORMAT);
    }
}

// Low power mode functions
void enterLowPowerMode()
{
    // Put MPU6050 in cycle mode (low power mode)
    Wire.beginTransmission(MPU6050);
    Wire.write(0x6B); // Power management register 1
    Wire.write(0x21); // Set cycle bit and sleep bit, clock source = gyro X
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU6050);
    Wire.write(0x6C); // Power management register 2
    Wire.write(0x3F); // Set all accelerometer and gyroscope axes to standby mode
    Wire.endTransmission(true);

    // STM32 low power mode - this is a simple implementation
    // For more aggressive power saving, you could use the STM32 sleep modes
    delay(10); // Small delay to ensure I2C transactions complete
}

void exitLowPowerMode()
{
    // Wake up MPU6050
    Wire.beginTransmission(MPU6050);
    Wire.write(0x6B); // Power management register 1
    Wire.write(0x01); // Clear sleep bit, keep clock source as gyro X
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU6050);
    Wire.write(0x6C); // Power management register 2
    Wire.write(0x00); // Set all accelerometer and gyroscope axes to active mode
    Wire.endTransmission(true);

    delay(10); // Small delay to ensure I2C transactions complete
}
