#include <STM32LowPower.h> // Low-power library for STM32

// Motor 1 Pins (PWM Control)
#define MOTOR1_PWM_A PB10 // Motor 1 Channel A
#define MOTOR1_PWM_B PB11 // Motor 1 Channel B

// Motor 2 Pins (PWM Control)
#define MOTOR2_PWM_A PB0 // Motor 2 Channel A
#define MOTOR2_PWM_B PB1 // Motor 2 Channel B

// Speed Sensor Pins
#define MOTOR1_SENSOR_PIN PA0 // Motor 1 Optical Speed Sensor
#define MOTOR2_SENSOR_PIN PA1 // Motor 2 Optical Speed Sensor

// Buzzer Pin
#define BUZZER_PIN PA8

// Encoder Variables
volatile long motor1Pulses = 0;
volatile long motor2Pulses = 0;

// Motor Control Variables
int motor1Speed = 50; // Default motor 1 speed (0-100%)
int motor2Speed = 50; // Default motor 2 speed (0-100%)

// Interrupt Service Routine for Motor 1 Sensor
void handleMotor1Sensor()
{
    motor1Pulses++;
}

// Interrupt Service Routine for Motor 2 Sensor
void handleMotor2Sensor()
{
    motor2Pulses++;
}

// Function to stop motors
void stopMotors()
{
    analogWrite(MOTOR1_PWM_A, 0);
    analogWrite(MOTOR1_PWM_B, 0);
    analogWrite(MOTOR2_PWM_A, 0);
    analogWrite(MOTOR2_PWM_B, 0);
}

// Function to initialize motors
void setupMotors()
{
    // Set motor pins as outputs (no need to specify PWM mode for STM32)
    pinMode(MOTOR1_PWM_A, OUTPUT);
    pinMode(MOTOR1_PWM_B, OUTPUT);
    pinMode(MOTOR2_PWM_A, OUTPUT);
    pinMode(MOTOR2_PWM_B, OUTPUT);

    // Stop motors initially
    stopMotors();
}

// Function to initialize speed sensors
void setupSpeedSensors()
{
    // Set speed sensor pins as inputs
    pinMode(MOTOR1_SENSOR_PIN, INPUT_PULLUP);
    pinMode(MOTOR2_SENSOR_PIN, INPUT_PULLUP);

    // Attach interrupts for speed sensors
    attachInterrupt(digitalPinToInterrupt(MOTOR1_SENSOR_PIN),
                    handleMotor1Sensor, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR2_SENSOR_PIN),
                    handleMotor2Sensor, RISING);
}

// Function to initialize buzzer
void setupBuzzer()
{
    // Set buzzer pin as PWM output
    pinMode(BUZZER_PIN, OUTPUT);
    analogWrite(BUZZER_PIN, 0); // Turn off buzzer initially
}



// Function to move motor 1 forward
void moveMotor1Forward(int speed)
{
    analogWrite(MOTOR1_PWM_A, map(speed, 0, 100, 0, 255));
    analogWrite(MOTOR1_PWM_B, 0);
}

// Function to move motor 1 backward
void moveMotor1Backward(int speed)
{
    analogWrite(MOTOR1_PWM_A, 0);
    analogWrite(MOTOR1_PWM_B, map(speed, 0, 100, 0, 255));
}

// Function to move motor 2 forward
void moveMotor2Forward(int speed)
{
    analogWrite(MOTOR2_PWM_A, map(speed, 0, 100, 0, 255));
    analogWrite(MOTOR2_PWM_B, 0);
}

// Function to move motor 2 backward
void moveMotor2Backward(int speed)
{
    analogWrite(MOTOR2_PWM_A, 0);
    analogWrite(MOTOR2_PWM_B, map(speed, 0, 100, 0, 255));
}

// Function to control buzzer with intensity
void controlBuzzer(int intensity)
{
    analogWrite(BUZZER_PIN, map(intensity, 0, 100, 0, 255));
}

void setup()
{
    Serial.begin(115200); // Initialize Serial for debugging

    // Initialize motors, sensors, and buzzer
    setupMotors();
    setupSpeedSensors();
    setupBuzzer();

    // Initialize Low-Power Mode
    LowPower.begin();

    Serial.println("System Initialized.");
}

void loop()
{
    // Example Usage: Control motors and buzzer

    // Move Motor 1 forward at 60% speed
    moveMotor1Forward(60);

    // Move Motor 2 backward at 40% speed
    moveMotor2Backward(40);

    // Wait for a while
    delay(2000);

    // Stop both motors
    stopMotors();

    // Turn on the buzzer at 50% intensity
    controlBuzzer(0);

    // Wait for a while
    delay(1000);

    // Turn off the buzzer
    controlBuzzer(0);

    // Print encoder values for debugging
    Serial.print("Motor 1 Pulses: ");
    Serial.println(motor1Pulses);
    Serial.print("Motor 2 Pulses: ");
    Serial.println(motor2Pulses);

    // Reset pulse counters
    motor1Pulses = 0;
    motor2Pulses = 0;

    // Enter low-power mode
//     LowPower.deepSleep();
}