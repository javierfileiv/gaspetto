#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <STM32LowPower.h> // Low-power library for STM32

// Pin Definitions
// Motor 1 Pins (PWM Control)
#define MOTOR1_PWM_A PB10 // Motor 1 Channel A
#define MOTOR1_PWM_B PB11 // Motor 1 Channel B

// Motor 2 Pins (PWM Control)
#define MOTOR2_PWM_A PB0  // Motor 2 Channel A
#define MOTOR2_PWM_B PB1  // Motor 2 Channel B

// Speed Sensor Pins
#define MOTOR1_SENSOR_PIN PA0 // Motor 1 Optical Speed Sensor
#define MOTOR2_SENSOR_PIN PA1 // Motor 2 Optical Speed Sensor

// Buzzer Pin
#define BUZZER_PIN PA8

// nRF24L01+ Pins
#define CE_PIN PA4
#define CSN_PIN PA3

// RF24 Object
RF24 radio(CE_PIN, CSN_PIN);

// IRQ Counter
volatile uint8_t irqCount = 0; // Counter to track IRQ events

// Encoder Variables
volatile long motor1Pulses = 0;
volatile long motor2Pulses = 0;

// Motor Control Variables
int motor1Speed = 50; // Default motor 1 speed (0-100%)
int motor2Speed = 50; // Default motor 2 speed (0-100%)

// Commands
#define CMD_FORWARD  "FWD"
#define CMD_BACKWARD "BWD"
#define CMD_LEFT     "LFT"
#define CMD_RIGHT    "RGT"
#define CMD_STOP     "STP"

// Interrupt Service Routine for nRF24 IRQ Pin
void onNrf24Interrupt() {
  irqCount++; // Increment the IRQ counter
}

// Interrupt Service Routine for Motor 1 Sensor
void handleMotor1Sensor() {
  motor1Pulses++;
}

// Interrupt Service Routine for Motor 2 Sensor
void handleMotor2Sensor() {
  motor2Pulses++;
}

// Function to initialize motors
void setupMotors() {
  // Set motor pins as PWM outputs
  pinMode(MOTOR1_PWM_A, PWM);
  pinMode(MOTOR1_PWM_B, PWM);
  pinMode(MOTOR2_PWM_A, PWM);
  pinMode(MOTOR2_PWM_B, PWM);

  // Stop motors initially
  stopMotors();
}

// Function to initialize speed sensors
void setupSpeedSensors() {
  // Set speed sensor pins as inputs
  pinMode(MOTOR1_SENSOR_PIN, INPUT_PULLUP);
  pinMode(MOTOR2_SENSOR_PIN, INPUT_PULLUP);

  // Attach interrupts for speed sensors
  attachInterrupt(digitalPinToInterrupt(MOTOR1_SENSOR_PIN), handleMotor1Sensor, RISING);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_SENSOR_PIN), handleMotor2Sensor, RISING);
}

// Function to initialize buzzer
void setupBuzzer() {
  // Set buzzer pin as PWM output
  pinMode(BUZZER_PIN, PWM);
  analogWrite(BUZZER_PIN, 0); // Turn off buzzer initially
}

// Function to stop motors
void stopMotors() {
  analogWrite(MOTOR1_PWM_A, 0);
  analogWrite(MOTOR1_PWM_B, 0);
  analogWrite(MOTOR2_PWM_A, 0);
  analogWrite(MOTOR2_PWM_B, 0);
}

// Function to move motor 1 forward
void moveMotor1Forward(int speed) {
  analogWrite(MOTOR1_PWM_A, map(speed, 0, 100, 0, 255));
  analogWrite(MOTOR1_PWM_B, 0);
}

// Function to move motor 1 backward
void moveMotor1Backward(int speed) {
  analogWrite(MOTOR1_PWM_A, 0);
  analogWrite(MOTOR1_PWM_B, map(speed, 0, 100, 0, 255));
}

// Function to move motor 2 forward
void moveMotor2Forward(int speed) {
  analogWrite(MOTOR2_PWM_A, map(speed, 0, 100, 0, 255));
  analogWrite(MOTOR2_PWM_B, 0);
}

// Function to move motor 2 backward
void moveMotor2Backward(int speed) {
  analogWrite(MOTOR2_PWM_A, 0);
  analogWrite(MOTOR2_PWM_B, map(speed, 0, 100, 0, 255));
}

// Function to control buzzer with intensity
void controlBuzzer(int intensity) {
  analogWrite(BUZZER_PIN, map(intensity, 0, 100, 0, 255));
}

void setup() {
  Serial.begin(115200); // Initialize Serial for debugging

  // Initialize nRF24L01+
  if (!radio.begin()) {
    Serial.println("nRF24L01+ module not detected. Check connections.");
    while (1);
  }
  radio.openReadingPipe(0, "00001"); // Set the communication address
  radio.setPALevel(RF24_PA_LOW);     // Set Power Amplifier level to low
  radio.setDataRate(RF24_1MBPS);     // Set data rate to 1Mbps
  radio.startListening();            // Set module to receive mode

  // Attach IRQ pin to interrupt
  pinMode(CSN_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CSN_PIN), onNrf24Interrupt, FALLING);

  // Initialize motors, sensors, and buzzer
  setupMotors();
  setupSpeedSensors();
  setupBuzzer();

  // Initialize Low-Power Mode
  LowPower.begin();

  Serial.println("System Initialized.");
}

void loop() {
  // Enter low-power mode if no IRQs are pending
  while (irqCount == 0) {
    LowPower.deepSleep();
  }

  // Process IRQs
  while (irqCount > 0) {
    noInterrupts(); // Temporarily disable interrupts to safely decrement irqCount
    irqCount--;     // Decrement the IRQ counter
    interrupts();   // Re-enable interrupts

    // Check the IRQ source
    uint8_t status = radio.get_status(); // Read the STATUS register
    Serial.print("STATUS Register: 0x");
    Serial.println(status, HEX);

    // Handle RX_DR (Data Received)
    if (status & _BV(RX_DR)) {
      Serial.println("Data Received (RX_DR)");
      char receivedCommand[32] = "";
      while (radio.available()) { // Check for available data
        radio.read(&receivedCommand, sizeof(receivedCommand));
        Serial.print("Received Command: ");
        Serial.println(receivedCommand);

        // Parse command and optional speed
        char command[4];
        int speed = -1;
        sscanf(receivedCommand, "%3s %d", command, &speed);

        // If a speed value is provided, update motor speeds
        if (speed >= 0 && speed <= 100) {
          motor1Speed = speed;
          motor2Speed = speed;
          Serial.print("Updated Motor Speed: ");
          Serial.println(speed);
        }

        // Process the command
        if (strcmp(command, CMD_FORWARD) == 0) {
          moveMotor1Forward(motor1Speed);
          moveMotor2Forward(motor2Speed);
        } else if (strcmp(command, CMD_BACKWARD) == 0) {
          moveMotor1Backward(motor1Speed);
          moveMotor2Backward(motor2Speed);
        } else if (strcmp(command, CMD_LEFT) == 0) {
          moveMotor1Backward(motor1Speed);
          moveMotor2Forward(motor2Speed);
        } else if (strcmp(command, CMD_RIGHT) == 0) {
          moveMotor1Forward(motor1Speed);
          moveMotor2Backward(motor2Speed);
        } else if (strcmp(command, CMD_STOP) == 0) {
          stopMotors();
        } else {
          Serial.println("Unknown Command.");
        }
      }

      // Clear RX_DR flag
      radio.write_register(NRF_STATUS, _BV(RX_DR));
    }
  }
}