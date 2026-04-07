#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <stdint.h>  // For uint8_t type
#include "pin_definitions.h"


// nRF24L01 registers
#define NRF24_REG_CONFIG      0x00
#define NRF24_REG_STATUS      0x07
#define NRF24_CMD_READ_REG    0x00
#define NRF24_CMD_WRITE_REG   0x20

// MPU6050 registers
#define MPU6050_WHO_AM_I 0x75
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

// Function prototypes
void togglePins();
bool testMPU6050();
void readMPU6050Data();
bool testNRF24L01();
uint8_t readNRF24Register(uint8_t reg);
void writeNRF24Register(uint8_t reg, uint8_t value);

// Global variables
unsigned long lastToggleTime = 0;
unsigned long lastI2CTime = 0;
unsigned long lastSPITime = 0;
const int toggleInterval = 500; // Toggle every 500ms
const int i2cReadInterval = 1000; // Read I2C every 1000ms
const int spiReadInterval = 2000; // Read SPI every 2000ms
bool pinState = false;

void setup()
{
    uint8_t counter = 0;

    // Initialize serial communication
    Serial.begin(115200);
    Serial.println("GPIO, I2C and SPI Test Program Starting");

    // Initialize GPIO pins as outputs
    pinMode(PIN_A_MOTOR_RIGHT, OUTPUT);
    pinMode(PIN_B_MOTOR_RIGHT, OUTPUT);
    pinMode(PIN_A_MOTOR_LEFT, OUTPUT);
    pinMode(PIN_B_MOTOR_LEFT, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    pinMode(NRF24_CE, OUTPUT);
    pinMode(NRF24_CSN, OUTPUT);

    //toggling gpios for some loops



    // Initialize all pins to LOW/inactive state
    digitalWrite(PIN_A_MOTOR_RIGHT, LOW);
    digitalWrite(PIN_B_MOTOR_RIGHT, LOW);
    digitalWrite(PIN_A_MOTOR_LEFT, LOW);
    digitalWrite(PIN_B_MOTOR_LEFT, LOW);
    digitalWrite(PIN_LED, HIGH); // disable LED on PC13


    digitalWrite(NRF24_CE, LOW);
    digitalWrite(NRF24_CSN, HIGH);  // CSN is active LOW

    // Initialize I2C communication
    Wire.begin();
    // Initialize SPI for nRF24L01
    SPI.begin();
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);
    SPI.setClockDivider(SPI_CLOCK_DIV8);  // 2MHz SPI clock (assuming 16MHz MCU clock)

    // Test MPU6050 connection
    if (testMPU6050()) {
        Serial.println("MPU6050 found and initialized successfully!");
    } else {
        Serial.println("Failed to connect to MPU6050!");
        // Blink rapidly to indicate error
        while (1) {
            digitalWrite(PIN_LED, HIGH);
            delay(500);
            digitalWrite(PIN_LED, LOW);
            delay(500);
        }
    }

    // Test nRF24L01 connection
    if (testNRF24L01()) {
        Serial.println("nRF24L01 found and initialized successfully!");
    } else {
        Serial.println("Failed to connect to nRF24L01!");
        // Blink slowly on PB11 to indicate error
        while (1) {
            digitalWrite(PIN_LED, HIGH);
            delay(2000);
            digitalWrite(PIN_LED, LOW);
            delay(2000);
        }
    }
}

void loop()
{
    unsigned long currentMillis = millis();

    // Toggle GPIO pins at defined interval
    if (currentMillis - lastToggleTime >= toggleInterval) {
        lastToggleTime = currentMillis;
        togglePins();
    }

    // Read MPU6050 data at defined interval
    if (currentMillis - lastI2CTime >= i2cReadInterval) {
        lastI2CTime = currentMillis;
        readMPU6050Data();
    }

    // Check nRF24L01 status at defined interval
    if (currentMillis - lastSPITime >= spiReadInterval) {
        lastSPITime = currentMillis;
        // Read the STATUS register from nRF24L01
        uint8_t status = readNRF24Register(NRF24_REG_STATUS);
        Serial.print("nRF24L01 STATUS: 0x");
        Serial.println(status, HEX);
    }
}

void togglePins()
{
    // Toggle pins in different patterns for easy identification on logic analyzer

    // Toggle state for all pins
    pinState = !pinState;

    // PB10 - simple toggle (same as pinState)
    digitalWrite(PIN_A_MOTOR_RIGHT, pinState);

    // PB11 - opposite of pinState
    digitalWrite(PIN_B_MOTOR_RIGHT, !pinState);

    // PB14 - toggle only every other time (double frequency)
    static bool pb14State = false;
    pb14State = !pb14State;
    digitalWrite(PIN_A_MOTOR_LEFT, pb14State);

    // PB15 - pulse pattern (short HIGH pulse then LOW)
    if (pinState) {
        digitalWrite(PIN_B_MOTOR_LEFT, HIGH);
        delay(200); // 50ms pulse
        digitalWrite(PIN_B_MOTOR_LEFT, LOW);
    }
}

bool testMPU6050()
{
    // Test the MPU6050 by reading the WHO_AM_I register
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 1, true);

    if (Wire.available()) {
        uint8_t whoAmI = Wire.read();
        Serial.print("WHO_AM_I register: 0x");
        Serial.println(whoAmI, HEX);

        // WHO_AM_I should return 0x68 for MPU6050
        if (whoAmI == 0x68) {
            // Initialize MPU6050
            Wire.beginTransmission(MPU6050_ADDR);
            Wire.write(MPU6050_PWR_MGMT_1);
            Wire.write(0x00); // Wake up the MPU6050 (clear sleep bit)
            Wire.endTransmission(true);
            return true;
        }
    }

    return false;
}

void readMPU6050Data()
{
    // Signal on PB10 that we're starting I2C communication
    digitalWrite(PIN_A_MOTOR_RIGHT, HIGH);

    Serial.println("Reading MPU6050 data...");

    // Read accelerometer data
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_XOUT_H);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 6, true);

    int16_t accelX = Wire.read() << 8 | Wire.read();
    int16_t accelY = Wire.read() << 8 | Wire.read();
    int16_t accelZ = Wire.read() << 8 | Wire.read();

    // Print accelerometer data
    Serial.print("Accelerometer: X=");
    Serial.print(accelX);
    Serial.print(", Y=");
    Serial.print(accelY);
    Serial.print(", Z=");
    Serial.println(accelZ);

    // Signal on PB10 that we're done with I2C communication
    digitalWrite(PIN_A_MOTOR_RIGHT, LOW);
}

// Helper function to read a register from nRF24L01
uint8_t readNRF24Register(uint8_t reg)
{
    digitalWrite(NRF24_CSN, LOW);
    SPI.transfer(NRF24_CMD_READ_REG | reg);
    uint8_t value = SPI.transfer(0xFF);  // Send dummy byte to read value
    digitalWrite(NRF24_CSN, HIGH);
    return value;
}

// Helper function to write to a register of nRF24L01
void writeNRF24Register(uint8_t reg, uint8_t value)
{
    digitalWrite(NRF24_CSN, LOW);
    SPI.transfer(NRF24_CMD_WRITE_REG | reg);
    SPI.transfer(value);
    digitalWrite(NRF24_CSN, HIGH);
}

// Test the nRF24L01 by reading the CONFIG register
bool testNRF24L01()
{
    // Signal that we're starting SPI communication
    digitalWrite(PIN_B_MOTOR_RIGHT, HIGH);

    Serial.println("Testing nRF24L01...");

    // Try reading the CONFIG register (default value should be 0x08)
    uint8_t config = readNRF24Register(NRF24_REG_CONFIG);
    Serial.print("CONFIG register: 0x");
    Serial.println(config, HEX);

    // Check if the CONFIG register has a valid value
    // Default power-on reset value for CONFIG is 0x08
    // We'll also accept other values since the module might have been
    // previously configured differently
    bool isValid = (config != 0xFF && config != 0x00);

    if (isValid) {
        // If we got a valid response, try writing to the CONFIG register and read it back
        uint8_t testValue = config ^ 0x02;  // Toggle one bit
        writeNRF24Register(NRF24_REG_CONFIG, testValue);

        // Read it back
        uint8_t readValue = readNRF24Register(NRF24_REG_CONFIG);
        Serial.print("CONFIG after write: 0x");
        Serial.println(readValue, HEX);

        // Restore original value
        writeNRF24Register(NRF24_REG_CONFIG, config);

        // Check if write worked
        isValid = (readValue == testValue);
    }

    // Signal that we're done with SPI communication
    digitalWrite(PIN_B_MOTOR_RIGHT, LOW);

    return isValid;
}
