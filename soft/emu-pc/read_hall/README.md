# Read Hall - Hall Effect Sensor Module

Module for reading hall effect sensors, typically used for wheel speed measurement (odometry) in motor control applications.

## Overview

The Read Hall module provides:
- Hall effect sensor reading for wheel encoders
- Rotation counting and speed measurement
- Distance calculation based on wheel parameters
- Interrupt-based or polling-based reading

## Purpose

Hall effect sensors are used in the Gaspetto Car for:
- **Wheel Speed Measurement**: Count wheel rotations
- **Odometry**: Calculate distance traveled
- **Speed Control**: Closed-loop motor control
- **Position Tracking**: Navigate to specific locations

## Structure

```
read_hall/
├── include/
│   └── ReadHall.h    # Hall sensor interface
└── src/
    └── ReadHall.cpp  # Implementation
```

## Hardware

### Hall Effect Sensor

Common types:
- **Digital Hall Sensor**: Outputs HIGH/LOW based on magnetic field
- **Hall Encoder**: Multiple pulses per rotation (e.g., 20 PPR)
- **Analog Hall Sensor**: Outputs voltage proportional to field strength

### Typical Setup

```
Wheel → Magnet(s) → Hall Sensor → Microcontroller GPIO
```

For differential drive robot:
- Left wheel hall sensor → GPIO pin (e.g., PA6)
- Right wheel hall sensor → GPIO pin (e.g., PA7)

## API Reference

### Initialization

```cpp
class ReadHall {
public:
    ReadHall(uint8_t pin);
    void init();
    void attachInterrupt(void (*callback)());
};
```

**Example:**
```cpp
ReadHall leftWheelEncoder(LEFT_HALL_PIN);
ReadHall rightWheelEncoder(RIGHT_HALL_PIN);

void leftWheelISR() {
    leftWheelEncoder.onPulse();
}

void setup() {
    leftWheelEncoder.init();
    leftWheelEncoder.attachInterrupt(leftWheelISR);
}
```

### Reading Values

```cpp
// Get pulse count
uint32_t getPulseCount();

// Get rotations
float getRotations();

// Get distance (requires wheel configuration)
float getDistance();

// Get speed (pulses per second)
float getSpeed();

// Reset counter
void reset();
```

## Usage Examples

### Basic Pulse Counting

```cpp
ReadHall encoder(HALL_PIN);

void setup() {
    encoder.init();
}

void loop() {
    uint32_t pulses = encoder.getPulseCount();
    Serial.print("Pulses: ");
    Serial.println(pulses);
    delay(1000);
}
```

### Distance Measurement

```cpp
// Configuration
const float WHEEL_DIAMETER_CM = 6.5;
const float PULSES_PER_ROTATION = 20;
const float CM_PER_PULSE = (PI * WHEEL_DIAMETER_CM) / PULSES_PER_ROTATION;

ReadHall encoder(HALL_PIN);

float getDistanceTraveled() {
    uint32_t pulses = encoder.getPulseCount();
    return pulses * CM_PER_PULSE;
}

void loop() {
    float distance = getDistanceTraveled();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    delay(1000);
}
```

### Speed Measurement

```cpp
ReadHall encoder(HALL_PIN);
unsigned long lastTime = 0;
uint32_t lastPulses = 0;

void loop() {
    unsigned long now = millis();
    if (now - lastTime >= 100) {  // Update every 100ms
        uint32_t currentPulses = encoder.getPulseCount();
        uint32_t deltaPulses = currentPulses - lastPulses;
        float deltaTime = (now - lastTime) / 1000.0;  // seconds

        float pulsesPerSecond = deltaPulses / deltaTime;
        float rpm = (pulsesPerSecond / PULSES_PER_ROTATION) * 60;

        Serial.print("Speed: ");
        Serial.print(rpm);
        Serial.println(" RPM");

        lastPulses = currentPulses;
        lastTime = now;
    }
}
```

### Interrupt-Based Reading

```cpp
volatile uint32_t pulseCount = 0;
ReadHall encoder(HALL_PIN);

void hallISR() {
    pulseCount++;
}

void setup() {
    encoder.init();
    attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, RISING);
}

void loop() {
    Serial.println(pulseCount);
    delay(1000);
}
```

## Wheel Configuration

### Calculating Pulses Per Rotation

For magnetic encoders with multiple magnets:
```cpp
const uint8_t MAGNETS_PER_WHEEL = 4;
const uint8_t PULSES_PER_ROTATION = MAGNETS_PER_WHEEL;
```

For hall encoders with encoder disk:
```cpp
const uint8_t ENCODER_SLOTS = 20;
const uint8_t PULSES_PER_ROTATION = ENCODER_SLOTS;
```

### Calculating Distance Per Pulse

```cpp
const float WHEEL_DIAMETER_CM = 6.5;
const float WHEEL_CIRCUMFERENCE_CM = PI * WHEEL_DIAMETER_CM;
const float CM_PER_PULSE = WHEEL_CIRCUMFERENCE_CM / PULSES_PER_ROTATION;
```

## Differential Drive Odometry

### Position Tracking

```cpp
class Odometry {
private:
    ReadHall leftEncoder;
    ReadHall rightEncoder;
    float x, y, theta;  // Robot pose

public:
    void update() {
        float leftDist = leftEncoder.getDistance();
        float rightDist = rightEncoder.getDistance();

        // Calculate movement
        float distance = (leftDist + rightDist) / 2.0;
        float deltaTheta = (rightDist - leftDist) / WHEEL_BASE_CM;

        // Update pose
        x += distance * cos(theta + deltaTheta/2);
        y += distance * sin(theta + deltaTheta/2);
        theta += deltaTheta;

        // Reset encoders
        leftEncoder.reset();
        rightEncoder.reset();
    }

    float getX() { return x; }
    float getY() { return y; }
    float getTheta() { return theta; }
};
```

### Navigation

```cpp
void moveToPosition(float targetX, float targetY) {
    Odometry odom;

    while (true) {
        odom.update();

        float dx = targetX - odom.getX();
        float dy = targetY - odom.getY();
        float distance = sqrt(dx*dx + dy*dy);

        if (distance < 5.0) {  // Within 5cm
            motors.stop();
            break;
        }

        float targetAngle = atan2(dy, dx);
        float angleError = targetAngle - odom.getTheta();

        // Simple proportional control
        motors.setSpeeds(50 + angleError*10, 50 - angleError*10);

        delay(50);
    }
}
```

## Closed-Loop Speed Control

### PID Speed Controller

```cpp
class SpeedController {
private:
    ReadHall encoder;
    float targetSpeed;
    float kp, ki, kd;
    float integral, lastError;

public:
    void setTargetSpeed(float speed) {
        targetSpeed = speed;
    }

    uint8_t update() {
        float currentSpeed = encoder.getSpeed();
        float error = targetSpeed - currentSpeed;

        integral += error;
        float derivative = error - lastError;

        float output = kp*error + ki*integral + kd*derivative;

        lastError = error;

        return constrain(output, 0, 100);
    }
};
```

## Calibration

### Finding Pulses Per Rotation

```cpp
void calibrateEncoder() {
    Serial.println("Rotate wheel exactly 10 times");
    Serial.println("Press button when done");

    uint32_t startPulses = encoder.getPulseCount();

    while (!buttonPressed()) {
        delay(100);
    }

    uint32_t endPulses = encoder.getPulseCount();
    uint32_t totalPulses = endPulses - startPulses;

    float pulsesPerRotation = totalPulses / 10.0;

    Serial.print("Pulses per rotation: ");
    Serial.println(pulsesPerRotation);
}
```

### Measuring Wheel Diameter

```cpp
void measureWheelDiameter() {
    Serial.println("Roll robot exactly 100cm");
    Serial.println("Press button when done");

    encoder.reset();

    while (!buttonPressed()) {
        delay(100);
    }

    uint32_t pulses = encoder.getPulseCount();
    float distanceCM = 100.0;

    float circumference = distanceCM / (pulses / PULSES_PER_ROTATION);
    float diameter = circumference / PI;

    Serial.print("Wheel diameter: ");
    Serial.print(diameter);
    Serial.println(" cm");
}
```

## Troubleshooting

### No Pulses Detected

**Possible causes:**
1. Hall sensor not powered
2. Magnet too far from sensor
3. Wrong pin configuration
4. Sensor polarity reversed

**Solutions:**
- Check 3.3V/5V supply to sensor
- Reduce sensor-to-magnet gap (< 5mm)
- Verify GPIO pin number
- Try swapping VCC/GND

### Erratic Readings

**Possible causes:**
1. Electrical noise from motors
2. Bouncing contacts
3. Loose connections

**Solutions:**
- Add 0.1µF capacitor across sensor pins
- Implement software debouncing
- Use shielded cables
- Separate sensor and motor power

### Speed Calculation Inaccurate

**Possible causes:**
1. Wrong pulses per rotation value
2. Timing measurement error
3. Integer overflow

**Solutions:**
- Calibrate encoder properly
- Use microsecond timing for fast rotation
- Use `unsigned long` for pulse count

## Integration with Movement Controller

```cpp
class MovementController {
private:
    MotorControl motors;
    ReadHall leftEncoder;
    ReadHall rightEncoder;
    SpeedController leftSpeedCtrl;
    SpeedController rightSpeedCtrl;

public:
    void setSpeed(float leftSpeed, float rightSpeed) {
        leftSpeedCtrl.setTargetSpeed(leftSpeed);
        rightSpeedCtrl.setTargetSpeed(rightSpeed);
    }

    void update() {
        // Read encoders
        float leftActual = leftEncoder.getSpeed();
        float rightActual = rightEncoder.getSpeed();

        // PID control
        uint8_t leftPWM = leftSpeedCtrl.update();
        uint8_t rightPWM = rightSpeedCtrl.update();

        // Apply to motors
        motors.setMotorSpeeds(true, leftPWM, true, rightPWM);
    }
};
```

## Testing

### Unit Test Example

```cpp
TEST(ReadHallTest, CountsPulses) {
    MockGPIO mockPin;
    ReadHall encoder(MOCK_PIN);

    encoder.init();

    // Simulate 10 pulses
    for (int i = 0; i < 10; i++) {
        mockPin.simulatePulse();
    }

    ASSERT_EQ(encoder.getPulseCount(), 10);
}
```

### Hardware Test

```cpp
void testEncoder() {
    encoder.reset();

    Serial.println("Rotate wheel 5 times...");
    delay(5000);

    uint32_t pulses = encoder.getPulseCount();
    float rotations = pulses / PULSES_PER_ROTATION;

    Serial.print("Measured rotations: ");
    Serial.println(rotations);

    if (abs(rotations - 5.0) < 0.5) {
        Serial.println("PASS");
    } else {
        Serial.println("FAIL - Check sensor alignment");
    }
}
```

## Performance

- **Interrupt-based**: Can handle > 10,000 pulses/second
- **Polling-based**: Limited by loop frequency (~1000 pulses/second)
- **Latency**: < 1ms with interrupts
- **Memory**: Minimal (few bytes per encoder)

## Best Practices

1. **Use interrupts** for high-speed or precision applications
2. **Debounce** if using mechanical encoders
3. **Calibrate** wheel parameters for accurate odometry
4. **Reset regularly** to prevent overflow (or use appropriate data types)
5. **Filter speed** measurements for smoother control

## References

- [Hall Effect Sensor Tutorial](https://www.electronicshub.org/hall-effect-sensor/)
- [Wheel Encoder Guide](https://www.societyofrobots.com/sensors_encoder.shtml)
- [Odometry for Mobile Robots](http://rossum.sourceforge.net/papers/DiffSteer/)
