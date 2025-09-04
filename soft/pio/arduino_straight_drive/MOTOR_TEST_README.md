# Motor PWM Threshold Testing

This folder contains separate test programs to find the minimum PWM values where your DRV8871 motors respond reliably.

## Programs

### 1. Motor Test Receiver (`motor_test_main.cpp`)
- Runs on STM32F411 (BlackPill) with DRV8871 motor drivers
- Receives PWM commands via nRF24L01+ or serial
- **NEW**: Includes IMU and PID for straight driving tests
- Direct motor control AND PID-controlled straight driving

### 2. Motor Test Sender (`motor_test_sender_main.cpp`)
- Runs on Arduino Uno with nRF24L01+
- Sends PWM commands wirelessly to the receiver
- Simple serial interface for testing

## Building

```bash
# Build receiver (STM32F411)
platformio run --environment motor_test

# Build sender (Arduino Uno)
platformio run --environment motor_test_sender

# Upload receiver
platformio run --environment motor_test --target upload

# Upload sender
platformio run --environment motor_test_sender --target upload
```

## Usage

### Commands (both serial and radio)

- `L<value>` - Set left motor PWM (-255 to +255)
- `R<value>` - Set right motor PWM (-255 to +255)
- `B<value>` - Set both motors PWM (-255 to +255)
- `S` - Stop all motors
- **`F<value>` - Straight driving with IMU+PID (NEW)**

### Examples
```
L50     # Left motor forward at PWM 50
R-30    # Right motor reverse at PWM 30
B100    # Both motors forward at PWM 100
F75     # Straight driving with PID at PWM 75 (NEW)
S       # Stop all motors
```

## Testing Procedure

1. Upload receiver firmware to STM32F411
2. Upload sender firmware to Arduino Uno
3. Connect motors to DRV8871 drivers on STM32F411
4. Open serial monitor on sender (Arduino Uno) at 115200 baud
5. Start with low values and work up:

```
S       # Start with motors stopped
L10     # Try left motor at PWM 10
L20     # Increase if no movement
L30     # Continue until motor starts reliably
L40
L50
...
```

6. Test both directions:
```
L50     # Forward
L-50    # Reverse
```

7. Repeat for right motor and both motors
8. Note the minimum reliable PWM values for your setup

## Straight Driving with PID (NEW)

9. Test PID straight driving (requires IMU):
```
F50     # Straight driving at PWM 50 with PID correction
F75     # Try higher speeds
F100    # Test at different PWM levels
S       # Stop PID driving
```

10. Observe how PID keeps the robot driving straight
11. Compare with direct `B<value>` commands (no correction)
12. Note minimum PWM where PID can effectively correct

## Expected Results

- Motors may not respond below ~20-40 PWM due to driver deadband
- The minimum value where motors start consistently is your `MIN_EFFECTIVE_DUTY`
- **PID straight driving requires higher minimum PWM for effective correction**
- Compare direct control vs PID-corrected behavior
- Use these values in the main car_main.cpp program

## Hardware Connections

### STM32F411 (Receiver)
- Motor pins defined in `pin_definitions.h`
- nRF24L01+: CE=PB0, CSN=PB1, SPI on default pins
- **MPU6050 IMU: I2C on default pins (SCL/SDA)**
- LED on PC13 for status

### Arduino Uno (Sender)
- nRF24L01+: CE=9, CSN=10, SPI on default pins
- Serial communication at 115200 baud

## Notes

- IMU initialization included for straight driving tests
- **PID control available with 'F' command for straight driving**
- Direct motor control still available (L, R, B commands)
- Radio settings match main program (channel 108, 250kbps)
- Can also test via serial directly on receiver if radio issues
- **PID parameters: Kp=1.2, Ki=0.02, Kd=0.10 (same as main program)**
