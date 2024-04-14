#include <Arduino.h>

// #define TEST_FWD_BWD 0
/* motor 1 is on the left with speed sensors down. */
#define DC_MOTOR_PWM_FREQ 25 /* 30 hz PWM frequency. */

// A=1 B=0  Motor 1 FWD
// A=0 B=1  Motor 1 BWD
// C=1 D=0  Motor 2 FWD
// C=0 D=1  Motor 2 BWD
#define DUTY_A 19
#define DUTY_B 0
#define DUTY_C 15
#define DUTY_D 0

enum motor {
    MOTOR_1 = 0,
    MOTOR_2 = 1,
};

enum direction {
    FORDWARD = 0,
    BACKWARD = 1,
};

const uint8_t m1_sensor_pin = PA1;
const uint8_t m2_sensor_pin = PA0;
const uint8_t buzzer_pin    = PA8;

const uint8_t pin_M1_FWD = PB10; // orange  TIM2_CH3
const uint8_t pin_M1_BWD = PB11; // green  TIM2_CH4
const uint8_t pin_M2_FWD = PB1;  // violet  TIM3_CH4
const uint8_t pin_M2_BWD = PB0;  // brown  TIM3_CH3

const uint32_t channel_M1_FWD =
STM_PIN_CHANNEL (pinmap_function (digitalPinToPinName (pin_M1_FWD), PinMap_PWM));
const uint32_t channel_M1_BWD =
STM_PIN_CHANNEL (pinmap_function (digitalPinToPinName (pin_M1_BWD), PinMap_PWM));
const uint32_t channel_M2_FWD =
STM_PIN_CHANNEL (pinmap_function (digitalPinToPinName (pin_M2_FWD), PinMap_PWM));
const uint32_t channel_M2_BWD =
STM_PIN_CHANNEL (pinmap_function (digitalPinToPinName (pin_M2_BWD), PinMap_PWM));

TIM_TypeDef* Timer2 =
(TIM_TypeDef*)pinmap_peripheral (digitalPinToPinName (pin_M1_FWD), PinMap_PWM);
HardwareTimer* m1_timer = new HardwareTimer (Timer2);

TIM_TypeDef* Timer3 =
(TIM_TypeDef*)pinmap_peripheral (digitalPinToPinName (pin_M2_FWD), PinMap_PWM);
HardwareTimer* m2_timer = new HardwareTimer (Timer3);
/* Constant for steps in disk. */
const float stepcount = 20.00; // 20 Slots in disk.
/* Constant for wheel diameter. */
const float wheeldiameter = 67.0; // Wheel diameter in millimeters.

volatile uint32_t m1_counter = 0;
volatile uint32_t m2_counter = 0;

static void setup_pwm_timer (void) {
    m1_timer->setPWM (channel_M1_FWD, pin_M1_FWD, DC_MOTOR_PWM_FREQ, 0);
    m1_timer->setPWM (channel_M1_BWD, pin_M1_BWD, DC_MOTOR_PWM_FREQ, 0);
    m2_timer->setPWM (channel_M2_FWD, pin_M2_FWD, DC_MOTOR_PWM_FREQ, 0);
    m2_timer->setPWM (channel_M2_BWD, pin_M2_BWD, DC_MOTOR_PWM_FREQ, 0);
}

void m1_speed_irq (void) {
    ++m1_counter;
}

void m2_speed_irq (void) {
    ++m2_counter;
}

static void setMotor (enum motor motor, enum direction direction, uint32_t speed) {
    /* wait for 100ms */
    if (motor == MOTOR_1) {
        m1_timer->setCaptureCompare (channel_M1_FWD, 0, PERCENT_COMPARE_FORMAT);
        m1_timer->setCaptureCompare (channel_M1_BWD, 0, PERCENT_COMPARE_FORMAT);
        delay (300);
        switch (direction) {
        case FORDWARD:
            m1_timer->setCaptureCompare (channel_M1_FWD, speed, PERCENT_COMPARE_FORMAT);
            break;
        case BACKWARD:
            m1_timer->setCaptureCompare (channel_M1_BWD, speed, PERCENT_COMPARE_FORMAT);
            break;
        }
    } else {
        m2_timer->setCaptureCompare (channel_M2_FWD, 0, PERCENT_COMPARE_FORMAT);
        m2_timer->setCaptureCompare (channel_M2_BWD, 0, PERCENT_COMPARE_FORMAT);
        delay (300);
        switch (direction) {
        case FORDWARD:
            m2_timer->setCaptureCompare (channel_M2_FWD, speed, PERCENT_COMPARE_FORMAT);
            break;
        case BACKWARD:
            m2_timer->setCaptureCompare (channel_M2_BWD, speed, PERCENT_COMPARE_FORMAT);
            break;
        }
    }
}
// Function to convert from centimeters to steps
static uint32_t centimeters_to_step (float cm) {

    float circumference = (wheeldiameter * 3.14) / 10; // Calculate wheel circumference in cm
    float cm_step  = circumference / stepcount; // CM per Step
    float f_result = cm / cm_step;              // Calculate result as a float

    return (uint32_t)f_result; // Convert to an integer (note this is NOT rounded)
}

static void MoveForward (int steps, int mspeed) {
    m1_counter = 0; //  reset counter A to zero
    m2_counter = 0; //  reset counter B to zero

    setMotor (MOTOR_1, FORDWARD, mspeed);
    setMotor (MOTOR_2, FORDWARD, mspeed);

    // Go forward until step value is reached
    //       while (steps > m1_counter && steps > m2_counter) {
    while (steps > m1_counter || steps > m2_counter)
        ;
    setMotor (MOTOR_1, FORDWARD, 0);
    setMotor (MOTOR_2, FORDWARD, 0);
}

// Function to Move in Reverse
static void MoveReverse (int steps, int mspeed) {
    m1_counter = 0; //  reset counter A to zero
    m2_counter = 0; //  reset counter B to zero

    setMotor (MOTOR_1, BACKWARD, mspeed);
    setMotor (MOTOR_2, BACKWARD, mspeed);

    // Go in reverse until step value is reached
    while (steps > m1_counter && steps > m2_counter)
        ;

    setMotor (MOTOR_1, FORDWARD, 0);
    setMotor (MOTOR_2, FORDWARD, 0);
}

// Function to Spin Right
static void SpinRight (int steps, int mspeed) {
    m1_counter = 0; //  reset counter A to zero
    m2_counter = 0; //  reset counter B to zero

    setMotor (MOTOR_1, FORDWARD, mspeed);
    setMotor (MOTOR_2, BACKWARD, mspeed);

    // Go until step value is reached
    while (steps > m1_counter && steps > m2_counter)
        ;
    // Stop when done
    setMotor (MOTOR_1, FORDWARD, 0);
    setMotor (MOTOR_2, FORDWARD, 0);
}

// Function to Spin Left
static void SpinLeft (int steps, int mspeed) {
    m1_counter = 0; //  reset counter A to zero
    m2_counter = 0; //  reset counter B to zero

    // Set Motor A forward
    // Set Motor B reverse
    setMotor (MOTOR_1, BACKWARD, mspeed);
    setMotor (MOTOR_2, FORDWARD, mspeed);

    // Go until step value is reached
    while (steps > m1_counter && steps > m2_counter)
        ;

    // Stop when done
    setMotor (MOTOR_1, FORDWARD, 0);
    setMotor (MOTOR_2, FORDWARD, 0);
}

void set_interrupt_speed_sensors (void) {
    uint8_t m1_irq_mode = 0;
    uint8_t m2_irq_mode = 0;
    pinMode (m1_sensor_pin, INPUT);
    pinMode (m2_sensor_pin, INPUT);

    if (digitalRead (m1_sensor_pin) == HIGH)
        /* Detect falling pulses. */
        m1_irq_mode = FALLING;
    else
        m1_irq_mode = RISING;

    if (digitalRead (m2_sensor_pin) == HIGH)
        /* Detect falling pulses. */
        m2_irq_mode = FALLING;
    else
        m2_irq_mode = RISING;

    attachInterrupt (digitalPinToInterrupt (m1_sensor_pin), m1_speed_irq, m1_irq_mode);
    attachInterrupt (digitalPinToInterrupt (m2_sensor_pin), m2_speed_irq, m2_irq_mode);
}

#define INIT_STATE LOW
void setup () {
    //     Serial.begin ();
    //     pinMode (pin_M1_FWD, OUTPUT);
    //     pinMode (pin_M1_BWD, OUTPUT);
    //     pinMode (pin_M2_FWD, OUTPUT);
    //     pinMode (pin_M2_BWD, OUTPUT);
    //     digitalWrite (pin_M1_FWD, INIT_STATE);
    //     digitalWrite (pin_M1_BWD, INIT_STATE);
    //     digitalWrite (pin_M2_FWD, INIT_STATE);
    //     digitalWrite (pin_M2_BWD, INIT_STATE);

    setup_pwm_timer ();
    set_interrupt_speed_sensors ();
}

void loop () {

    // Test Motor Movement  - Experiment with your own sequences here
    MoveForward (2, 50);
    //     analogWriteFrequency (25);
    //     analogWriteResolution (16);
    //     analogWrite (pin_M2_FWD, 10);
    //     analogWrite (pin_M1_FWD, 10);
    delay (10000);
    MoveReverse (2, 50);
    delay (10000);
    MoveForward (2, 50);
    delay (10000);
    MoveReverse (2, 50);
    delay (10000);
    SpinRight (2, 50);
    delay (10000);
    SpinLeft (2, 50);
    while (1)
        ;
}
