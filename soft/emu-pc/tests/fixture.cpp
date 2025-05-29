#include "fixture.h"

using testing::NotNull;

void enter_low_power_mode(void)
{
    SwitchToLowPowerMode();
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Fixture::expect_car_init()
{
    expect_movement_controller_init();
    mock_RadioController.expect_radio_initialization();
    EXPECT_CALL(_mock_arduino, SwitchToLowPowerMode);
}

void Fixture::expect_movement_controller_init()
{
    expect_motor_control_init();
    /* Configure IRQ pins and set ISR. */
    EXPECT_CALL(_mock_arduino, pinMode(SPEED_SENSOR_LEFT_PIN, INPUT));
    EXPECT_CALL(_mock_arduino, pinMode(SPEED_SENSOR_RIGHT_PIN, INPUT));
    EXPECT_CALL(_mock_arduino, attachInterrupt(SPEED_SENSOR_LEFT_PIN, NotNull(), RISING));
    EXPECT_CALL(_mock_arduino, attachInterrupt(SPEED_SENSOR_RIGHT_PIN, NotNull(), RISING));
}

void Fixture::expect_motor_control_init()
{
    /* Configure pins and set PWM freq. */
    EXPECT_CALL(_mock_arduino, pinMode(MOTOR_LEFT_PIN_A, OUTPUT));
    EXPECT_CALL(_mock_arduino, pinMode(MOTOR_LEFT_PIN_B, OUTPUT));
    EXPECT_CALL(_mock_arduino, pinMode(MOTOR_RIGHT_PIN_A, OUTPUT));
    EXPECT_CALL(_mock_arduino, pinMode(MOTOR_RIGHT_PIN_B, OUTPUT));
    EXPECT_CALL(_mock_arduino, analogWriteFrequency(PWM_FREQ));
    /* Stop both motors. */
    EXPECT_CALL(_mock_arduino, digitalWrite(MOTOR_LEFT_PIN_A, LOW));
    EXPECT_CALL(_mock_arduino, digitalWrite(MOTOR_LEFT_PIN_B, LOW));
    EXPECT_CALL(_mock_arduino, digitalWrite(MOTOR_RIGHT_PIN_A, LOW));
    EXPECT_CALL(_mock_arduino, digitalWrite(MOTOR_RIGHT_PIN_B, LOW));
}

void Fixture::expect_enter_low_power_mode()
{
    EXPECT_CALL(_mock_arduino, SwitchToLowPowerMode);
}

void Fixture::expect_move_forward(uint32_t leftSpeed, uint32_t rightSpeed)
{
    uint32_t _leftPercent = map(leftSpeed, 0, 100, 0, 255);
    uint32_t _rightPercent = map(rightSpeed, 0, 100, 0, 255);

    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_LEFT_PIN_A, _leftPercent));
    EXPECT_CALL(_mock_arduino, digitalWrite(MOTOR_LEFT_PIN_B, 0));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_RIGHT_PIN_A, _rightPercent));
    EXPECT_CALL(_mock_arduino, digitalWrite(MOTOR_RIGHT_PIN_B, 0));
}
void Fixture::expect_move_backward(uint32_t leftSpeed, uint32_t rightSpeed)
{
    uint32_t _leftPercent = map(leftSpeed, 0, 100, 0, 255);
    uint32_t _rightPercent = map(rightSpeed, 0, 100, 0, 255);

    EXPECT_CALL(_mock_arduino, digitalWrite(MOTOR_LEFT_PIN_A, 0));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_LEFT_PIN_B, _leftPercent));
    EXPECT_CALL(_mock_arduino, digitalWrite(MOTOR_RIGHT_PIN_A, 0));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_RIGHT_PIN_B, _rightPercent));
}

void Fixture::expect_turn_right(uint32_t leftSpeed, uint32_t rightSpeed)
{
    uint32_t _leftPercent = map(leftSpeed, 0, 100, 0, 255);
    uint32_t _rightPercent = map(rightSpeed, 0, 100, 0, 255);

    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_LEFT_PIN_A, _leftPercent));
    EXPECT_CALL(_mock_arduino, digitalWrite(MOTOR_LEFT_PIN_B, 0));
    EXPECT_CALL(_mock_arduino, digitalWrite(MOTOR_RIGHT_PIN_A, 0));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_RIGHT_PIN_B, _rightPercent));
}

void Fixture::expect_turn_left(uint32_t leftSpeed, uint32_t rightSpeed)
{
    uint32_t _leftPercent = map(leftSpeed, 0, 100, 0, 255);
    uint32_t _rightPercent = map(rightSpeed, 0, 100, 0, 255);

    EXPECT_CALL(_mock_arduino, digitalWrite(MOTOR_LEFT_PIN_A, 0));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_LEFT_PIN_B, _leftPercent));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_RIGHT_PIN_A, _rightPercent));
    EXPECT_CALL(_mock_arduino, digitalWrite(MOTOR_RIGHT_PIN_B, 0));
}

void Fixture::expect_both_motors_stop()
{
    expect_stop_motor_left();
    expect_stop_motor_right();
}

void Fixture::expect_stop_motor_left()
{
    EXPECT_CALL(_mock_arduino, digitalWrite(MOTOR_LEFT_PIN_A, 0));
    EXPECT_CALL(_mock_arduino, digitalWrite(MOTOR_LEFT_PIN_B, 0));
}

void Fixture::expect_stop_motor_right()
{
    EXPECT_CALL(_mock_arduino, digitalWrite(MOTOR_RIGHT_PIN_A, 0));
    EXPECT_CALL(_mock_arduino, digitalWrite(MOTOR_RIGHT_PIN_B, 0));
}

void Fixture::expect_radio_process_event(Event *evt)
{
    mock_RadioController.expect_receive_event(evt);
}

/* Actions on Active Object. */
void Fixture::stop_car()
{
    /* Post STOP event. */
    car.postEvent(stopEvent);
    expect_both_motors_stop();
    expect_enter_low_power_mode();
    car.processNextEvent();
}
