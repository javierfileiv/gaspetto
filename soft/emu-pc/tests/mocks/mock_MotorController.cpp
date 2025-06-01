#include "mock_MotorController.h"

using testing::Return;

uint32_t MockMotorController::mapDutyCycle(int dutyCycle)
{
    return MotorController::mapDutyCycle(dutyCycle);
}

uint32_t MockMotorController::CentimetersToCount(float cm)
{
    return MotorController::CentimetersToCount(cm);
}

void MockMotorController::expect_turn_right(uint8_t *target_left, uint8_t distance_left,
                                            uint8_t *target_right, uint8_t distance_right)
{
    uint32_t pwm = this->mapDutyCycle(MOTOR_PWM);
    *target_left = this->CentimetersToCount(distance_left);
    *target_right = this->CentimetersToCount(distance_right);

    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[LEFT].pin[A], pwm, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[LEFT].pin[B], 0, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[RIGHT].pin[A], 0, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[RIGHT].pin[B], pwm, PERCENT_COMPARE_FORMAT));
}

void MockMotorController::expect_turn_left(uint8_t *target_left, uint8_t distance_left,
                                           uint8_t *target_right, uint8_t distance_right)
{
    uint32_t pwm = this->mapDutyCycle(MOTOR_PWM);
    *target_left = this->CentimetersToCount(distance_left);
    *target_right = this->CentimetersToCount(distance_right);

    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[LEFT].pin[A], 0, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[LEFT].pin[B], pwm, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[RIGHT].pin[A], pwm, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[RIGHT].pin[B], 0, PERCENT_COMPARE_FORMAT));
}

void MockMotorController::expect_move_forward(uint8_t *target_left, uint8_t distance_left,
                                              uint8_t *target_right, uint8_t distance_right)
{
    uint32_t pwm = this->mapDutyCycle(MOTOR_PWM);
    *target_left = this->CentimetersToCount(distance_left);
    *target_right = this->CentimetersToCount(distance_right);

    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[LEFT].pin[A], pwm, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[LEFT].pin[B], 0, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[RIGHT].pin[A], pwm, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[RIGHT].pin[B], 0, PERCENT_COMPARE_FORMAT));
}

void MockMotorController::expect_move_backward(uint8_t *target_left, uint8_t distance_left,
                                               uint8_t *target_right, uint8_t distance_right)
{
    uint32_t pwm = this->mapDutyCycle(MOTOR_PWM);
    *target_left = this->CentimetersToCount(distance_left);
    *target_right = this->CentimetersToCount(distance_right);

    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[LEFT].pin[A], 0, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[LEFT].pin[B], pwm, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[RIGHT].pin[A], 0, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[RIGHT].pin[B], pwm, PERCENT_COMPARE_FORMAT));
}

void MockMotorController::expect_stop_motor_left()
{
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[LEFT].pin[A], 0, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[LEFT].pin[B], 0, PERCENT_COMPARE_FORMAT));
}

void MockMotorController::expect_stop_motor_right()
{
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[RIGHT].pin[A], 0, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[RIGHT].pin[B], 0, PERCENT_COMPARE_FORMAT));
}

void MockMotorController::expect_motor_controller_init()
{
    expect_motor_pins_init();
    EXPECT_CALL(_mock_arduino, setPWM);
    EXPECT_CALL(_mock_arduino, setPWM);
    EXPECT_CALL(_mock_arduino, setPWM);
    EXPECT_CALL(_mock_arduino, setPWM);
    expect_speed_sensor_init();
}

void MockMotorController::expect_motor_pins_init()
{
    EXPECT_CALL(_mock_arduino, pinMode(motor[LEFT].pin[A], OUTPUT));
    EXPECT_CALL(_mock_arduino, pinMode(motor[LEFT].pin[B], OUTPUT));
    EXPECT_CALL(_mock_arduino, pinMode(motor[RIGHT].pin[A], OUTPUT));
    EXPECT_CALL(_mock_arduino, pinMode(motor[RIGHT].pin[B], OUTPUT));
}

void MockMotorController::expect_speed_sensor_init()
{
    EXPECT_CALL(_mock_arduino, pinMode(motor[LEFT].speed_sensor_pin, INPUT));
    EXPECT_CALL(_mock_arduino, pinMode(motor[RIGHT].speed_sensor_pin, INPUT));
    EXPECT_CALL(_mock_arduino,
                attachInterrupt(motor[LEFT].speed_sensor_pin, testing::NotNull(), RISING));
    EXPECT_CALL(_mock_arduino,
                attachInterrupt(motor[RIGHT].speed_sensor_pin, testing::NotNull(), RISING));
}
