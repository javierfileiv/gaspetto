#include "mock_MotorController.h"

#define __MOTOR_PWM__ 17

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
    uint32_t pwm = this->mapDutyCycle(__MOTOR_PWM__);
    *target_left = this->CentimetersToCount(distance_left);
    *target_right = this->CentimetersToCount(distance_right);

    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_LEFT_PIN_A, pwm));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_LEFT_PIN_B, 0));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_RIGHT_PIN_A, 0));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_RIGHT_PIN_B, pwm));
}

void MockMotorController::expect_turn_left(uint8_t *target_left, uint8_t distance_left,
                                           uint8_t *target_right, uint8_t distance_right)
{
    uint32_t pwm = this->mapDutyCycle(__MOTOR_PWM__);
    *target_left = this->CentimetersToCount(distance_left);
    *target_right = this->CentimetersToCount(distance_right);

    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_LEFT_PIN_A, 0));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_LEFT_PIN_B, pwm));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_RIGHT_PIN_A, pwm));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_RIGHT_PIN_B, 0));
}

void MockMotorController::expect_move_forward(uint8_t *target_left, uint8_t distance_left,
                                              uint8_t *target_right, uint8_t distance_right)
{
    uint32_t pwm = this->mapDutyCycle(__MOTOR_PWM__);
    *target_left = this->CentimetersToCount(distance_left);
    *target_right = this->CentimetersToCount(distance_right);

    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_LEFT_PIN_A, pwm));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_LEFT_PIN_B, 0));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_RIGHT_PIN_A, pwm));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_RIGHT_PIN_B, 0));
}

void MockMotorController::expect_move_backward(uint8_t *target_left, uint8_t distance_left,
                                               uint8_t *target_right, uint8_t distance_right)
{
    uint32_t pwm = this->mapDutyCycle(__MOTOR_PWM__);
    *target_left = this->CentimetersToCount(distance_left);
    *target_right = this->CentimetersToCount(distance_right);

    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_LEFT_PIN_A, 0));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_LEFT_PIN_B, pwm));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_RIGHT_PIN_A, 0));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_RIGHT_PIN_B, pwm));
}

void MockMotorController::expect_stop_motor_left()
{
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_LEFT_PIN_A, 0));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_LEFT_PIN_B, 0));
}

void MockMotorController::expect_stop_motor_right()
{
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_RIGHT_PIN_A, 0));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_RIGHT_PIN_B, 0));
}

void MockMotorController::expect_motor_controller_init()
{
    this->expect_motor_pins_init();
    this->expect_speed_sensor_init();
}

void MockMotorController::expect_motor_pins_init()
{
    EXPECT_CALL(_mock_arduino, pinMode(MOTOR_LEFT_PIN_A, OUTPUT));
    EXPECT_CALL(_mock_arduino, pinMode(MOTOR_LEFT_PIN_B, OUTPUT));
    EXPECT_CALL(_mock_arduino, pinMode(MOTOR_RIGHT_PIN_A, OUTPUT));
    EXPECT_CALL(_mock_arduino, pinMode(MOTOR_RIGHT_PIN_B, OUTPUT));
}

void MockMotorController::expect_speed_sensor_init()
{
    EXPECT_CALL(_mock_arduino, pinMode(SPEED_SENSOR_LEFT_PIN, INPUT));
    EXPECT_CALL(_mock_arduino, pinMode(SPEED_SENSOR_RIGHT_PIN, INPUT));
    EXPECT_CALL(_mock_arduino, attachInterrupt(SPEED_SENSOR_LEFT_PIN, testing::NotNull(), RISING));
    EXPECT_CALL(_mock_arduino, attachInterrupt(SPEED_SENSOR_RIGHT_PIN, testing::NotNull(), RISING));
}
