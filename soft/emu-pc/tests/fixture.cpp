#include "fixture.h"

#include "mock_MotorController.h"

void Fixture::expect_car_init()
{
    mockMotorController.expect_motor_controller_init();
    mock_RadioController.expect_radio_initialization();
    EXPECT_CALL(mock_arduino, SwitchToLowPowerMode);
}

void Fixture::expect_motor_pins_init()
{
    mockMotorController.expect_motor_pins_init();
}

void Fixture::expect_speed_sensor_init()
{
    mockMotorController.expect_speed_sensor_init();
}

void Fixture::expect_enter_low_power_mode()
{
    EXPECT_CALL(mock_arduino, SwitchToLowPowerMode);
}

void Fixture::expect_turn_right(uint8_t *target_left, uint8_t distance_left, uint8_t *target_right,
                                uint8_t distance_right)
{
    mockMotorController.expect_turn_right(target_left, distance_left, target_right, distance_right);
}

void Fixture::expect_turn_left(uint8_t *target_left, uint8_t distance_left, uint8_t *target_right,
                               uint8_t distance_right)
{
    mockMotorController.expect_turn_left(target_left, distance_left, target_right, distance_right);
}

void Fixture::expect_move_forward(uint8_t *target_left, uint8_t distance_left,
                                  uint8_t *target_right, uint8_t distance_right)
{
    mockMotorController.expect_move_forward(target_left, distance_left, target_right,
                                            distance_right);
}
void Fixture::expect_move_backward(uint8_t *target_left, uint8_t distance_left,
                                   uint8_t *target_right, uint8_t distance_right)
{
    mockMotorController.expect_move_backward(target_left, distance_left, target_right,
                                             distance_right);
}

void Fixture::expect_stop_motor_left()
{
    mockMotorController.expect_stop_motor_left();
}

void Fixture::expect_stop_motor_right()
{
    mockMotorController.expect_stop_motor_right();
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
