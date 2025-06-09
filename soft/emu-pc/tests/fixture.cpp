#include "fixture.h"

#include "LogData.h"

using testing::NotNull;
using testing::Return;
using testing::Invoke;
using testing::DoAll;

void enter_low_power_mode(void)
{
    SwitchToLowPowerMode();
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#define motor carMovementController._motorControl.motor

void Fixture::expect_car_init()
{
    expect_movement_controller_init();
    expect_radio_initialization();
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
    EXPECT_CALL(_mock_arduino, pinMode(motor[LEFT].pin[DIR], OUTPUT));
    EXPECT_CALL(_mock_arduino, pinMode(motor[LEFT].pin[SPEED], OUTPUT));
    EXPECT_CALL(_mock_arduino, pinMode(motor[RIGHT].pin[DIR], OUTPUT));
    EXPECT_CALL(_mock_arduino, pinMode(motor[RIGHT].pin[SPEED], OUTPUT));
    EXPECT_CALL(_mock_arduino, digitalWrite(motor[LEFT].pin[DIR], LOW));
    EXPECT_CALL(_mock_arduino, digitalWrite(motor[LEFT].pin[SPEED], LOW));
    EXPECT_CALL(_mock_arduino, digitalWrite(motor[RIGHT].pin[DIR], LOW));
    EXPECT_CALL(_mock_arduino, digitalWrite(motor[RIGHT].pin[SPEED], LOW));
    EXPECT_CALL(_mock_arduino, pause).Times(2);
    EXPECT_CALL(_mock_arduino, setPWM);
    EXPECT_CALL(_mock_arduino, setPWM);
    EXPECT_CALL(_mock_arduino, setPWM);
    EXPECT_CALL(_mock_arduino, setPWM);
    EXPECT_CALL(_mock_arduino, setCaptureCompare(_, LOW, PERCENT_COMPARE_FORMAT)).Times(4);
}

void Fixture::expect_enter_low_power_mode()
{
    EXPECT_CALL(_mock_arduino, SwitchToLowPowerMode);
}

void Fixture::expect_move_forward(uint32_t leftSpeed, uint32_t rightSpeed)
{
    uint32_t _leftPercent = map(leftSpeed, 0, 100, 0, 255);
    uint32_t _rightPercent = map(rightSpeed, 0, 100, 0, 255);

    EXPECT_CALL(_mock_arduino,
                setCaptureCompare(motor[LEFT].pin[DIR], _leftPercent, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino,
                setCaptureCompare(motor[LEFT].pin[SPEED], 0, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino,
                setCaptureCompare(motor[RIGHT].pin[DIR], _rightPercent, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino,
                setCaptureCompare(motor[RIGHT].pin[SPEED], 0, PERCENT_COMPARE_FORMAT));
}

void Fixture::expect_move_backward(uint32_t leftSpeed, uint32_t rightSpeed)
{
    uint32_t _leftPercent = map(leftSpeed, 0, 100, 0, 255);
    uint32_t _rightPercent = map(rightSpeed, 0, 100, 0, 255);

    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[LEFT].pin[DIR], 0, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino,
                setCaptureCompare(motor[LEFT].pin[SPEED], _leftPercent, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[RIGHT].pin[DIR], 0, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino,
                setCaptureCompare(motor[RIGHT].pin[SPEED], _rightPercent, PERCENT_COMPARE_FORMAT));
}

void Fixture::expect_turn_left(uint32_t leftSpeed, uint32_t rightSpeed)
{
    uint32_t _leftPercent = map(leftSpeed, 0, 100, 0, 255);
    uint32_t _rightPercent = map(rightSpeed, 0, 100, 0, 255);

    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[LEFT].pin[DIR], 0, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino,
                setCaptureCompare(motor[LEFT].pin[SPEED], _leftPercent, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino,
                setCaptureCompare(motor[RIGHT].pin[DIR], _rightPercent, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino,
                setCaptureCompare(motor[RIGHT].pin[SPEED], 0, PERCENT_COMPARE_FORMAT));
}

void Fixture::expect_turn_right(uint32_t leftSpeed, uint32_t rightSpeed)
{
    uint32_t _leftPercent = map(leftSpeed, 0, 100, 0, 255);
    uint32_t _rightPercent = map(rightSpeed, 0, 100, 0, 255);

    EXPECT_CALL(_mock_arduino,
                setCaptureCompare(motor[LEFT].pin[DIR], _leftPercent, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino,
                setCaptureCompare(motor[LEFT].pin[SPEED], 0, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[RIGHT].pin[DIR], 0, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino,
                setCaptureCompare(motor[RIGHT].pin[SPEED], _rightPercent, PERCENT_COMPARE_FORMAT));
}

void Fixture::expect_stop_motor_left()
{
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[LEFT].pin[DIR], 0, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino,
                setCaptureCompare(motor[LEFT].pin[SPEED], 0, PERCENT_COMPARE_FORMAT));
}

void Fixture::expect_stop_motor_right()
{
    EXPECT_CALL(_mock_arduino, setCaptureCompare(motor[RIGHT].pin[DIR], 0, PERCENT_COMPARE_FORMAT));
    EXPECT_CALL(_mock_arduino,
                setCaptureCompare(motor[RIGHT].pin[SPEED], 0, PERCENT_COMPARE_FORMAT));
}

void Fixture::expect_both_motors_stop()
{
    expect_stop_motor_left();
    expect_stop_motor_right();
}

void Fixture::expect_radio_initialization()
{
    EXPECT_CALL(_mock_RF24, _begin()).WillOnce(Return(true));
    EXPECT_CALL(_mock_RF24, _setPALevel(PA_LEVEL, true));
    EXPECT_CALL(_mock_RF24, _setDataRate(DATA_RATE)).WillOnce(Return(true));
    EXPECT_CALL(_mock_RF24, _setPayloadSize(Event::packetSize()));
    EXPECT_CALL(_mock_RF24, _openWritingPipe(::testing::_))
            .WillOnce(::testing::Invoke([this](const uint8_t *ptr) {
                for (int i = 0; i < 5; ++i) {
                    ASSERT_EQ(ptr[i], test_writing_addr[i])
                            << "Mismatch at writing address byte " << i;
                }
            }));
    EXPECT_CALL(_mock_RF24, _openReadingPipe(1, ::testing::_))
            .WillOnce(::testing::Invoke([this](uint8_t, const uint8_t *ptr) {
                for (int i = 0; i < 5; ++i) {
                    ASSERT_EQ(ptr[i], test_reading_addr[i])
                            << "Mismatch at reading address byte " << i;
                }
            }));
    EXPECT_CALL(_mock_RF24, _powerUp());
    EXPECT_CALL(_mock_RF24, _startListening());
}

void Fixture::expect_receive_event(Event *evt)
{
    EXPECT_CALL(_mock_RF24, _available(_)).WillOnce(Return(evt ? true : false));
    if (evt)
        EXPECT_CALL(_mock_RF24, _read(_, Event::packetSize()))
                .WillOnce([evt](void *buf, uint8_t len) {
                    if (evt) {
                        EventPacket packet;
                        evt->toPacket(packet);
                        memcpy(buf, &packet, len);
                    }
                });
}

void Fixture::ProcessRadio()
{
    radioController.processRadio();
}

void Fixture::RxRadioEvent(Event evt)
{
    if (eventQueue.IsFull()) {
        std::cout << "RadioController::SendEvent: Queue is full." << std::endl;
        return;
    }
    eventQueue.enqueue(evt);
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
