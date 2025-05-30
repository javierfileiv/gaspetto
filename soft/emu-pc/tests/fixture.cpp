#include "fixture.h"

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

void Fixture::expect_turn_left(uint32_t leftSpeed, uint32_t rightSpeed)
{
    uint32_t _leftPercent = map(leftSpeed, 0, 100, 0, 255);
    uint32_t _rightPercent = map(rightSpeed, 0, 100, 0, 255);

    EXPECT_CALL(_mock_arduino, digitalWrite(MOTOR_LEFT_PIN_A, 0));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_LEFT_PIN_B, _leftPercent));
    EXPECT_CALL(_mock_arduino, analogWrite(MOTOR_RIGHT_PIN_A, _rightPercent));
    EXPECT_CALL(_mock_arduino, digitalWrite(MOTOR_RIGHT_PIN_B, 0));
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
                    std::cout << "_read called with buf=" << buf << " len=" << (int)len
                              << std::endl;
                    if (evt) {
                        EventPacket packet;
                        evt->toPacket(packet);
                        memcpy(buf, &packet, len);
                    }
                });
}

void Fixture::expect_transmit_event(Event evt)
{
    EventPacket packet;

    evt.toPacket(packet);
    auto do_check = [&packet](const void *buf, size_t len) {
        ASSERT_EQ(len, Event::packetSize());
        ASSERT_EQ(memcmp(buf, &packet, Event::packetSize()), 0);
    };
    EXPECT_CALL(_mock_RF24, _write(NotNull(), Event::packetSize()))
            .WillOnce(DoAll(Invoke(do_check), Return(true)));
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
