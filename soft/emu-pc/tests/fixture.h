#ifndef GCAR_FIXTURE_H
#define GCAR_FIXTURE_H

#include "CarEvents.h"
#include "CarStates.h"
#include "Context.h"
#include "EventQueue.h"
#include "GCar.h"
#include "ProcessingState.h"
#include "RadioController.h"
#include "State.h"
#include "config_event.h"
#include "config_radio.h"
#include "mock_Arduino.h"
#include "mock_MovementController.h"
#include "mock_RF24.h"

#include <gtest/gtest.h>

using testing::_;
using testing::Eq;

static const uint8_t *test_writing_addr = gaspetto_box_pipe_name;
static const uint8_t *test_reading_addr = gcar_pipe_name;

extern bool low_power_mode;
void enter_low_power_mode(void);

class Fixture : public ::testing::Test {
public:
    Fixture()
            : radioController(_mock_RF24, &eventQueue, test_writing_addr, test_reading_addr)
            , _mock_movementController()
            , eventQueue()
            , ctx{ &eventQueue, &_mock_movementController, &radioController, nullptr,
                   &idleState,  &processingState,          MOTOR_FREQ }
            , gaspettoCar(ctx)
    {
    }

    void SetUp() override
    {
        gaspettoCar.setLowPowerModeCallback(enter_low_power_mode);
        expect_car_init();
        gaspettoCar.init(StateId::IDLE);
        ASSERT_EQ(gaspettoCar.getCurrentState(), &idleState);
    }

    void expect_car_init();
    /* Movement. */
    void expect_movement_controller_init();
    void expect_enter_low_power_mode();
    void set_exit_low_power_mode();
    bool get_low_power_mode() const
    {
        return low_power_mode;
    }
    void expect_move_forward(float speed, uint32_t timeout_ms);
    void expect_move_backward(float speed, uint32_t timeout_ms);
    void expect_turn_left(float target_yaw, float speed, uint32_t timeout_ms);
    void expect_turn_right(float target_yaw, float speed, uint32_t timeout_ms);
    void expect_stop_both_motors();
    void expect_update_movement();
    void expect_target_reached();
    void expect_target_not_reached();

public:
    /* Radio. */
    void expect_radio_initialization();
    void radio_receive_event(Event *evt = nullptr);
    void radio_receive_raw(const void *data, uint8_t len = Event::packetSize());
    void expect_transmit_event(Event evt);
    void expect_process_radio_no_event();
    void RxRadioEvent(Event evt);
    void expect_send_event(Event *evt = nullptr);
    void expect_radio_process_event(Event *evt = nullptr);

protected:
    /* States must be declared before car since car's Context references them. */
    IdleState idleState;
    ProcessingState processingState;
    Context ctx;
    GCar gaspettoCar;
    Event forwardEvent{ EventId::ACTION, CommandId::MOTOR_FORWARD };
    Event backwardEvent{ EventId::ACTION, CommandId::MOTOR_BACKWARD };
    Event stopEvent{ EventId::ACTION, CommandId::MOTOR_STOP };
    Event leftEvent{ EventId::ACTION, CommandId::MOTOR_TURN_LEFT };
    Event rightEvent{ EventId::ACTION, CommandId::MOTOR_TURN_RIGHT };
    Event queueClearEvent{ EventId::ACTION, CommandId::QUEUE_CLEAR };

private:
    EventQueue eventQueue;
    testing::StrictMock<MockMovementController> _mock_movementController;
    testing::StrictMock<MockArduino> _mock_arduino;
    testing::StrictMock<MockRF24> _mock_RF24;
    RadioController radioController;
    ::testing::InSequence seq;
};

#endif /* GCAR_FIXTURE_H */
