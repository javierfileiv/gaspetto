#pragma once

#include "ActiveObject.h"
#include "Arduino.h"
#include "EventQueue.h"
#include "ProcessingState.h"
#include "State.h"

#ifndef ARDUINO
#include <cstdint>
#define __ASSERT_USE_STDERR
#include <assert.h>
#else
#define assert(x)                              \
    do {                                       \
        Serial.print(F("Assertion failed: ")); \
        Serial.print(F(#x));                   \
        Serial.print(F(" in "));               \
        Serial.print(__FILE__);                \
        Serial.print(F(" at line "));          \
        Serial.println(__LINE__);              \
        while (1) {                            \
            delay(1000);                       \
        }                                      \
    } while (0)
#endif

class MotorController;
class RadioController;

class GaspettoCar : public ActiveObject {
public:
    GaspettoCar(State *idle, State *running, EventQueue *queue,
                MotorController *motorController = nullptr,
                RadioController *radioController = nullptr);

    void Init(StateId initial_state);
    /* Set motor directions. */
    void SetMotor(bool forward_motor_left, uint8_t motor_left_speed, uint8_t distance_cm_left,
                  bool forward_motor_right, uint8_t motor_right_speed, uint8_t distance_cm_right);
    int postEvent(Event evt) override;
    void processNextEvent() override;
    void enterLowPowerMode() override;
    void stopMotorRight();
    void stopMotorLeft();
    void ResetCounterMotorRight();
    void ResetCounterMotorLeft();
    MotorController *getMotorController() const
    {
        return motorController;
    }
    RadioController *getRadioController() const
    {
        return radioController;
    }

private:
    MotorController *motorController;
    RadioController *radioController;
    void InitMotorPins();
    void InitSpeedSensor();
    uint32_t CentimetersToStep(float cm);
    bool isTargetReached();
};
