#ifndef LOGDATA_H
#define LOGDATA_H

#include <stdint.h>

struct TelemetryData {
    int16_t left_motor_speed_measured;
    int16_t right_motor_speed_measured;
    int16_t left_motor_pwm_applied;
    int16_t right_motor_pwm_applied;
    int16_t target_left_speed;
    int16_t target_right_speed;
    uint16_t loop_time_ms;
};

extern TelemetryData telemetry_data;

#endif /* LOGDATA_H */
