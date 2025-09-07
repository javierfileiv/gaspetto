#ifndef CONFIG_MOTOR_H
#define CONFIG_MOTOR_H

#define FORWARD true
#define BACKWARD false
#define MOTOR_PWM 15
#define DISTANCE_CM_FWD_BWD 50
#define DISTANCE_CM_TURN_RIGHT 15
#define DISTANCE_CM_TURN_LEFT 20

const uint32_t PWM_FREQ = 15; /* Set PWM frequency to 35Hz. */
const uint32_t MOTOR_LEFT_PIN_A = PB11; /* Example PWM pin for motor left. D4 on salaea. */
const uint32_t MOTOR_LEFT_PIN_B = PB10; /* Direction pin for motor left.  D5 on salaea. */
const uint32_t MOTOR_RIGHT_PIN_A = PB0; /* PWM pin for motor right. D1 on salaea. */
const uint32_t MOTOR_RIGHT_PIN_B = PB1; /* Direction pin for motor right. D2 on salaea. */
const uint32_t SPEED_SENSOR_LEFT_PIN = PA1; /* Pin for left speed/distance sensor. D0 on salaea. */
const uint32_t SPEED_SENSOR_RIGHT_PIN = PA0; /* Pin for right speed/distance sensor. D3 on salaea.
                                              */

#endif /* CONFIG_MOTOR_H */
