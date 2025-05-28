#ifndef CONFIG_MOTOR_H
#define CONFIG_MOTOR_H

#define FORWARD true
#define BACKWARD false
#define MOTOR_PWM 15
#define DISTANCE_CM_FWD_BWD 50
#define DISTANCE_CM_TURN_RIGHT 15
#define DISTANCE_CM_TURN_LEFT 20

const uint32_t PWM_FREQ = 35; /* Set PWM frequency to 35Hz. */
const uint32_t MOTOR_RIGHT_PIN_A = PB0; /* PWM pin for motor right. */
const uint32_t MOTOR_RIGHT_PIN_B = PB1; /* Direction pin for motor right. */
const uint32_t MOTOR_LEFT_PIN_A = PB11; /* Example PWM pin for motor left.  */
const uint32_t MOTOR_LEFT_PIN_B = PB10; /* Direction pin for motor left.  */
const uint32_t SPEED_SENSOR_LEFT_PIN = PA1; /* Pin for left speed/distance sensor. */
const uint32_t SPEED_SENSOR_RIGHT_PIN = PA0; /* Pin for right speed/distance sensor. */

#endif /* CONFIG_MOTOR_H */
