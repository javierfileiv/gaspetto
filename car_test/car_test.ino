#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <queue.h>
#include <semphr.h>

#include "BuzzerT.h"
#include "Tones.h"

#define QUEUE_LENGTH 5
#define ITEM_SIZE sizeof(uint32_t)

// #define TEST_FWD_BWD 0
/* motor 1 is on the left with speed sensors down. */
#define DC_MOTOR_PWM_FREQ 25 /* 30 hz PWM frequency. */

// A=1 B=0  Motor 1 FWD
// A=0 B=1  Motor 1 BWD
// C=1 D=0  Motor 2 FWD
// C=0 D=1  Motor 2 BWD
#define DUTY_A 19
#define DUTY_B 0
#define DUTY_C 15
#define DUTY_D 0

enum motor {
    MOTOR_1 = 0,
    MOTOR_2 = 1,
};

enum direction { FORDWARD = 0, BACKWARD = 1 };

static void vLEDFlashTask(void* pvParameters);
static void vPrintTask(void* pvParameters);
static void m1_stop_task(void* pvParameters);

const uint8_t m1_sensor_pin = PA1;
const uint8_t m2_sensor_pin = PA0;
const uint8_t buzzer_pin = PA8;

const uint8_t pin_M1_FWD = PB10; // orange   TIM2_CH3
const uint8_t pin_M1_BWD = PB11; // green  TIM2_CH4
const uint8_t pin_M2_BWD = PB1;  // violet   TIM3_CH4
const uint8_t pin_M2_FWD = PB0;  // brown    TIM3_CH3

const uint32_t channel_M1_FWD = STM_PIN_CHANNEL(
    pinmap_function(digitalPinToPinName(pin_M1_FWD), PinMap_PWM));
const uint32_t channel_M1_BWD = STM_PIN_CHANNEL(
    pinmap_function(digitalPinToPinName(pin_M1_BWD), PinMap_PWM));
// const uint32_t channel_M1_BWD = STM_PIN_CHANNEL(
//     pinmap_function(digitalPinToPinName(pin_M2_BWD), PinMap_PWM));
// const uint32_t channel_M2_BWD = STM_PIN_CHANNEL(
//     pinmap_function(digitalPinToPinName(pin_M2_FWD), PinMap_PWM));

const uint8_t LED_PIN = LED_BUILTIN;

/* Constant for steps in disk. */
const float stepcount = 20.00; // 20 Slots in disk.
/* Constant for wheel diameter. */
const float wheeldiameter = 67.0; // Wheel diameter in millimeters.

volatile uint32_t m1_counter = 0;
volatile uint32_t m2_counter = 0;

HardwareTimer* m1_timer = NULL;
// HardwareTimer* m2_timer = NULL;
// BuzzerT Buzzer(buzzer_pin);

QueueHandle_t m1_queue = NULL;
SemaphoreHandle_t m1_sem = NULL;
QueueHandle_t m2_queue = NULL;
SemaphoreHandle_t m2_sem = NULL;

void m1_speed_irq(void)
{
    ++m1_counter;
}

void m2_speed_irq(void)
{
    ++m2_counter;
}

static void setMotor(enum motor motor, enum direction direction, uint32_t speed)
{
    //     m2_timer->setPWM(channel_M1_BWD, pin_M2_BWD, DC_MOTOR_PWM_FREQ, 0);

    /* wait for 100ms */

    if (motor == MOTOR_1) {
        m1_timer->setPWM(channel_M1_FWD, pin_M1_FWD, DC_MOTOR_PWM_FREQ, 0);
        m1_timer->setPWM(channel_M1_BWD, pin_M1_BWD, DC_MOTOR_PWM_FREQ, 0);
        vTaskDelay(100UL * configTICK_RATE_HZ / 1000UL);

        // Wait for a semaphore
        if (xSemaphoreTake(m1_sem, portMAX_DELAY) == pdTRUE) {

            switch (direction) {
            case FORDWARD:
                m1_timer->setPWM(channel_M1_FWD, pin_M1_FWD, DC_MOTOR_PWM_FREQ,
                                 speed);
                break;
            case BACKWARD:
                m1_timer->setPWM(channel_M1_BWD, pin_M1_BWD, DC_MOTOR_PWM_FREQ,
                                 speed);
                break;
            }
            // The semaphore has been successfully taken and the resource can be
            // accessed safely.
        }
    } else {
        return;
        switch (direction) {
        case FORDWARD:
            digitalWrite(pin_M2_FWD, LOW);
            break;
        case BACKWARD:
            digitalWrite(pin_M2_FWD, HIGH);
            break;
            m1_timer->setPWM(channel_M1_BWD, pin_M1_BWD, DC_MOTOR_PWM_FREQ,
                             speed);
            m1_timer->resumeChannel(channel_M1_BWD);
        }
    }
    if (xQueueSend(m1_queue, &speed, portMAX_DELAY) != pdPASS) {
        while (1) {
        }
    }
}
// Function to convert from centimeters to steps
static uint32_t centimeters_to_step(float cm)
{

    float circumference =
        (wheeldiameter * 3.14) / 10; // Calculate wheel circumference in cm
    float cm_step = circumference / stepcount; // CM per Step
    float f_result = cm / cm_step;             // Calculate result as a float

    return (
        uint32_t)f_result; // Convert to an integer (note this is NOT rounded)
}

static void set_buzzer(void)
{
    // 	Buzzer.Melody(Startup);
    // 	TIM_TypeDef *Timer1 = (TIM_TypeDef
    // *)pinmap_peripheral(digitalPinToPinName(buzzer_pin), PinMap_PWM);
    // 	HardwareTimer *Timer1_instance = new HardwareTimer(Timer1);

    // 	uint32_t channel =
    // STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(buzzer_pin),
    // PinMap_PWM)); 	Timer1_instance->setPWM(channel, buzzer_pin, 650,
    // 0);
}

static void MoveForward(int steps, int mspeed)
{
    m1_counter = 0; //  reset counter A to zero
    m2_counter = 0; //  reset counter B to zero

    setMotor(MOTOR_1, FORDWARD, mspeed);
    setMotor(MOTOR_2, FORDWARD, mspeed);

    //   // Go forward until step value is reached
    //   while (steps > m1_counter && steps > m2_counter) {
    //     if (m1_counter >= steps)
    //       setMotor(MOTOR_1, FORDWARD, 0);
    //     if (m2_counter >= steps)
    //       setMotor(MOTOR_2, FORDWARD, 0);
    //   }
}

// Function to Move in Reverse
static void MoveReverse(int steps, int mspeed)
{
    m1_counter = 0; //  reset counter A to zero
    m2_counter = 0; //  reset counter B to zero

    setMotor(MOTOR_1, BACKWARD, mspeed);
    setMotor(MOTOR_2, BACKWARD, mspeed);

    // Go in reverse until step value is reached
    while (steps > m1_counter && steps > m2_counter)
        ;

    setMotor(MOTOR_1, FORDWARD, 0);
    setMotor(MOTOR_2, FORDWARD, 0);
}

// Function to Spin Right
static void SpinRight(int steps, int mspeed)
{
    m1_counter = 0; //  reset counter A to zero
    m2_counter = 0; //  reset counter B to zero

    setMotor(MOTOR_1, FORDWARD, mspeed);
    setMotor(MOTOR_2, BACKWARD, mspeed);

    // Go until step value is reached
    while (steps > m1_counter && steps > m2_counter)
        ;
    // Stop when done
    setMotor(MOTOR_1, FORDWARD, 0);
    setMotor(MOTOR_2, FORDWARD, 0);
}

// Function to Spin Left
static void SpinLeft(int steps, int mspeed)
{
    m1_counter = 0; //  reset counter A to zero
    m2_counter = 0; //  reset counter B to zero

    // Set Motor A forward
    // Set Motor B reverse
    setMotor(MOTOR_1, BACKWARD, mspeed);
    setMotor(MOTOR_2, FORDWARD, mspeed);

    // Go until step value is reached
    while (steps > m1_counter && steps > m2_counter)
        ;

    // Stop when done
    setMotor(MOTOR_1, FORDWARD, 0);
    setMotor(MOTOR_2, FORDWARD, 0);
}

static void setup_pwm_timer(void)
{
    TIM_TypeDef* Timer1 = (TIM_TypeDef*)pinmap_peripheral(
        digitalPinToPinName(pin_M1_FWD), PinMap_PWM);
    m1_timer = new HardwareTimer(Timer1);
    m1_timer->pause();

    //     TIM_TypeDef* Timer1 = (TIM_TypeDef*)pinmap_peripheral(
    //         digitalPinToPinName(pin_M2_BWD), PinMap_PWM);
    //     m2_timer = new HardwareTimer(Timer1);
    //     m2_timer->pause();
}

void set_interrupt_speed_sensors(void)
{
    uint8_t m1_irq_mode = 0;
    uint8_t m2_irq_mode = 0;
    pinMode(m1_sensor_pin, INPUT);
    pinMode(m2_sensor_pin, INPUT);

    if (digitalRead(m1_sensor_pin) == HIGH)
        /* Detect falling pulses. */
        m1_irq_mode = FALLING;
    else
        m1_irq_mode = RISING;

    if (digitalRead(m2_sensor_pin) == HIGH)
        /* Detect falling pulses. */
        m2_irq_mode = FALLING;
    else
        m2_irq_mode = RISING;

    attachInterrupt(digitalPinToInterrupt(m1_sensor_pin), m1_speed_irq,
                    m1_irq_mode);
    attachInterrupt(digitalPinToInterrupt(m2_sensor_pin), m2_speed_irq,
                    m2_irq_mode);
}

// static void task_buzzer(void *pvParameters)
// {
// 	while(1) {
// 		Buzzer.Melody(Startup);
// 		Buzzer.Play=true;
// 	}
// }

#define INIT_STATE LOW
void setup()
{
    Serial.begin();
    digitalWrite(pin_M1_FWD, INIT_STATE);
    digitalWrite(pin_M1_BWD, INIT_STATE);
    digitalWrite(pin_M2_BWD, INIT_STATE);
    digitalWrite(pin_M2_FWD, INIT_STATE);

    m1_queue = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE);
    if (m1_queue == NULL) {
        while (1) {
        }
    }

    m2_queue = xQueueCreate(QUEUE_LENGTH, ITEM_SIZE);
    if (m2_queue == NULL) {
        while (1) {
        }
    }

    m1_sem = xSemaphoreCreateBinary();
    if (m1_sem == NULL) {
        while (1) {
        }
    }
    xSemaphoreGive(m1_sem);

    m2_sem = xSemaphoreCreateBinary();
    if (m2_sem == NULL) {
        while (1) {
        }
    }

    xSemaphoreGive(m2_sem);

    setup_pwm_timer();
    set_interrupt_speed_sensors();
    set_buzzer();
    // #ifdef TEST_FWD_BWD
    // #else

    // TIM_TypeDef *Timer2 = (TIM_TypeDef
    // *)pinmap_peripheral(digitalPinToPinName(pin_M1_FWD),
    // PinMap_PWM); HardwareTimer *Timer2_instance = new
    // HardwareTimer(Timer2);

    // uint32_t channel =
    // STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_M1_FWD),
    // PinMap_PWM)); Timer2_instance->setPWM(channel, pin_M1_FWD,
    // DC_MOTOR_PWM_FREQ, DUTY_A); channel =
    // STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_M1_BWD),
    // PinMap_PWM)); Timer2_instance->setPWM(channel, pin_M1_BWD,
    // DC_MOTOR_PWM_FREQ, DUTY_B);

    // TIM_TypeDef *Timer1 = (TIM_TypeDef
    // *)pinmap_peripheral(digitalPinToPinName(pin_M2_BWD),
    // PinMap_PWM); HardwareTimer *Timer1_instance = new
    // HardwareTimer(Timer1);

    // channel =
    // STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_M2_BWD),
    // PinMap_PWM)); Timer1_instance->setPWM(channel, pin_M2_BWD,
    // DC_MOTOR_PWM_FREQ, DUTY_C); channel =
    // STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(pin_M2_FWD),
    // PinMap_PWM)); Timer1_instance->setPWM(channel, pin_M2_FWD,
    // DC_MOTOR_PWM_FREQ, DUTY_D);
    // #endif
    // create blink task
    xTaskCreate(vLEDFlashTask, "Task1", configMINIMAL_STACK_SIZE + 100, NULL,
                tskIDLE_PRIORITY + 2, NULL);

    // create print task
    xTaskCreate(vPrintTask, "Task2", configMINIMAL_STACK_SIZE + 100, NULL,
                tskIDLE_PRIORITY + 1, NULL);

    // create print task
    xTaskCreate(m1_stop_task, "M1 STOP", configMINIMAL_STACK_SIZE + 100, NULL,
                tskIDLE_PRIORITY + 1, NULL);

    // xTaskCreate(task_buzzer,
    // 	    "TaskBuzz",
    // 	    configMINIMAL_STACK_SIZE + 100,
    // 	    NULL,
    // 	    tskIDLE_PRIORITY + 1,
    // 	    NULL);

    // start FreeRTOS
    vTaskStartScheduler();

    // should never return
    Serial.println(F("Die"));
    while (1)
        ;
}

//------------------------------------------------------------------------------
// high priority for blinking LED
static void vLEDFlashTask(void* pvParameters)
{
    UNUSED(pvParameters);
    pinMode(LED_PIN, OUTPUT);

    // Flash led every 200 ms.
    for (;;) {
        // Turn LED off.
        digitalWrite(LED_PIN, HIGH);

        // Sleep for 50 milliseconds.
        vTaskDelay((100L * configTICK_RATE_HZ) / 1000L);

        // Turn LED on.
        digitalWrite(LED_PIN, LOW);

        // Sleep for 150 milliseconds.
        vTaskDelay((100L * configTICK_RATE_HZ) / 1000L);
    }
}

static void m1_stop_task(void* pvParameters)
{
    UNUSED(pvParameters);
    uint32_t steps = 0;

    for (;;) {
        if (xQueueReceive(m1_queue, &steps, portMAX_DELAY) == pdPASS) {
            do {
                /* wait for 100 ms */
                vTaskDelay(100UL * configTICK_RATE_HZ / 1000UL);
            } while (steps > m1_counter);
            setMotor(MOTOR_1, FORDWARD, 0);
            xSemaphoreGive(m1_sem);
        } else {
            while (10)
                ;
        }
    }
}

//------------------------------------------------------------------------------
static void vPrintTask(void* pvParameters)
{
    UNUSED(pvParameters);
    while (1) {
        // Sleep for one second.
        vTaskDelay(configTICK_RATE_HZ);

        Serial.print(F("Count MOTOR 1: "));
        Serial.println(m1_counter);

        Serial.print(F("Count MOTOR 2: "));
        Serial.println(m2_counter);
    }
}
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
void loop()
{

#ifdef TEST_FWD_BWD
    // Full speed forward
#if TEST_FWD_BWD == 1
    digitalWrite(pin_M1_FWD, HIGH);
    digitalWrite(pin_M1_BWD, LOW);
    // #elif TEST_FWD_BWD == 2
    digitalWrite(pin_M2_BWD, HIGH);
    digitalWrite(pin_M2_FWD, LOW);
#endif
    delay(1000);

    // stop
    digitalWrite(pin_M1_FWD, LOW);
    digitalWrite(pin_M1_BWD, LOW);
    digitalWrite(pin_M2_BWD, LOW);
    digitalWrite(pin_M2_FWD, LOW);

    delay(1000);

    // Full speed backward
#if TEST_FWD_BWD == 1
    digitalWrite(pin_M1_FWD, LOW);
    digitalWrite(pin_M1_BWD, HIGH);
    // #elif TEST_FWD_BWD == 2
    digitalWrite(pin_M2_BWD, LOW);
    digitalWrite(pin_M2_FWD, HIGH);
#endif

    delay(1000);

    // stop
    digitalWrite(pin_M1_FWD, LOW);
    digitalWrite(pin_M1_BWD, LOW);
    digitalWrite(pin_M2_BWD, LOW);
    digitalWrite(pin_M2_FWD, LOW);

    delay(1000);
#else
    // Test Motor Movement  - Experiment with your own sequences here
    MoveForward(centimeters_to_step(5),
                20);     // Forward half a metre at 255 speed
    delay(1000);         // Wait one second
    MoveReverse(10, 20); // Reverse 10 steps at 255 speed
    delay(1000);         // Wait one second
    MoveForward(10, 21); // Forward 10 steps at 150 speed
    delay(1000);         // Wait one second
    MoveReverse(centimeters_to_step(25.4),
                20);   // Reverse 25.4 cm at 200 speed
    delay(1000);       // Wait one second
    SpinRight(20, 22); // Spin right 20 steps at 255 speed
    delay(1000);       // Wait one second
    SpinLeft(60, 23);  // Spin left 60 steps at 175 speed
    while (1)
        delay(5000);
#endif
}
