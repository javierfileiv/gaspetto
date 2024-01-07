#include <Arduino.h>
#include <STM32FreeRTOS.h>
#include <BuzzerT.h>
#include <Tones.h>

#define TEST_FWD_BWD 0
/* motor 1 is on the left with speed sensors down. */
#define DC_MOTOR_PWM_FREQ 20 /* 30 hz PWM frequency. */

// A=1 B=0  Motor 1 FWD
// A=0 B=1  Motor 1 BWD
// C=1 D=0  Motor 2 FWD
// C=0 D=1  Motor 2 BWD
#define DUTY_A 19
#define DUTY_B 0
#define DUTY_C 15
#define DUTY_D 0

static void vLEDFlashTask(void *pvParameters);
static void vPrintTask(void *pvParameters);

const uint8_t in_A_driver_pin = PB10; // orange   TIM2_CH3
const uint8_t in_B_driver_pin = PB11; // green  TIM2_CH4
const uint8_t in_C_driver_pin = PB1;  // violet   TIM3_CH4
const uint8_t in_D_driver_pin = PB0;  // brown    TIM3_CH3
const uint8_t LED_PIN = LED_BUILTIN;
const uint8_t m1_sensor_pin = PA1;
const uint8_t m2_sensor_pin = PA0;
const uint8_t buzzer_pin = PA8;

/* Constant for steps in disk. */
const float stepcount = 20.00; // 20 Slots in disk.
/* Constant for wheel diameter. */
const float wheeldiameter = 67.0; // Wheel diameter in millimeters.

volatile uint32_t m1_counter = 0;
volatile uint32_t m2_counter = 0;

HardwareTimer *m1_timer = NULL;
HardwareTimer *m2_timer = NULL;
uint32_t channelA, channelB, channelC, channelD;
// BuzzerT Buzzer(buzzer_pin);

void m1_speed_irq(void)
{
	++m1_counter;
}

void m2_speed_irq(void)
{
	++m2_counter;
}

// Function to convert from centimeters to steps
uint32_t centimeters_to_step(float cm)
{

	float circumference = (wheeldiameter * 3.14) / 10; // Calculate wheel circumference in cm
	float cm_step = circumference / stepcount;	   // CM per Step
	float f_result = cm / cm_step;			   // Calculate result as a float

	return (uint32_t)f_result; // Convert to an integer (note this is NOT rounded)
}
// void set_buzzer(void)
// {
// 	Buzzer.Melody(Startup);
// 	TIM_TypeDef *Timer1 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(buzzer_pin), PinMap_PWM);
// 	HardwareTimer *Timer1_instance = new HardwareTimer(Timer1);

// 	uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(buzzer_pin), PinMap_PWM));
// 	Timer1_instance->setPWM(channel, buzzer_pin, 650, 0);
// }

void MoveForward(int steps, int mspeed)
{
	m1_counter = 0; //  reset counter A to zero
	m2_counter = 0; //  reset counter B to zero

	// Go forward until step value is reached
	while (steps > m1_counter && steps > m2_counter)
	{

		if (steps > m1_counter)
			m1_timer->setPWM(channelA, in_A_driver_pin, DC_MOTOR_PWM_FREQ, mspeed);
		else
			m1_timer->setPWM(channelA, in_A_driver_pin, DC_MOTOR_PWM_FREQ, 0);
		if (steps > m2_counter)
			m2_timer->setPWM(channelC, in_C_driver_pin, DC_MOTOR_PWM_FREQ, mspeed);
		else
			m2_timer->setPWM(channelC, in_C_driver_pin, DC_MOTOR_PWM_FREQ, 0);
	}

	m1_timer->pause();
	m2_timer->pause();

}

// Function to Move in Reverse
void MoveReverse(int steps, int mspeed)
{
	m1_counter = 0; //  reset counter A to zero
	m2_counter = 0; //  reset counter B to zero

	// Go in reverse until step value is reached
	while (steps > m1_counter && steps > m2_counter)
	{

		if (steps > m1_counter)
			m1_timer->setPWM(channelB, in_B_driver_pin, DC_MOTOR_PWM_FREQ, mspeed);
		else
			m1_timer->setPWM(channelB, in_B_driver_pin, DC_MOTOR_PWM_FREQ, 0);
		if (steps > m2_counter)
			m2_timer->setPWM(channelD, in_D_driver_pin, DC_MOTOR_PWM_FREQ, mspeed);
		else
			m2_timer->setPWM(channelD, in_D_driver_pin, DC_MOTOR_PWM_FREQ, 0);
	}

	m1_timer->pause();
	m2_timer->pause();
}

// Function to Spin Right
void SpinRight(int steps, int mspeed)
{
	m1_counter = 0; //  reset counter A to zero
	m2_counter = 0; //  reset counter B to zero

	// Go until step value is reached
	while (steps > m1_counter && steps > m2_counter)
	{

		if (steps > m1_counter)
			m1_timer->setPWM(channelB, in_B_driver_pin, DC_MOTOR_PWM_FREQ, mspeed);
		else
			m1_timer->setPWM(channelB, in_B_driver_pin, DC_MOTOR_PWM_FREQ, 0);

		if (steps > m2_counter)
			m2_timer->setPWM(channelC, in_C_driver_pin, DC_MOTOR_PWM_FREQ, mspeed);
		else
			m2_timer->setPWM(channelC, in_C_driver_pin, DC_MOTOR_PWM_FREQ, 0);
	}

	// Stop when done
	m1_timer->pause();
	m2_timer->pause();

}

// Function to Spin Left
void SpinLeft(int steps, int mspeed)
{
	m1_counter = 0; //  reset counter A to zero
	m2_counter = 0; //  reset counter B to zero

	// Set Motor A forward
	// Set Motor B reverse

	// Go until step value is reached
	while (steps > m1_counter && steps > m2_counter)
	{

		if (steps > m1_counter)
			m1_timer->setPWM(channelA, in_A_driver_pin, DC_MOTOR_PWM_FREQ, mspeed);
		else
			m1_timer->setPWM(channelA, in_A_driver_pin, DC_MOTOR_PWM_FREQ, 0);

		if (steps > m2_counter)
			m2_timer->setPWM(channelD, in_D_driver_pin, DC_MOTOR_PWM_FREQ, mspeed);
		else
			m2_timer->setPWM(channelD, in_D_driver_pin, DC_MOTOR_PWM_FREQ, 0);
	}

	// Stop when done
	m1_timer->pause();
	m2_timer->pause();

}

void setup_controller(void)
{
	TIM_TypeDef *Timer2 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(in_A_driver_pin), PinMap_PWM);
	m1_timer = new HardwareTimer(Timer2);

	TIM_TypeDef *Timer3 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(in_C_driver_pin), PinMap_PWM);
	m2_timer = new HardwareTimer(Timer3);

	channelA = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(in_A_driver_pin), PinMap_PWM));
	channelB = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(in_B_driver_pin), PinMap_PWM));
	channelC = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(in_C_driver_pin), PinMap_PWM));
	channelD = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(in_D_driver_pin), PinMap_PWM));
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

	attachInterrupt(digitalPinToInterrupt(m1_sensor_pin), m1_speed_irq, m1_irq_mode);
	attachInterrupt(digitalPinToInterrupt(m2_sensor_pin), m2_speed_irq, m2_irq_mode);
}

// static void task_buzzer(void *pvParameters)
// {
// 	while(1) {
// 		Buzzer.Melody(Startup);
// 		Buzzer.Play=true;
// 	}
// }

void setup()
{
	Serial.begin();
	pinMode(in_A_driver_pin, OUTPUT);
	pinMode(in_B_driver_pin, OUTPUT);
	pinMode(in_C_driver_pin, OUTPUT);
	pinMode(in_D_driver_pin, OUTPUT);

	setup_controller();
	set_interrupt_speed_sensors();
	// set_buzzer();
#ifdef TEST_FWD_BWD
	digitalWrite(in_A_driver_pin, LOW);
	digitalWrite(in_B_driver_pin, LOW);
	digitalWrite(in_C_driver_pin, LOW);
	digitalWrite(in_D_driver_pin, LOW);
#else

	// TIM_TypeDef *Timer2 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(in_A_driver_pin), PinMap_PWM);
	// HardwareTimer *Timer2_instance = new HardwareTimer(Timer2);

	// uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(in_A_driver_pin), PinMap_PWM));
	// Timer2_instance->setPWM(channel, in_A_driver_pin, DC_MOTOR_PWM_FREQ, DUTY_A);
	// channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(in_B_driver_pin), PinMap_PWM));
	// Timer2_instance->setPWM(channel, in_B_driver_pin, DC_MOTOR_PWM_FREQ, DUTY_B);

	// TIM_TypeDef *Timer3 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(in_C_driver_pin), PinMap_PWM);
	// HardwareTimer *Timer3_instance = new HardwareTimer(Timer3);

	// channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(in_C_driver_pin), PinMap_PWM));
	// Timer3_instance->setPWM(channel, in_C_driver_pin, DC_MOTOR_PWM_FREQ, DUTY_C);
	// channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(in_D_driver_pin), PinMap_PWM));
	// Timer3_instance->setPWM(channel, in_D_driver_pin, DC_MOTOR_PWM_FREQ, DUTY_D);
#endif
	// create blink task
	xTaskCreate(vLEDFlashTask,
		    "Task1",
		    configMINIMAL_STACK_SIZE + 50,
		    NULL,
		    tskIDLE_PRIORITY + 2,
		    NULL);

	// create print task
	xTaskCreate(vPrintTask,
		    "Task2",
		    configMINIMAL_STACK_SIZE + 100,
		    NULL,
		    tskIDLE_PRIORITY + 1,
		    NULL);

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
// WARNING idle loop has a very small stack (configMINIMAL_STACK_SIZE)
// loop must never block
void loop()
{

#ifdef TEST_FWD_BWD
	// Full speed forward
#if TEST_FWD_BWD == 1
	digitalWrite(in_A_driver_pin, HIGH);
	digitalWrite(in_B_driver_pin, LOW);
#elif TEST_FWD_BWD == 2
	digitalWrite(in_C_driver_pin, HIGH);
	digitalWrite(in_D_driver_pin, LOW);
#endif
	delay(1000);

	// stop
	digitalWrite(in_A_driver_pin, LOW);
	digitalWrite(in_B_driver_pin, LOW);
	digitalWrite(in_C_driver_pin, LOW);
	digitalWrite(in_D_driver_pin, LOW);

	delay(1000);

	// Full speed backward
#if TEST_FWD_BWD == 1
	digitalWrite(in_A_driver_pin, LOW);
	digitalWrite(in_B_driver_pin, HIGH);
#elif TEST_FWD_BWD == 2
	digitalWrite(in_C_driver_pin, LOW);
	digitalWrite(in_D_driver_pin, HIGH);
#endif

	delay(1000);

	// stop
	digitalWrite(in_A_driver_pin, LOW);
	digitalWrite(in_B_driver_pin, LOW);
	digitalWrite(in_C_driver_pin, LOW);
	digitalWrite(in_D_driver_pin, LOW);

	delay(1000);
#else
	// Test Motor Movement  - Experiment with your own sequences here
	MoveForward(centimeters_to_step(50), 50);  // Forward half a metre at 255 speed
	delay(1000);				    // Wait one second
	MoveReverse(10, 100);			    // Reverse 10 steps at 255 speed
	delay(1000);				    // Wait one second
	MoveForward(10, 65);			    // Forward 10 steps at 150 speed
	delay(1000);				    // Wait one second
	MoveReverse(centimeters_to_step(25.4), 90); // Reverse 25.4 cm at 200 speed
	delay(1000);				    // Wait one second
	SpinRight(20, 100);			    // Spin right 20 steps at 255 speed
	delay(1000);				    // Wait one second
	SpinLeft(60, 65);			    // Spin left 60 steps at 175 speed
	delay(1000);				    // Wait one second
	MoveForward(1, 100);			    // Forward 1 step at 255 speed
	while (1)
		delay(5000);
#endif
}

//------------------------------------------------------------------------------
// high priority for blinking LED
static void vLEDFlashTask(void *pvParameters)
{
	UNUSED(pvParameters);
	pinMode(LED_PIN, OUTPUT);

	// Flash led every 200 ms.
	for (;;)
	{
		// Turn LED off.
		digitalWrite(LED_PIN, HIGH);

		// Sleep for 50 milliseconds.
		vTaskDelay((100L * configTICK_RATE_HZ) / 1000L);

		// Turn LED on.
		digitalWrite(LED_PIN, LOW);

		// Sleep for 150 milliseconds.
		vTaskDelay((1000L * configTICK_RATE_HZ) / 1000L);
	}
}
//------------------------------------------------------------------------------
static void vPrintTask(void *pvParameters)
{
	UNUSED(pvParameters);
	while (1)
	{
		// Sleep for one second.
		vTaskDelay(configTICK_RATE_HZ);

		Serial.print(F("Count MOTOR 1: "));
		Serial.println(m1_counter);

		Serial.print(F("Count MOTOR 2: "));
		Serial.println(m2_counter);
	}
}
//------------------------------------------------------------------------------
