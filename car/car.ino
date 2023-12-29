#include <Arduino.h>
#include <STM32FreeRTOS.h>

#if defined(TIM2) // PB10 and PB11
TIM_TypeDef *_timer2_ = TIM2;
#else
#error "Choose another timer for your device"
#endif

#if defined(TIM3) // PB0 and PB1
TIM_TypeDef *_timer3_ = TIM3;
#else
#error "Choose another timer for your device"
#endif

const uint8_t LED_PIN = LED_BUILTIN;

volatile uint32_t count = 0;

// handle for blink task
TaskHandle_t blink;

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

		// Print count for previous second.
		Serial.print(F("Count: "));
		Serial.print(count);

		// Print unused stack for threads.
		Serial.print(F(", Unused Stack: "));
		Serial.print(uxTaskGetStackHighWaterMark(blink));
		Serial.print(' ');
		Serial.print(uxTaskGetStackHighWaterMark(0));
		Serial.print(' ');
		Serial.println(uxTaskGetStackHighWaterMark(xTaskGetIdleTaskHandle()));

		// Zero count.
		count = 0;
	}
}
//------------------------------------------------------------------------------

// #define TEST_FWD_BWD
/* motor 1 is on the left with speed sensor down. */
#define MOTOR_N 1
#define DC_MOTOR_PWM_FREQ	30 /* 30 hz PWM frequency. */

const uint8_t SENSOR_M1 = PC14;
const uint8_t SENSOR_M2 = PC15;

const uint8_t IN_A = PB11; // green  TIM2_CH4
const uint8_t IN_B = PB10; // orange   TIM2_CH3
const uint8_t IN_C = PB1; // violet   TIM3_CH4
const uint8_t IN_D = PB0; // brown    TIM3_CH3

#define DUTY_A	50
#define DUTY_B	1
#define DUTY_C	0
#define DUTY_D	0
void setup()
{
	Serial.begin();

	pinMode(SENSOR_M1, INPUT);
	pinMode(SENSOR_M2, INPUT);

	pinMode(IN_A, OUTPUT);
	pinMode(IN_B, OUTPUT);
	pinMode(IN_C, OUTPUT);
	pinMode(IN_D, OUTPUT);

#ifdef TEST_FWD_BWD
	digitalWrite(IN_A, LOW);
	digitalWrite(IN_B, LOW);
	digitalWrite(IN_C, LOW);
	digitalWrite(IN_D, LOW);
#else

	TIM_TypeDef *Timer2 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(IN_A), PinMap_PWM);
	HardwareTimer *Timer2_instance = new HardwareTimer(Timer2);

	uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(IN_A), PinMap_PWM));
	Timer2_instance->setPWM(channel, IN_A, DC_MOTOR_PWM_FREQ, DUTY_A);

	channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(IN_B), PinMap_PWM));
	Timer2_instance->setPWM(channel, IN_B, DC_MOTOR_PWM_FREQ, DUTY_B);


	TIM_TypeDef *Timer3 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(IN_C), PinMap_PWM);
	HardwareTimer *Timer3_instance = new HardwareTimer(Timer3);

	channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(IN_C), PinMap_PWM));
	Timer3_instance->setPWM(channel, IN_C, DC_MOTOR_PWM_FREQ, DUTY_C);

	channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(IN_D), PinMap_PWM));
	Timer3_instance->setPWM(channel, IN_D, DC_MOTOR_PWM_FREQ, DUTY_D);
#endif
	// create blink task
	xTaskCreate(vLEDFlashTask,
		    "Task1",
		    configMINIMAL_STACK_SIZE + 50,
		    NULL,
		    tskIDLE_PRIORITY + 2,
		    &blink);

	// create print task
	xTaskCreate(vPrintTask,
		    "Task2",
		    configMINIMAL_STACK_SIZE + 100,
		    NULL,
		    tskIDLE_PRIORITY + 1,
		    NULL);

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
#if MOTOR_N==1
	digitalWrite(IN_A, HIGH);
	digitalWrite(IN_B, LOW);
#elif MOTOR_N==2
	digitalWrite(IN_C, HIGH);
	digitalWrite(IN_D, LOW);
#endif
	delay(1000);


	// stop
	digitalWrite(IN_A, LOW);
	digitalWrite(IN_B, LOW);
	digitalWrite(IN_C, LOW);
	digitalWrite(IN_D, LOW);

	delay(5000);

	// Full speed backward
#if MOTOR_N==1
	digitalWrite(IN_A, LOW);
	digitalWrite(IN_B, HIGH);
#elif MOTOR_N==2
	digitalWrite(IN_C, LOW);
	digitalWrite(IN_D, HIGH);
#endif

	delay(1000);

	// stop
	digitalWrite(IN_A, LOW);
	digitalWrite(IN_B, LOW);
	digitalWrite(IN_C, LOW);
	digitalWrite(IN_D, LOW);

	delay(5000);
#else
	delay(5000);
#endif
}
