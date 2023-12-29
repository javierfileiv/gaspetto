#include <Arduino.h>
#include <STM32FreeRTOS.h>

#define DC_MOTOR_PWM_FREQ	20000 /* 20 Khz PWM frequency. */
#if defined(TIM1) // PB0 and PB1
TIM_TypeDef *_timer1 = TIM1;
#else
#error "Choose another timer for your device"
#endif

#if defined(TIM2) // PB10 and PB11
TIM_TypeDef *_timer2_ = TIM2;
#else
#error "Choose another timer for your device"
#endif

#if defined(TIM3) // PB4 and PB5
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
#define OPTION
#ifdef OPTION
const uint8_t IN_A = PB4;  // green  TIM3_CH1
const uint8_t IN_B = PB5;  // orange   TIM3_CH2
const uint8_t IN_C = PB3;  // violet   TIM2_CH2
const uint8_t IN_D = PA15; // brown    TIM2_CH1_ETR
#else
const uint8_t IN_B = PB1; // TIM1_CH3N
const uint8_t IN_A = PB0; // TIM1_CH2N
const uint8_t IN_C = PB11; // TIM2_CH4
const uint8_t IN_D = PB10; // TIM2_CH3
#endif

#define DUTY_A	0
#define DUTY_B	0
#define DUTY_C	50
#define DUTY_D	0
void setup()
{
	Serial.begin(9600);

	pinMode(IN_A, OUTPUT);
	pinMode(IN_B, OUTPUT);
	pinMode(IN_C, OUTPUT);
	pinMode(IN_D, OUTPUT);
#ifdef OPTION
	TIM_TypeDef *Timer3 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(IN_A), PinMap_PWM);
	HardwareTimer *Timer3_instance = new HardwareTimer(Timer3);
	TIM_TypeDef *Timer2 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(IN_C), PinMap_PWM);
	HardwareTimer *Timer2_instance = new HardwareTimer(Timer2);

	uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(IN_A), PinMap_PWM));
	Timer3_instance->setPWM(channel, IN_A, DC_MOTOR_PWM_FREQ, DUTY_A);

	channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(IN_B), PinMap_PWM));
	Timer3_instance->setPWM(channel, IN_B, DC_MOTOR_PWM_FREQ, DUTY_B);


	channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(IN_C), PinMap_PWM));
	Timer2_instance->setPWM(channel, IN_C, DC_MOTOR_PWM_FREQ, DUTY_C);

	channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(IN_D), PinMap_PWM));
	Timer2_instance->setPWM(channel, IN_D, DC_MOTOR_PWM_FREQ, DUTY_D);
#else
	TIM_TypeDef *Timer1 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(IN_B), PinMap_PWM);
	HardwareTimer *Timer1_instance = new HardwareTimer(Timer1);

	uint32_t channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(IN_B), PinMap_PWM));
	Timer1_instance->setPWM(channel, IN_B, 1000, 90); // 1 KHertz, 90% dutycycle

	channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(IN_A), PinMap_PWM));
	Timer1_instance->setPWM(channel, IN_A, 1000, 20); // 1 KHertz, 20% dutycycle


	TIM_TypeDef *Timer2 = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(IN_C), PinMap_PWM);
	HardwareTimer *Timer2_instance = new HardwareTimer(Timer2);

	channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(IN_C), PinMap_PWM));
	Timer2_instance->setPWM(channel, IN_C, 5, 10); // 5 Hertz, 10% dutycycle

	channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(IN_D), PinMap_PWM));
	Timer2_instance->setPWM(channel, IN_D, 5, 90); // 5 Hertz, 90% dutycycle
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
	// we measure 10 times and make the mean
	long measure = 0;
	while (1)
	{
		delay(2000);
	}
}
