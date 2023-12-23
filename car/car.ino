#include <Arduino.h>
#include <STM32FreeRTOS.h>


const uint8_t LED_PIN = LED_BUILTIN;
const uint8_t INT1 = PB3;
const uint8_t INT2 = PA15;
const uint8_t INT3 = PB5;
const uint8_t INT4 = PB4;

volatile uint32_t count = 0;

const int pinHall = A1;
#define Hall_Sensor_Gnd_Pin_0 5

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
void setup()
{
	Serial.begin(9600);
	// wait for Leonardo
	while (!Serial)
	{
	}

	pinMode(pinHall, INPUT);
	pinMode(INT1, OUTPUT);
	pinMode(INT2, OUTPUT);
	pinMode(INT3, OUTPUT);
	pinMode(INT4, OUTPUT);
	digitalWrite(INT1, LOW);
	digitalWrite(INT2, LOW);
	digitalWrite(INT3, LOW);
	digitalWrite(INT4, LOW);




	Serial.begin(9600);
	pinMode(Hall_Sensor_Gnd_Pin_0, OUTPUT);
	digitalWrite(Hall_Sensor_Gnd_Pin_0, LOW);

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
		// must insure increment is atomic
		// in case of context switch for print
		noInterrupts();
		count++;
		interrupts();
		for (int i = 0; i < 10; i++)
		{

			measure += analogRead(pinHall);
		}
		measure /= 10;
		// voltage in mV
		float outputV = measure * 5000.0 / 1023;
		Serial.print("Output Voltaje = ");
		Serial.print(outputV);
		Serial.print(" mV   ");
		Serial.print(" \n");

		// flux density
		float magneticFlux = outputV * 53.33 - 133.3;
		Serial.print("Magnetic Flux Density = ");
		Serial.print(magneticFlux);
		Serial.print(" mT");
		Serial.print(" \n");
		delay(2000);
	}
}
