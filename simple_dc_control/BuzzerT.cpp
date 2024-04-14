#include <Arduino.h>
#include "BuzzerT.h"

BuzzerT::BuzzerT(uint8_t buzzer_pin)
{
	this->buzzer_pin = buzzer_pin;
	TIM_TypeDef *TimerSTM = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(buzzer_pin), PinMap_PWM);
	this->channel = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(buzzer_pin), PinMap_PWM));
	this->TIMER = new HardwareTimer(TimerSTM);
	DisableBuzzer();
}

void BuzzerT::Melody(int (&image)[32][2])
{
	if (Play)
	{
		Freq = (int)image[counter][0];
		Dura = (int)image[counter][1];

		if (Freq != lastFreq)
		{
			if (Freq > 0)
			{
				if (Freq != lastConfig)
				{
					ConfigTimer(Freq);
					lastConfig = Freq;
				}
				EnableBuzzer();
			}
			else
			{
				DisableBuzzer();
			}
			lastFreq = Freq;
			lastStart = millis();
		}

		if (millis() >= (lastStart + Dura*10))
		{
			counter++;
			if (counter == 32)
			{
				DisableBuzzer();
				Play = false;
				counter = 0;
			}
		}
	}
}

void BuzzerT::ConfigTimer(int Freq)
{
	this->TIMER->pause();
	this->frequency = Freq;
}

void BuzzerT::DisableBuzzer() {
	this->TIMER->setPWM(this->channel, this->buzzer_pin, 0, 0);
	this->TIMER->pause();
	digitalWrite(this->buzzer_pin, HIGH);

}
void BuzzerT::EnableBuzzer()
{
	this->TIMER->setPWM(this->channel, this->buzzer_pin, this->frequency, 50);
}
