#include <Arduino.h>
#include <stdint.h>

const int pwmPin1 = PB0; // Timer 2 Channel 3
const int pwmPin2 = PB1; // Timer 2 Channel 4

void setup()
{
    pinMode(pwmPin1, OUTPUT); // Set PB10 as output
    pinMode(pwmPin2, OUTPUT); // Set PB11 as output
}

void loop()
{
    // Example PWM control: Sweep the duty cycle
    analogWriteFrequency(75); // Set frequency to 75Hz
    // freq 1KHz
    // 255 --100 %
    // 240--- 95 %
    // 230--- 90 %
    // 225--- 85 %
    // 220--- 85 %
    // 210--- 85 %
    // 209--- 85 %
    // 208--- 80 %
    // 200--- 75 %
    // 25 --- 10 %
    // 20 --- 5 %
    // 15 --- 5 %
    const uint8_t dutyCycle1 = 10; // Set the desired duty cycle (0-255 for 8-bit resolution)
    const uint8_t dutyCycle2 = 50; // Set the desired duty cycle (0-255 for 8-bit resolution)
    analogWrite(pwmPin1, map(dutyCycle1, 0, 100, 0,
                             255)); // Set PWM duty cycle (0-255 for 8-bit resolution)
    analogWrite(pwmPin2, map(dutyCycle2, 0, 100, 0,
                             255)); // Set PWM duty cycle (0-255 for 8-bit
                                    // resolution) Channel 3 (PB10)
                                    //   for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle+=15) {
                                    //     analogWrite(pwmPin1, dutyCycle); // Set PWM duty cycle
                                    //     (0-255 for 8-bit resolution) delay(10);
                                    //   }
                                    //   for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle-=15) {
                                    //     analogWrite(pwmPin1, dutyCycle);
                                    //     delay(10);
                                    //   }

    //   // Channel 4 (PB11) - Let's make it out of phase for visualization
    //   for (int dutyCycle = 255; dutyCycle >= 0; dutyCycle--) {
    //     analogWrite(pwmPin2, dutyCycle);
    //     delay(10);
    //   }
    //   for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
    //     analogWrite(pwmPin2, dutyCycle);
    //     delay(10);
    //   }
}
