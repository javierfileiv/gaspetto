#include <RF24.h>
#include <SPI.h>
#include <nRF24L01.h>

/*  Pin Definitions for nRF24L01+. */
#define CE_PIN PA8
#define CSN_PIN PA9

/*  RF24 Object. */
RF24 radio(CE_PIN, CSN_PIN);

/*  nRF24L01+ Configuration. */
const byte address[6] = "00001"; /*  Address for communication. */

/*  Pin Definitions for ADC and GPIO. */
const uint8_t adcPins[4] = { PB0, PB1, PB10, PB11 }; /*  ADC Channels. */
const uint8_t groupPins[20] = { PA0,  PA1,  PA2,  PA3,  PA4,  PA5, PA6, PA7,  PB12, PB13,
                                PB14, PB15, PC13, PC14, PC15, PA8, PA9, PA10, PA11, PA12 }; /*  GPIO
                                                                                             * Pins
                                                                                             * for
                                                                                             * GND.
                                                                                             */
/*  control. */

/*  Number of groups and sensors per group. */
const uint8_t numGroups = 5;
const uint8_t sensorsPerGroup = 4;

/*  ADC Values array. */
uint16_t adcValues[4] = { 0 }; /*  Only store one group's readings. */

void setup()
{
    /*  Initialize Serial for debugging. */
    Serial.begin(115200);

    /*  Initialize nRF24L01+. */
    if (!radio.begin()) {
        logln("nRF24L01+ module not detected. Check connections.");
        while (1)
            ;
    }
    radio.openWritingPipe(address); /*  Set the communication address. */
    radio.setPALevel(RF24_PA_LOW); /*  Set Power Amplifier level to low. */
    radio.setDataRate(RF24_1MBPS); /*  Set data rate to 1Mbps. */
    radio.setRetries(15, 15); /*  Set retries (delay, count). */
    radio.stopListening(); /*  Set module to transmit mode. */

    /*  Set GPIO pins as OUTPUT and set them HIGH (disable all sensors). */
    for (uint8_t i = 0; i < 20; i++) {
        pinMode(groupPins[i], OUTPUT);
        digitalWrite(groupPins[i], HIGH); /*  Disable sensor. */
    }

    /*  Set ADC pins as INPUT. */
    for (uint8_t i = 0; i < 4; i++) {
        pinMode(adcPins[i], INPUT_ANALOG);
    }
}

void loop()
{
    /*  Iterate over each group. */
    for (uint8_t group = 0; group < numGroups; group++) {
        /*  Enable the current group of sensors. */
        for (uint8_t i = 0; i < sensorsPerGroup; i++) {
            digitalWrite(groupPins[group * sensorsPerGroup + i], LOW);
        }

        /*  Small delay to allow stabilization. */
        delayMicroseconds(50);

        /*  Read ADC values for the current group. */
        for (uint8_t i = 0; i < sensorsPerGroup; i++) {
            adcValues[i] = analogRead(adcPins[i]);
        }

        /*  Disable the current group of sensors. */
        for (uint8_t i = 0; i < sensorsPerGroup; i++) {
            digitalWrite(groupPins[group * sensorsPerGroup + i], HIGH);
        }

        /*  Transmit the ADC values via nRF24L01+. */
        if (radio.write(&adcValues, sizeof(adcValues))) {
            log("Group ");
            log(group);
            logln(" data sent successfully!");
        } else {
            log("Group ");
            log(group);
            logln(" data transmission failed.");
        }

        /*  Small delay before switching to the next group. */
        delay(100); /*  Adjust as needed. */
    }

    /*  Delay before the next cycle. */
    delay(1000);
}
