#include <Arduino.h>
#include <SPI.h>
#include <RF24.h>
#include "pin_definitions.h"


// Initialize radio object
RF24 radio(NRF24_CE, NRF24_CSN);

// ==========================================
// 2. SETUP
// ==========================================
void setup() {
  // Initialize USB CDC Serial Monitor
  Serial.begin(115200);
  delay(3000); // Wait for Serial Monitor to open
  Serial.println("\n\n--- HARDWARE TEST STARTED ---");

  // --- LED Configuration ---
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH); // Turn OFF LED initially

  // --- MOSFETs Configuration (Active-Low Logic) ---
  // IMPORTANT: PB14 must be OPEN DRAIN to safely handle 5V!
  pinMode(PIN_MOSFET_5V_LEDS, OUTPUT_OPEN_DRAIN);
  pinMode(PIN_MOSFET_3V3_SENSORS, OUTPUT_OPEN_DRAIN);

  // Turn OFF all rails by default (HIGH = OFF for P-Channel MOSFETs)
  digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);
  digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH);
  Serial.println("1. MOSFETs initialized and TURNED OFF.");

  // --- nRF24L01+ SPI Test ---
  Serial.println("\n2. Testing nRF24L01+ SPI communication...");

  // Initialize SPI bus pins for STM32
  SPI.setMISO(PA6);
  SPI.setMOSI(PA7);
  SPI.setSCLK(PA5);
  SPI.begin();

  if (radio.begin()) {
    Serial.println("   [SUCCESS] nRF24L01+ detected! SPI is working.");
    radio.setPALevel(RF24_PA_LOW); // Set low power for testing
    radio.printDetails();          // Print chip info to Serial
  } else {
    Serial.println("   [ERROR] nRF24L01+ not found!");
    Serial.println("   -> Check soldering on PA3, PA4, PA5, PA6, PA7.");
    Serial.println("   -> Check if 3.3V is reaching the nRF board.");
  }
  Serial.println("\n--- POWER RAILS TEST CYCLE ---");
}

#define DELAY 3000
// ==========================================
// 3. MAIN LOOP (Test Cycle)
// ==========================================
void loop() {
  // --- Test 3.3V Rail (Sensors) ---
  digitalWrite(PIN_LED, HIGH);         // Turn ON board LED
  Serial.println("-> TURNING ON 5V_SWITCHED (LED Cache)...");
  digitalWrite(PIN_MOSFET_5V_LEDS, LOW);      // Turn ON Q1
  Serial.println("-> TURNING ON 3V3_SWITCHED (Sensors)...");
  digitalWrite(PIN_MOSFET_3V3_SENSORS, LOW);     // Turn ON Q2

  // Quick double blink to indicate cycle restart
  for(int i = 0; i < 2; i++) {
      digitalWrite(PIN_LED, LOW);
      delay(100);
      digitalWrite(PIN_LED, HIGH);
      delay(100);
  }
  delay(DELAY);                                // Keep ON for 10 seconds

  digitalWrite(PIN_LED, HIGH);        // Turn OFF board LED
  Serial.println("-> TURNING OFF 5V_SWITCHED...");
  digitalWrite(PIN_MOSFET_5V_LEDS, HIGH);     // Turn OFF Q1
  Serial.println("-> TURNING OFF 3V3_SWITCHED...");
  digitalWrite(PIN_MOSFET_3V3_SENSORS, HIGH);    // Turn OFF Q2
  // Quick double blink to indicate cycle restart
  for(int i = 0; i < 2; i++) {
      digitalWrite(PIN_LED, LOW);
      delay(100);
      digitalWrite(PIN_LED, HIGH);
      delay(100);
  }

  Serial.println("Cycle complete. Restarting in 2 seconds.");
  delay(DELAY);
}
