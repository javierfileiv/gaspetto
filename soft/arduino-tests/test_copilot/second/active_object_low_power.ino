#include <STM32LowPower.h> // Include the STM32 Low Power library
#include <Queue.h>         // Include ArduinoQueue library

// Define events
enum Event {
    BUTTON_PRESSED
};

// Define states
enum State {
    IDLE,
    PROCESSING,
    DONE
};

// Create a queue to hold events
ArduinoQueue<Event> eventQueue(10); // Maximum of 10 events in the queue

// Pin definitions
const int LED_PIN = PC13;  // Built-in LED (STM32 Bluepill)
const int BUTTON_PIN = PA0; // Button pin (use a pin with EXTI capability)

// Tracks the current state of the system
State currentState = IDLE;

// Debounce variables
volatile bool buttonPressedFlag = false; // Set by ISR
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50; // 50ms debounce

void setup() {
    // Initialize Low Power library
    LowPower.begin();

    // Configure pins
    pinMode(LED_PIN, OUTPUT);       // Set LED pin as output
    pinMode(BUTTON_PIN, INPUT_PULLUP); // Set button pin as input with pull-up resistor

    // Properly initialize lastDebounceTime to the current time
    lastDebounceTime = millis();

    // Attach interrupt for the button press
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);

    Serial.begin(9600); // Initialize serial communication
    Serial.println("System initialized.");
}

void loop() {
    // Check if the button is flagged as pressed
    if (buttonPressedFlag) {
        unsigned long currentTime = millis();
        if (currentTime - lastDebounceTime > debounceDelay) {
            lastDebounceTime = currentTime; // Update debounce time
            eventQueue.enqueue(BUTTON_PRESSED); // Push event to the queue
            buttonPressedFlag = false; // Reset the flag
        }
    }

    // Process events from the queue
    while (!eventQueue.isEmpty()) {
        Event event = eventQueue.dequeue(); // Get the next event
        handleEvent(event);                 // Process the event
    }

    // Perform actions based on the current state
    switch (currentState) {
        case IDLE:
            Serial.println("Entering low-power mode...");
            LowPower.stop(); // Enter low-power mode
            break;

        case PROCESSING:
            Serial.println("Processing...");
            delay(1000); // Simulate some processing work
            currentState = DONE; // Transition to DONE state
            break;

        case DONE:
            Serial.println("Processing complete. Returning to IDLE state.");
            currentState = IDLE; // Transition back to IDLE state
            break;
    }
}

// Interrupt Service Routine (ISR) for the button press
void handleButtonPress() {
    buttonPressedFlag = true; // Only set a flag
}

// Handle events (update states based on events)
void handleEvent(Event event) {
    if (event == BUTTON_PRESSED) {
        if (currentState == IDLE) {
            Serial.println("Button pressed. Transitioning to PROCESSING state.");
            currentState = PROCESSING; // Transition to PROCESSING state
        } else {
            Serial.println("Button pressed, but system is not in IDLE state.");
        }
    }
}