#include <iostream>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <atomic>

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

// Create a thread-safe queue to hold events
class EventQueue {
private:
    std::queue<Event> events;
    std::mutex queueMutex;
    std::condition_variable condition;

public:
    void enqueue(Event event) {
        {
            std::lock_guard<std::mutex> lock(queueMutex);
            events.push(event);
        }
        condition.notify_one();
    }

    Event dequeue() {
        std::unique_lock<std::mutex> lock(queueMutex);
        condition.wait(lock, [this]() { return !events.empty(); });
        Event event = events.front();
        events.pop();
        return event;
    }

    bool isEmpty() {
        std::lock_guard<std::mutex> lock(queueMutex);
        return events.empty();
    }
};

// Global variables
std::atomic<unsigned long> millisCounter(0); // Simulated millis()
EventQueue eventQueue;                       // Global event queue
std::atomic<bool> running(true);             // Flag to stop threads
std::atomic<bool> lowPowerMode(false);       // Simulates low-power mode
State currentState;                   // Current state of the system
std::atomic<bool> buttonPressedFlag(false);  // Simulates button interrupt flag
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 50;      // 50ms debounce

// Simulated millis() function
unsigned long millis() {
    return millisCounter.load();
}

// Simulated interrupt handler for button press
void handleButtonPress() {
    unsigned long currentTime = millis();
    if (currentTime - lastDebounceTime > debounceDelay && !buttonPressedFlag) { // Debounce check
        lastDebounceTime = currentTime;                   // Update debounce time
        eventQueue.enqueue(BUTTON_PRESSED);              // Push event to the queue
        buttonPressedFlag = true;                        // Set the flag
        lowPowerMode = false;                            // Simulate wake-up from low-power mode
    }
}

// Button press simulation thread
void buttonThread() {
    while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(2)); // Simulate button press every 2 seconds
        std::cout << "Simulating button press...\n";
        handleButtonPress(); // Call the "interrupt handler"
    }
}

// Millis simulation thread
void millisThread() {
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Increment every millisecond
        millisCounter.fetch_add(1);
    }
}

// Simulated low-power mode
void enterLowPowerMode() {
    std::cout << "Entering low-power mode...\n";
    lowPowerMode = true;
    while (lowPowerMode && running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Simulate low-power sleep
    }
    std::cout << "Waking up from low-power mode...\n";
}

// Handle events (update states based on events)
void handleEvent(Event event) {
    if (event == BUTTON_PRESSED) {
        if (currentState == IDLE) {
            std::cout << "Button pressed. Transitioning to PROCESSING state.\n";
            currentState = PROCESSING; // Transition to PROCESSING state
        } else {
            std::cout << "Button pressed, but system is not in IDLE state.\n";
        }
    }
}

void setup() {
    // Initialize the system
    std::cout << "System initialized.\n";
    currentState = IDLE; // Start in IDLE state
}

// Main processing loop
void loop() {
    // Process events from the queue
    while (!eventQueue.isEmpty()) {
        Event event = eventQueue.dequeue(); // Get the next event
        handleEvent(event);                 // Process the event
    }

    // Perform actions based on the current state
    switch (currentState) {
        case IDLE:
            enterLowPowerMode(); // Enter low-power mode
            break;

        case PROCESSING:
            std::cout << "Processing...\n";
            std::this_thread::sleep_for(std::chrono::seconds(20)); // Simulate some processing work
            currentState = DONE; // Transition to DONE state
            break;

        case DONE:
            std::cout << "Processing complete. Returning to IDLE state.\n";
            currentState = IDLE; // Transition back to IDLE state
            break;
    }
}

int main() {
    // Start the simulation threads
    std::thread millisSim(millisThread);
    std::thread buttonSim(buttonThread);

    // Main loop
    while (running) {
        loop();
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Add a small delay to prevent CPU overuse
    }

    // Stop the threads
    running = false;
    millisSim.join();
    buttonSim.join();

    return 0;
}