#include <iostream>
#include <thread>
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

// Thread-safe event queue using a preallocated array
class EventQueue {
private:
    static const size_t capacity = 10; // Fixed size of the queue
    Event events[capacity];           // Preallocated array for events
    size_t head = 0;                  // Index of the first element
    size_t tail = 0;                  // Index of the next available slot
    size_t count = 0;                 // Number of elements in the queue

public:
    // Enqueue an event
    bool enqueue(Event event) {
        if (count == capacity) {
            // Queue is full
            return false;
        }
        events[tail] = event;
        tail = (tail + 1) % capacity; // Wrap around if necessary
        ++count;
        return true;
    }

    // Dequeue an event
    bool dequeue(Event& event) {
        if (count == 0) {
            // Queue is empty
            return false;
        }
        event = events[head];
        head = (head + 1) % capacity; // Wrap around if necessary
        --count;
        return true;
    }

    // Check if the queue is empty
    bool empty() const {
        return count == 0;
    }

    // Check if the queue is full
    bool full() const {
        return count == capacity;
    }

    // Get the current size of the queue
    size_t size() const {
        return count;
    }
};

// Active Object class encapsulates the main logic
class ActiveObject {
private:
    State currentState = IDLE;
    EventQueue& eventQueue; // Reference to the shared event queue
    unsigned long lastDebounceTime = 0;
    const unsigned long debounceDelay = 50; // 50ms debounce

    std::atomic<bool>& lowPowerMode; // Reference to low-power mode flag

public:
    ActiveObject(EventQueue& queue, std::atomic<bool>& lowPower)
        : eventQueue(queue), lowPowerMode(lowPower) {}

    void handleEvent(Event event) {
        if (event == BUTTON_PRESSED) {
            if (currentState == IDLE) {
                std::cout << "Button pressed. Transitioning to PROCESSING state.\n";
                currentState = PROCESSING;
            } else {
                std::cout << "Button pressed, but system is not in IDLE state.\n";
            }
        }
    }

    void loop() {
        // Process events from the queue
        Event event;
        while (eventQueue.dequeue(event)) {
            handleEvent(event);
        }

        // Perform actions based on the current state
        switch (currentState) {
            case IDLE:
                enterLowPowerMode();
                break;

            case PROCESSING:
                std::cout << "Processing...\n";
                std::this_thread::sleep_for(std::chrono::seconds(1)); // Simulate work
                currentState = DONE;
                break;

            case DONE:
                std::cout << "Processing complete. Returning to IDLE state.\n";
                currentState = IDLE;
                break;
        }
    }

    void enterLowPowerMode() {
        std::cout << "Entering low-power mode...\n";
        lowPowerMode = true;
        while (lowPowerMode) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Simulate low-power sleep
        }
        std::cout << "Waking up from low-power mode...\n";
    }

    void debounceAndEnqueue(unsigned long currentTime) {
        if (currentTime - lastDebounceTime > debounceDelay) {
            lastDebounceTime = currentTime;
            if (!eventQueue.full()) {
                eventQueue.enqueue(BUTTON_PRESSED);
                lowPowerMode = false; // Wake the system
            } else {
                std::cout << "Event queue is full! Unable to enqueue event.\n";
            }
        }
    }
};

// Global variables
std::atomic<unsigned long> millisCounter(0); // Simulated millis()
EventQueue eventQueue;                       // Shared event queue
std::atomic<bool> running(true);             // Flag to stop threads
std::atomic<bool> lowPowerMode(false);       // Simulates low-power mode

// Simulated millis function
unsigned long millis() {
    return millisCounter.load();
}

// Button press simulation thread
void buttonThread(ActiveObject& activeObject) {
    while (running) {
        std::this_thread::sleep_for(std::chrono::seconds(2)); // Simulate button press every 2 seconds
        std::cout << "Simulating button press...\n";
        activeObject.debounceAndEnqueue(millis());
    }
}

// Millis simulation thread
void millisThread() {
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); // Increment every millisecond
        millisCounter.fetch_add(1);
    }
}

int main() {
    // Create the ActiveObject instance
    ActiveObject activeObject(eventQueue, lowPowerMode);

    // Start the simulation threads
    std::thread millisSim(millisThread);
    std::thread buttonSim(buttonThread, std::ref(activeObject));

    // Main loop
    while (running) {
        activeObject.loop();
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Add a small delay to prevent CPU overuse
    }

    // Stop the threads
    running = false;
    millisSim.join();
    buttonSim.join();

    return 0;
}