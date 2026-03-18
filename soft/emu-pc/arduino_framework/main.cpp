#include "Arduino.h"
#include "CarEvents.h"
#include "GaspettoCar_ino.h"
#include "Serial.h"
#include "implementations.h"

#include <atomic>
#include <chrono>
#include <thread>

static std::atomic<bool> running(true);

Event evt_copy;
EventPacket pkt;

/*  Millis simulation thread. */
static void emu_millisThread()
{
    while (running) {
        std::this_thread::sleep_for(std::chrono::microseconds(1)); /*  Increment every
                                                                      microsecond. */
        microsCounter.fetch_add(1);
    }
}

Event getEvent(void)
{
    evt_copy = event;
    return evt_copy;
}

#ifndef ARDUINO
extern "C" Event getEmulatedEvent(void);

extern "C" Event getEmulatedEvent(void)
{
    return getEvent();
}
#endif

static void keyboardInput(void)
{
    while (true) {
        if (Serial.available()) {
            char ch = Serial.read();
#ifdef GASPETTO_CAR
            gaspetto_car_input_switch(ch);
#ifndef USE_RADIO_CONTROLLER
            ISR();
#endif
#endif
#ifdef GASPETTO_BOX
            gaspetto_box_input_switch(ch);
            ISR();
#endif
#if NRF_SENDER
            nrf_sender_input_switch(ch);
#endif
            std::cout << "Exiting low-power mode due to keyboard input.\n";
            lowPowerMode.store(false);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); /*  Polling delay. */
    }
}

int main()
{
    Serial.println("Starting simulation...\n");
    /*  Start the simulation threads. */
    std::thread millisSim(emu_millisThread);
    std::thread keyboardSim(keyboardInput);
    /*  Setuo the system. */
    setup();
    /*  Main loop. */
    while (running) {
        loop();
        std::this_thread::sleep_for(std::chrono::milliseconds(10)); /*  Add a small delay to
                                                                       prevent CPU overuse.
                                                                     */
    }

    /*  Stop the threads. */
    running = false;
    millisSim.join();
    keyboardSim.join();
    return 0;
}
