#include "Arduino.h"
#include "Event.h"
#include "GaspettoCar_ino.h"
#include "Serial.h"
#include "implementations.h"

#include <atomic>
#include <thread>

static std::atomic<bool> running(true);

Event event;
Event evt_copy;
EventPacket pkt;

/*  Millis simulation thread. */
static void emu_millisThread()
{
    while (running) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1)); /*  Increment every
                                                                      millisecond. */
        millisCounter.fetch_add(1);
    }
}

Event getEvent(void)
{
    evt_copy = event;
    return evt_copy;
}

static void keyboardInput(void)
{
    while (true) {
        if (Serial.available()) {
            char ch = Serial.read();
            (void)ch;
#ifdef GASPETTO_CAR
            gaspetto_car_input_switch(ch);
#endif
#ifdef GASPETTO_BOX
            gaspetto_box_input_switch(ch);
            ISR();
#endif
#if NRF_SENDER
            nrf_sender_input_switch(ch);
#endif
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
