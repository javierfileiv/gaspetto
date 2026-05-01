#pragma once

#include "ActiveObject.h"
#include "Arduino.h"
#include "CarEvents.h"
#include "CarStates.h"
#include "CommandPacket.h"
#include "Context.h"
#include "Log.h"

#include <array>
#include <atomic>

#ifdef ARDUINO
#include <Adafruit_NeoPixel.h>
#endif

#ifndef ARDUINO
extern std::atomic<bool> lowPowerMode;
#endif

constexpr uint8_t BOX_MAX_STATES = static_cast<uint8_t>(StateId::MAX_STATE_ID);

struct AdcDecodeEntry {
    uint16_t minValue;
    uint16_t maxValue;
    BoxPieceId piece;
    const char *label;
};

struct SlotReading {
    uint16_t rawValue;
    BoxPieceId piece;
};

struct AdsRouteInfo {
    bool usesI2c3;
    uint8_t address;
    uint8_t channel;
};

class GaspettoBox : public GenericActiveObject<StateId, Event, BOX_MAX_STATES> {
public:
    /** GaspettoBox(): Constructor for the GaspettoBox class.
     *  @ctx: Reference to the Context instance containing dependencies.
     */
    GaspettoBox(Context &ctx);
    /** Init(): Initialize the GaspettoBox state machine.
     *  @initialStateId: The initial state ID to start the state machine.
     */
    void init(StateId initialStateId = StateId::IDLE);

    /** postEvent(): Post an event to the event queue.
     * @evt: The event to be posted.
     * @return 0 on success, -1 on failure.
     */
    int postEvent(Event evt) override;

    /** work(): Process the next event in the event queue.
     *  Delegates to the current state.
     */
    void work() override;

    /** enterLowPowerMode(): Enter low power mode.
     */
    void enterLowPowerMode() override;

    /** debounceAndEnqueue(): Debounce the event and enqueue it if valid.
     * @evt: The event to be debounced and enqueued.
     * @currentTime: The current time in milliseconds.
     */
    void debounceAndEnqueue(Event &evt, unsigned long currentTime);

    void initHardware();
#ifndef ARDUINO
    void injectBoardPieces(const BoxBoardPieces &boardPieces);
    void injectRawAdcValues(const std::array<uint16_t, BOX_TOTAL_SLOTS> &rawValues);
#endif

    static bool buildProgramFromPieces(const BoxBoardPieces &boardPieces, CommandPacket &packet,
                                       bool &isEmpty);

    void restoreFromStop();
    void setSensorRailEnabled(bool enabled);
    void runScanAnimation();
    void runSuccessAnimation();
    void runBuildErrorAnimation();
    void runEmptyBoardAnimation();
    void runRfErrorAnimation();
    void scanSlots();
    bool buildProgram(CommandPacket &packet, bool &isEmpty);
    bool sendProgram(const CommandPacket &packet);
    bool sendClearQueueCommand();
    bool isCurrentlyScanning() const;
    void startScanning();
    void interruptScanning();

    static bool routeForSlot(std::size_t slot, AdsRouteInfo &route);

protected:
    virtual void delayMs(int ms) const;

private:
    void prepareForStop();
    void SystemClock_Config();
    void configurePins();
    void setLedRailEnabled(bool enabled);
    void blackoutLeds();
    uint16_t rawValueForPiece(BoxPieceId piece) const;
    BoxPieceId decodePiece(uint16_t rawValue) const;
    const char *pieceToString(BoxPieceId piece) const;

    Context &_ctx;
    std::array<SlotReading, BOX_TOTAL_SLOTS> lastScan;
#ifdef ARDUINO
    Adafruit_NeoPixel leds_;
#endif
    bool initialized_;
    unsigned long lastDebounceTime_;
    bool isScanning_;
    static constexpr unsigned long debounceDelay = 50; /* 50ms debounce delay */
};
