#include "ActiveObject.h"
#include "Arduino.h"
#include "CarEvents.h"
#include "CarStates.h"
#include "Context.h"
#include "Log.h"

#ifndef ARDUINO
extern std::atomic<bool> lowPowerMode;
#endif

constexpr uint8_t BOX_MAX_STATES = static_cast<uint8_t>(StateId::MAX_STATE_ID);

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

private:
    Context &_ctx;
};
