#include "ProcessingState.h"

#include "GBox.h"

void ProcessingState::enter()
{
    GBox *box = static_cast<GBox *>(active_object_);
    CommandPacket packet{};
    bool isEmpty = false;
    const bool appending = box->isProgramRunning();

    box->restoreFromStop();
    box->setSensorRailEnabled(true);

    /* The scan animation belongs to a fresh program. When one is already
     * running, the yellow breath keeps the strip instead: the new program is
     * only added to the car's queue, and the car keeps working */
    if (!appending) {
        box->runScanAnimation();
    }
    box->scanSlots();

    const bool programReady = box->buildProgram(packet, isEmpty);
    box->setSensorRailEnabled(false);

    if (!programReady) {
        if (isEmpty) {
            box->runEmptyBoardAnimation();
        } else {
            box->runBuildErrorAnimation();
        }
        box->transitionTo(StateId::IDLE);
        return;
    }

    const bool radioOk = box->sendProgram(packet);
    if (radioOk && !appending) {
        box->runSuccessAnimation();
    } else if (!radioOk) {
        box->runRfErrorAnimation();
    }

    box->transitionTo(StateId::IDLE);
}

void ProcessingState::processEvent(Event &evt)
{
    GBox *box = static_cast<GBox *>(active_object_);

    switch (evt.getEventId()) {
    case EventId::BUTTON_PRESSED:
        /* Button pressed during processing - interrupt and send clear queue command */
        box->interruptScanning();
        box->sendClearQueueCommand();
        box->transitionTo(StateId::IDLE);
        break;
    default:
        break;
    }
}
