#include "ProcessingState.h"

#include "GaspettoBox.h"

void ProcessingState::enter()
{
    GaspettoBox *box = static_cast<GaspettoBox *>(active_object_);
    CommandPacket packet{};
    bool isEmpty = false;

    box->restoreFromStop();
    box->setSensorRailEnabled(true);
    box->runScanAnimation();
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
    if (radioOk) {
        box->runSuccessAnimation();
    } else {
        box->runRfErrorAnimation();
    }

    box->transitionTo(StateId::IDLE);
}

void ProcessingState::processEvent(Event &evt)
{
    GaspettoBox *box = static_cast<GaspettoBox *>(active_object_);

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
