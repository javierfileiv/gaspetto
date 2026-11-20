#include "LinkMonitor.h"

bool LinkMonitor::service(uint32_t nowMs)
{
    /* Unsigned subtraction: the clock wrapping after forty-nine days must not
     * freeze the state */
    if (_state == State::Alive && (nowMs - _lastAnswerMs) >= kAliveWindowMs) {
        _state = State::Lost;
    }

    /* One ping per period. The clock starts at zero, so the first one leaves
     * after a second; the boot health check already takes longer than that,
     * which is why the first loop pings at once on the real box */
    if ((nowMs - _lastPingMs) >= kPingPeriodMs) {
        _lastPingMs = nowMs;
        return true;
    }

    return false;
}

void LinkMonitor::noteAnswer(uint32_t nowMs)
{
    _lastAnswerMs = nowMs;
    _state = State::Alive;
}
