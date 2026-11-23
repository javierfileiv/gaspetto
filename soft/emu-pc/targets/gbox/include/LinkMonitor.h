#pragma once

#include <cstdint>

/* The box pings the car once a second and watches the answers, instead of
 * trusting a single request: a link that stopped answering has to be told
 * apart from one that was never up. The monitor owns the schedule and the
 * state; the owner sends the request when service() asks for it, so nothing
 * here touches the radio and the whole thing runs on an injected clock */
class LinkMonitor {
public:
    enum class State : uint8_t { Unknown, Alive, Lost };

    /* service(): advance to nowMs; true when a ping must be sent now */
    bool service(uint32_t nowMs);

    /* noteAnswer(): an answer arrived at nowMs */
    void noteAnswer(uint32_t nowMs);

    State state() const
    {
        return _state;
    }

    uint32_t lastAnswerMs() const
    {
        return _lastAnswerMs;
    }

private:
    static constexpr uint32_t kPingPeriodMs = 1000;
    /* Two lost pings plus a margin: one missed answer must not call the link
     * broken, and the window stays well under the five seconds a verdict is
     * shown */
    static constexpr uint32_t kAliveWindowMs = 2500;
    /* A normal loop calls service() every few milliseconds: a longer gap means
     * a blocking animation or a scan, not a silent link */
    static constexpr uint32_t kBlockedGapMs = 500;

    State _state = State::Unknown;
    uint32_t _lastPingMs = 0;
    uint32_t _lastAnswerMs = 0;
    uint32_t _lastServiceMs = 0;
};
