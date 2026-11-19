#pragma once

#include "Arduino.h"

#ifdef ARDUINO
#include <Print.h>
/* string.h, not <cstring>: the AVR toolchain ships no C++ standard
 * library headers at all, <cstring> simply does not exist there */
#include <string.h>

#ifdef GASPETTO_LOG_OVER_NRF24
#include "RF24.h"
/* The tag lives in shared_types: that is the one include path every
 * project that logs over the radio already carries */
#include "PacketTag.h"
#endif
#else
#include "Serial.h"

#include <iostream>
#include <string>
#endif

#ifdef ARDUINO
namespace gaspetto_log
{
#ifdef GASPETTO_LOG_OVER_NRF24
/* Radio payload width of a log fragment: every Gaspetto packet is 32 bytes.
 * Named because an array parameter decays to a pointer, so sizeof() on one
 * would give the pointer size (2 on AVR) instead of this */
constexpr uint8_t kLogPayloadSize = 32;
/* A line travels in 30-byte fragments: two bytes of the payload name the tag
 * and the fragment flags */
constexpr uint8_t kLogFragmentData = 30;
/* A burst of logs waits here instead of being pushed at once: sixteen
 * fragments cover a command's worth of lines */
constexpr uint8_t kLogQueueFragments = 16;
constexpr uint32_t kLogServicePeriodMs = 5;

class NrfLogOutput : public Print {
public:
    NrfLogOutput()
            : _radio(nullptr)
            , _lineLen(0)
            , _ready(false)
            , _queueHead(0)
            , _queueTail(0)
            , _queueCount(0)
            , _lastServiceMs(0)
    {
        memset(_line, 0, sizeof(_line));
        memset(_queue, 0, sizeof(_queue));
        memset(_logAddress, 0, sizeof(_logAddress));
        memset(_restoreAddress, 0, sizeof(_restoreAddress));
    }

    void attach(RF24 &radio, const uint8_t logAddress[5], const uint8_t restoreAddress[5])
    {
        _radio = &radio;
        memcpy(_logAddress, logAddress, sizeof(_logAddress));
        memcpy(_restoreAddress, restoreAddress, sizeof(_restoreAddress));
    }

    /* A fragment can only be framed and transmitted once the chip answered
     * begin(): before that the SPI clock may still be off, and touching the
     * registers blocks the board on a status flag that never comes */
    void markReady()
    {
        if (_radio != nullptr) {
            /* sendPayload() transmits with the no-acknowledge command, and the
             * chip only accepts it once EN_DYN_ACK is set; the library leaves
             * the feature register clear, so without this call the payload is
             * never loaded and the logs disappear without an error */
            _radio->enableDynamicAck();
        }
        _ready = true;
    }

    size_t write(uint8_t b) override
    {
        /* Boot lines cannot ride a radio that is not up yet, so they go to
         * the wired console instead of being dropped: losing the first lines
         * after a reset is exactly when they matter most */
        if (!_ready) {
            Serial.write(b);
            return 1;
        }

        _line[_lineLen++] = static_cast<char>(b);
        if (b == '\n') {
            flushLineFinal();
        } else if (_lineLen >= kLogFragmentData) {
            flushLineContinuation();
        }
        return 1;
    }

    void flush() override
    {
        if (_lineLen > 0) {
            flushLineFinal();
        }
    }

    /* One fragment per period: a burst is spread out so the acknowledged
     * traffic keeps the channel and the receiver's small FIFO has time to
     * drain. Called by the main loop, never by the caller of LOG */
    void service()
    {
        if (!_ready || _radio == nullptr || _queueCount == 0) {
            return;
        }

        const uint32_t now = millis();
        if ((now - _lastServiceMs) < kLogServicePeriodMs) {
            return;
        }

        _lastServiceMs = now;
        sendPayload(_queue[_queueTail]);
        _queueTail = static_cast<uint8_t>((_queueTail + 1U) % kLogQueueFragments);
        --_queueCount;
    }

    /* Best effort drain before a jump that will not come back: the last lines
     * are exactly the ones that explain the reset */
    void flushPending()
    {
        if (_radio == nullptr) {
            return;
        }

        while (_queueCount > 0) {
            sendPayload(_queue[_queueTail]);
            _queueTail = static_cast<uint8_t>((_queueTail + 1U) % kLogQueueFragments);
            --_queueCount;
        }
    }

private:
    void flushLineContinuation()
    {
        if (_lineLen == 0) {
            return;
        }

        uint8_t payload[kLogPayloadSize] = { 0 };

        payload[0] = static_cast<uint8_t>(PacketTag::Log);
        payload[1] = 0x80 | kLogFragmentData; // Continuation flag + data length
        memcpy(&payload[2], _line, kLogFragmentData);
        enqueue(payload);
        _lineLen = 0;
    }

    void flushLineFinal()
    {
        if (_lineLen == 0) {
            return;
        }

        uint8_t payload[kLogPayloadSize] = { 0 };

        payload[0] = static_cast<uint8_t>(PacketTag::Log);
        payload[1] = (_lineLen & 0x7F); // End packet: no continuation flag
        memcpy(&payload[2], _line, _lineLen);
        enqueue(payload);
        _lineLen = 0;
    }

    void enqueue(const uint8_t payload[kLogPayloadSize])
    {
        if (_radio == nullptr) {
            return;
        }

        if (_queueCount == kLogQueueFragments) {
            /* Keep the newest line: the oldest one is the least likely to
             * explain what is happening now */
            _queueTail = static_cast<uint8_t>((_queueTail + 1U) % kLogQueueFragments);
            --_queueCount;
        }

        memcpy(_queue[_queueHead], payload, kLogPayloadSize);
        _queueHead = static_cast<uint8_t>((_queueHead + 1U) % kLogQueueFragments);
        ++_queueCount;
    }

    void sendPayload(const uint8_t payload[kLogPayloadSize])
    {
        /* A log line is best effort, so it leaves without acknowledgement.
         * With auto-ack the chip retransmits fifteen times when nobody drains
         * the log pipe, about 22 ms, and the caller stalls waiting for an
         * answer that will not come: on the car that is several control
         * cycles lost per line. The no-ack transmit goes out once, and the
         * bounded wait only covers the airtime */
        constexpr uint32_t kLogTxTimeoutMs = 2;

        _radio->stopListening();
        _radio->openWritingPipe(_logAddress);
        /* kLogPayloadSize, not sizeof(payload): the parameter is a pointer
         * inside this function, and sizeof would be 2 on AVR. The chip would
         * then transmit two bytes and pad the rest with zeros, which is what
         * made every log line arrive with its content blank */
        _radio->startWrite(payload, kLogPayloadSize, true);
        _radio->txStandBy(kLogTxTimeoutMs, true);
        _radio->openWritingPipe(_restoreAddress);
        _radio->startListening();
    }

    RF24 *_radio;
    char _line[32];
    uint8_t _lineLen;
    bool _ready;
    uint8_t _logAddress[5];
    uint8_t _restoreAddress[5];
    uint8_t _queue[kLogQueueFragments][kLogPayloadSize];
    uint8_t _queueHead;
    uint8_t _queueTail;
    uint8_t _queueCount;
    uint32_t _lastServiceMs;
};

inline NrfLogOutput &output()
{
    static NrfLogOutput logOutput;
    return logOutput;
}

inline void attachNrf24(RF24 &radio, const uint8_t logAddress[5], const uint8_t restoreAddress[5])
{
    output().attach(radio, logAddress, restoreAddress);
}

/* The radio is only usable once begin() answered: every firmware that logs
 * over the air calls this right after its own radio initialisation */
inline void markNrf24Ready()
{
    output().markReady();
}

/* Called by the main loop: sends at most one queued fragment per period */
inline void service()
{
    output().service();
}

/* Called before a jump that will not come back */
inline void flushPending()
{
    output().flushPending();
}

#else
inline Print &output()
{
    return Serial;
}

inline void service()
{
}

inline void flushPending()
{
}
#endif
} // namespace gaspetto_log
#endif

class Log {
public:
#ifndef ARDUINO
    template <typename T> static void log(const T &data)
    {
#ifdef GASPETTO_LOG
        std::cout << data;
#endif
    }

    template <typename T> static void log(const T &data, int base)
    {
#ifdef GASPETTO_LOG
        if (base == HEX) {
            std::cout << std::hex << std::uppercase << (int)data << std::dec;
            std::cout << std::flush;
        } else if (base == OCT) {
            std::cout << std::oct << (int)data << std::dec;
            std::cout << std::flush;
        } else if (base == BIN) {
            unsigned int v = static_cast<unsigned int>(data);
            std::string s;
            do {
                s = (v % 2 ? '1' : '0') + s;
                v /= 2;
            } while (v);
            std::cout << s;
            std::cout << std::flush;
        } else {
            std::cout << (int)data;
            std::cout << std::flush;
        }
#endif
    }

    /* Print new line without arguments. */
    static void logln()
    {
#ifdef GASPETTO_LOG
        std::cout << std::endl << std::flush;
        std::cout << std::flush;
#endif
    }

    template <typename T> static void logln(const T &data)
    {
#ifdef GASPETTO_LOG
        std::cout << data << std::endl;
        std::cout << std::flush;
#endif
    }

    static void service()
    {
    }

    static void flushPending()
    {
    }
#else /* ARDUINO */
#ifdef GASPETTO_LOG_OVER_NRF24
    static void attachNrf24(RF24 &radio, const uint8_t logAddress[5],
                            const uint8_t restoreAddress[5])
    {
        gaspetto_log::attachNrf24(radio, logAddress, restoreAddress);
    }

    static void markNrf24Ready()
    {
        gaspetto_log::markNrf24Ready();
    }
#endif

    /* The transport queues its fragments; the main loop drains them */
    static void service()
    {
        gaspetto_log::service();
    }

    /* Best effort drain before a reset or a jump to DFU */
    static void flushPending()
    {
        gaspetto_log::flushPending();
    }

    template <typename T> static void log(const T &data)
    {
#ifdef GASPETTO_LOG
        gaspetto_log::output().print(data);
#endif
    }

    template <typename T> static void log(const T &data, int base)
    {
#ifdef GASPETTO_LOG
        gaspetto_log::output().print(data, base);
#endif
    }

    template <typename T> static void logln(const T &data)
    {
#ifdef GASPETTO_LOG
        gaspetto_log::output().println(data);
#endif
    }

    static void logln(void)
    {
#ifdef GASPETTO_LOG
        gaspetto_log::output().println();
#endif
    }
#endif /* ARDUINO */
};

#define LOG(...) Log::log(__VA_ARGS__)
#define LOGLN(...) Log::logln(__VA_ARGS__)
