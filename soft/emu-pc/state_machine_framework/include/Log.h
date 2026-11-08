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

class NrfLogOutput : public Print {
public:
    NrfLogOutput()
            : _radio(nullptr)
            , _lineLen(0)
    {
        memset(_line, 0, sizeof(_line));
        memset(_logAddress, 0, sizeof(_logAddress));
        memset(_restoreAddress, 0, sizeof(_restoreAddress));
    }

    void attach(RF24 &radio, const uint8_t logAddress[5], const uint8_t restoreAddress[5])
    {
        _radio = &radio;
        memcpy(_logAddress, logAddress, sizeof(_logAddress));
        memcpy(_restoreAddress, restoreAddress, sizeof(_restoreAddress));
    }

    size_t write(uint8_t b) override
    {
        _line[_lineLen++] = static_cast<char>(b);
        if (b == '\n') {
            flushLineFinal();
        } else if (_lineLen >= 30) {
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

private:
    void flushLineContinuation()
    {
        if (_lineLen == 0 || _radio == nullptr) {
            return;
        }

        uint8_t payload[32];
        payload[0] = static_cast<uint8_t>(PacketTag::Log);
        payload[1] = 0x80 | 30; // Continuation flag (bit 7) + 30 bytes of data

        memcpy(&payload[2], _line, 30);

        sendPayload(payload);
        _lineLen = 0;
    }

    void flushLineFinal()
    {
        if (_lineLen == 0) {
            return;
        }

        if (_radio == nullptr) {
            _lineLen = 0;
            return;
        }

        uint8_t payload[32] = { 0 };
        payload[0] = static_cast<uint8_t>(PacketTag::Log);
        payload[1] = (_lineLen & 0x7F); // End packet: no continuation flag, length in bits 6-0

        memcpy(&payload[2], _line, _lineLen);

        sendPayload(payload);
        _lineLen = 0;
    }

    void sendPayload(const uint8_t payload[kLogPayloadSize])
    {
        _radio->stopListening();
        _radio->openWritingPipe(_logAddress);
        /* kLogPayloadSize, not sizeof(payload): the parameter is a pointer
         * inside this function, and sizeof would be 2 on AVR. The chip would
         * then transmit two bytes and pad the rest with zeros, which is what
         * made every log line arrive with its content blank */
        _radio->write(payload, kLogPayloadSize);
        _radio->openWritingPipe(_restoreAddress);
        _radio->startListening();
    }

    RF24 *_radio;
    char _line[32];
    uint8_t _lineLen;
    uint8_t _logAddress[5];
    uint8_t _restoreAddress[5];
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

#else
inline Print &output()
{
    return Serial;
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
#else /* ARDUINO */
#ifdef GASPETTO_LOG_OVER_NRF24
    static void attachNrf24(RF24 &radio, const uint8_t logAddress[5],
                            const uint8_t restoreAddress[5])
    {
        gaspetto_log::attachNrf24(radio, logAddress, restoreAddress);
    }
#endif

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
