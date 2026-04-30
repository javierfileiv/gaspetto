#ifndef LOG_H
#define LOG_H

#include "Arduino.h"

#ifdef ARDUINO
#include <Print.h>
#include <cstring>

#ifdef GASPETTO_LOG_OVER_NRF24
#include "RF24.h"
#endif
#else
#include "Serial.h"

#include <iostream>
#endif

#ifdef ARDUINO
namespace gaspetto_log
{
#ifdef GASPETTO_LOG_OVER_NRF24
class NrfLogOutput : public Print {
public:
    NrfLogOutput()
            : _radio(nullptr)
            , _lineLen(0)
    {
        std::memset(_line, 0, sizeof(_line));
        std::memset(_logAddress, 0, sizeof(_logAddress));
        std::memset(_restoreAddress, 0, sizeof(_restoreAddress));
    }

    void attach(RF24 &radio, const uint8_t logAddress[5], const uint8_t restoreAddress[5])
    {
        _radio = &radio;
        std::memcpy(_logAddress, logAddress, sizeof(_logAddress));
        std::memcpy(_restoreAddress, restoreAddress, sizeof(_restoreAddress));
    }

    size_t write(uint8_t b) override
    {
        _line[_lineLen++] = static_cast<char>(b);
        if (b == '\n') {
            flushLineFinal();
        } else if (_lineLen >= 31) {
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
        payload[0] = 0x80 | 31; // Continuation flag (bit 7) + 31 bytes of data

        std::memcpy(&payload[1], _line, 31);

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
        payload[0] = (_lineLen & 0x7F); // End packet: no continuation flag, length in bits 6-0

        std::memcpy(&payload[1], _line, _lineLen);

        sendPayload(payload);
        _lineLen = 0;
    }

    void sendPayload(const uint8_t payload[32])
    {
        _radio->stopListening();
        _radio->openWritingPipe(_logAddress);
        _radio->write(payload, sizeof(payload));
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
    template <typename T> void log(const T &data)
    {
#ifdef GASPETTO_LOG
        std::cout << data;
#endif
    }

    template <typename T> void print(const T &data, int base)
    {
        if (base == HEX)
            std::cout << std::hex << std::uppercase << (int)data << std::dec;
        else if (base == OCT)
            std::cout << std::oct << (int)data << std::dec;
        else if (base == BIN) {
            unsigned int v = static_cast<unsigned int>(data);
            std::string s;
            do {
                s = (v % 2 ? '1' : '0') + s;
                v /= 2;
            } while (v);
            std::cout << s;
        } else
            std::cout << (int)data;
    }

    /* Print new line without arguments. */
    void logln()
    {
#ifdef GASPETTO_LOG
        std::cout << std::endl;
#endif
    }

    template <typename T> void logln(const T &data)
    {
#ifdef GASPETTO_LOG
        std::cout << data << std::endl;
#endif
    }

    template <typename T> void log(const T &data, int base)
    {
#ifdef GASPETTO_LOG
        print(data, base);
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

    template <typename T> void log(const T &data)
    {
#ifdef GASPETTO_LOG
        gaspetto_log::output().print(data);
#endif
    }

    template <typename T> void log(const T &data, int base)
    {
#ifdef GASPETTO_LOG
        gaspetto_log::output().print(data, base);
#endif
    }

    template <typename T> void logln(const T &data)
    {
#ifdef GASPETTO_LOG
        gaspetto_log::output().println(data);
#endif
    }

    void logln(void)
    {
#ifdef GASPETTO_LOG
        gaspetto_log::output().println();
#endif
    }
#endif /* ARDUINO */
};
#endif /* LOG_H */
