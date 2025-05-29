#ifndef LOG_H
#define LOG_H

#include "Arduino.h"

#ifndef ARDUINO
#include "Serial.h"

#include <iostream>
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
    template <typename T> void log(const T &data)
    {
#ifdef GASPETTO_LOG
        Serial.print(data);
#endif
    }

    template <typename T> void log(const T &data, int base)
    {
#ifdef GASPETTO_LOG
        Serial.print(data, base);
#endif
    }

    template <typename T> void logln(const T &data)
    {
#ifdef GASPETTO_LOG
        Serial.println(data);
#endif
    }

    void logln(void)
    {
#ifdef GASPETTO_LOG
        Serial.println();
#endif
    }
#endif /* ARDUINO */
};
#endif /* LOG_H */
