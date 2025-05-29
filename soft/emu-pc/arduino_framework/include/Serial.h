#pragma once

#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

class SerialEmulator {
public:
    template <typename T> void print(const T &data)
    {
        std::cout << data;
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

    /*  Mimics Serial.println. */
    template <typename T> void println(const T &data)
    {
        std::cout << data << std::endl;
    }

    template <typename T> void println(const T &data, int base)
    {
        print(data, base);
        std::cout << std::endl;
    }

    /*  Overload for no-argument println (just a newline). */
    void println()
    {
        std::cout << std::endl;
    }

    void begin(unsigned long baud)
    {
        std::cout << "Serial started at baud rate: " << baud << std::endl;
    }

    int available();
    int read();
    SerialEmulator();
    ~SerialEmulator();

private:
    void fillBuffer();
    std::queue<char> input_buffer;
    std::mutex input_mutex;
    struct termios oldt, newt;
};

/*  Create a global instance to mimic Arduino's Serial object. */
static SerialEmulator Serial;
