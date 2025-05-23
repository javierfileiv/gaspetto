#include "Serial.h"

#include <iostream>
#include <mutex>
#include <queue>
#include <string>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

SerialEmulator::SerialEmulator()
{
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
}

SerialEmulator::~SerialEmulator()
{
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
}

int SerialEmulator::available()
{
    std::lock_guard<std::mutex> lock(input_mutex);
    fillBuffer();
    return !input_buffer.empty();
}

int SerialEmulator::read()
{
    std::lock_guard<std::mutex> lock(input_mutex);
    fillBuffer();
    if (!input_buffer.empty()) {
        char ch = input_buffer.front();
        input_buffer.pop();
        return ch;
    }
    return -1;
}

void SerialEmulator::fillBuffer()
{
    fd_set set;
    struct timeval timeout = { 0, 0 };
    FD_ZERO(&set);
    FD_SET(STDIN_FILENO, &set);
    int rv = select(STDIN_FILENO + 1, &set, NULL, NULL, &timeout);
    if (rv > 0) {
        char ch;
        ssize_t n = ::read(STDIN_FILENO, &ch, 1);
        if (n > 0)
            input_buffer.push(ch);
    }
}
