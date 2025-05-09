#pragma once
#include <iostream>
#include <string>

class SerialEmulator {
public:
  // Mimics Serial.print
  template <typename T> void print(const T &data) { std::cout << data; }

  // Mimics Serial.println
  template <typename T> void println(const T &data) {
    std::cout << data << std::endl;
  }

  // Overload for no-argument println (just a newline)
  void println() { std::cout << std::endl; }

  void begin(unsigned long baud) {
    std::cout << "Serial started at baud rate: " << baud << std::endl;
  }
};

// Create a global instance to mimic Arduino's Serial object
static SerialEmulator Serial;