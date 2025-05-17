#include "Arduino.h"

class RF24 {
public:
    RF24(int cePin, int csnPin)
    {
    }

    void begin()
    {
        available_ = true;
    }

    bool available()
    {
        return available_;
    }

    void read(uint8_t *data, size_t size)
    {
        // In a real implementation, this would read data from the hardware
    }

    void write(const uint8_t *data, size_t size)
    {
        // Placeholder for writing data to the RF24 module
        // In a real implementation, this would send data to the hardware
    }

public:
    bool available_ = false;
};
