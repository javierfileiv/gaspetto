/* filepath: /home/fixp/sourceCode/gaspetto/soft/emu-pc/arduino_framework/RF24.h */
#pragma once

#include <cstdint>
#include <string>

/**
 * @file RF24_PC_Emulation.h
 *
 * PC emulation of the nRF24L01+ transceiver to allow testing RF24 code on a PC
 * This emulation mimics the interface of the original RF24 library but uses
 * inter-process communication to simulate radio communications.
 */

/* Maintain the same enumerations as the original RF24 library */
typedef enum {
    RF24_PA_MIN = 0,
    RF24_PA_LOW,
    RF24_PA_HIGH,
    RF24_PA_MAX,
    RF24_PA_ERROR
} rf24_pa_dbm_e;
typedef enum { RF24_1MBPS = 0, RF24_2MBPS, RF24_250KBPS } rf24_datarate_e;
typedef enum { RF24_CRC_DISABLED = 0, RF24_CRC_8, RF24_CRC_16 } rf24_crclength_e;

/* Define pin type for compatibility */
typedef int rf24_gpio_pin_t;

class RF24 {
    uint8_t addr_width = 5; /* Default address width. */

public:
    RF24(rf24_gpio_pin_t _cepin, rf24_gpio_pin_t _cspin, uint32_t _spi_speed = 10000000);
    RF24(uint32_t _spi_speed = 10000000);
    virtual ~RF24() = default;

    /* Core functionality */
    virtual bool begin(void);
    virtual void setPALevel(uint8_t level, bool lnaEnable = true);
    virtual bool setDataRate(rf24_datarate_e speed);
    virtual void setPayloadSize(uint8_t size);
    virtual void openWritingPipe(const uint8_t *address);
    virtual void openReadingPipe(uint8_t number, const uint8_t *address);
    virtual void printDetails(void);
    virtual void printPrettyDetails(void);
    virtual void powerUp(void);
    virtual void startListening(void);
    virtual void stopListening(void);
    virtual bool available(uint8_t *pipe_num);
    virtual void read(void *buf, uint8_t len);
    virtual bool write(const void *buf, uint8_t len);
    void setAddressWidth(uint8_t a_width);
    /* PC emulation specific methods */
    virtual void simulateReceivedPacket(uint8_t pipe, const void *data, uint8_t len);
    virtual void simulateFailedTransmission();
    virtual void simulateSuccessfulTransmission();
};
