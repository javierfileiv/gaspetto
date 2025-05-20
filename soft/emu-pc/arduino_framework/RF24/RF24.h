/* filepath: /home/fixp/sourceCode/gaspetto/soft/emu-pc/arduino_framework/RF24.h */
#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <vector>

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
typedef enum {
    RF24_FIFO_OCCUPIED,
    RF24_FIFO_EMPTY,
    RF24_FIFO_FULL,
    RF24_FIFO_INVALID
} rf24_fifo_state_e;
typedef enum {
    RF24_IRQ_NONE = 0,
    RF24_TX_DF = 1 << 4, /* Data Failed (MAX_RT) */
    RF24_TX_DS = 1 << 5, /* Data Sent */
    RF24_RX_DR = 1 << 6, /* Data Ready */
    RF24_IRQ_ALL = (1 << 4) | (1 << 5) | (1 << 6)
} rf24_irq_flags_e;

/* Define pin type for compatibility */
typedef int rf24_gpio_pin_t;

class RF24 {
private:
    /* Internal state */
    rf24_gpio_pin_t ce_pin;
    rf24_gpio_pin_t csn_pin;
    uint32_t spi_speed;
    bool powered_up;
    bool listening;
    bool p_variant;
    uint8_t payload_size;
    uint8_t channel;
    rf24_datarate_e data_rate;
    rf24_pa_dbm_e pa_level;
    bool dynamic_payloads;
    bool ack_payloads_enabled;
    uint8_t addr_width;
    rf24_crclength_e crc_length;
    bool auto_ack;
    uint8_t status;
    uint8_t config_reg;

    /* PC emulation specific members */
    std::string pipe_name;
    std::map<uint8_t, std::vector<uint8_t> > pipe_addresses;
    std::queue<std::vector<uint8_t> > rx_fifo;
    std::queue<std::vector<uint8_t> > tx_fifo;
    std::mutex tx_mutex;
    std::mutex rx_mutex;

    /* IPC communication handler */
    class IPCHandler {
    public:
        static IPCHandler &getInstance();
        bool sendPacket(const std::vector<uint8_t> &address, const std::vector<uint8_t> &data);
        bool receivePacket(const std::vector<uint8_t> &address, std::vector<uint8_t> &data);

    private:
        IPCHandler() = default;
    };

    /* Internal methods */
    void _init_obj();
    bool _init_radio();
    bool _init_pins();
    void ce(bool level);
    void csn(bool mode);
    uint8_t read_register(uint8_t reg);
    void read_register(uint8_t reg, uint8_t *buf, uint8_t len);
    void write_register(uint8_t reg, uint8_t value);
    void write_register(uint8_t reg, const uint8_t *buf, uint8_t len);
    void write_payload(const void *buf, uint8_t len, const uint8_t writeType);
    void read_payload(void *buf, uint8_t len);
    void flush_rx();
    void flush_tx();
    uint8_t _data_rate_reg_value(rf24_datarate_e speed);
    uint8_t _pa_level_reg_value(uint8_t level, bool lnaEnable);

public:
    /* Constructor */
    RF24(rf24_gpio_pin_t _cepin, rf24_gpio_pin_t _cspin, uint32_t _spi_speed = 10000000);
    RF24(uint32_t _spi_speed = 10000000);

    /* Core functionality */
    bool begin(void);
    bool begin(rf24_gpio_pin_t _cepin, rf24_gpio_pin_t _cspin);
    void startListening(void);
    void stopListening(void);
    bool available(void);
    bool available(uint8_t *pipe_num);
    void read(void *buf, uint8_t len);
    bool write(const void *buf, uint8_t len);
    bool write(const void *buf, uint8_t len, const bool multicast);

    /* Setup methods */
    void openWritingPipe(const uint8_t *address);
    void openReadingPipe(uint8_t number, const uint8_t *address);
    void closeReadingPipe(uint8_t pipe);
    void setPayloadSize(uint8_t size);
    uint8_t getPayloadSize(void);
    uint8_t getDynamicPayloadSize(void);
    void enableAckPayload(void);
    void disableAckPayload(void);
    void enableDynamicPayloads(void);
    void disableDynamicPayloads(void);
    void enableDynamicAck();
    bool isPVariant(void);
    void setAutoAck(bool enable);
    void setAutoAck(uint8_t pipe, bool enable);
    void setPALevel(uint8_t level, bool lnaEnable = true);
    uint8_t getPALevel(void);
    uint8_t getARC(void);
    bool setDataRate(rf24_datarate_e speed);
    rf24_datarate_e getDataRate(void);
    void setCRCLength(rf24_crclength_e length);
    rf24_crclength_e getCRCLength(void);
    void disableCRC(void);
    void setAddressWidth(uint8_t a_width);
    void setChannel(uint8_t channel);
    uint8_t getChannel(void);
    void setRetries(uint8_t delay, uint8_t count);

    /* Power management */
    void powerDown(void);
    void powerUp(void);
    bool isChipConnected();

    /* Advanced operation */
    void startFastWrite(const void *buf, uint8_t len, const bool multicast, bool startTx = true);
    bool startWrite(const void *buf, uint8_t len, const bool multicast);
    void reUseTX();
    bool rxFifoFull();
    rf24_fifo_state_e isFifo(bool about_tx);
    bool isFifo(bool about_tx, bool check_empty);
    bool writeFast(const void *buf, uint8_t len);
    bool writeFast(const void *buf, uint8_t len, const bool multicast);
    bool writeBlocking(const void *buf, uint8_t len, uint32_t timeout);
    bool txStandBy();
    bool txStandBy(uint32_t timeout, bool startTx = false);
    bool writeAckPayload(uint8_t pipe, const void *buf, uint8_t len);
    bool testCarrier(void);
    bool testRPD(void);
    void toggleAllPipes(bool isEnabled);
    void setRadiation(uint8_t level, rf24_datarate_e speed, bool lnaEnable = true);

    /* Status management */
    uint8_t update();
    void clearStatusFlags(uint8_t flags = RF24_IRQ_ALL);
    void setStatusFlags(uint8_t flags = RF24_IRQ_NONE);
    uint8_t getStatusFlags();

    /* Debug */
    void printDetails(void);
    void printPrettyDetails(void);
    uint16_t sprintfPrettyDetails(char *debugging_information);
    void printStatus(uint8_t flags);
    void encodeRadioDetails(uint8_t *encoded_status);
    bool isValid();

    /* For compatibility with older code */
    void openReadingPipe(uint8_t number, uint64_t address);
    void openWritingPipe(uint64_t address);
    bool isAckPayloadAvailable(void);
    void maskIRQ(bool tx_ok, bool tx_fail, bool rx_ready);
    void whatHappened(bool &tx_ok, bool &tx_fail, bool &rx_ready);
    void stopListening(const uint8_t *txAddress);
    void stopListening(const uint64_t txAddress);

    /* Configuration for PC emulation */
    uint32_t txDelay = 5;
    uint32_t csDelay = 5;

    /* PC emulation specific methods */
    void setEmulationPipe(const std::string &name);
    std::string getEmulationPipe() const;
    void simulateReceivedPacket(uint8_t pipe, const void *data, uint8_t len);
    void simulateFailedTransmission();
    void simulateSuccessfulTransmission();
    void setNetworkLatency(uint32_t ms);
    void setPacketLossRate(float percentage);
};
