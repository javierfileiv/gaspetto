/* filepath: /home/fixp/sourceCode/gaspetto/soft/emu-pc/arduino_framework/RF24.cpp */
#include "RF24.h"

#include <chrono>
#include <cstring>
#include <iostream>
#include <random>
#include <sstream>
#include <thread>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace
{
void rf24_log(const std::string &msg)
{
#if NRF_LOG
    std::cout << msg << std::endl;
#endif
}
void rf24_log_noendl(const std::string &msg)
{
#if NRF_LOG
    std::cout << msg;
#endif
}
}

/* Static instance for IPC handler. */
RF24::IPCHandler &RF24::IPCHandler::getInstance()
{
    static IPCHandler instance;

    return instance;
}

/* Constructor. */
RF24::RF24(rf24_gpio_pin_t _cepin, rf24_gpio_pin_t _cspin, uint32_t _spi_speed)
        : ce_pin(_cepin)
        , csn_pin(_cspin)
        , spi_speed(_spi_speed)
        , powered_up(false)
        , listening(false)
        , p_variant(true)
        , payload_size(32)
        , channel(76)
        , data_rate(RF24_1MBPS)
        , pa_level(RF24_PA_MAX)
        , dynamic_payloads(false)
        , ack_payloads_enabled(false)
        , addr_width(5)
        , crc_length(RF24_CRC_16)
        , auto_ack(true)
        , status(0)
        , config_reg(0)
{
    rf24_log("[RF24::RF24] this=" + std::to_string(reinterpret_cast<uintptr_t>(this)) +
             ", addr_width=" + std::to_string((int)addr_width));
    _init_obj();
}

RF24::RF24(uint32_t _spi_speed)
        : RF24(0, 0, _spi_speed)
{
}

void RF24::_init_obj()
{
    /* Set default pipe name based on process ID for uniqueness. */
    std::ostringstream oss;

    oss << "rf24_emu_" << getpid();
    pipe_name = oss.str();
    /* Initialize pipe addresses with zeros. */
    for (uint8_t i = 0; i < 6; i++)
        pipe_addresses[i] = std::vector<uint8_t>(5, 0);
    /* Log initialization. */
    rf24_log("RF24 PC Emulation initialized with pipe: " + pipe_name);
}

bool RF24::begin()
{
    return _init_radio();
}

bool RF24::begin(rf24_gpio_pin_t _cepin, rf24_gpio_pin_t _cspin)
{
    ce_pin = _cepin;
    csn_pin = _cspin;

    return _init_radio();
}

bool RF24::_init_radio()
{
    /* Initialize pins. */
    _init_pins();
    /* Reset status. */
    status = 0;
    /* Set default configuration. */
    config_reg = 0x0C; /* CRC enabled, 16-bit CRC. */
    /* Default to channel 76. */
    setChannel(76);
    /* Default to max power. */
    setPALevel(RF24_PA_MAX);
    /* Default data rate. */
    setDataRate(RF24_1MBPS);
    /* Open pipes with default addresses. */
    uint8_t default_addr[5] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };
    for (uint8_t i = 0; i < 6; i++) {
        openReadingPipe(i, default_addr);
        default_addr[0]++;
    }
    /* Flush FIFOs. */
    flush_rx();
    flush_tx();
    /* Power up. */
    powerUp();
    rf24_log("RF24 PC Emulation radio initialized.");
    return true;
}

bool RF24::_init_pins()
{
    /* In PC emulation, pins are virtual. */
    rf24_log("RF24 PC Emulation pins initialized (CE: " + std::to_string(ce_pin) +
             ", CSN: " + std::to_string(csn_pin) + ").");
    return true;
}

void RF24::startListening()
{
    /* Set RX mode. */
    config_reg |= 0x01; /* Set PRIM_RX bit. */
    /* Restore pipe0 address if needed. */
    if (pipe_addresses[0].size() > 0)
        write_register(0x0A, pipe_addresses[0].data(), pipe_addresses[0].size());
    /* Flush RX FIFO. */
    flush_rx();
    /* Set CE pin high to enable RX mode. */
    ce(true);
    /* Reset status flags. */
    status &= ~(RF24_RX_DR | RF24_TX_DS | RF24_TX_DF);
    /* Mark as listening. */
    listening = true;
}

void RF24::stopListening()
{
    /* Set CE low to disable RX/TX. */
    ce(false);
    /* Flush TX FIFO if ACK payloads were enabled. */
    if (ack_payloads_enabled) {
        flush_tx();
    }
    /* Clear PRIM_RX bit for TX mode. */
    config_reg &= ~0x01;
    /* Set CE high again to activate TX. */
    ce(true);
    /* Mark as not listening. */
    listening = false;
    /* Small delay to allow mode change. */
    std::this_thread::sleep_for(std::chrono::microseconds(txDelay));
    rf24_log("RF24 PC Emulation stopped listening.");
}

bool RF24::available()
{
    if (!listening)
        return false;
    /* Check if we have data in RX FIFO. */
    std::lock_guard<std::mutex> lock(rx_mutex);
    return !rx_fifo.empty();
}

bool RF24::available(uint8_t *pipe_num)
{
    if (available()) {
        if (pipe_num)
            *pipe_num = 0; /* For emulation, always use pipe 0. */
        return true;
    }
    return false;
}

void RF24::read(void *buf, uint8_t len)
{
    std::lock_guard<std::mutex> lock(rx_mutex);
    if (rx_fifo.empty()) {
        memset(buf, 0, len);
        return;
    }
    std::vector<uint8_t> &payload = rx_fifo.front();
    uint8_t read_size = (len < payload.size()) ? len : payload.size();
    memcpy(buf, payload.data(), read_size);
    if (len > payload.size())
        memset(static_cast<uint8_t *>(buf) + read_size, 0, len - payload.size());
    if (read_size < payload.size()) {
        payload.erase(payload.begin(), payload.begin() + read_size);
    } else {
        rx_fifo.pop();
    }
    status &= ~RF24_RX_DR;
    rf24_log("RF24 PC Emulation read " + std::to_string((int)read_size) + " bytes.");
}

bool RF24::write(const void *buf, uint8_t len)
{
    return write(buf, len, false);
}

bool RF24::write(const void *buf, uint8_t len, const bool multicast)
{
    bool success;

    if (len > 32)
        len = 32; /* Maximum packet size. */
    /* Exit listening mode if needed. */
    bool reset_listening = listening;
    if (listening)
        stopListening();
    /* Create a copy of the data. */
    std::vector<uint8_t> payload(static_cast<const uint8_t *>(buf),
                                 static_cast<const uint8_t *>(buf) + len);
    /* Get writing pipe address. */
    std::vector<uint8_t> &tx_addr = pipe_addresses[0];
    /* Simulate network transmission. */
    success = IPCHandler::getInstance().sendPacket(tx_addr, payload);
    /* Simulate transmission delay. */
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    /* Set status flags based on result. */
    if (success) {
        status |= RF24_TX_DS; /* Data sent. */
        rf24_log("RF24 PC Emulation successfully transmitted " + std::to_string(len) + " bytes.");
    } else {
        status |= RF24_TX_DF; /* Data failed. */
        rf24_log("RF24 PC Emulation failed to transmit data.");
    }
    /* Return to listening mode if needed. */
    if (reset_listening)
        startListening();
    return success;
}

void RF24::openWritingPipe(const uint8_t *address)
{
    std::ostringstream addr_str;

    if (address == nullptr) {
        rf24_log("[RF24] openWritingPipe: address is nullptr!");
        return;
    }
    /* Store the address. */
    memcpy(pipe_addresses[0].data(), address, addr_width);
    for (uint8_t i = 0; i < addr_width; i++)
        addr_str << std::hex << (int)address[i] << " ";
    rf24_log("RF24 PC Emulation writing pipe opened with address: " + addr_str.str() + ".");
}

void RF24::openReadingPipe(uint8_t number, const uint8_t *address)
{
    rf24_log("[RF24::openReadingPipe] this=" + std::to_string(reinterpret_cast<uintptr_t>(this)) +
             ", addr_width=" + std::to_string((int)addr_width));
    uint8_t en_rx_addr;
    std::ostringstream addr_str;
    /* Validate parameters to prevent segfaults */
    if (number > 5 || address == nullptr) {
        rf24_log("[RF24] openReadingPipe: Invalid pipe number (" + std::to_string((int)number) +
                 ") or address is nullptr!");
        return;
    }
    /* Ensure the vector at pipe_addresses[number] is the correct size. */
    if (pipe_addresses.find(number) == pipe_addresses.end()) {
        pipe_addresses[number] = std::vector<uint8_t>(addr_width, 0);
    }
    if (pipe_addresses[number].size() < addr_width) {
        rf24_log("[RF24] openReadingPipe: pipe_addresses[" + std::to_string((int)number) +
                 "].size()=" + std::to_string(pipe_addresses[number].size()) +
                 ", expected=" + std::to_string((int)addr_width));
        pipe_addresses[number].resize(addr_width, 0);
    }
    /* Store the address. */
    memcpy(pipe_addresses[number].data(), address, addr_width);
    /* Enable the data pipe. */
    en_rx_addr = read_register(0x02); /* EN_RXADDR. */
    en_rx_addr |= (1 << number);
    write_register(0x02, en_rx_addr);
    for (uint8_t i = 0; i < addr_width; i++) {
        addr_str << std::hex << (int)address[i] << " ";
    }
    rf24_log("RF24 PC Emulation reading pipe " + std::to_string((int)number) +
             " opened with address: " + addr_str.str() + ".");
}

void RF24::setChannel(uint8_t ch)
{
    if (ch > 125)
        ch = 125;
    channel = ch;
    write_register(0x05, ch);
    rf24_log("RF24 PC Emulation channel set to " + std::to_string((int)ch) + ".");
}

uint8_t RF24::getChannel()
{
    return channel;
}

void RF24::setPayloadSize(uint8_t size)
{
    if (size > 32)
        size = 32;
    payload_size = size;
    rf24_log("RF24 PC Emulation payload size set to " + std::to_string((int)size) + " bytes.");
}

uint8_t RF24::getPayloadSize()
{
    return payload_size;
}

void RF24::powerUp()
{
    if (!powered_up) {
        /* Set the PWR_UP bit. */
        config_reg |= 0x02;
        write_register(0x00, config_reg);
        /* Delay for power-up. */
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        powered_up = true;
        rf24_log("RF24 PC Emulation powered up.");
    }
}

void RF24::powerDown()
{
    /* Clear the PWR_UP bit. */
    config_reg &= ~0x02;
    write_register(0x00, config_reg);
    powered_up = false;
    rf24_log("RF24 PC Emulation powered down.");
}

void RF24::ce(bool level)
{
    /* In PC emulation, this is just for logging. */
}

void RF24::csn(bool mode)
{
    /* In PC emulation, this is just for logging. */
    if (mode)
        rf24_log("RF24 PC Emulation CSN pin set HIGH.");
    else
        rf24_log("RF24 PC Emulation CSN pin set LOW.");
}

void RF24::flush_rx()
{
    std::lock_guard<std::mutex> lock(rx_mutex);

    while (!rx_fifo.empty())
        rx_fifo.pop();
}

void RF24::flush_tx()
{
    std::lock_guard<std::mutex> lock(tx_mutex);

    while (!tx_fifo.empty())
        tx_fifo.pop();
    rf24_log("RF24 PC Emulation TX FIFO flushed.");
}

uint8_t RF24::read_register(uint8_t reg)
{
    uint8_t result = 0;

    /* Emulate register read based on register address. */
    switch (reg) {
    case 0x00: /* CONFIG. */
        result = config_reg;
        break;
    case 0x05: /* RF_CH. */
        result = channel;
        break;
    case 0x07: /* STATUS. */
        result = status;
        break;
    default:
        /* For unimplemented registers, return 0. */
        result = 0;
    }
    return result;
}

void RF24::read_register(uint8_t reg, uint8_t *buf, uint8_t len)
{
    /* Simple emulation - fill with zeros. */
    memset(buf, 0, len);
    /* For address registers, return stored addresses. */
    if (reg >= 0x0A && reg <= 0x10) { /* RX_ADDR_P0 through TX_ADDR. */
        uint8_t pipe = reg - 0x0A;
        if (pipe <= 5) /* Valid pipes are 0-5. */
            memcpy(buf, pipe_addresses[pipe].data(), std::min(len, (uint8_t)5));
    }
}

void RF24::write_register(uint8_t reg, uint8_t value)
{
    /* Emulate register write based on register address. */
    switch (reg) {
    case 0x00: /* CONFIG. */
        config_reg = value;
        break;
    case 0x05: /* RF_CH. */
        channel = value;
        break;
    case 0x07: /* STATUS. */
        /* Only clear bits that are written with 1. */
        status &= ~value;
        break;
    default:
        /* For unimplemented registers, do nothing. */
        break;
    }
}

void RF24::write_register(uint8_t reg, const uint8_t *buf, uint8_t len)
{
    /* For address registers, store addresses. */
    if (reg >= 0x0A && reg <= 0x10) { /* RX_ADDR_P0 through TX_ADDR. */
        uint8_t pipe = reg - 0x0A;

        if (pipe <= 5) { /* Valid pipes are 0-5. */
            memcpy(pipe_addresses[pipe].data(), buf, std::min(len, (uint8_t)5));
        }
    }
}

/* PC emulation specific methods. */
void RF24::setEmulationPipe(const std::string &name)
{
    pipe_name = name;
    rf24_log("RF24 PC Emulation pipe set to: " + pipe_name + ".");
}

std::string RF24::getEmulationPipe() const
{
    return pipe_name;
}

void RF24::simulateReceivedPacket(uint8_t pipe, const void *data, uint8_t len)
{
    std::lock_guard<std::mutex> lock(rx_mutex);

    if (pipe > 5)
        return;
    if (!listening)
        return;
    std::vector<uint8_t> payload(static_cast<const uint8_t *>(data),
                                 static_cast<const uint8_t *>(data) + len);
    /* Add to RX FIFO. */
    rx_fifo.push(payload);
    /* Set RX_DR flag. */
    status |= RF24_RX_DR;
}

void RF24::simulateFailedTransmission()
{
    status |= RF24_TX_DF;
    rf24_log("RF24 PC Emulation simulated failed transmission.");
}

void RF24::simulateSuccessfulTransmission()
{
    status |= RF24_TX_DS;
    rf24_log("RF24 PC Emulation simulated successful transmission.");
}

/* Other methods with minimal implementations for simplicity. */

bool RF24::isChipConnected()
{
    return true;
}

bool RF24::isPVariant()
{
    return p_variant;
}

uint8_t RF24::getDynamicPayloadSize()
{
    if (rx_fifo.empty())
        return 0;

    std::lock_guard<std::mutex> lock(rx_mutex);
    return rx_fifo.front().size();
}

void RF24::enableDynamicPayloads()
{
    dynamic_payloads = true;
    rf24_log("RF24 PC Emulation dynamic payloads enabled.");
}

void RF24::disableDynamicPayloads()
{
    dynamic_payloads = false;
    rf24_log("RF24 PC Emulation dynamic payloads disabled.");
}

void RF24::enableAckPayload()
{
    ack_payloads_enabled = true;
    enableDynamicPayloads();
    rf24_log("RF24 PC Emulation ACK payloads enabled.");
}

void RF24::disableAckPayload()
{
    ack_payloads_enabled = false;
    rf24_log("RF24 PC Emulation ACK payloads disabled.");
}

void RF24::setAutoAck(bool enable)
{
    auto_ack = enable;
    rf24_log("RF24 PC Emulation auto-ack " + std::string(enable ? "enabled" : "disabled") + ".");
}

void RF24::setPALevel(uint8_t level, bool lnaEnable)
{
    pa_level = static_cast<rf24_pa_dbm_e>(level > RF24_PA_MAX ? RF24_PA_MAX : level);
    rf24_log("RF24 PC Emulation PA level set to " + std::to_string((int)pa_level) +
             (lnaEnable ? " with LNA" : "") + ".");
}

uint8_t RF24::getPALevel()
{
    return pa_level;
}

bool RF24::setDataRate(rf24_datarate_e speed)
{
    data_rate = speed;
    rf24_log("RF24 PC Emulation data rate set to " +
             std::string(speed == RF24_250KBPS ? "250kbps" :
                                                 (speed == RF24_1MBPS ? "1Mbps" : "2Mbps")) +
             ".");
    return true;
}

rf24_datarate_e RF24::getDataRate()
{
    return data_rate;
}

void RF24::setCRCLength(rf24_crclength_e length)
{
    crc_length = length;

    /* Update config register. */
    if (length == RF24_CRC_DISABLED) {
        config_reg &= ~0x0C; /* Clear EN_CRC and CRC0 bits. */
    } else if (length == RF24_CRC_8) {
        config_reg |= 0x08; /* Set EN_CRC. */
        config_reg &= ~0x04; /* Clear CRC0. */
    } else {
        config_reg |= 0x0C; /* Set EN_CRC and CRC0. */
    }

    rf24_log("RF24 PC Emulation CRC length set to " +
             std::string(length == RF24_CRC_DISABLED ?
                                 "disabled" :
                                 (length == RF24_CRC_8 ? "8-bit" : "16-bit")) +
             ".");
}

rf24_crclength_e RF24::getCRCLength()
{
    return crc_length;
}

void RF24::disableCRC()
{
    setCRCLength(RF24_CRC_DISABLED);
}

void RF24::setAddressWidth(uint8_t width)
{
    if (width < 3)
        width = 3;
    if (width > 5)
        width = 5;
    addr_width = width;
    rf24_log("[RF24::setAddressWidth] this=" + std::to_string(reinterpret_cast<uintptr_t>(this)) +
             " addr_width=" + std::to_string((int)addr_width));
    rf24_log("RF24 PC Emulation address width set to " + std::to_string((int)width) + " bytes.");
}

bool RF24::isValid()
{
    return true;
}

void RF24::printDetails()
{
    rf24_log("------- RF24 PC Emulation Details -------");
    rf24_log("Status: 0x" + std::to_string((int)status) + ".");
    rf24_log("Channel: " + std::to_string((int)channel) + ".");
    rf24_log("PA Level: " + std::to_string((int)pa_level) + ".");
    rf24_log("Data Rate: " +
             std::string(data_rate == RF24_250KBPS ?
                                 "250kbps" :
                                 (data_rate == RF24_1MBPS ? "1Mbps" : "2Mbps")) +
             ".");
    rf24_log("CRC Length: " +
             std::string(crc_length == RF24_CRC_DISABLED ?
                                 "Disabled" :
                                 (crc_length == RF24_CRC_8 ? "8-bit" : "16-bit")) +
             ".");
    rf24_log("Payload Size: " + std::to_string((int)payload_size) + ".");
    rf24_log("Auto-Ack: " + std::string(auto_ack ? "Enabled" : "Disabled") + ".");
    rf24_log("Dynamic Payloads: " + std::string(dynamic_payloads ? "Enabled" : "Disabled") + ".");
    rf24_log("ACK Payloads: " + std::string(ack_payloads_enabled ? "Enabled" : "Disabled") + ".");
    rf24_log("Power Status: " + std::string(powered_up ? "Powered Up" : "Powered Down") + ".");
    rf24_log("Listening: " + std::string(listening ? "Yes" : "No") + ".");
    rf24_log("----------------------------------------");
}

/* IPC Handler implementation (simplified for this example). */
bool RF24::IPCHandler::sendPacket(const std::vector<uint8_t> &address,
                                  const std::vector<uint8_t> &data)
{
    return false;
}

bool RF24::IPCHandler::receivePacket(const std::vector<uint8_t> &address,
                                     std::vector<uint8_t> &data)
{
    /* This would be called from a separate thread polling for incoming data. */
    return false;
}

/* Stub implementations for compatibility. */
void RF24::openReadingPipe(uint8_t number, uint64_t address)
{
    uint8_t addr_bytes[8];
    uint8_t converted[5];
    memcpy(addr_bytes, &address, 8);
    /* Convert from little endian to big endian. */
    converted[0] = addr_bytes[0];
    converted[1] = addr_bytes[1];
    converted[2] = addr_bytes[2];
    converted[3] = addr_bytes[3];
    converted[4] = addr_bytes[4];
    openReadingPipe(number, converted);
}

void RF24::openWritingPipe(uint64_t address)
{
    uint8_t addr_bytes[8];
    uint8_t converted[5];
    memcpy(addr_bytes, &address, 8);
    /* Convert from little endian to big endian. */
    converted[0] = addr_bytes[0];
    converted[1] = addr_bytes[1];
    converted[2] = addr_bytes[2];
    converted[3] = addr_bytes[3];
    converted[4] = addr_bytes[4];
    openWritingPipe(converted);
}

/* Minimal implementations for other methods. */
bool RF24::isAckPayloadAvailable()
{
    return false;
}
void RF24::maskIRQ(bool tx_ok, bool tx_fail, bool rx_ready)
{
}
void RF24::whatHappened(bool &tx_ok, bool &tx_fail, bool &rx_ready)
{
    tx_ok = (status & RF24_TX_DS) != 0;
    tx_fail = (status & RF24_TX_DF) != 0;
    rx_ready = (status & RF24_RX_DR) != 0;
    status &= ~(RF24_TX_DS | RF24_TX_DF | RF24_RX_DR);
}
void RF24::stopListening(const uint8_t *txAddress)
{
    stopListening();
    openWritingPipe(txAddress);
}
void RF24::stopListening(const uint64_t txAddress)
{
    stopListening();
    openWritingPipe(txAddress);
}
bool RF24::writeFast(const void *buf, uint8_t len)
{
    return write(buf, len);
}
bool RF24::writeFast(const void *buf, uint8_t len, const bool multicast)
{
    return write(buf, len, multicast);
}
bool RF24::writeBlocking(const void *buf, uint8_t len, uint32_t timeout)
{
    return write(buf, len);
}
bool RF24::txStandBy()
{
    return true;
}
bool RF24::txStandBy(uint32_t timeout, bool startTx)
{
    return true;
}
bool RF24::writeAckPayload(uint8_t pipe, const void *buf, uint8_t len)
{
    return false;
}
bool RF24::testCarrier()
{
    return false;
}
bool RF24::testRPD()
{
    return false;
}
uint8_t RF24::getARC()
{
    return 0;
}
void RF24::setRetries(uint8_t delay, uint8_t count)
{
}
void RF24::toggleAllPipes(bool isEnabled)
{
}
void RF24::setRadiation(uint8_t level, rf24_datarate_e speed, bool lnaEnable)
{
    setPALevel(level, lnaEnable);
    setDataRate(speed);
}
void RF24::closeReadingPipe(uint8_t pipe)
{
}
void RF24::enableDynamicAck()
{
}
void RF24::setAutoAck(uint8_t pipe, bool enable)
{
}
bool RF24::rxFifoFull()
{
    return false;
}
rf24_fifo_state_e RF24::isFifo(bool about_tx)
{
    return about_tx ? RF24_FIFO_EMPTY : RF24_FIFO_EMPTY;
}
bool RF24::isFifo(bool about_tx, bool check_empty)
{
    return check_empty;
}
void RF24::reUseTX()
{
}
void RF24::startFastWrite(const void *buf, uint8_t len, const bool multicast, bool startTx)
{
    write(buf, len, multicast);
}
bool RF24::startWrite(const void *buf, uint8_t len, const bool multicast)
{
    return write(buf, len, multicast);
}
uint8_t RF24::update()
{
    return status;
}
void RF24::clearStatusFlags(uint8_t flags)
{
    status &= ~flags;
}
void RF24::setStatusFlags(uint8_t flags)
{
}
uint8_t RF24::getStatusFlags()
{
    return status;
}
void RF24::printStatus(uint8_t flags)
{
}
void RF24::printPrettyDetails()
{
    printDetails();
}
uint16_t RF24::sprintfPrettyDetails(char *debugging_information)
{
    return 0;
}
void RF24::encodeRadioDetails(uint8_t *encoded_status)
{
}
void RF24::setNetworkLatency(uint32_t ms)
{
}
void RF24::setPacketLossRate(float percentage)
{
}
