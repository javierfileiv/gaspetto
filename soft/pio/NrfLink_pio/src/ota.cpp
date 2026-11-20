#include "ota.h"

#include "Arduino.h"
#include "Crc32.h"
#include "OtaProtocol.h"
#include "PacketTag.h"
#include "RF24.h"
#include "config_radio.h"
#include "link_modes.h"
#include "radio_send.h"
#include "radio_timing.h"

#include <string.h>

/* The relay borrows the tool's radio, its mode, the deferred mode restore and
 * the log reassembly: they stay in main.cpp, which owns the hardware, and this
 * file only reads them */
extern RF24 radio;
extern uint8_t gMode;
extern uint8_t gPendingMode;
extern uint32_t gPendingModeAt;

extern void setMode(uint8_t mode);
extern void drainLogFragment(uint8_t pipe, const uint8_t *payload);

/* ========================================== */
/* OTA relay                                  */
/* ========================================== */
/* The PC streams a binary image on this port: a 0x01 start byte, the image
 * length and its CRC as two little-endian words, then the image itself. Every
 * 28-byte chunk becomes one Data packet. The nRF24 acknowledges each packet at
 * the hardware level, so the car never answers during the data phase: it only
 * speaks once after the erase, once for the CRC, and on a real error. That is
 * what keeps the channel out of the turnaround race two thousand times over */
/* Control codes, not letters: NrfLink prints hex addresses and log lines
 * on the same port, and a plain 'F' would be read as a refusal */
constexpr uint8_t kOtaAck = 0x06;
constexpr uint8_t kOtaNak = 0x15;
/* Flow control, and the only byte that travels during the data phase: one
 * credit per group of packets means "I have taken that much, send more" */
constexpr uint8_t kOtaCredit = 0x11;
constexpr uint8_t kOtaCreditPackets = 4;
constexpr uint16_t kOtaTimeoutMs = 5000;
/* The car erases the staging sector before it can take a byte, and writes the
 * record through another erase when the image is complete */
constexpr uint16_t kOtaBeginAckMs = 6000;
constexpr uint16_t kOtaEndAckMs = 10000;
/* The PC keeps the port open after the verdict to read the car's boot lines,
 * and those ride _box_: stay in the box mode for the same window */
constexpr uint32_t kPostOtaModeDelayMs = 15000;
/* A failed write already means the car's FIFO was full and cost two
 * milliseconds of hardware retransmissions. Three tries cover a short backlog
 * and still keep the whole stall under ten milliseconds, which is what the
 * serial buffer can absorb while the PC keeps sending */
constexpr uint8_t kOtaWriteAttempts = 3;

bool readExact(uint8_t *buffer, uint8_t count, uint16_t timeoutMs)
{
    uint8_t got = 0;
    const uint32_t start = millis();

    while (got < count) {
        if (Serial.available() > 0) {
            buffer[got++] = static_cast<uint8_t>(Serial.read());
            continue;
        }
        if (millis() - start > timeoutMs) {
            return false;
        }
    }

    return true;
}

/* After a failed transfer the PC may still be sending: eat what is left so a
 * 0x01 inside the image cannot start a bogus session from the main loop */
void drainSerial(uint16_t ms)
{
    const uint32_t start = millis();

    while (millis() - start < ms) {
        while (Serial.available() > 0) {
            Serial.read();
        }
    }
}

void writeLe32(uint8_t *at, uint32_t value)
{
    at[0] = static_cast<uint8_t>(value);
    at[1] = static_cast<uint8_t>(value >> 8);
    at[2] = static_cast<uint8_t>(value >> 16);
    at[3] = static_cast<uint8_t>(value >> 24);
}

bool sendOtaPacket(OtaKind kind, uint16_t seq, const uint8_t *data, uint8_t length)
{
    OtaPacket packet = {};

    packet.tag = static_cast<uint8_t>(PacketTag::Ota);
    packet.kind = static_cast<uint8_t>(kind);
    packet.seq = seq;

    if (data != nullptr && length > 0) {
        memcpy(packet.data, data, length);
    }

    return radio_send(radio, &packet, sizeof(packet), kTxTimeoutMs);
}

bool sendOtaBegin(uint32_t length, uint32_t crc)
{
    uint8_t header[10] = {};

    header[0] = kWireProtocolVersion;
    writeLe32(&header[2], length);
    writeLe32(&header[6], crc);
    return sendOtaPacket(OtaKind::Begin, 0, header, sizeof(header));
}

enum class OtaWait : uint8_t { Ok, Error, Timeout };

/* Wait for one of the car's rare answers. The radio acknowledgement only says
 * a packet reached the chip; this one carries what the car decided */
OtaWait waitOtaResponse(uint16_t seq, uint16_t timeoutMs, uint8_t &errorCode)
{
    const uint32_t start = millis();

    while (millis() - start < timeoutMs) {
        uint8_t pipe;
        uint8_t buffer[sizeof(OtaPacket)];
        OtaPacket packet;

        if (!radio.available(&pipe)) {
            continue;
        }

        radio.read(buffer, sizeof(buffer));
        if (buffer[0] == static_cast<uint8_t>(PacketTag::Log)) {
            /* The car's own lines explain a refusal, show them live */
            drainLogFragment(pipe, buffer);
            continue;
        }
        if (buffer[0] != static_cast<uint8_t>(PacketTag::Ota)) {
            continue; /* a packet meant for another reader */
        }

        memcpy(&packet, buffer, sizeof(packet));
        if (packet.kind == static_cast<uint8_t>(OtaKind::Error)) {
            errorCode = packet.data[0];
            return OtaWait::Error;
        }
        if (packet.kind == static_cast<uint8_t>(OtaKind::Ack) && packet.seq == seq) {
            return OtaWait::Ok;
        }
    }

    return OtaWait::Timeout;
}

/* The car only speaks up to complain, so a look between two packets is enough:
 * waiting here would give the channel away for nothing */
bool pollReceiverError(uint8_t &errorCode)
{
    uint8_t pipe;

    while (radio.available(&pipe)) {
        uint8_t buffer[sizeof(OtaPacket)];
        OtaPacket packet;

        radio.read(buffer, sizeof(buffer));
        if (buffer[0] == static_cast<uint8_t>(PacketTag::Log)) {
            drainLogFragment(pipe, buffer);
            continue;
        }
        if (buffer[0] != static_cast<uint8_t>(PacketTag::Ota)) {
            continue;
        }

        memcpy(&packet, buffer, sizeof(packet));
        if (packet.kind == static_cast<uint8_t>(OtaKind::Error)) {
            errorCode = packet.data[0];
            return true;
        }
    }

    return false;
}

/* The OTA entry borrows the box mode to reach the car. This puts the tool back
 * where the operator left it on every exit path, so a quiet tool stays quiet
 * and a car emulator resumes its telemetry once the transfer is over */
class OtaModeGuard {
public:
    OtaModeGuard(uint8_t entryMode, bool &carRestarts)
            : _entryMode(entryMode)
            , _carRestarts(carRestarts)
    {
    }

    ~OtaModeGuard()
    {
        if (gMode == _entryMode) {
            return;
        }

        /* An accepted image makes the car reset, and its boot lines ride _box_:
         * stay in the box mode for the window the PC keeps the port open, then
         * give the operator's mode back from the loop */
        if (_carRestarts) {
            gPendingMode = _entryMode;
            gPendingModeAt = millis();
            return;
        }

        setMode(_entryMode);
    }

private:
    uint8_t _entryMode;
    bool &_carRestarts;
};

void runOtaStream()
{
    uint8_t header[8];
    uint8_t chunk[sizeof(OtaPacket::data)];
    uint32_t length = 0;
    uint32_t crc = 0;
    uint32_t expected = 0;
    uint32_t received = 0;
    uint32_t streamCrc = kCrc32Init;
    uint16_t seq = 0;
    uint16_t radioRetries = 0;
    uint8_t error = 0;
    bool carRestarts = false;
    const OtaModeGuard restoreMode(gMode, carRestarts);

    /* A device command leaves on the car pipe, whatever the current mode */
    if (gMode != kModeBox) {
        setMode(kModeBox);
    }

    if (!readExact(header, sizeof(header), kOtaTimeoutMs)) {
        Serial.println(F("[ota] no header, dropped"));
        return;
    }

    length = static_cast<uint32_t>(header[0]) | (static_cast<uint32_t>(header[1]) << 8) |
             (static_cast<uint32_t>(header[2]) << 16) | (static_cast<uint32_t>(header[3]) << 24);
    crc = static_cast<uint32_t>(header[4]) | (static_cast<uint32_t>(header[5]) << 8) |
          (static_cast<uint32_t>(header[6]) << 16) | (static_cast<uint32_t>(header[7]) << 24);

    expected = length;

    Serial.print(F("[ota] image "));
    Serial.print(length);
    Serial.print(F(" bytes, crc 0x"));
    Serial.println(crc, HEX);

    /* The radio acknowledgement is not the signal that matters here: the car
     * erases the staging sector for two seconds and its FIFO refuses packets
     * meanwhile. What counts is the application's answer, after it */
    bool accepted = false;
    for (uint8_t attempt = 0; attempt < 3 && !accepted; ++attempt) {
        Serial.print(F("[ota] begin attempt "));
        Serial.print(attempt + 1);
        Serial.println(sendOtaBegin(length, crc) ? F(", the radio acknowledged it") :
                                                   F(", no answer from the chip"));

        const OtaWait begin = waitOtaResponse(kOtaBeginAck, kOtaBeginAckMs, error);
        if (begin == OtaWait::Ok) {
            accepted = true;
        } else if (begin == OtaWait::Error) {
            Serial.print(F("[ota] begin refused, error "));
            Serial.println(error);
            Serial.write(kOtaNak);
            drainSerial(300);
            return;
        }
    }

    if (!accepted) {
        Serial.println(F("[ota] begin not accepted"));
        Serial.write(kOtaNak);
        drainSerial(300);
        return;
    }

    Serial.println(F("[ota] begin accepted"));
    /* The image waits until the car is listening again: the PC takes this byte
     * as the order to start, and would otherwise overflow our serial buffer
     * while we are still busy with the answer above */
    Serial.write(kOtaAck);

    while (length > 0) {
        const uint8_t take = length > sizeof(chunk) ? sizeof(chunk) : static_cast<uint8_t>(length);
        bool delivered = false;

        if (!readExact(chunk, take, kOtaTimeoutMs)) {
            /* The PC cannot be blamed any more: it never sends more than the
             * credits below allow, so a shortfall here means the serial link
             * itself lost bytes */
            Serial.print(F("[ota] stream stopped after "));
            Serial.print(received);
            Serial.print(F(" of "));
            Serial.print(expected);
            Serial.print(F(" bytes, radio retries "));
            Serial.println(radioRetries);
            Serial.write(kOtaNak);
            drainSerial(300);
            return;
        }

        /* The tail of the last packet is padded with the erased-flash value;
         * the receiver only counts the bytes the record covers */
        memset(&chunk[take], 0xFF, sizeof(chunk) - take);

        for (uint8_t attempt = 0; attempt < kOtaWriteAttempts && !delivered; ++attempt) {
            delivered = sendOtaPacket(OtaKind::Data, seq, chunk, sizeof(chunk));
            if (!delivered) {
                ++radioRetries;
                /* The car reads one packet per loop, so a full FIFO needs a
                 * few milliseconds to free a slot: burning the three attempts
                 * at once would report a refusal it could have avoided */
                delay(3);
            }
        }

        if (!delivered) {
            Serial.print(F("[ota] radio refused packet "));
            Serial.println(seq);
            Serial.write(kOtaNak);
            drainSerial(300);
            return;
        }

        if (pollReceiverError(error)) {
            Serial.print(F("[ota] receiver error "));
            Serial.println(error);
            Serial.write(kOtaNak);
            drainSerial(300);
            return;
        }

        received += take;
        /* Only the bytes the image really covers: the tail of the last packet
         * is padding, and the PC's CRC does not include it */
        streamCrc = crc32Update(streamCrc, chunk, take);
        ++seq;
        length -= take;

        /* The car reads one packet per loop, and its FIFO only holds three:
         * sending the next 28 bytes immediately would fill it and the packet
         * after that would be refused. One millisecond keeps the pipe full
         * without overrunning it */
        delay(1);

        /* One credit per full group of packets. The PC may not send more than
         * its credits allow, which keeps this board's serial buffer from
         * overflowing: without it the PC runs ahead, the buffer fills, and the
         * UART drops the surplus without anyone noticing */
        if (seq % kOtaCreditPackets == 0) {
            Serial.write(kOtaCredit);
        }
    }

    /* The PC is the authority on the image, so the relay only has to check
     * that what it received is what the PC announced. Catching it here says
     * the loss happened on the serial link, not in the car */
    streamCrc = crc32Finish(streamCrc);
    if (streamCrc != crc) {
        Serial.print(F("[ota] stream corrupt: crc 0x"));
        Serial.print(streamCrc, HEX);
        Serial.print(F(" instead of 0x"));
        Serial.println(crc, HEX);
        Serial.write(kOtaNak);
        drainSerial(300);
        return;
    }

    bool ended = false;
    for (uint8_t attempt = 0; attempt < 3 && !ended; ++attempt) {
        sendOtaPacket(OtaKind::End, seq, nullptr, 0);

        const OtaWait wait = waitOtaResponse(seq, kOtaEndAckMs, error);
        if (wait == OtaWait::Ok) {
            ended = true;
        } else if (wait == OtaWait::Error) {
            Serial.print(F("[ota] receiver error "));
            Serial.println(error);
            break;
        }
    }

    carRestarts = ended;
    Serial.println(ended ? F("[ota] image accepted") : F("[ota] end not accepted"));
    Serial.write(ended ? kOtaAck : kOtaNak);
}
void runOtaSelfTest()
{
    uint8_t chunk[sizeof(OtaPacket::data)];
    bool carRestarts = false;
    const OtaModeGuard restoreMode(gMode, carRestarts);

    if (gMode != kModeBox) {
        setMode(kModeBox);
    }

    for (uint8_t index = 0; index < sizeof(chunk); ++index) {
        chunk[index] = index;
    }

    Serial.println(F("[ota] test image: Begin, two Data, End"));
    Serial.print(F("[ota] begin "));
    const bool beginSent = sendOtaBegin(2 * sizeof(chunk), 0x12345678UL);
    Serial.println(beginSent ? F("ACK") : F("no ACK"));

    if (!beginSent) {
        return;
    }

    /* The Begin makes the car erase its staging sector for about two seconds.
     * Its RF FIFO only holds three packets and nobody drains it while the app
     * is busy, so sending the rest now would have the End dropped without an
     * acknowledgement. Wait for the application answer first, like the real
     * relay does */
    uint8_t error = 0;
    const OtaWait begin = waitOtaResponse(kOtaBeginAck, kOtaBeginAckMs, error);
    Serial.println(begin == OtaWait::Ok ? F("[ota] begin accepted") :
                                          F("[ota] begin refused or silent"));

    if (begin != OtaWait::Ok) {
        return;
    }

    for (uint16_t seq = 0; seq < 2; ++seq) {
        chunk[0] = static_cast<uint8_t>(seq);
        Serial.print(F("[ota] data "));
        Serial.print(seq);
        Serial.print(' ');
        Serial.println(sendOtaPacket(OtaKind::Data, seq, chunk, sizeof(chunk)) ? F("ACK") :
                                                                                 F("no ACK"));
        /* Leave the car time to read the packet and program its 28 bytes */
        delay(10);
    }

    Serial.print(F("[ota] end "));
    Serial.println(sendOtaPacket(OtaKind::End, 2, nullptr, 0) ? F("ACK") : F("no ACK"));
}

/* Called from the loop: once the window that reads the car's boot lines is
 * over, put the tool back in the mode the operator had */
void otaService()
{
    if (gPendingMode == 0 || (millis() - gPendingModeAt) < kPostOtaModeDelayMs) {
        return;
    }

    const uint8_t mode = gPendingMode;

    gPendingMode = 0;
    Serial.print(F("[ota] log window over, back to mode "));
    Serial.println(mode);
    setMode(mode);
}
