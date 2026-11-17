#include "OtaReceiver.h"

#include "AppRecord.h"
#include "Crc32.h"
#include "FlashLayout.h"
#include "FlashWriter.h"
#include "Log.h"
#include "PacketTag.h"
#include "RadioController.h"
#include "RecordStore.h"

#include <Arduino.h>

namespace
{
/* The radio gives up on a missing hardware acknowledgement after about twenty
 * milliseconds, so three tries stay well inside the sender's own timeout */
constexpr uint8_t kAnswerAttempts = 3;

uint32_t readLe32(const uint8_t *at)
{
    return static_cast<uint32_t>(at[0]) | (static_cast<uint32_t>(at[1]) << 8) |
           (static_cast<uint32_t>(at[2]) << 16) | (static_cast<uint32_t>(at[3]) << 24);
}
} /* namespace */

OtaReceiver::OtaReceiver(RadioController &radio)
        : _radio(radio)
{
}

void OtaReceiver::handle(const OtaPacket &packet)
{
    switch (static_cast<OtaKind>(packet.kind)) {
    case OtaKind::Begin:
        handleBegin(packet);
        break;
    case OtaKind::Data:
        handleData(packet);
        break;
    case OtaKind::End:
        handleEnd(packet);
        break;
    default:
        /* Ack and Error belong to the sending side */
        break;
    }
}

void OtaReceiver::handleBegin(const OtaPacket &packet)
{
    const uint8_t version = packet.data[0];
    const uint32_t length = readLe32(&packet.data[2]);
    const uint32_t crc = readLe32(&packet.data[6]);

    if (version != kWireProtocolVersion) {
        respond(OtaKind::Error, 0, OtaErrorCode::ProtocolMismatch);
        return;
    }
    if (length == 0 || length > kStagingCapacity) {
        respond(OtaKind::Error, 0, OtaErrorCode::TooLarge);
        return;
    }

    LOG(F("OTA: begin "));
    LOG(length);
    LOGLN(F(" bytes"));

    /* The sector has to be erased before anything can be programmed, and the
     * erase stalls the CPU for about two seconds */
    flashUnlock();
    const bool erased = flashEraseSector(kStagingAddress);
    flashLock();

    if (!erased) {
        respond(OtaKind::Error, 0, OtaErrorCode::FlashFailure);
        return;
    }

    /* A repeated Begin is a restart, not a duplicate: the sender repeats it
     * when the previous answer was lost, and by then the sector is already
     * erased, so counting from zero again costs nothing */
    _length = length;
    _crc = crc;
    _expected = 0;
    _active = true;
    respond(OtaKind::Ack, kOtaBeginAck, OtaErrorCode::None);
}

void OtaReceiver::handleData(const OtaPacket &packet)
{
    /* No answer here on purpose. The radio acknowledges every packet at the
     * hardware level, which is what paces the stream, and answering from the
     * application as well would force two turnarounds per packet on a channel
     * that only one side can use at a time */
    if (!_active) {
        return;
    }
    if (packet.seq < _expected) {
        /* The sender retried something already written: its acknowledgement
         * was lost, the bytes are in flash, and silence is the right answer */
        return;
    }
    if (packet.seq != _expected) {
        /* A packet really did go missing: the stream is past it, so this
         * session cannot produce a valid image and the sender has to restart */
        LOGLN(F("OTA: sequence gap"));
        _active = false;
        respond(OtaKind::Error, _expected, OtaErrorCode::SequenceGap);
        return;
    }

    const uint32_t offset = static_cast<uint32_t>(packet.seq) * kOtaPayload;
    bool ok = true;

    flashUnlock();
    for (uint32_t word = 0; word < kOtaPayload / 4 && ok; ++word) {
        const uint32_t value = readLe32(&packet.data[word * 4]);
        ok = flashProgramWord(kStagingAddress + offset + word * 4, value);
    }
    flashLock();

    if (!ok) {
        LOGLN(F("OTA: data write failed"));
        _active = false;
        respond(OtaKind::Error, packet.seq, OtaErrorCode::FlashFailure);
        return;
    }

    ++_expected;
}

void OtaReceiver::handleEnd(const OtaPacket &packet)
{
    if (!_active) {
        /* The session was abandoned on a gap or a bad write, and the sender
         * already got that answer: repeating it would only cost a packet */
        return;
    }

    const uint16_t total = static_cast<uint16_t>((_length + kOtaPayload - 1) / kOtaPayload);
    if (_expected != total) {
        respond(OtaKind::Error, _expected, OtaErrorCode::SequenceGap);
        return;
    }

    const uint32_t actual = crc32(flashPointer(kStagingAddress), _length);
    if (actual != _crc) {
        LOGLN(F("OTA: CRC mismatch"));
        _active = false;
        respond(OtaKind::Error, 0, OtaErrorCode::CrcMismatch);
        return;
    }

    AppRecord record = {};

    record.magic = kRecordMagic;
    record.format = kRecordFormat;
    record.flags = kRecordFlagCopyPending;
    record.length = _length;
    record.crc = _crc;

    flashUnlock();
    const bool written = recordWrite(&record);
    flashLock();

    if (!written) {
        _active = false;
        respond(OtaKind::Error, 0, OtaErrorCode::FlashFailure);
        return;
    }

    LOGLN(F("OTA: image accepted, restarting"));

    _active = false;
    respond(OtaKind::Ack, packet.seq, OtaErrorCode::None);

#ifdef ARDUINO
    /* Let the acknowledgement leave before the bootloader takes over */
    delay(50);
    NVIC_SystemReset();
#endif
}

void OtaReceiver::respond(OtaKind kind, uint16_t seq, OtaErrorCode error)
{
    OtaPacket packet = {};

    packet.tag = static_cast<uint8_t>(PacketTag::Ota);
    packet.kind = static_cast<uint8_t>(kind);
    packet.seq = seq;
    if (kind == OtaKind::Error) {
        packet.data[0] = static_cast<uint8_t>(error);
    }

    /* The sender asks for the next packet as soon as this answer lands, so the
     * channel is ours and a hardware acknowledgement that did not come means
     * it was still turning around from its own transmission */
    for (uint8_t attempt = 0; attempt < kAnswerAttempts; ++attempt) {
        if (_radio.sendBuffer(&packet, sizeof(packet))) {
            break;
        }
        delay(1);
    }
}
