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

    /* A lost acknowledgement makes the sender repeat the Begin; erasing the
     * sector again would cost two seconds for nothing */
    if (_active && _length == length && _crc == crc) {
        respond(OtaKind::Ack, kOtaBeginAck, OtaErrorCode::None);
        return;
    }

    _length = length;
    _crc = crc;
    _expected = 0;
    _active = true;
    respond(OtaKind::Ack, kOtaBeginAck, OtaErrorCode::None);
}

void OtaReceiver::handleData(const OtaPacket &packet)
{
    if (!_active) {
        respond(OtaKind::Error, packet.seq, OtaErrorCode::SequenceGap);
        return;
    }
    if (packet.seq < _expected) {
        /* A retransmission of something already written: acknowledge it again
         * so the sender moves on instead of restarting the session */
        respond(OtaKind::Ack, packet.seq, OtaErrorCode::None);
        return;
    }
    if (packet.seq > _expected) {
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
    respond(OtaKind::Ack, packet.seq, OtaErrorCode::None);
}

void OtaReceiver::handleEnd(const OtaPacket &packet)
{
    if (!_active) {
        respond(OtaKind::Error, 0, OtaErrorCode::SequenceGap);
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

    _radio.sendBuffer(&packet, sizeof(packet));
}
