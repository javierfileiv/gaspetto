#pragma once

#include "OtaProtocol.h"

class RadioController;

/* Receives a new application image over the radio and stages it in sector 5.
 * The application runs from its own sectors, so writing the staging area is
 * safe; once the whole image is there and its CRC matches, the receiver writes
 * the boot-request record and resets, and the bootloader copies it into place */
class OtaReceiver {
public:
    explicit OtaReceiver(RadioController &radio);

    /* Called by the radio controller for every OTA packet */
    void handle(const OtaPacket &packet);

private:
    void handleBegin(const OtaPacket &packet);
    void handleData(const OtaPacket &packet);
    void handleEnd(const OtaPacket &packet);
    void respond(OtaKind kind, uint16_t seq, OtaErrorCode error);

    RadioController &_radio;
    uint32_t _length = 0;
    uint32_t _crc = 0;
    uint16_t _expected = 0;
    bool _active = false;
};
