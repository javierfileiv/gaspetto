#pragma once

#include "AppRecord.h"
#include "FlashLayout.h"
#include "FlashWriter.h"

#include <string.h>

/* The validation record lives alone in its sector, so writing it costs one
 * 16 kB erase and never touches the image it describes. The caller unlocks the
 * flash before and locks it after: a transfer writes the record right after
 * the staging area, and unlocking twice would only be noise */
inline bool recordWrite(const AppRecord *record)
{
    const uint8_t *bytes = reinterpret_cast<const uint8_t *>(record);

    if (!flashEraseSector(kRecordAddress)) {
        return false;
    }

    for (uint32_t offset = 0; offset < sizeof(AppRecord); offset += 4) {
        uint32_t word = 0;

        for (uint32_t byte = 0; byte < 4; ++byte) {
            word |= static_cast<uint32_t>(bytes[offset + byte]) << (byte * 8);
        }

        if (!flashProgramWord(kRecordAddress + offset, word)) {
            return false;
        }
    }

    return true;
}

inline void recordRead(AppRecord *record)
{
    memcpy(record, flashPointer(kRecordAddress), sizeof(AppRecord));
}
