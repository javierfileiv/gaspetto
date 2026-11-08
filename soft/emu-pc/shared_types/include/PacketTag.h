#pragma once

#include <stdint.h>

/* First byte of every radio packet that is not a program. A CommandPacket
 * keeps its command count in that byte instead and the count never exceeds
 * 31, so the tags sit well above it: the receiver tells the packet kinds
 * apart with one comparison on byte 0 instead of guessing from byte 1,
 * which is what the old heuristic got wrong for an event carrying a low
 * CommandId (it decoded it as a program) */
enum class PacketTag : uint8_t {
    Event = 0x80,
    Telemetry = 0x81,
    Log = 0x82,
};
