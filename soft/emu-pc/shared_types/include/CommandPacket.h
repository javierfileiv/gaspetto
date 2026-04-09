#pragma once

#include "CommandId.h"

#include <array>
#include <cstddef>
#include <cstdint>

constexpr std::size_t BOX_TOTAL_SLOTS = 20;
constexpr std::size_t BOX_MAIN_SLOTS = 14;
constexpr std::size_t BOX_LOOP_SLOTS = 6;
constexpr std::size_t BOX_LED_SLOTS = 3;
constexpr std::size_t BOX_MAX_PAYLOAD_COMMANDS = 31;

enum class BoxPieceId : uint8_t {
    EMPTY = 0,
    FORWARD,
    BACKWARD,
    TURN_RIGHT,
    TURN_LEFT,
    STOP,
    LOOP_CALL,
    INVALID,
};

using BoxBoardPieces = std::array<BoxPieceId, BOX_TOTAL_SLOTS>;

struct __attribute__((packed)) CommandPacket {
    uint8_t count;
    uint8_t commands[BOX_MAX_PAYLOAD_COMMANDS];
};

static_assert(sizeof(CommandPacket) == 32, "CommandPacket must fit in a single nRF24 payload");
